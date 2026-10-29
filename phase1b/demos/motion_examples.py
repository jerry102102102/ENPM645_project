"""Scripted A/B/C motion examples; no learned intent, expert or wrench allocation."""

import argparse
import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np

from phase0_tactile.scripts.run_experiment import clip_norm, rotate_y, smooth
from phase0_v2.simulation.dual_gripper import point_wrench
from phase1b.scripts.build_scene import solve
from phase1b.simulation import CONFIG, ROOT, PandaWorld

DURATION = 8.0
SCHEDULES = {
    "A": dict(
        name="Rotate and hold", angle_deg=20.0, lift_m=0.03, external_force_N=0.0
    ),
    "B": dict(name="Lift then hold", angle_deg=0.0, lift_m=0.08, external_force_N=0.0),
    "C": dict(
        name="Reposition and resist constant load",
        angle_deg=-12.0,
        lift_m=0.0,
        external_force_N=20.0,
    ),
}


def pose_schedule(t, case, human=False):
    # Human target leads the predetermined arm motion; this is not an inference controller.
    u, du = smooth(t, 0.6 if human else 1.4, 2.4)
    cfg = SCHEDULES[case]
    return (
        np.deg2rad(cfg["angle_deg"]) * u,
        cfg["lift_m"] * u,
        np.deg2rad(cfg["angle_deg"]) * du,
        cfg["lift_m"] * du,
    )


class DemoWorld(PandaWorld):
    def __init__(self, case):
        super().__init__("B")
        self.case = case

    def refresh(self, t, scenario=None):
        if t < 0:
            return super().refresh(t, "B")
        m, d, p = self.m, self.d, self.p
        d.qfrc_applied[:] = 0
        d.xfrc_applied[:] = 0
        mujoco.mj_fwdPosition(m, d)
        mujoco.mj_fwdVelocity(m, d)
        angle, dz, omega, vz = pose_schedule(t, self.case, True)
        goal = p["initial_angle"] + angle
        radius = rotate_y(goal) @ self.human_local
        target = self.center + np.array([0, 0, dz]) + radius
        velocity = clip_norm(
            np.cross([0, omega, 0], radius) + [0, 0, vz], p["human_target_speed_limit"]
        )
        hp = d.site_xpos[self.hsite].copy()
        jac = np.zeros((3, m.nv))
        mujoco.mj_jacSite(m, d, jac, None, self.hsite)
        fh = clip_norm(
            p["kh"] * (target - hp) + p["dh"] * (velocity - jac @ d.qvel),
            p["human_force_limit"],
        )
        ep = d.xpos[self.panel] + d.xmat[self.panel].reshape(3, 3) @ np.array(
            p["disturbance_point"]
        )
        u, _ = smooth(t, 4.0, 0.3)
        fe = np.array([0.0, 0.0, -SCHEDULES[self.case]["external_force_N"] * u])
        for force, point in ((fh, hp), (fe, ep)):
            mujoco.mj_applyFT(
                m, d, force, np.zeros(3), point, self.panel, d.qfrc_applied
            )
        assert not np.any(d.qfrc_applied[self.va]) and not np.any(d.xfrc_applied)
        mujoco.mj_forward(m, d)
        self.truth = dict(
            human_goal=goal,
            human_target_trajectory=target,
            human_application_point=hp,
            human_applied_force=fh,
            external_application_point=ep,
            external_disturbance_force=fe,
            human_applied_wrench=point_wrench(fh, hp, d.xipos[self.panel]),
            external_disturbance_wrench=point_wrench(fe, ep, d.xipos[self.panel]),
        )
        return self.truth


def plan(case):
    w = PandaWorld()
    q = []
    errors = []
    # Only this separate planning instance has qpos assigned by IK.
    for t in np.arange(0, DURATION + 0.005, 0.01):
        angle, z, _, _ = pose_schedule(t, case)
        R = rotate_y(w.p["initial_angle"] + angle)
        center = w.center + [0, 0, z]
        for arm, (side, x) in enumerate((("left", -0.2), ("right", 0.2))):
            result = solve(
                w.m,
                w.d,
                side,
                center + R @ np.array([x, 0.1, 0]),
                R,
                w.d.qpos[w.qa[arm]].copy(),
            )
            if result is None:
                raise RuntimeError(f"IK failed {case} {side} {t}")
            errors.append(result["position_error_m"])
        q.append(w.d.qpos[w.qa].copy())
    return np.array(q), float(max(errors))


def collect(case, output):
    output = Path(output)
    if output.exists():
        raise FileExistsError(output)
    trajectory, ikerror = plan(case)
    w = DemoWorld(case)
    w.settle()
    records = []
    diagnostics = []
    pad_ids = sum(w.pad_g, [])
    for tick in range(round(DURATION / 0.002)):
        t = tick * 0.002
        index = t / 0.01
        lower = int(np.floor(index))
        fraction = index - lower
        command = (1 - fraction) * trajectory[lower] + fraction * trajectory[
            min(lower + 1, len(trajectory) - 1)
        ]
        # Drive through joint actuators only; no kinematic writes to execution state.
        w.set_commands(command, np.array([-40.0, 40.0, -40.0, 40.0]))
        w.refresh(t)
        o, diag = w.snapshot()
        unexpected = []
        for c in w.d.contact:
            expected = w.pg in (c.geom1, c.geom2) and (
                c.geom1 in pad_ids or c.geom2 in pad_ids
            )
            if not expected and c.dist < -1e-5:
                unexpected.append(float(-c.dist))
        margin = np.minimum(
            w.d.qpos[w.qa] - w.m.jnt_range[w.arm_joints, 0],
            w.m.jnt_range[w.arm_joints, 1] - w.d.qpos[w.qa],
        )
        total = (
            o["object_contact_wrench"]
            + w.truth["human_applied_wrench"]
            + w.truth["external_disturbance_wrench"]
        )
        residual = np.linalg.norm(
            w.m.body_mass[w.panel] * (diag["panel_acceleration"] - w.m.opt.gravity)
            - total[:3]
        )
        requested = (
            w.m.actuator_gainprm[w.aa, 0] * command
            + w.m.actuator_biasprm[w.aa, 1] * w.d.qpos[w.qa]
            + w.m.actuator_biasprm[w.aa, 2] * w.d.qvel[w.va]
        )
        diagnostics.append(
            dict(
                timestamp=t,
                contact_counts=np.r_[o["contact_state_left"], o["contact_state_right"]],
                max_pad_penetration=diag["penetration"],
                unexpected_penetration=max(unexpected, default=0),
                minimum_joint_margin=margin.min(),
                saturated=np.any(np.abs(requested - w.d.actuator_force[w.aa]) > 1e-8),
                force_balance_residual=residual,
                warnings=w.d.warning.number.copy(),
            )
        )
        if tick % 5 == 0:
            R = w.d.xmat[w.panel].reshape(3, 3)
            records.append(
                dict(
                    **o,
                    **w.truth,
                    qpos=w.d.qpos.copy(),
                    qvel=w.d.qvel.copy(),
                    arm_position_command=command.copy(),
                    panel_pitch_deg=np.rad2deg(np.arctan2(-R[2, 0], R[0, 0])),
                    desired_assist_torque=np.nan,
                    expert_label_valid=False,
                    command_source="scripted_joint_motion",
                )
            )
        w.step()
    arrays = {k: np.array([r[k] for r in records]) for k in records[0]}
    d = {k: np.array([r[k] for r in diagnostics]) for k in diagnostics[0]}
    t = arrays["timestamp"]
    late = t >= 5.5
    angle_change = float(
        np.mean(arrays["panel_pitch_deg"][late]) - arrays["panel_pitch_deg"][0]
    )
    lift = float(
        np.mean(arrays["object_position"][late, 2]) - arrays["object_position"][0, 2]
    )
    joint_change = float(
        np.max(np.abs(arrays["arm_joint_position"] - arrays["arm_joint_position"][0]))
    )
    peak = int(np.argmax(np.linalg.norm(arrays["human_applied_force"], axis=1)))
    if case == "C":
        np.testing.assert_array_equal(
            arrays["external_disturbance_force"][t >= 4.3],
            np.tile([0.0, 0.0, -20.0], (int((t >= 4.3).sum()), 1)),
        )
    metrics = dict(
        human_peak_time_s=float(t[peak]),
        human_peak_force_vector_N=arrays["human_applied_force"][peak].tolist(),
        case=case,
        name=SCHEDULES[case]["name"],
        samples=len(records),
        physics_steps=len(diagnostics),
        observed_pitch_change_deg=angle_change,
        observed_lift_m=lift,
        max_joint_change_rad=joint_change,
        max_joint_speed_rad_s=float(np.abs(arrays["arm_joint_velocity"]).max()),
        min_contacts=d["contact_counts"].min(axis=0).tolist(),
        max_pad_penetration_m=float(d["max_pad_penetration"].max()),
        unexpected_penetration_m=float(d["unexpected_penetration"].max()),
        min_joint_margin_rad=float(d["minimum_joint_margin"].min()),
        saturated_steps=int(d["saturated"].sum()),
        max_force_balance_residual_N=float(d["force_balance_residual"].max()),
        max_human_force_N=float(
            np.linalg.norm(arrays["human_applied_force"], axis=1).max()
        ),
        late_human_force_mean_N=arrays["human_applied_force"][late]
        .mean(axis=0)
        .tolist(),
        late_external_force_N=arrays["external_disturbance_force"][late]
        .mean(axis=0)
        .tolist(),
        hold_pitch_range_deg=float(np.ptp(arrays["panel_pitch_deg"][late])),
        hold_z_range_m=float(np.ptp(arrays["object_position"][late, 2])),
        ik_max_position_error_m=ikerror,
    )
    expected_motion = (
        abs(angle_change) > 15
        if case == "A"
        else lift > 0.06
        if case == "B"
        else angle_change < -8
    )
    metrics["passed"] = bool(
        expected_motion
        and joint_change > 0.05
        and d["contact_counts"].min() >= 1
        and d["max_pad_penetration"].max() < 0.001
        and d["unexpected_penetration"].max() == 0
        and d["minimum_joint_margin"].min() > 0
        and not d["warnings"].any()
        and d["force_balance_residual"].max() < 1e-5
        and metrics["hold_pitch_range_deg"] < 0.2
        and metrics["hold_z_range_m"] < 0.002
    )
    output.mkdir(parents=True)
    np.savez_compressed(output / "samples.npz", **arrays)
    np.savez_compressed(output / "physics.npz", **d)
    np.savez_compressed(
        output / "planned_commands.npz",
        timestamps=np.arange(len(trajectory)) * 0.01,
        arm_position_command=trajectory,
    )
    meta = dict(
        demo_version="scripted-motion/1.0",
        scenario_version="3.0.0",
        hardware_version="2.0.0",
        schedule=SCHEDULES[case],
        duration_s=DURATION,
        human_motion_start_s=0.6,
        robot_motion_start_s=1.4,
        motion_duration_s=2.4,
        constant_load_start_s=4.3,
        external_ramp_start_s=4.0,
        external_ramp_duration_s=0.3,
        force_frame="world XYZ; forces in N; visualization conventions stored separately",
        scope="Scripted motion/load demonstration only, not intent inference, force tracking or expert data",
        clock="Samples are same-time post-command/pre-step snapshots at 100 Hz; physics diagnostics at 500 Hz",
        hashes={
            str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in [
                Path(__file__),
                CONFIG,
                ROOT / w.config["model"],
                ROOT / "phase1b/config/hardware_v2.json",
                ROOT / "phase1b/simulation.py",
                ROOT / "phase1b/scripts/build_scene.py",
            ]
        },
        metrics=metrics,
    )
    (output / "report.json").write_text(json.dumps(meta, indent=2) + "\n")
    print(json.dumps(metrics), flush=True)
    return metrics


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--case", choices=list("ABC"))
    a = p.parse_args()
    if a.output.exists():
        p.error("choose a fresh output directory")
    for case in a.case or "ABC":
        result = collect(case, a.output / case)
        if not result["passed"]:
            raise RuntimeError("Motion demonstration failed acceptance: " + case)


if __name__ == "__main__":
    main()
