"""Articulated dual Panda baseline: arm position actuators, custom tactile pads."""

import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np

from phase1b.geometry import clip_norm, point_wrench, rotate_y, smooth
from phase1b.hardware import snapshot, verify_vendor
from phase1b.sensing import observe

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "phase1b/config/dual_panda_v1.json"


class PandaWorld:
    def __init__(self, scenario="B"):
        if scenario not in "ABC" or len(scenario) != 1:
            raise ValueError("scenario must be A, B or C")
        self.config = json.loads(CONFIG.read_text())
        path = ROOT / self.config["model"]
        if hashlib.sha256(path.read_bytes()).hexdigest() != self.config["model_sha256"]:
            raise ValueError("model hash mismatch")
        verify_vendor(ROOT / "phase1b/vendor/franka_emika_panda")
        if (
            hashlib.sha256(
                (
                    ROOT / "phase1b/vendor/franka_emika_panda/provenance.json"
                ).read_bytes()
            ).hexdigest()
            != self.config["vendor_manifest_hash"]
        ):
            raise ValueError("vendor manifest mismatch")
        self.scenario = scenario
        self.p = self.config["parameters"]
        self.m = mujoco.MjModel.from_xml_path(str(path))
        self.d = mujoco.MjData(self.m)
        m, d = self.m, self.d
        m.geom_friction[:, 0] = self.p["friction"]
        d.qpos[:] = self.config["qpos"]
        self.panel = m.body("panel").id
        self.pg = m.geom("panel_geom").id
        self.vadr = m.jnt_dofadr[m.joint("panel_free").id]
        self.roots = [m.body(s).id for s in ("left", "right")]
        self.pad_names = [
            [s + "_" + p for p in ("upper", "lower")] for s in ("left", "right")
        ]
        self.pad_b = [[m.body(p).id for p in names] for names in self.pad_names]
        self.pad_g = [
            [m.geom(p + "_pad").id for p in names] for names in self.pad_names
        ]
        self.arm_joints = np.array(
            [
                [m.joint(n).id for n in self.config["arm_joint_names"][s]]
                for s in ("left", "right")
            ]
        )
        self.qa = m.jnt_qposadr[self.arm_joints]
        self.va = m.jnt_dofadr[self.arm_joints]
        self.aa = np.array(
            [
                [m.actuator(f"{s}_actuator{i}").id for i in range(1, 8)]
                for s in ("left", "right")
            ]
        )
        self.pad_joints = np.array(
            [[m.joint(n + "_slide").id for n in names] for names in self.pad_names]
        )
        self.pad_qa = m.jnt_qposadr[self.pad_joints]
        self.pad_va = m.jnt_dofadr[self.pad_joints]
        self.pad_aa = np.array(
            [m.actuator(n + "_preload").id for names in self.pad_names for n in names]
        )
        self.qtarget = d.qpos[self.qa].copy()
        self.hsite = m.site("human_grasp").id
        self.human_local = m.site_pos[self.hsite].copy()
        self.center = np.array([0.0, 0.0, 0.8])
        self.tick = 0
        self.set_commands(self.qtarget, np.array([-40.0, 40.0, -40.0, 40.0]))
        mujoco.mj_forward(m, d)
        hardware = ROOT / "phase1b/config/hardware_v2.json"
        if snapshot(self) != json.loads(hardware.read_text()):
            raise ValueError("compiled hardware mismatch")

    def set_commands(self, arm_position, pad_force):
        arm_position = np.asarray(arm_position)
        pad_force = np.asarray(pad_force)
        if (
            arm_position.shape != (2, 7)
            or pad_force.shape != (4,)
            or not np.isfinite(arm_position).all()
            or not np.isfinite(pad_force).all()
        ):
            raise ValueError("invalid command shape/value")
        ids = np.r_[self.aa.ravel(), self.pad_aa]
        values = np.r_[arm_position.ravel(), pad_force]
        if np.any(values < self.m.actuator_ctrlrange[ids, 0]) or np.any(
            values > self.m.actuator_ctrlrange[ids, 1]
        ):
            raise ValueError("command limit exceeded")
        self.d.ctrl[self.aa] = arm_position
        self.d.ctrl[self.pad_aa] = pad_force

    def environment_targets(self, t, scenario):
        """Runner-owned human/environment inputs; never controller observations."""
        p = self.p
        u, du = smooth(t, 0.7, p["ramp_duration"])
        delta = p["direction"] * p["target_angle"] if scenario == "A" else 0.0
        goal = p["initial_angle"] + delta * u
        radius = rotate_y(goal) @ self.human_local
        target = self.center + radius
        velocity = clip_norm(
            np.cross([0.0, delta * du, 0.0], radius), p["human_target_speed_limit"]
        )
        fe = np.zeros(3)
        if scenario == "C":
            on, _ = smooth(t, p["disturbance_start"], 0.15)
            off, _ = smooth(t, p["disturbance_end"], 0.15)
            fe[2] = -p["direction"] * p["disturbance_magnitude"] * (on - off)
        return goal, target, velocity, fe

    def refresh(self, t, scenario=None):
        scenario = self.scenario if scenario is None else scenario
        m, d, p = self.m, self.d, self.p
        d.qfrc_applied[:] = 0
        d.xfrc_applied[:] = 0
        mujoco.mj_fwdPosition(m, d)
        mujoco.mj_fwdVelocity(m, d)
        goal, target, velocity, fe = self.environment_targets(t, scenario)
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
        for force, point in [(fh, hp), (fe, ep)]:
            mujoco.mj_applyFT(
                m, d, force, np.zeros(3), point, self.panel, d.qfrc_applied
            )
        # Only the free panel receives the scripted human/environment forces.
        assert np.count_nonzero(d.qfrc_applied[self.va]) == 0
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

    def settle(self):
        for _ in range(round(self.p["settle_seconds"] / self.m.opt.timestep)):
            self.refresh(-1, "B")
            mujoco.mj_step(self.m, self.d)
        self.origin = float(self.d.time)
        self.tick = 0
        self.refresh(0)

    def step(self):
        mujoco.mj_step(self.m, self.d)
        self.tick += 1
        self.refresh(self.tick * self.m.opt.timestep)

    def snapshot(self):
        o, diag = observe(self)
        m, d = self.m, self.d
        data = {
            k: o[k]
            for k in (
                "object_pose",
                "object_velocity",
                "gripper_pose",
                "gripper_velocity",
            )
            if k in o
        }
        for side in ("left", "right"):
            data["wrist_ft_" + side] = o["wrist_ft_" + side]
            data["tactile_" + side] = o["tactile_" + side]
            data["contact_state_" + side] = o["contact_state_" + side]
            data["wrist_sensor_" + side] = np.r_[
                d.sensor(side + "_wrist_force").data,
                d.sensor(side + "_wrist_torque").data,
            ].copy()
        data.update(
            timestamp=self.tick * m.opt.timestep,
            engine_timestamp=float(d.time),
            physics_tick=self.tick,
            arm_joint_position=d.qpos[self.qa].copy(),
            arm_joint_velocity=d.qvel[self.va].copy(),
            arm_actuator_torque=d.qfrc_actuator[self.va].copy(),
            arm_actuator_force=d.actuator_force[self.aa].copy(),
            gripper_joint_position=d.qpos[self.pad_qa].copy(),
            gripper_joint_velocity=d.qvel[self.pad_va].copy(),
            gripper_actuator_force=d.actuator_force[self.pad_aa].copy(),
        )
        data["end_effector_pose"] = data["gripper_pose"].copy()
        data["object_position"] = o["object_pose"][:3].copy()
        data["object_orientation"] = o["object_pose"][3:].copy()
        data["object_angular_velocity"] = o["object_velocity"][:3].copy()
        data["object_linear_velocity"] = o["object_velocity"][3:].copy()
        data["object_contact_wrench"] = o["aggregated_object_wrench"].copy()
        data["gripper_velocity"] = np.zeros((2, 6))
        for g, b in enumerate(self.roots):
            mujoco.mj_objectVelocity(
                m, d, mujoco.mjtObj.mjOBJ_BODY, b, data["gripper_velocity"][g], 0
            )
        data["end_effector_velocity"] = data["gripper_velocity"].copy()
        for g, side in enumerate(("left", "right")):
            data["wrist_ft_" + side + "_world_at_panel_com"] = o[
                "independent_object_wrenches"
            ][g].copy()
        return data, diag
