"""Physics/contact checks for edge v2; legacy controls are reported separately."""

import argparse
import json
import tempfile
from pathlib import Path

import mujoco
import numpy as np

from phase0_v2.sensing.contacts import gripper_balance, observe
from phase1a.scripts.run_pipeline_smoke import run_episode
from phase1a.scripts.validate_dataset import compare_npz, validate_smoke
from phase1a.simulation.scenario_loader import (
    CONFIG,
    EDGE_CONFIG,
    REPO,
    load_scenario,
    make_sim,
)


def physical_checks():
    cfg = load_scenario(EDGE_CONFIG)
    episodes = {}
    for scenario in "ABC":
        sim = make_sim(cfg)
        sim.settle()
        m, d = sim.m, sim.d
        metrics = {
            "max_linear_balance_N": 0.0,
            "max_angular_balance_Nm": 0.0,
            "max_gripper_balance_abs_N_Nm": 0.0,
            "max_wrench_transform_error_N_Nm": 0.0,
            "max_penetration_m": 0.0,
            "max_grasp_frame_deviation_m": 0.0,
            "max_human_attachment_error_m": 0.0,
        }
        minimum = np.full((2, 2), 100)
        floor_count = 0
        for step in range(2000):
            truth, control = sim.forces(step * 0.002, scenario)
            o, diag = observe(sim)
            R = d.xmat[sim.panel].reshape(3, 3)
            inertia = m.body_inertia[sim.panel]
            omega = d.qvel[sim.vadr + 3 : sim.vadr + 6]
            moment = R @ (
                inertia * d.qacc[sim.vadr + 3 : sim.vadr + 6]
                + np.cross(omega, inertia * omega)
            )
            total = (
                o["aggregated_object_wrench"]
                + truth["human_applied_wrench"]
                + truth["external_disturbance_wrench"]
            )
            values = {
                "max_linear_balance_N": np.linalg.norm(
                    m.body_mass[sim.panel]
                    * (diag["panel_acceleration"] - m.opt.gravity)
                    - total[:3]
                ),
                "max_angular_balance_Nm": np.linalg.norm(moment - total[3:]),
                "max_gripper_balance_abs_N_Nm": np.max(
                    np.abs(
                        gripper_balance(
                            sim, o, control["executed_virtual_drive_wrenches"]
                        )
                    )
                ),
                "max_wrench_transform_error_N_Nm": diag["wrench_transform_error"],
                "max_penetration_m": diag["penetration"],
                "max_grasp_frame_deviation_m": max(
                    np.linalg.norm(
                        R.T @ (d.xpos[b] - d.xpos[sim.panel])
                        - cfg["scene"]["grippers"][side][
                            "nominal_grasp_position_panel_local"
                        ]
                    )
                    for side, b in zip(("left", "right"), sim.roots)
                ),
                "max_human_attachment_error_m": np.linalg.norm(
                    R.T @ (d.site_xpos[sim.hsite] - d.xpos[sim.panel]) - sim.human_local
                ),
            }
            metrics = {k: max(metrics[k], float(v)) for k, v in values.items()}
            minimum = np.minimum(
                minimum, [o["contact_state_left"], o["contact_state_right"]]
            )
            floor_count += any(
                m.geom("floor").id in (c.geom1, c.geom2) for c in d.contact
            )
            for side in ("left", "right"):
                assert (
                    np.isfinite(o["tactile_" + side]).all()
                    and np.linalg.norm(o["tactile_" + side]) > 0
                )
            mujoco.mj_step(m, d)
        assert np.all(minimum > 0) and floor_count == 0 and sum(d.warning.number) == 0
        for name in (
            "max_linear_balance_N",
            "max_angular_balance_Nm",
            "max_gripper_balance_abs_N_Nm",
            "max_wrench_transform_error_N_Nm",
        ):
            assert metrics[name] < 1e-8, (scenario, name, metrics[name])
        assert (
            metrics["max_penetration_m"] < 0.001
            and metrics["max_human_attachment_error_m"] < 1e-12
        )
        episodes[scenario] = {
            "passed": True,
            "physics_steps": 2000,
            "minimum_contacts_per_pad": minimum.tolist(),
            "floor_contact_steps": floor_count,
            "warnings": int(sum(d.warning.number)),
            **metrics,
        }
    controls = {}
    for mode, removed in [
        ("left_removed", [0]),
        ("right_removed", [1]),
        ("both_removed", [0, 1]),
        ("drives_disabled", []),
    ]:
        sim = make_sim(cfg)
        sim.settle()
        initial_z = float(sim.d.xpos[sim.panel, 2])
        for g in removed:
            sim.m.geom_contype[sim.pad_g[g]] = 0
            sim.m.geom_conaffinity[sim.pad_g[g]] = 0
        for step in range(100):
            sim.forces(
                -1.0, "B", disable_human=True, disable_robot=mode == "drives_disabled"
            )
            mujoco.mj_step(sim.m, sim.d)
        sim.forces(
            -1.0, "B", disable_human=True, disable_robot=mode == "drives_disabled"
        )
        o, _ = observe(sim)
        counts = [o["contact_state_left"].tolist(), o["contact_state_right"].tolist()]
        for g in removed:
            side = ("left", "right")[g]
            assert (
                sum(counts[g]) == 0
                and np.max(np.abs(o["wrist_ft_" + side])) == 0
                and np.max(np.abs(o["tactile_" + side])) == 0
            )
        if len(removed) == 1:
            assert sum(counts[1 - removed[0]]) > 0
        drop = initial_z - float(o["object_pose"][2])
        if mode in ("both_removed", "drives_disabled"):
            assert drop > 0.15
        controls[mode] = {
            "passed": True,
            "panel_drop_m": drop,
            "contact_counts": counts,
        }
    return {
        "episodes": episodes,
        "edge_contact_removal_controls": controls,
        "passed": True,
        "fixed_semantics": "Only nominal anchors and human material attachment are fixed. Robot grasp frames remain compliant; deviation is measured, not clamped.",
    }


def legacy_replay():
    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp) / "A"
        run_episode(out, "A", load_scenario(CONFIG))
        count = sum(
            compare_npz(REPO / "results/phase1a/smoke/A" / filename, out / filename)
            for filename in (
                "observations.npz",
                "next_observations.npz",
                "supervision.npz",
                "controller.npz",
                "privileged.npz",
            )
        )
    return {
        "passed": True,
        "historical_A_exact_arrays": count,
        "historical_datasets": validate_smoke(REPO / "results/phase1a/smoke"),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--output",
        type=Path,
        default=REPO / "results/phase1a/edge_v2/geometry_validation.json",
    )
    a = ap.parse_args()
    if a.output.exists():
        ap.error("choose a fresh report path")
    result = {
        "edge_geometry_physics": physical_checks(),
        "legacy_regression": legacy_replay(),
    }
    a.output.write_text(json.dumps(result, indent=2) + "\n")
    print(a.output, flush=True)


if __name__ == "__main__":
    main()
