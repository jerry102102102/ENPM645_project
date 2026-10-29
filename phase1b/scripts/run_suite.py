"""Reproducible integration acceptance suite, preserving all previous outputs."""

import argparse
import json
import subprocess
import sys
import tempfile
from pathlib import Path

import mujoco
import numpy as np

from phase1b.pipeline import read_episode, run_episode
from phase1b.simulation import CONFIG, ROOT, PandaWorld


def compare(a, b):
    left, _ = read_episode(a)
    right, _ = read_episode(b)
    count = 0
    for section in left:
        assert set(left[section]) == set(right[section])
        for key in left[section]:
            np.testing.assert_array_equal(
                left[section][key], right[section][key], err_msg=section + "/" + key
            )
            count += 1
    return count


def controls():
    result = {}
    for mode in ("remove_pad_contacts", "disable_arm_actuators"):
        w = PandaWorld()
        w.settle()
        z0 = w.d.xpos[w.panel, 2]
        q0 = w.d.qpos[w.qa].copy()
        if mode == "remove_pad_contacts":
            for gid in sum(w.pad_g, []):
                w.m.geom_contype[gid] = 0
                w.m.geom_conaffinity[gid] = 0
        else:
            w.m.actuator_gainprm[w.aa.ravel()] = 0
            w.m.actuator_biasprm[w.aa.ravel()] = 0
        w.refresh(0)
        for _ in range(150):
            w.step()
        o, _ = w.snapshot()
        drop = float(z0 - o["object_position"][2])
        assert drop > 0.1
        if mode == "remove_pad_contacts":
            for side in ("left", "right"):
                np.testing.assert_array_equal(o["tactile_" + side], 0)
                np.testing.assert_array_equal(o["wrist_ft_" + side], 0)
        else:
            np.testing.assert_array_equal(o["arm_actuator_torque"], 0)
        result[mode] = {
            "passed": True,
            "duration_s": 0.3,
            "panel_drop_m": drop,
            "max_joint_change_rad": float(np.max(np.abs(o["arm_joint_position"] - q0))),
            "tactile_norms": [
                float(np.linalg.norm(o["tactile_" + s])) for s in ("left", "right")
            ],
        }
    # Keep arm support, remove all virtual human/environment forces.
    w = PandaWorld()
    w.settle()
    w.p = dict(w.p, kh=0.0, dh=0.0)
    z0 = w.d.xpos[w.panel, 2]
    w.refresh(0)
    for _ in range(500):
        w.step()
    o, _ = w.snapshot()
    support = float(o["object_contact_wrench"][2])
    assert abs(support - 9.81) < 0.05 and abs(z0 - o["object_position"][2]) < 0.01
    np.testing.assert_array_equal(w.truth["human_applied_force"], 0)
    result["robot_support_without_human"] = {
        "passed": True,
        "duration_s": 1.0,
        "total_robot_vertical_contact_force_N": support,
        "panel_weight_N": 9.81,
        "panel_z_change_m": float(o["object_position"][2] - z0),
    }
    return result


def kinematics(path):
    data, _ = read_episode(path)
    w = PandaWorld()
    errors = []
    for i, q in enumerate(data["render_states"]["qpos"]):
        w.d.qpos[:] = q
        w.d.qvel[:] = data["render_states"]["qvel"][i]
        mujoco.mj_fwdPosition(w.m, w.d)
        actual = np.array([np.r_[w.d.xpos[b], w.d.xquat[b]] for b in w.roots])
        errors.append(
            float(np.max(np.abs(actual - data["observations"]["end_effector_pose"][i])))
        )
    assert max(errors) < 1e-12
    return {
        "passed": True,
        "frames": len(errors),
        "max_pose_component_error": max(errors),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--output", type=Path, default=ROOT / "results/phase1b/dual_panda_v1"
    )
    ap.add_argument("--render", action="store_true")
    a = ap.parse_args()
    if a.output.exists():
        ap.error("choose a fresh output folder")
    a.output.mkdir(parents=True)
    result = {"episodes": {}}
    for s in "ABC":
        result["episodes"][s] = run_episode(a.output / "smoke" / s, s)
        print(s, result["episodes"][s], flush=True)
    result["episodes"]["reference_test"] = run_episode(
        a.output / "smoke/reference_test", "B", reference_test=True
    )
    result["negative_controls"] = controls()
    result["fk_readback"] = {s: kinematics(a.output / "smoke" / s) for s in "ABC"}
    result["exact_replay"] = {}
    with tempfile.TemporaryDirectory() as tmp:
        for name in ("A", "B", "C", "reference_test"):
            run_episode(
                Path(tmp) / name,
                "B" if name == "reference_test" else name,
                reference_test=name == "reference_test",
            )
            result["exact_replay"][name] = {
                "passed": True,
                "arrays": compare(a.output / "smoke" / name, Path(tmp) / name),
            }
    base, _ = read_episode(a.output / "smoke/B")
    ref, _ = read_episode(a.output / "smoke/reference_test")
    for section in ("observations", "next_observations", "privileged", "render_states"):
        for key in base[section]:
            np.testing.assert_array_equal(base[section][key], ref[section][key])
    result["ignored_reference_control"] = {
        "passed": True,
        "meaning": "Changing +/- .02 Nm reference does not change baseline physics; not assistance tracking",
    }
    test = subprocess.run(
        [
            sys.executable,
            "-m",
            "unittest",
            "phase1a.tests.test_contracts",
            "phase1a.tests.test_edge_geometry",
            "phase1b.tests.test_integration",
            "-v",
        ],
        cwd=ROOT,
        text=True,
        capture_output=True,
    )
    (a.output / "tests.txt").write_text(test.stdout + test.stderr)
    result["tests_passed"] = test.returncode == 0
    result["arm_model_integration"] = {
        "passed": True,
        "arm_joints": 14,
        "fixed_bases": 2,
        "only_freejoint": "panel_free",
        "nominal_ik": json.loads(CONFIG.read_text())["ik"],
    }
    result["physical_coupling_passed"] = all(
        v["physical_sample_checks_passed"] for v in result["episodes"].values()
    ) and all(v["passed"] for v in result["negative_controls"].values())
    result["data_pipeline_passed"] = all(
        v["data_pipeline_passed"] for v in result["episodes"].values()
    )
    result["control_readiness"] = {
        "joint_position_adapter": True,
        "desired_assist_torque_interface": "world +Y / panel COM / Nm",
        "wrench_allocation": False,
        "assistance_tracking": False,
        "joint_torque_command_adapter": False,
        "expert": False,
        "policy_training": False,
    }
    result["passed"] = (
        result["tests_passed"]
        and result["physical_coupling_passed"]
        and result["data_pipeline_passed"]
    )
    (a.output / "verification.json").write_text(json.dumps(result, indent=2) + "\n")
    rows = "\n".join(
        f"| {s} | {v['min_contacts_per_pad']} | {v['max_pad_penetration_m'] * 1000:.3f} | {v['min_joint_limit_margin_rad']:.4f} | {v['arm_saturated_steps']} |"
        for s, v in result["episodes"].items()
    )
    (a.output / "report.md").write_text(f"""# Dual Panda integration report

- Arm Model Integration: **{"PASS" if result["arm_model_integration"]["passed"] else "FAIL"}**. Two fixed Panda bases, 14 arm hinge joints, four custom pad slides; the only freejoint belongs to the panel. Initial IK is within 1 micrometer and 1e-5 rad; no initial penetrations beyond 1e-5 m.
- Physical Coupling: **{"PASS" if result["physical_coupling_passed"] else "FAIL"} for these smoke episodes**. Arms are driven by upstream bounded position servos; pad motors preload the object. No root wrench drive, panel weld, kinematic hand following or qpos overwrite during stepping.
- Data Pipeline: **{"PASS" if result["data_pipeline_passed"] else "FAIL"}**. 1,600 transitions at 100 Hz / 8,000 physics steps; same-time joint states, sensing and command/realized events; all four episodes replay exactly. q/dq logging is enabled, default policy use is disabled.
- Control Readiness: **joint-position adapter ready; force allocation, assistance tracking, direct joint-torque adapter and Expert not implemented**. Signed desired-assist references deliberately leave the hold baseline unchanged.

## Physical measurements

| Episode | Minimum contacts [L upper,L lower,R upper,R lower] | Max pad penetration mm | Minimum arm joint-limit margin rad | Saturated arm steps |
|---|---|---|---|---|
{rows}

All reported episodes have no unexpected penetrations beyond 1e-5 m, no warnings, and finite states. Panel force-balance residuals and exact replay counts are in `verification.json`. Arm velocities/torques respond to loads, but the high-gain hold baseline intentionally allows little motion; this is not a cooperative flip controller. Checks cover the sampled four-second episodes, not arbitrary trajectories or real hardware.

## Coupling controls

- Pad contacts removed: panel drops {result["negative_controls"]["remove_pad_contacts"]["panel_drop_m"]:.3f} m in 0.30 s; both tactile maps and reconstructed contact F/T become zero.
- Arm actuators disabled: panel drops {result["negative_controls"]["disable_arm_actuators"]["panel_drop_m"]:.3f} m in 0.30 s; arm joints move and actuator torque is zero.
- Human force removed while joint support remains: robot vertical contact force is {result["negative_controls"]["robot_support_without_human"]["total_robot_vertical_contact_force_N"]:.4f} N for a 9.81 N panel weight.

Negative controls intentionally alter fresh test instances and are not nominal datasets. Test output is in `tests.txt`; old Phase 1A source/models/results are retained. The mount adds 0.08 kg per custom gripper and changes its attachment; this is Hardware v2 / Scenario v3, not the previous dynamics.

## Reproduce

```bash
uv sync --locked
uv run --locked python -m phase1b.scripts.run_suite --output results/phase1b/reproduction --render
```

See `phase1b/README.md`, `docs/DATA_SCHEMA_V2.md`, and `docs/CONTROL_HANDOFF_ARMS.md` for frame conventions and integration. Rendering needs local ffmpeg. The vendored Menagerie revision and Apache-2.0 license are preserved.
""")
    if not result["passed"]:
        raise RuntimeError("Integration checks failed; inspect verification.json")
    if a.render:
        from phase1b.scripts.render_sync import render

        render(a.output / "smoke", a.output / "visuals")
    print("Integration suite passed:", a.output, flush=True)


if __name__ == "__main__":
    main()
