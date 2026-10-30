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
from phase1b.simulation import ROOT, PandaWorld


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
    ap = argparse.ArgumentParser(
        description="Verify the single dual-Panda system and its three-layer contract"
    )
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--render", action="store_true")
    a = ap.parse_args()
    if a.output.exists():
        ap.error("choose a fresh output folder")
    a.output.mkdir(parents=True)
    test = subprocess.run(
        [
            sys.executable,
            "-m",
            "unittest",
            "phase1b.tests.test_contracts",
            "phase1b.tests.test_integration",
            "-v",
        ],
        cwd=ROOT,
        text=True,
        capture_output=True,
    )
    (a.output / "tests.txt").write_text(test.stdout + test.stderr)
    if test.returncode:
        raise RuntimeError("Contract tests failed; see tests.txt")
    result = dict(
        tests_passed=True,
        episodes={},
        exact_replay={},
        motion={},
        interface_ready=True,
        controller_implemented="reference-only and position-hold adapters",
        assistance_tracking_validated=False,
        expert_labels_validated=False,
    )
    for case in "ABC":
        path = a.output / "episodes" / case
        result["episodes"][case] = run_episode(path, case)
        print(case, result["episodes"][case], flush=True)
    result["negative_controls"] = controls()
    result["fk_readback"] = {s: kinematics(a.output / "episodes" / s) for s in "ABC"}
    with tempfile.TemporaryDirectory() as tmp:
        for case in "ABC":
            run_episode(Path(tmp) / case, case)
            result["exact_replay"][case] = compare(
                a.output / "episodes" / case, Path(tmp) / case
            )
    from phase1b.demos.motion_examples import collect

    for case in "ABC":
        result["motion"][case] = collect(case, a.output / "motion" / case)
    result["passed"] = all(
        v["physical_sample_checks_passed"] for v in result["episodes"].values()
    ) and all(v["passed"] for v in result["motion"].values())
    (a.output / "verification.json").write_text(json.dumps(result, indent=2) + "\n")
    if not result["passed"]:
        raise RuntimeError("Physical acceptance failed; inspect verification.json")
    if a.render:
        from phase1b.demos.render_examples import render

        render(a.output / "motion", a.output / "motion/visuals")
    print("Integration suite passed:", a.output, flush=True)


if __name__ == "__main__":
    main()
