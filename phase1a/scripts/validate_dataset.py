"""Read-back validation, exact replay, original-v2 regression and no-op reference control."""

import argparse
import json
import tempfile
from pathlib import Path

import numpy as np

from phase1a.data.episode_io import DatasetReader, validate_dataset
from phase1a.scripts.run_pipeline_smoke import run_episode
from phase1a.simulation.scenario_loader import REPO, load_scenario

EPISODES = [
    ("A", "A", 4.0, False),
    ("B", "B", 4.0, False),
    ("C", "C", 4.0, False),
    ("reference_test", "B", 5.0, True),
]


def compare_npz(a, b):
    count = 0
    with np.load(a, allow_pickle=False) as x, np.load(b, allow_pickle=False) as y:
        assert set(x.files) == set(y.files)
        for k in x.files:
            equal = (
                np.array_equal(x[k], y[k], equal_nan=True)
                if x[k].dtype.kind in "fc"
                else np.array_equal(x[k], y[k])
            )
            assert equal, (str(a), k)
            count += 1
    return count


def regression(smoke):
    from phase0_v2.scripts.run_pilot import run_episode as original_episode
    from phase0_v2.scripts.validate_pilot import controls

    config = load_scenario()
    result = {"deterministic_replay": {}, "original_v2_comparison": {}}
    with tempfile.TemporaryDirectory() as temp:
        temp = Path(temp)
        for name, scenario, duration, reference in EPISODES:
            target = temp / name
            run_episode(
                target, scenario, config, duration=duration, reference_test=reference
            )
            arrays = sum(
                compare_npz(smoke / name / f, target / f)
                for f in (
                    "observations.npz",
                    "next_observations.npz",
                    "controller.npz",
                    "supervision.npz",
                    "privileged.npz",
                )
            )
            result["deterministic_replay"][name] = {
                "passed": True,
                "exact_arrays_including_nan_masks": arrays,
            }
        zero = temp / "zero_reference"
        run_episode(zero, "B", config, duration=5.0, reference_test=False)
        arrays = sum(
            compare_npz(zero / f, temp / "reference_test" / f)
            for f in ("observations.npz", "next_observations.npz", "privileged.npz")
        )
        z = DatasetReader(zero).sections["controller"]
        r = DatasetReader(temp / "reference_test").sections["controller"]
        for key in (
            "executed_virtual_drive_wrenches",
            "executed_actuator_commands",
            "realized_object_contact_wrench",
        ):
            assert np.array_equal(z[key], r[key])
            arrays += 1
        result["ignored_reference_physics_control"] = {
            "passed": True,
            "exact_arrays": arrays,
            "duration_s": 5,
            "meaning": "+/- scripted reference leaves baseline commands and all sensor trajectories unchanged",
        }
        for scenario in "ABC":
            dest = temp / ("v2_" + scenario)
            original_episode(dict(config["parameters"]), scenario, dest)
            new = DatasetReader(smoke / scenario)
            o = new.sections["observations"]
            c = new.sections["controller"]
            with (
                np.load(dest / "observations.npz") as old,
                np.load(dest / "controller.npz") as ctrl,
            ):
                pairs = {
                    "position_m": (o["object_position"], old["object_pose"][:, :3]),
                    "orientation_quaternion": (
                        o["object_orientation"],
                        old["object_pose"][:, 3:],
                    ),
                    "linear_velocity_m_s": (
                        o["object_linear_velocity"],
                        old["object_velocity"][:, 3:],
                    ),
                    "angular_velocity_rad_s": (
                        o["object_angular_velocity"],
                        old["object_velocity"][:, :3],
                    ),
                    "root_command_N_Nm": (
                        c["executed_virtual_drive_wrenches"][:, 0],
                        ctrl["executed_virtual_drive_wrenches"],
                    ),
                    "contact_at_actuation_N_Nm": (
                        c["contact_wrench_at_actuation"][:, 0],
                        old["aggregated_object_wrench"],
                    ),
                }
                errors = {
                    key: float(np.max(np.abs(a - b))) for key, (a, b) in pairs.items()
                }
                assert max(errors.values()) < 1e-9, errors
            result["original_v2_comparison"][scenario] = {
                "passed": True,
                "max_absolute_errors": errors,
                "absolute_tolerance": 1e-9,
            }
    result["v2_physical_controls"] = controls()
    assert result["v2_physical_controls"]["passed"]
    result["passed"] = True
    return result


def validate_smoke(path):
    return {name: validate_dataset(path / name) for name, _, _, _ in EPISODES}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", type=Path, default=REPO / "results/phase1a/smoke")
    ap.add_argument("--regression", action="store_true")
    ap.add_argument("--report", type=Path)
    a = ap.parse_args()
    result = {"datasets": validate_smoke(a.input)}
    if a.regression:
        result["regression"] = regression(a.input)
    text = json.dumps(result, indent=2) + "\n"
    if a.report:
        if a.report.exists():
            ap.error("report already exists; choose a fresh path")
        a.report.write_text(text)
    print(text)


if __name__ == "__main__":
    main()
