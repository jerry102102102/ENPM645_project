"""Read-only offline annotation sidecars; never step MuJoCo or promote labels."""

import argparse
import hashlib
import json
import subprocess
import xml.etree.ElementTree as ET
from dataclasses import asdict
from pathlib import Path

import numpy as np

from phase1b.expert.reference import (
    DisturbanceTruth,
    ExpertConfig,
    PublicSample,
    ReferenceGenerator,
    TeacherGoal,
)

ROOT = Path(__file__).resolve().parents[2]
VERSION = "enpm645-expert-shadow/1.0"


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_arrays(path):
    with np.load(path, allow_pickle=False) as f:
        return {k: f[k] for k in f.files}


def stack(rows):
    return {k: np.array([r[k] for r in rows]) for k in rows[0]}


def generate_candidates(observations, teacher_inputs, config):
    """Explicit teacher-only stream, separate E0/E1/E2 state, same public samples."""
    t = np.asarray(observations["timestamp"])
    if (
        t.ndim != 1
        or len(t) == 0
        or not np.isfinite(t).all()
        or t[0] < 0
        or not np.allclose(np.diff(t), config.policy_dt, atol=1e-12, rtol=0)
    ):
        raise ValueError("nonempty consecutive 100 Hz observations required")
    np.testing.assert_array_equal(t, teacher_inputs["timestamp"])
    # Do not infer intent from A/B/C or from the sign of a force.
    for key, shape in (
        ("ground_truth_intent", (len(t),)),
        ("human_goal_angle", (len(t),)),
        ("external_disturbance_wrench", (len(t), 6)),
    ):
        if key not in teacher_inputs or np.asarray(teacher_inputs[key]).shape != shape:
            raise ValueError(
                f"explicit timestamp-aligned teacher field required: {key} {shape}"
            )
    if not np.isfinite(teacher_inputs["external_disturbance_wrench"]).all():
        raise ValueError("nonfinite disturbance truth")
    generators = {key: ReferenceGenerator(key, config) for key in ("E0", "E1", "E2")}
    rows = {key: [] for key in generators}
    derived = []
    for i, timestamp in enumerate(t):
        obs = {k: v[i] for k, v in observations.items()}
        public = PublicSample.from_observation(obs)
        goal = TeacherGoal(
            float(timestamp),
            str(teacher_inputs["ground_truth_intent"][i]),
            float(teacher_inputs["human_goal_angle"][i]),
        )
        disturbance = DisturbanceTruth(
            float(timestamp), float(teacher_inputs["external_disturbance_wrench"][i, 4])
        )
        for key, g in generators.items():
            args = (
                {}
                if key == "E0"
                else {"goal": goal}
                if key == "E1"
                else {"goal": goal, "disturbance": disturbance}
            )
            rows[key].append(g.step(public, **args).record())
        e0 = rows["E0"][-1]
        derived.append(
            dict(
                timestamp=float(timestamp),
                theta_y=public.pitch,
                omega_y=public.omega_y,
                delta_theta_y=e0["delta_pitch"],
                delta_theta_valid=e0["delta_pitch_valid"],
                acceleration_y=e0["acceleration_estimate"],
                acceleration_valid=e0["acceleration_estimate_valid"],
                interaction_torque_estimate=e0["interaction_torque_estimate"],
                interaction_estimate_valid=e0["interaction_torque_estimate_valid"],
                tau_left_y=public.robot_torque_left_y,
                tau_right_y=public.robot_torque_right_y,
                off_axis_omega_norm=public.off_axis_omega_norm,
            )
        )
    return {key: stack(values) for key, values in rows.items()}, stack(derived)


def episode_metadata(source, teacher_path, episode_id, config):
    meta = json.loads((source / "metadata.json").read_text())
    scenario_path = ROOT / "phase1b/config/dual_panda_v1.json"
    hardware_path = ROOT / "phase1b/config/hardware_v2.json"
    scenario = json.loads(scenario_path.read_text())
    hardware = json.loads(hardware_path.read_text())
    model_path = ROOT / scenario["model"]
    for key, path in (
        ("model_hash", model_path),
        ("config_hash", scenario_path),
        ("hardware_hash", hardware_path),
    ):
        if meta.get(key) != digest(path):
            raise ValueError("source episode geometry/config mismatch: " + key)
    panel = ET.parse(model_path).find(".//body[@name='panel']/geom[@name='panel_geom']")
    dimensions = (2 * np.fromstring(panel.get("size"), sep=" ")).tolist()
    idx = hardware["body_names"].index("panel")
    p = scenario["parameters"]
    try:
        commit = subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True
        ).strip()
        dirty = bool(
            subprocess.check_output(
                ["git", "status", "--porcelain"], cwd=ROOT, text=True
            ).strip()
        )
    except (OSError, subprocess.CalledProcessError):
        commit, dirty = None, None
    if not isinstance(episode_id, str) or not episode_id.strip():
        raise ValueError("explicit episode_id required")
    shared = dict(
        episode_id=episode_id,
        dataset_version=VERSION,
        seed=p["seed"],
        commit_hash=commit,
        working_tree_dirty=dirty,
        source_episode_commit=meta.get("commit_hash"),
        schema_version=meta["schema_version"],
        mujoco_version=hardware["mujoco_version"],
        scenario_version=scenario["scenario_version"],
        hardware_version=scenario["hardware_version"],
        model_hash=digest(model_path),
        config_hash=digest(scenario_path),
        hardware_hash=digest(hardware_path),
        panel=dict(
            mass=hardware["arrays"]["body_mass"][idx],
            principal_inertia=hardware["arrays"]["body_inertia"][idx],
            dimensions=dimensions,
            friction=p["friction"],
        ),
        grasp=dict(
            robot_left=[-0.2, 0.1, 0],
            robot_right=[0.2, 0.1, 0],
            human=[0.27, -0.1, 0],
            frame="panel_local",
        ),
        human={
            k: p[k]
            for k in ("kh", "dh", "human_force_limit", "human_target_speed_limit")
        },
        disturbance=dict(protocol="explicit teacher-only stream", magnitude_range=None),
        experts=dict(
            ids=["E0", "E1", "E2"],
            version="1.0",
            configuration=asdict(config),
            limits_validated=False,
            pitch_coordination_validated=False,
        ),
        controller=dict(
            components=meta.get("components"),
            version=None,
            actuator_type="position_servo_rad",
            tracking_validated=False,
        ),
        timing=dict(
            physics_dt=meta["physics_dt"],
            policy_dt=config.policy_dt,
            episode_duration=meta["duration_seconds"],
            settle_duration=p["settle_seconds"],
        ),
        frames=dict(
            torque_axis="world +Y",
            reference_point="instantaneous panel COM",
            quaternion_order="wxyz",
            ft_source="contact_reconstructed",
            twist_order="angular_then_linear",
        ),
        quality=dict(
            episode_valid=None,
            failure_reason="shadow annotation only; source episode quality not revalidated",
            validation_version="expert-math/1.0",
            execution_valid=False,
            supervision_eligible=False,
        ),
        source_files={
            str((source / name).resolve()): digest(source / name)
            for name in (
                "metadata.json",
                "observations.npz",
                "privileged.npz",
                "controller.npz",
                "object_commands.npz",
            )
        },
        code_hashes={
            str(path.relative_to(ROOT)): digest(path)
            for path in (
                *Path(__file__).parent.glob("*.py"),
                ROOT / "phase1b/interfaces.py",
            )
        },
    )
    private = dict(
        teacher_input_path=str(teacher_path.resolve()),
        teacher_input_hash=digest(teacher_path),
        source_privileged_path=str((source / "privileged.npz").resolve()),
        scenario_label=meta.get("scenario"),
        disturbance_protocol_parameters={
            k: p[k]
            for k in (
                "disturbance_point",
                "disturbance_magnitude",
                "disturbance_start",
                "disturbance_end",
            )
        },
        human_protocol_parameters={
            k: p[k]
            for k in ("direction", "initial_angle", "target_angle", "ramp_duration")
        },
        note="Exact targets, times and force directions are private; teacher inputs are authoritative for this annotation.",
    )
    return shared, private


def annotate_episode(source, output, config, *, teacher_inputs_path, episode_id):
    source, output, teacher_path = Path(source), Path(output), Path(teacher_inputs_path)
    if output.exists():
        raise FileExistsError(output)
    meta, private = episode_metadata(source, teacher_path, episode_id, config)
    candidates, derived = generate_candidates(
        load_arrays(source / "observations.npz"), load_arrays(teacher_path), config
    )
    output.mkdir(parents=True)
    np.savez_compressed(output / "derived_features.npz", **derived)
    for key, arrays in candidates.items():
        # These files include hidden-goal-derived references: quarantine the entire trace.
        (output / "expert_internal").mkdir(exist_ok=True)
        np.savez_compressed(output / "expert_internal" / f"{key}.npz", **arrays)
        (output / "supervision").mkdir(exist_ok=True)
        n = len(arrays["timestamp"])
        np.savez_compressed(
            output / "supervision" / f"{key}.npz",
            timestamp=arrays["timestamp"],
            expert_assist_torque_ref=np.full(n, np.nan),
            expert_mode=arrays["expert_mode"],
            expert_mode_valid=np.zeros(n, bool),
            expert_torque_valid=np.zeros(n, bool),
            expert_label_valid=np.zeros(n, bool),
            candidate_valid=arrays["candidate_valid"],
            execution_valid=np.zeros(n, bool),
            supervision_eligible=np.zeros(n, bool),
            label_status=arrays["label_status"],
            label_invalid_reason=arrays["label_invalid_reason"],
        )
    meta["output_hashes"] = {
        str(p.relative_to(output)): digest(p) for p in output.rglob("*.npz")
    }
    (output / "metadata.json").write_text(
        json.dumps(meta, indent=2, allow_nan=False) + "\n"
    )
    (output / "privileged_metadata.json").write_text(
        json.dumps(private, indent=2, allow_nan=False) + "\n"
    )
    validate_sidecar(output)
    return output


def validate_sidecar(path):
    path = Path(path)
    meta = json.loads((path / "metadata.json").read_text())
    if meta["dataset_version"] != VERSION:
        raise ValueError("unknown sidecar schema")
    for file, expected in meta["source_files"].items():
        if digest(file) != expected:
            raise ValueError("source episode changed: " + file)
    for file, expected in meta["output_hashes"].items():
        if digest(path / file) != expected:
            raise ValueError("shadow output changed: " + file)
    private = json.loads((path / "privileged_metadata.json").read_text())
    if digest(private["teacher_input_path"]) != private["teacher_input_hash"]:
        raise ValueError("teacher input changed")
    for file, expected in meta["code_hashes"].items():
        if digest(ROOT / file) != expected:
            raise ValueError("annotation code revision changed: " + file)
    timestamps = load_arrays(path / "derived_features.npz")["timestamp"]
    c = ExpertConfig(**meta["experts"]["configuration"])
    if (
        timestamps.ndim != 1
        or not len(timestamps)
        or not np.isfinite(timestamps).all()
        or timestamps[0] < 0
        or not np.allclose(np.diff(timestamps), c.policy_dt, atol=1e-12, rtol=0)
    ):
        raise ValueError("invalid sidecar clock")
    for key in ("E0", "E1", "E2"):
        trace = load_arrays(path / "expert_internal" / f"{key}.npz")
        labels = load_arrays(path / "supervision" / f"{key}.npz")
        np.testing.assert_array_equal(trace["timestamp"], timestamps)
        np.testing.assert_array_equal(labels["timestamp"], timestamps)
        for flag in (
            "expert_label_valid",
            "expert_torque_valid",
            "expert_mode_valid",
            "execution_valid",
            "supervision_eligible",
        ):
            if labels[flag].dtype != bool or labels[flag].any():
                raise ValueError("shadow labels must remain ineligible")
        if not np.isnan(labels["expert_assist_torque_ref"]).all():
            raise ValueError("formal torque unavailable")
        if trace["execution_valid"].any() or trace["supervision_eligible"].any():
            raise ValueError("candidate is not execution evidence")
        torque = trace["candidate_assist_torque_ref"]
        if not np.isfinite(torque).all() or np.any(
            np.abs(torque) > c.torque_max + 1e-12
        ):
            raise ValueError("invalid candidate torque")
        uncertain = trace["expert_mode"] == "UNCERTAIN"
        if np.any(torque[uncertain] != 0):
            raise ValueError("UNCERTAIN assistance forbidden")
        np.testing.assert_array_equal(
            labels["candidate_valid"], trace["candidate_valid"]
        )
        np.testing.assert_array_equal(labels["label_status"], trace["label_status"])
        increments = np.abs(np.diff(np.r_[0, torque]))
        if np.any(
            (increments > c.torque_rate_max * c.policy_dt + 1e-12)
            & ~(uncertain & trace["rate_limit_overridden"])
        ):
            raise ValueError("unrecorded torque slew violation")
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--config",
        type=Path,
        required=True,
        help="Explicit ExpertConfig JSON; no assumed physical limits",
    )
    parser.add_argument(
        "--teacher-inputs",
        type=Path,
        required=True,
        help="Privileged NPZ with timestamp, human_goal_angle, ground_truth_intent, external_disturbance_wrench",
    )
    parser.add_argument("--episode-id", required=True)
    args = parser.parse_args()
    print(
        annotate_episode(
            args.input,
            args.output,
            ExpertConfig(**json.loads(args.config.read_text())),
            teacher_inputs_path=args.teacher_inputs,
            episode_id=args.episode_id,
        )
    )


if __name__ == "__main__":
    main()
