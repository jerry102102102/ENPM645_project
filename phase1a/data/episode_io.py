"""Separated NPZ transitions; explicit time/event semantics and a causal reader."""

import json
from pathlib import Path

import numpy as np

from phase1a.data.observation_builder import (
    OBS_SHAPES,
    POLICY_SHAPES,
    SensorObservation,
    policy_observation_builder,
)

SCHEMA_VERSION = "enpm645-transition/1.0"


def schema():
    return dict(
        schema_version=SCHEMA_VERSION,
        observation_shapes={k: list(v) for k, v in OBS_SHAPES.items()},
        policy_feature_allowlist=list(POLICY_SHAPES),
        files={
            "observations.npz": "o_t only",
            "next_observations.npz": "o_(t+1), never policy input for transition t",
            "supervision.npz": "all expert labels invalid in Phase 1A",
            "controller.npz": "requested reference, five executed command steps, post-step realized contact wrench",
            "privileged.npz": "human goal, applied human/environment wrench, scenario and seed",
        },
        time=dict(
            unit="simulated seconds",
            origin="after 2-second settle",
            observation="pre-decision, previous command held",
            decision="same simulation time as o_t, later event",
            next_observation="post physics, before next decision",
            reference_hz=100,
            controller_hz=500,
            physics_hz=500,
            steps_per_transition=5,
            a_t="ordered five-step command sequence; not one command held for all 10 ms",
        ),
        frames=dict(
            wrist_ft="panel-on-gripper, each local root, about root origin",
            world_wrench="gripper-on-panel, world, about instantaneous panel COM",
            executed_virtual_drive="world, force/torque applied at each gripper root origin",
            allocated_gripper_wrenches="gripper-on-panel target, world, about each gripper root origin",
            tactile="each pad sensor frame z/x/y angular bins; force N/bin",
            quaternion_order="wxyz",
            object_velocity="world",
            desired_assistance="world +Y about panel COM, Nm, excludes baseline gravity/stabilization",
        ),
        availability="unavailable numeric fields use NaN and false masks; runtime controller uses None",
        tracking="unavailable: support_only does not execute references; total realized contact torque is not additional assistance",
    )


def snapshot_dict(o):
    return dict(
        timestamp=o.timestamp,
        engine_timestamp=o.engine_timestamp,
        physics_tick=o.physics_tick,
        **o.fields,
    )


def stack_records(records):
    return {key: np.asarray([r[key] for r in records]) for key in records[0]}


def save_episode(path, sections, metadata):
    path = Path(path)
    if path.exists():
        raise FileExistsError(path)
    path.mkdir(parents=True)
    metadata["array_layout"] = {}
    for name, records in sections.items():
        arrays = stack_records(records)
        metadata["array_layout"][name] = {
            key: {"shape": ["N", *arr.shape[1:]], "dtype": str(arr.dtype)}
            for key, arr in arrays.items()
        }
        np.savez_compressed(path / (name + ".npz"), **arrays)
    (path / "metadata.json").write_text(
        json.dumps(metadata, indent=2, allow_nan=False) + "\n"
    )


class DatasetReader:
    def __init__(self, path):
        self.path = Path(path)
        self.metadata = json.loads((self.path / "metadata.json").read_text())
        self.sections = {}
        for name in (
            "observations",
            "next_observations",
            "supervision",
            "controller",
            "privileged",
        ):
            with np.load(self.path / (name + ".npz"), allow_pickle=False) as f:
                self.sections[name] = {k: f[k] for k in f.files}

    def policy_history(self, index, history_length=25):
        o = self.sections["observations"]
        n = len(o["timestamp"])
        if index < 0 or index >= n:
            raise IndexError(index)
        frames = [
            SensorObservation(
                float(o["timestamp"][j]),
                float(o["engine_timestamp"][j]),
                int(o["physics_tick"][j]),
                {k: o[k][j] for k in OBS_SHAPES},
            )
            for j in range(max(0, index - history_length + 1), index + 1)
        ]
        return policy_observation_builder(
            frames, float(o["timestamp"][index]), history_length
        )


def validate_dataset(path, require_support_only=True):
    if not __debug__:
        raise RuntimeError("Validation requires Python without -O")
    from phase1a.simulation.scenario_loader import (
        config_hash,
        hardware_binding,
        load_episode_scenario,
    )

    reader = DatasetReader(path)
    s = reader.sections
    m = reader.metadata
    config = load_episode_scenario(m)
    for key, value in hardware_binding(config).items():
        assert m.get(key) == value, "Episode hardware binding mismatch: " + key
    o = s["observations"]
    nxt = s["next_observations"]
    c = s["controller"]
    e = s["supervision"]
    p = s["privileged"]
    n = len(o["timestamp"])
    dt = m["physics_dt"]
    stride = m["physics_steps_per_transition"]
    if n == 0:
        raise ValueError("empty episode")
    assert m["schema_version"] == SCHEMA_VERSION
    for key in ("scenario_id", "scenario_version", "model_hash"):
        assert m[key] == config[key], key
    assert m["config_hash"] == config_hash(config)
    assert (
        dt == config["scene"]["physics_dt"]
        and stride == config["timing"]["physics_steps_per_transition"]
    )
    for name, section in s.items():
        assert set(section) == set(m["array_layout"][name]), name
        for key, arr in section.items():
            layout = m["array_layout"][name][key]
            assert (
                arr.shape == (n, *layout["shape"][1:])
                and str(arr.dtype) == layout["dtype"]
            ), key
            optional_fields = {
                "expert_assist_torque_ref",
                "requested_total_object_wrench",
                "allocated_gripper_wrenches",
                "tracking_error_Nm",
            }
            if arr.dtype.kind in "fc" and key not in optional_fields:
                assert np.isfinite(arr).all(), key
    assert np.allclose(
        np.linalg.norm(o["object_orientation"], axis=1), 1, atol=1e-8, rtol=0
    )
    assert np.allclose(
        np.linalg.norm(nxt["object_orientation"], axis=1), 1, atol=1e-8, rtol=0
    )
    assert (
        set(o)
        == set(nxt)
        == set(OBS_SHAPES) | {"timestamp", "engine_timestamp", "physics_tick"}
    )
    for k, shape in OBS_SHAPES.items():
        assert (
            o[k].shape == nxt[k].shape == (n, *shape)
            and np.isfinite(o[k]).all()
            and np.isfinite(nxt[k]).all()
        ), k
        assert np.array_equal(nxt[k][:-1], o[k][1:]), "transition discontinuity: " + k
    for sec in (c, e, p):
        assert len(sec["timestamp"]) == n and np.array_equal(
            sec["timestamp"], o["timestamp"]
        )
    assert np.all(o["timestamp"] <= c["decision_timestamp"]) and np.all(
        c["decision_timestamp"] < nxt["timestamp"]
    )
    assert np.array_equal(c["reference_timestamp"], c["decision_timestamp"])
    assert np.all(c["history_latest_timestamp"] <= c["decision_timestamp"])
    assert np.array_equal(nxt["physics_tick"], o["physics_tick"] + stride)
    assert np.array_equal(o["physics_tick"], np.arange(n) * stride)
    assert np.allclose(o["timestamp"], o["physics_tick"] * dt, atol=1e-12, rtol=0)
    assert np.allclose(
        nxt["timestamp"] - o["timestamp"], stride * dt, atol=1e-12, rtol=0
    )
    assert np.allclose(
        o["engine_timestamp"] - m["engine_time_origin"],
        o["timestamp"],
        atol=1e-8,
        rtol=0,
    )
    assert np.allclose(
        nxt["engine_timestamp"] - m["engine_time_origin"],
        nxt["timestamp"],
        atol=1e-8,
        rtol=0,
    )
    expected_ticks = o["physics_tick"][:, None] + np.arange(stride)
    assert np.array_equal(c["physics_step_ticks"], expected_ticks)
    assert np.allclose(
        c["controller_call_timestamps"], expected_ticks * dt, atol=1e-12, rtol=0
    )
    assert np.allclose(
        c["realized_substep_timestamps"], (expected_ticks + 1) * dt, atol=1e-12, rtol=0
    )
    assert np.array_equal(
        p["applied_wrench_timestamps"], c["controller_call_timestamps"]
    )
    assert np.array_equal(
        c["realized_object_contact_wrench"], nxt["object_contact_wrench"]
    )
    assert (
        not e["expert_label_valid"].any()
        and np.isnan(e["expert_assist_torque_ref"]).all()
    )
    assert np.all(e["expert_source"] == "unavailable")
    assert (
        c["controller_mode"].shape
        == c["assist_reference_executed"].shape
        == (n, stride)
    )
    assert c["assist_reference_executed"].dtype == bool
    assert not np.any(
        (c["controller_mode"] == "support_only") & c["assist_reference_executed"]
    )
    if require_support_only:
        assert not c["assist_reference_executed"].any() and np.all(
            c["controller_mode"] == "support_only"
        )
    for k, shape in [
        ("requested_total_object_wrench", (n, stride, 6)),
        ("allocated_gripper_wrenches", (n, stride, 2, 6)),
        ("tracking_error_Nm", (n,)),
    ]:
        assert c[k].shape == shape and c[k + "_valid"].dtype == bool, k
        mask = c[k + "_valid"]
        assert mask.shape == ((n,) if k == "tracking_error_Nm" else (n, stride))
        assert np.isfinite(c[k][mask]).all() and np.isnan(c[k][~mask]).all(), k
        if require_support_only:
            assert not mask.any(), k
    # No validated baseline/assistance separation evaluator is installed in Phase 1A.
    assert (
        not c["tracking_error_Nm_valid"].any()
        and m["tracking_evaluation"] == "unavailable"
    )
    assert e["expert_label_valid"].dtype == bool
    assert np.isfinite(c["desired_assist_torque_ref"]).all()
    assert c["requested_assistance_wrench"].shape == (n, 6)
    expected = np.zeros((n, 6))
    expected[:, 4] = c["desired_assist_torque_ref"]
    assert np.array_equal(expected, c["requested_assistance_wrench"])
    assert np.array_equal(
        c["realized_contact_wrench_substep"][:, -1], c["realized_object_contact_wrench"]
    )
    assert c["contact_status"].shape == (n, stride, 2, 2)
    assert np.array_equal(
        c["contact_status"][:, 0],
        np.stack([o["contact_state_left"], o["contact_state_right"]], axis=1),
    )
    assert (
        c["saturation_status"].shape == (n, stride, 2, 2)
        and c["saturation_status"].dtype == bool
    )
    assert (
        c["pad_saturation_status"].shape == (n, stride, 4)
        and c["pad_saturation_status"].dtype == bool
    )
    assert (
        np.all(c["reference_source"] == "scripted_test") and c["reference_valid"].all()
    )
    assert np.all(c["reference_frame_id"] == "world") and np.all(
        c["reference_point"] == "panel_com"
    )
    assert c["executed_virtual_drive_wrenches"].shape == (n, stride, 2, 6)
    assert c["executed_actuator_commands"].shape == (n, stride, 4)
    assert np.array_equal(
        c["actual_pad_actuator_force"], c["executed_actuator_commands"]
    )
    drives = c["executed_virtual_drive_wrenches"]
    pads = c["executed_actuator_commands"]
    scene = config["scene"]
    assert np.isfinite(drives).all() and np.isfinite(pads).all()
    assert (
        np.max(np.linalg.norm(drives[..., :3], axis=-1))
        <= scene["root_drive_force_limit_N"] + 1e-10
    )
    assert (
        np.max(np.linalg.norm(drives[..., 3:], axis=-1))
        <= scene["root_drive_torque_limit_Nm"] + 1e-10
    )
    limits = np.array(scene["pad_actuator_limits_N"])
    assert np.all(pads >= limits[:, 0] - 1e-10) and np.all(pads <= limits[:, 1] + 1e-10)
    if require_support_only:
        assert np.array_equal(drives, c["baseline_virtual_drive_proposal"])
    assert (
        np.max(c["linear_balance_residual_N"]) < 1e-8
        and np.max(c["contact_generalized_error"]) < 1e-8
    )
    assert np.all(c["contact_status"] > 0)
    assert np.max(c["penetration_m"]) < 0.001 and sum(m["warning_counts"]) == 0
    for i in (0, n // 2, n - 1):
        hist = reader.policy_history(i)
        assert (
            set(hist.features) == set(POLICY_SHAPES)
            and hist.timestamps[-1] == o["timestamp"][i]
        )
    return dict(
        passed=True,
        transitions=n,
        physics_steps=n * stride,
        expert_valid_count=int(e["expert_label_valid"].sum()),
        assist_executed_count=int(c["assist_reference_executed"].sum()),
        tracking_evaluation="unavailable",
        max_balance_residual_N=float(np.max(c["linear_balance_residual_N"])),
        max_penetration_m=float(np.max(c["penetration_m"])),
    )
