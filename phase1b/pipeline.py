"""Versioned joint-state transitions with ordered command/measurement events."""

import hashlib
import json
from pathlib import Path

import numpy as np

from phase1a.interfaces.controller_contract import AssistTorqueReference, immutable
from phase1b.interfaces import (
    ArmControllerInput,
    PositionHoldBaseline,
    policy_observation,
    validate_command,
)
from phase1b.simulation import CONFIG, ROOT, PandaWorld


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def stack(records):
    return {k: np.array([row[k] for row in records]) for k in records[0]}


def controller_input(world, o):
    return ArmControllerInput(
        float(o["timestamp"]),
        world.m.opt.timestep,
        {k: immutable(v) for k, v in o.items()},
        immutable(world.qtarget),
        immutable([-40.0, 40.0, -40.0, 40.0]),
        immutable(world.m.actuator_ctrlrange[world.aa]),
        immutable(world.m.actuator_ctrlrange[world.pad_aa]),
    )


def reference_at(t, enabled=False):
    value = 0.02 if enabled and 1 <= t < 2 else -0.02 if enabled and 3 <= t < 4 else 0.0
    return AssistTorqueReference(t, value, source="scripted_test")


def run_episode(
    output,
    scenario="B",
    duration=4.0,
    controller_factory=PositionHoldBaseline,
    reference_test=False,
):
    output = Path(output)
    if output.exists():
        raise FileExistsError(output)
    count = round(duration * 100)
    if count <= 0 or not np.isclose(count / 100, duration):
        raise ValueError("duration must be a positive multiple of .01 s")
    world = PandaWorld(scenario)
    world.settle()
    controller = controller_factory()
    sections = {
        k: []
        for k in (
            "observations",
            "next_observations",
            "controller",
            "privileged",
            "supervision",
            "diagnostics",
        )
    }
    raw_states = []
    for i in range(count):
        before, _ = world.snapshot()
        truth = {k: np.array(v, copy=True) for k, v in world.truth.items()}
        policy_observation(before)  # Assert the default profile can be built.
        reference = reference_at(float(before["timestamp"]), reference_test)
        commands = []
        diagnostics = []
        raw_states.append(dict(qpos=world.d.qpos.copy(), qvel=world.d.qvel.copy()))
        for _ in range(5):
            current, _ = world.snapshot()
            inp = controller_input(world, current)
            command = controller.step(reference, inp)
            validate_command(command, inp)
            world.set_commands(command.arm_position_target, command.pad_force_command)
            world.refresh(world.tick * world.m.opt.timestep)
            at, diag = world.snapshot()
            requested = (
                world.m.actuator_gainprm[world.aa, 0] * world.d.ctrl[world.aa]
                + world.m.actuator_biasprm[world.aa, 1] * world.d.qpos[world.qa]
                + world.m.actuator_biasprm[world.aa, 2] * world.d.qvel[world.va]
            )
            forces = world.d.actuator_force[world.aa].copy()
            torques = world.d.qfrc_actuator[world.va].copy()
            c = dict(
                command_timestamp=current["timestamp"],
                arm_position_target=command.arm_position_target.copy(),
                pad_force_command=command.pad_force_command.copy(),
                arm_requested_servo_torque=requested.copy(),
                arm_actuator_torque_at_actuation=torques,
                arm_actuator_force_at_actuation=forces,
                arm_force_saturated=np.abs(requested - forces) > 1e-8,
                controller_mode=command.controller_mode,
                assist_reference_executed=command.assist_reference_executed,
            )
            # Full-model contact audit, not only panel-to-pad contacts.
            pad_ids = sum(world.pad_g, [])
            unexpected = []
            for contact in world.d.contact:
                expected = world.pg in (contact.geom1, contact.geom2) and (
                    contact.geom1 in pad_ids or contact.geom2 in pad_ids
                )
                if not expected and contact.dist < -1e-5:
                    unexpected.append(-contact.dist)
            joint_margin = np.minimum(
                world.d.qpos[world.qa] - world.m.jnt_range[world.arm_joints, 0],
                world.m.jnt_range[world.arm_joints, 1] - world.d.qpos[world.qa],
            )
            total = (
                at["object_contact_wrench"]
                + world.truth["human_applied_wrench"]
                + world.truth["external_disturbance_wrench"]
            )
            residual = (
                world.m.body_mass[world.panel]
                * (diag["panel_acceleration"] - world.m.opt.gravity)
                - total[:3]
            )
            diagnostics.append(
                dict(
                    timestamp=current["timestamp"],
                    penetration_m=diag["penetration"],
                    unexpected_penetration_count=len(unexpected),
                    unexpected_penetration_max_m=max(unexpected, default=0),
                    arm_joint_limit_margin_rad=joint_margin,
                    contact_counts=np.r_[
                        at["contact_state_left"], at["contact_state_right"]
                    ],
                    panel_balance_residual_N=np.linalg.norm(residual),
                    arm_external_generalized_force=world.d.qfrc_applied[
                        world.va
                    ].copy(),
                    xfrc_applied_norm=np.linalg.norm(world.d.xfrc_applied),
                    warnings=world.d.warning.number.copy(),
                )
            )
            world.step()
            after, _ = world.snapshot()
            c.update(
                realized_timestamp=after["timestamp"],
                arm_actuator_torque_after_step=after["arm_actuator_torque"],
                realized_object_contact_wrench=after["object_contact_wrench"],
            )
            commands.append(c)
        sections["observations"].append(before)
        sections["next_observations"].append(after)
        c = stack(commands)
        c.update(
            timestamp=before["timestamp"],
            reference_timestamp=reference.timestamp,
            desired_assist_torque=reference.torque_nm,
            reference_valid=reference.valid,
            reference_source=reference.source,
            tracking_error_Nm=np.nan,
            tracking_error_valid=False,
        )
        sections["controller"].append(c)
        sections["privileged"].append(
            dict(timestamp=before["timestamp"], scenario_label=scenario, **truth)
        )
        sections["supervision"].append(
            dict(
                timestamp=before["timestamp"],
                expert_assist_torque_ref=np.nan,
                expert_label_valid=False,
            )
        )
        sections["diagnostics"].append(stack(diagnostics))
    output.mkdir(parents=True)
    layouts = {}
    for name, rows in sections.items():
        arrays = stack(rows)
        np.savez_compressed(output / (name + ".npz"), **arrays)
        layouts[name] = {
            k: {"shape": list(v.shape), "dtype": str(v.dtype)}
            for k, v in arrays.items()
        }
    np.savez_compressed(output / "render_states.npz", **stack(raw_states))
    meta = dict(
        schema_version="enpm645-articulated-transition/1.0",
        scenario_version=world.config["scenario_version"],
        hardware_version=world.config["hardware_version"],
        scenario=scenario,
        physics_dt=world.m.opt.timestep,
        logging_hz=100,
        steps_per_transition=5,
        engine_time_origin=world.origin,
        duration_seconds=duration,
        model_hash=world.config["model_sha256"],
        config_hash=sha(CONFIG),
        hardware_hash=sha(ROOT / "phase1b/config/hardware_v2.json")
        if (ROOT / "phase1b/config/hardware_v2.json").exists()
        else None,
        source_hashes={
            str(p.relative_to(ROOT)): sha(p)
            for p in sorted(
                [
                    *(ROOT / "phase1b").glob("*.py"),
                    ROOT / "phase0_v2/sensing/contacts.py",
                    ROOT / "phase0_tactile/scripts/run_experiment.py",
                    ROOT / "phase0_v2/simulation/dual_gripper.py",
                ]
            )
        },
        joint_names=world.config["arm_joint_names"],
        controller_factory=controller_factory.__module__
        + ":"
        + controller_factory.__name__,
        reference_test=reference_test,
        policy_profile_default="baseline",
        joint_states_logged=True,
        joint_states_used_by_default_policy=False,
        expert_labels="unavailable",
        control_status="position_actuator_pipeline; assistance tracking not certified",
        array_layout=layouts,
    )
    (output / "metadata.json").write_text(json.dumps(meta, indent=2) + "\n")
    return validate_episode(output)


def read_episode(path):
    path = Path(path)
    data = {}
    for name in (
        "observations",
        "next_observations",
        "controller",
        "privileged",
        "supervision",
        "diagnostics",
        "render_states",
    ):
        with np.load(path / (name + ".npz"), allow_pickle=False) as f:
            data[name] = {k: f[k] for k in f.files}
    return data, json.loads((path / "metadata.json").read_text())


def validate_episode(path):
    data, meta = read_episode(path)
    o = data["observations"]
    n = data["next_observations"]
    c = data["controller"]
    d = data["diagnostics"]
    cfg = json.loads(CONFIG.read_text())
    assert meta["model_hash"] == cfg["model_sha256"] == sha(ROOT / cfg["model"])
    assert meta["config_hash"] == sha(CONFIG)
    if meta["hardware_hash"] is not None:
        assert meta["hardware_hash"] == sha(ROOT / "phase1b/config/hardware_v2.json")
    for f, digest in meta["source_hashes"].items():
        assert sha(ROOT / f) == digest, f
    count = len(o["timestamp"])
    np.testing.assert_allclose(o["timestamp"], np.arange(count) * 0.01, atol=1e-12)
    np.testing.assert_allclose(
        o["engine_timestamp"], meta["engine_time_origin"] + o["timestamp"], atol=1e-10
    )
    np.testing.assert_allclose(
        c["command_timestamp"],
        o["timestamp"][:, None] + np.arange(5) * 0.002,
        atol=1e-12,
    )
    np.testing.assert_allclose(
        c["realized_timestamp"], c["command_timestamp"] + 0.002, atol=1e-12
    )
    for section in ("controller", "privileged", "supervision"):
        np.testing.assert_array_equal(o["timestamp"], data[section]["timestamp"])
    for key in o:
        np.testing.assert_array_equal(n[key][:-1], o[key][1:])
    np.testing.assert_array_equal(
        c["arm_actuator_torque_after_step"][:, -1], n["arm_actuator_torque"]
    )
    np.testing.assert_array_equal(
        c["realized_object_contact_wrench"][:, -1], n["object_contact_wrench"]
    )
    np.testing.assert_array_equal(
        c["arm_actuator_torque_at_actuation"], c["arm_actuator_force_at_actuation"]
    )  # unit gear, hinge transmission
    assert o["arm_joint_position"].shape == (count, 2, 7)
    assert o["arm_joint_velocity"].shape == (count, 2, 7)
    world = PandaWorld(meta["scenario"])
    np.testing.assert_array_equal(
        o["arm_joint_position"], data["render_states"]["qpos"][:, world.qa]
    )
    np.testing.assert_array_equal(
        o["arm_joint_velocity"], data["render_states"]["qvel"][:, world.va]
    )
    np.testing.assert_array_equal(
        o["gripper_joint_position"], data["render_states"]["qpos"][:, world.pad_qa]
    )
    np.testing.assert_array_equal(
        o["gripper_joint_velocity"], data["render_states"]["qvel"][:, world.pad_va]
    )
    np.testing.assert_array_equal(o["physics_tick"], np.arange(count) * 5)
    for command, ids in (
        (c["arm_position_target"], world.aa),
        (c["pad_force_command"], world.pad_aa),
    ):
        assert np.isfinite(command).all()
        assert np.all(command >= world.m.actuator_ctrlrange[ids, 0]) and np.all(
            command <= world.m.actuator_ctrlrange[ids, 1]
        )
    assert np.all(
        np.abs(c["arm_actuator_torque_at_actuation"])
        <= world.m.actuator_forcerange[world.aa, 1] + 1e-8
    )
    assert (
        np.isnan(data["supervision"]["expert_assist_torque_ref"]).all()
        and not data["supervision"]["expert_label_valid"].any()
    )
    assert (
        np.isnan(c["tracking_error_Nm"]).all() and not c["tracking_error_valid"].any()
    )
    assert not np.any(d["arm_external_generalized_force"]) and not np.any(
        d["xfrc_applied_norm"]
    )
    for section in ("observations", "next_observations", "diagnostics"):
        for key, value in data[section].items():
            assert np.isfinite(value).all(), (section, key)
    for name in ("arm_joint_position", "arm_joint_velocity", "arm_actuator_torque"):
        assert name not in policy_observation({k: v[0] for k, v in o.items()})
    physical = bool(
        d["contact_counts"].min() >= 1
        and d["penetration_m"].max() < 0.001
        and d["unexpected_penetration_count"].max() == 0
        and d["warnings"].max() == 0
        and d["arm_joint_limit_margin_rad"].min() > 0
        and d["panel_balance_residual_N"].max() < 1e-5
    )
    return dict(
        data_pipeline_passed=True,
        physical_sample_checks_passed=physical,
        samples=count,
        physics_steps=count * 5,
        min_contacts_per_pad=d["contact_counts"].min(axis=(0, 1)).tolist(),
        max_pad_penetration_m=float(d["penetration_m"].max()),
        unexpected_penetration_count=int(d["unexpected_penetration_count"].sum()),
        min_joint_limit_margin_rad=float(d["arm_joint_limit_margin_rad"].min()),
        max_panel_balance_residual_N=float(d["panel_balance_residual_N"].max()),
        max_joint_speed_rad_s=float(np.abs(o["arm_joint_velocity"]).max()),
        arm_saturated_steps=int(np.any(c["arm_force_saturated"], axis=(2, 3)).sum()),
        panel_z_range_m=[
            float(o["object_position"][:, 2].min()),
            float(o["object_position"][:, 2].max()),
        ],
    )
