"""Versioned joint-state transitions with ordered command/measurement events."""

import json
from pathlib import Path

import numpy as np

from phase1b.control import ControlChain, decision_row, object_row
from phase1b.hardware import sha
from phase1b.interfaces import (
    ArmControllerInput,
    AssistanceDecision,
    ObjectCommand,
    immutable,
    policy_observation,
    validate_command,
)
from phase1b.simulation import CONFIG, ROOT, PandaWorld


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


def run_episode(
    output,
    scenario="B",
    duration=4.0,
    **components,
):
    output = Path(output)
    if output.exists():
        raise FileExistsError(output)
    count = round(duration * 100)
    if count <= 0 or not np.isclose(count / 100, duration):
        raise ValueError("duration must be a positive multiple of .01 s")
    world = PandaWorld(scenario)
    world.settle()
    controller = ControlChain(**components)
    sections = {
        k: []
        for k in (
            "observations",
            "next_observations",
            "decisions",
            "object_commands",
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
        commands = []
        diagnostics = []
        raw_states.append(dict(qpos=world.d.qpos.copy(), qvel=world.d.qvel.copy()))
        for _ in range(5):
            current, _ = world.snapshot()
            inp = controller_input(world, current)
            command = controller.step(inp)
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
        decision = decision_row(controller.decision)
        obj = object_row(controller.command)
        sections["decisions"].append(decision)
        sections["object_commands"].append(obj)
        c = stack(commands)
        c.update(
            timestamp=before["timestamp"],
            reference_timestamp=decision["timestamp"],
            desired_assist_torque=decision["assist_torque_y_nm"],
            reference_valid=decision["valid"],
            reference_source=decision["source"],
            decision_timestamp=np.full(5, decision["timestamp"]),
            object_command_timestamp=np.full(5, obj["timestamp"]),
            requested_object_wrench=np.full((5, 6), np.nan),
            requested_object_wrench_valid=np.zeros(5, bool),
            allocated_gripper_wrenches=np.full((5, 2, 6), np.nan),
            allocation_valid=np.zeros(5, bool),
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
        schema_version="enpm645-three-layer-transition/1.0",
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
        hardware_hash=sha(ROOT / "phase1b/config/hardware_v2.json"),
        source_hashes={
            str(p.relative_to(ROOT)): sha(p)
            for p in sorted(
                [
                    *(ROOT / "phase1b").glob("*.py"),
                ]
            )
        },
        joint_names=world.config["arm_joint_names"],
        components={
            key: type(getattr(controller, key)).__module__
            + ":"
            + type(getattr(controller, key)).__qualname__
            for key in ("policy", "object_controller", "dual_arm_controller")
        },
        policy_profile_default=controller.profile,
        history_length=controller.history_length,
        max_assist_torque_nm=controller.max_assist_torque_nm,
        max_rotation_rate_rad_s=controller.max_rotation_rate_rad_s,
        joint_states_logged=True,
        joint_states_used_by_default_policy=controller.profile == "with_joint_states",
        expert_labels="unavailable",
        control_status="three-layer interfaces; assistance tracking and Expert labels not validated",
        array_layout=layouts,
    )
    (output / "metadata.json").write_text(json.dumps(meta, indent=2) + "\n")
    result = validate_episode(output)
    (output / "verification.json").write_text(json.dumps(result, indent=2) + "\n")
    return result


def read_episode(path):
    path = Path(path)
    data = {}
    for name in (
        "observations",
        "next_observations",
        "decisions",
        "object_commands",
        "controller",
        "privileged",
        "supervision",
        "diagnostics",
        "render_states",
    ):
        with np.load(path / (name + ".npz"), allow_pickle=False) as f:
            data[name] = {k: f[k] for k in f.files}
    meta = json.loads((path / "metadata.json").read_text())
    if meta["schema_version"] != "enpm645-three-layer-transition/1.0":
        raise ValueError(
            "Use the historical Git revision to read older dataset schemas"
        )
    return data, meta


def validate_episode(path):
    data, meta = read_episode(path)
    o = data["observations"]
    n = data["next_observations"]
    c = data["controller"]
    d = data["diagnostics"]
    cfg = json.loads(CONFIG.read_text())
    assert meta["model_hash"] == cfg["model_sha256"] == sha(ROOT / cfg["model"])
    assert meta["config_hash"] == sha(CONFIG)
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
    result = dict(
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

    result.update(validate_layer_data(data, meta))
    return result


def validate_layer_data(data, meta):
    d, o, c = (data[k] for k in ("decisions", "object_commands", "controller"))
    times = data["observations"]["timestamp"]
    for section in (d, o):
        np.testing.assert_array_equal(section["timestamp"], times)
    for key in ("decision_timestamp", "object_command_timestamp"):
        np.testing.assert_array_equal(c[key], np.repeat(times[:, None], 5, axis=1))
    for key, source in (
        ("reference_timestamp", "timestamp"),
        ("desired_assist_torque", "assist_torque_y_nm"),
        ("reference_source", "source"),
        ("reference_valid", "valid"),
    ):
        np.testing.assert_array_equal(c[key], d[source])
    for section, arrays in data.items():
        if section == "render_states":
            continue
        for key, value in arrays.items():
            layout = meta["array_layout"][section][key]
            if (
                value.shape != tuple(layout["shape"])
                or str(value.dtype) != layout["dtype"]
            ):
                raise ValueError("array layout mismatch")
    for i, t in enumerate(times):
        if not d["confidence_present"][i] and not np.isnan(d["confidence"][i]):
            raise ValueError("unavailable confidence must be NaN")
        decision = AssistanceDecision(
            float(t),
            str(d["mode"][i]),
            float(d["assist_torque_y_nm"][i]),
            float(d["confidence"][i]) if d["confidence_present"][i] else None,
            str(d["source"][i]),
            bool(d["valid"][i]),
        )
        params = {}
        for key in ("stiffness", "damping"):
            if o[key + "_valid"][i]:
                params[key] = o[key][i]
            elif not np.isnan(o[key][i]).all():
                raise ValueError("unavailable impedance must be NaN")
        command = ObjectCommand(
            float(o["timestamp"][i]),
            str(o["mode"][i]),
            o["pose_ref_world"][i],
            o["twist_ref_world"][i],
            o["assist_wrench_world"][i],
            params,
            str(o["reference_point"][i]),
            bool(o["valid"][i]),
            str(o["frame_id"][i]),
        )
        if (
            command.mode != decision.mode
            or command.valid != decision.valid
            or command.assist_wrench_world[4] != decision.assist_torque_y_nm
        ):
            raise ValueError("decision/object mismatch")
        if abs(decision.assist_torque_y_nm) > meta["max_assist_torque_nm"]:
            raise ValueError("assistance bound exceeded")
        if (
            np.linalg.norm(command.twist_ref_world[:3])
            > meta["max_rotation_rate_rad_s"]
        ):
            raise ValueError("rotation reference rate exceeded")
        if command.mode == "UNCERTAIN":
            if c["assist_reference_executed"][i].any():
                raise ValueError("UNCERTAIN execution claim")
            measured = np.r_[
                data["observations"]["object_position"][i],
                data["observations"]["object_orientation"][i],
            ]
            np.testing.assert_allclose(
                command.pose_ref_world, measured, atol=1e-12, rtol=0
            )
    for key, mask in (
        ("requested_object_wrench", "requested_object_wrench_valid"),
        ("allocated_gripper_wrenches", "allocation_valid"),
    ):
        if c[mask].any() or not np.isnan(c[key]).all():
            raise ValueError(
                "allocation/total wrench unavailable in contract-only pipeline"
            )
    return dict(
        interface_ready=True,
        assistance_tracking_validated=False,
        expert_labels_validated=False,
        controller_status="adapter supplied; no full motion/force controller certification",
    )
