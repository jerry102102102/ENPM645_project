"""No allocator/expert: frozen v2 -> causal history -> reference -> adapter -> physics."""

import argparse
import importlib
import json
import platform
from pathlib import Path

import mujoco
import numpy as np

from phase1a.data.episode_io import (
    save_episode,
    schema,
    snapshot_dict,
    validate_dataset,
)
from phase1a.data.observation_builder import policy_observation_builder
from phase1a.interfaces.controller_contract import (
    AssistTorqueReference,
    SupportOnlyAdapter,
    validate_input,
    validate_output,
)
from phase1a.simulation.scenario_loader import REPO, config_hash, load_scenario
from phase1a.simulation.transition_world import TransitionWorld


def scripted_reference(timestamp, config, enabled):
    sign = (
        next(
            (
                s
                for start, end, s in config["scripted_reference"]["schedule_seconds"]
                if start <= timestamp < end
            ),
            0,
        )
        if enabled
        else 0
    )
    return AssistTorqueReference(
        timestamp,
        float(sign * config["scripted_reference"]["amplitude_Nm"]),
        source="scripted_test",
        valid=True,
    )


def optional(value, shape):
    return np.full(shape, np.nan) if value is None else value.copy()


def run_episode(
    path,
    scenario,
    config,
    *,
    duration=4.0,
    reference_test=False,
    controller_factory=SupportOnlyAdapter,
):
    world = TransitionWorld(config, scenario)
    controller = controller_factory()
    dt = world.dt
    stride = config["timing"]["physics_steps_per_transition"]
    n = round(duration / (dt * stride))
    if not np.isclose(n * dt * stride, duration):
        raise ValueError("duration must contain complete transitions")
    sections = {
        k: []
        for k in (
            "observations",
            "next_observations",
            "supervision",
            "controller",
            "privileged",
        )
    }
    history = []
    for index in range(n):
        before = world.current
        history.append(before)
        history = history[-25:]
        policy_history = policy_observation_builder(history, before.timestamp)
        reference = scripted_reference(before.timestamp, config, reference_test)
        start_truth = world.truth
        steps = []
        applied_truth = []
        for _ in range(stride):
            inp = world.controller_input()
            validate_input(reference, inp)
            output = controller.step(reference, inp)
            validate_output(output, inp)
            truth, applied, after = world.advance(output)
            applied_truth.append(truth)
            steps.append(
                dict(
                    controller_call_timestamp=inp.timestamp,
                    physics_step_tick=after.physics_tick - 1,
                    realized_timestamp=after.timestamp,
                    requested_total_object_wrench=optional(
                        output.requested_total_object_wrench, (6,)
                    ),
                    requested_total_object_wrench_valid=output.requested_total_object_wrench
                    is not None,
                    allocated_gripper_wrenches=optional(
                        output.allocated_gripper_wrenches, (2, 6)
                    ),
                    allocated_gripper_wrenches_valid=output.allocated_gripper_wrenches
                    is not None,
                    executed_actuator_commands=output.executed_actuator_commands.copy(),
                    executed_virtual_drive_wrenches=output.executed_virtual_drive_wrenches.copy(),
                    baseline_virtual_drive_proposal=inp.baseline_drive_proposal.copy(),
                    baseline_nominal_support=inp.baseline_nominal_support.copy(),
                    baseline_feedback=inp.baseline_feedback.copy(),
                    realized_contact_wrench_substep=after.fields[
                        "object_contact_wrench"
                    ].copy(),
                    controller_mode=output.controller_mode,
                    assist_reference_executed=output.assist_reference_executed,
                    saturation_status=output.saturation_status.copy(),
                    pad_saturation_status=output.pad_saturation_status.copy(),
                    contact_status=output.contact_status.copy(),
                    **applied,
                )
            )
        c = {k: np.array([v[k] for v in steps]) for k in steps[0]}
        for old, new in [
            ("controller_call_timestamp", "controller_call_timestamps"),
            ("physics_step_tick", "physics_step_ticks"),
            ("realized_timestamp", "realized_substep_timestamps"),
        ]:
            c[new] = c.pop(old)
        c.update(
            timestamp=before.timestamp,
            decision_timestamp=before.timestamp,
            reference_timestamp=reference.timestamp,
            desired_assist_torque_ref=reference.torque_nm,
            reference_source=reference.source,
            reference_valid=reference.valid,
            reference_frame_id=reference.frame_id,
            reference_point=reference.reference_point,
            requested_assistance_wrench=reference.assistance_wrench(),
            history_latest_timestamp=float(policy_history.timestamps[-1]),
            history_length=len(policy_history.timestamps),
            realized_object_contact_wrench=world.current.fields[
                "object_contact_wrench"
            ].copy(),
            tracking_error_Nm=np.nan,
            tracking_error_Nm_valid=False,
        )
        p = dict(
            timestamp=before.timestamp,
            human_goal=start_truth["human_goal"],
            human_target_trajectory=start_truth["human_target_trajectory"],
            human_applied_force=start_truth["human_applied_wrench"][:3],
            human_applied_torque=start_truth["human_applied_wrench"][3:],
            external_disturbance_force=start_truth["external_disturbance_wrench"][:3],
            external_disturbance_torque=start_truth["external_disturbance_wrench"][3:],
            human_application_point=start_truth["human_application_point"],
            external_application_point=start_truth["external_application_point"],
            applied_wrench_timestamps=c["controller_call_timestamps"],
            applied_human_wrench_substeps=np.array(
                [v["human_applied_wrench"] for v in applied_truth]
            ),
            applied_external_wrench_substeps=np.array(
                [v["external_disturbance_wrench"] for v in applied_truth]
            ),
            scenario_label=scenario,
            human_goal_type=config["tasks"][scenario]["human_goal"],
            random_seed=config["parameters"]["seed"],
        )
        sections["observations"].append(snapshot_dict(before))
        sections["next_observations"].append(snapshot_dict(world.current))
        sections["supervision"].append(
            dict(
                timestamp=before.timestamp,
                expert_assist_torque_ref=np.nan,
                expert_label_valid=False,
                expert_source="unavailable",
            )
        )
        sections["controller"].append(c)
        sections["privileged"].append(p)
    meta = dict(
        schema_version=schema()["schema_version"],
        scenario_id=config["scenario_id"],
        scenario_version=config["scenario_version"],
        model_hash=config["model_hash"],
        config_hash=config_hash(config),
        duration_seconds=duration,
        physics_dt=dt,
        physics_steps_per_transition=stride,
        engine_time_origin=world.origin,
        controller_factory=controller_factory.__module__
        + ":"
        + controller_factory.__name__,
        controller_modes=sorted(
            {mode for row in sections["controller"] for mode in row["controller_mode"]}
        ),
        reference_test=reference_test,
        assist_reference_executed=bool(
            any(
                row["assist_reference_executed"].any() for row in sections["controller"]
            )
        ),
        tracking_evaluation="unavailable",
        expert_labels="unavailable",
        reference_amplitude_Nm=config["scripted_reference"]["amplitude_Nm"],
        reference_attainability="not established by data pipeline validation",
        mujoco_version=mujoco.__version__,
        numpy_version=np.__version__,
        python_version=platform.python_version(),
        platform=platform.platform(),
        warning_counts=world.sim.d.warning.number.tolist(),
    )
    save_episode(path, sections, meta)
    return meta


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--output", type=Path, default=REPO / "results/phase1a/smoke")
    ap.add_argument(
        "--controller",
        default="phase1a.interfaces.controller_contract:SupportOnlyAdapter",
    )
    a = ap.parse_args()
    if a.output.exists():
        ap.error("choose a fresh output directory; previous results are immutable")
    config = load_scenario()
    module, cls = a.controller.split(":")
    factory = getattr(importlib.import_module(module), cls)
    a.output.mkdir(parents=True)
    (a.output / "schema.json").write_text(json.dumps(schema(), indent=2) + "\n")
    result = {}
    for name, scenario, duration, reference in [
        ("A", "A", 4.0, False),
        ("B", "B", 4.0, False),
        ("C", "C", 4.0, False),
        ("reference_test", "B", 5.0, True),
    ]:
        meta = run_episode(
            a.output / name,
            scenario,
            config,
            duration=duration,
            reference_test=reference,
            controller_factory=factory,
        )
        if name == "A":
            (a.output / "schema.json").write_text(
                json.dumps({**schema(), "array_layout": meta["array_layout"]}, indent=2)
                + "\n"
            )
        result[name] = validate_dataset(
            a.output / name, require_support_only=factory is SupportOnlyAdapter
        )
        print(name, result[name], flush=True)
    (a.output / "validation.json").write_text(json.dumps(result, indent=2) + "\n")


if __name__ == "__main__":
    main()
