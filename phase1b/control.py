"""100 Hz policy/object references, held for five 500 Hz arm steps."""

from dataclasses import replace

import numpy as np

from phase1b.interfaces import (
    ArmCommand,
    AssistanceDecision,
    ObjectCommand,
    PositionHoldDualArmAdapter,
    ReferenceOnlyObjectAdapter,
    current,
    history_view,
    number,
    public_input,
    validate_command,
)


class ScriptedContractPolicy:
    """Exercise interface cases, not a human-intent policy or Expert."""

    def predict(self, history):
        t = float(history.timestamps[-1])
        mode, torque, valid = (
            ("ROTATE_POS", 0.02, True),
            ("ROTATE_NEG", -0.02, True),
            ("HOLD", 0.01, True),
            ("HOLD", -0.01, True),
            ("UNCERTAIN", 0.0, True),
            ("UNCERTAIN", 0.0, False),
        )[round(t * 100) % 6]
        return AssistanceDecision(t, mode, torque, None, "scripted_test", valid)


def decision_row(d):
    return dict(
        timestamp=d.timestamp,
        mode=d.mode,
        assist_torque_y_nm=d.assist_torque_y_nm,
        confidence=np.nan if d.confidence is None else d.confidence,
        confidence_present=d.confidence is not None,
        source=d.source,
        valid=d.valid,
    )


def object_row(c):
    row = dict(
        timestamp=c.timestamp,
        mode=c.mode,
        pose_ref_world=c.pose_ref_world,
        twist_ref_world=c.twist_ref_world,
        assist_wrench_world=c.assist_wrench_world,
        frame_id=c.frame_id,
        reference_point=c.reference_point,
        valid=c.valid,
    )
    for key in ("stiffness", "damping"):
        row[key] = c.impedance_parameters.get(key, np.full(6, np.nan))
        row[key + "_valid"] = key in c.impedance_parameters
    return row


class ControlChain:
    """Three-layer orchestrator; all callbacks receive public copies."""

    def __init__(
        self,
        policy=None,
        object_controller=None,
        dual_arm_controller=None,
        profile="baseline",
        history_length=50,
        max_assist_torque_nm=0.02,
        max_rotation_rate_rad_s=0.5,
    ):
        if type(history_length) is not int or history_length <= 0:
            raise ValueError("positive integer history_length required")
        number(max_assist_torque_nm, "max_assist_torque_nm")
        if max_assist_torque_nm <= 0:
            raise ValueError("positive assistance bound required")
        self.policy = policy if policy is not None else ScriptedContractPolicy()
        self.object_controller = (
            object_controller
            if object_controller is not None
            else ReferenceOnlyObjectAdapter()
        )
        self.dual_arm_controller = (
            dual_arm_controller
            if dual_arm_controller is not None
            else PositionHoldDualArmAdapter()
        )
        self.profile, self.history_length = profile, history_length
        number(max_rotation_rate_rad_s, "max_rotation_rate_rad_s")
        if max_rotation_rate_rad_s <= 0:
            raise ValueError("positive rotation rate bound required")
        self.max_rotation_rate_rad_s = max_rotation_rate_rad_s
        self.max_assist_torque_nm = max_assist_torque_nm
        self.history = []
        self.ticks = 0

    def step(self, inp):
        inp = public_input(inp)
        current(inp.timestamp, self.ticks * 0.002, 0)
        if self.ticks % 5 == 0:
            self.history.append(inp.observation)
            self.history = self.history[-self.history_length :]
            decision = self.policy.predict(
                history_view(self.history, inp.timestamp, self.profile)
            )
            if not isinstance(decision, AssistanceDecision):
                raise TypeError("policy must return AssistanceDecision")
            current(decision.timestamp, inp.timestamp, 0)
            if abs(decision.assist_torque_y_nm) > self.max_assist_torque_nm:
                raise ValueError("assistance request exceeds configured bound")
            command = self.object_controller.step(decision, public_input(inp), 0.01)
            if not isinstance(command, ObjectCommand):
                raise TypeError("object controller must return ObjectCommand")
            current(command.timestamp, inp.timestamp, 0)
            if command.mode != decision.mode or command.valid != decision.valid:
                raise ValueError("object controller changed decision mode/validity")
            if command.assist_wrench_world[4] != decision.assist_torque_y_nm:
                raise ValueError(
                    "MVP object command must preserve requested assistance"
                )
            if (
                np.linalg.norm(command.twist_ref_world[:3])
                > self.max_rotation_rate_rad_s
            ):
                raise ValueError("rotation reference rate exceeds configured bound")
            if command.mode == "UNCERTAIN":
                measured = np.r_[
                    inp.observation["object_position"],
                    inp.observation["object_orientation"],
                ]
                if not np.allclose(
                    command.pose_ref_world, measured, atol=1e-12, rtol=0
                ):
                    raise ValueError("UNCERTAIN cannot introduce a new pose target")
            self.command = replace(
                command
            )  # own arrays before a callback can mutate them
            self.decision = decision

        current(self.command.timestamp, inp.timestamp, 0.008)
        result = self.dual_arm_controller.step(replace(self.command), public_input(inp))
        validate_command(result, inp)
        if self.decision.mode == "UNCERTAIN" and result.assist_reference_executed:
            raise ValueError("UNCERTAIN cannot report executed assistance")
        self.ticks += 1
        return ArmCommand(
            result.arm_position_target.copy(),
            result.pad_force_command.copy(),
            result.controller_mode,
            result.assist_reference_executed,
        )
