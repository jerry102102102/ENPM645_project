"""Track A / Track B boundary. All wrenches use [force xyz, torque xyz]."""

from dataclasses import dataclass
from typing import Literal, Protocol

import numpy as np

Source = Literal["scripted_test", "model_based_expert", "learned_policy"]


def immutable(a):
    a = np.array(a, copy=True)
    a.setflags(write=False)
    return a


def finite_array(a, shape, name):
    if not isinstance(a, np.ndarray) or a.shape != shape or not np.isfinite(a).all():
        raise ValueError(name + " must be finite ndarray " + str(shape))


@dataclass(frozen=True)
class AssistTorqueReference:
    timestamp: float
    torque_nm: float | None
    frame_id: str = "world"
    reference_point: str = "panel_com"
    source: Source = "scripted_test"
    valid: bool = True

    def __post_init__(self):
        if not np.isfinite(self.timestamp) or self.timestamp < 0:
            raise ValueError("invalid reference timestamp")
        if self.frame_id != "world" or self.reference_point != "panel_com":
            raise ValueError(
                "Reference must be world +Y about instantaneous panel COM, Nm"
            )
        if self.source not in ("scripted_test", "model_based_expert", "learned_policy"):
            raise ValueError("unknown reference source")
        if type(self.valid) is not bool:
            raise ValueError("valid must be bool")
        if self.valid and (self.torque_nm is None or not np.isfinite(self.torque_nm)):
            raise ValueError("valid reference requires finite torque")
        if not self.valid and self.torque_nm is not None:
            raise ValueError("unavailable torque must be None")

    def assistance_wrench(self):
        return (
            np.array([0.0, 0.0, 0.0, 0.0, self.torque_nm, 0.0]) if self.valid else None
        )


@dataclass(frozen=True)
class ControllerInput:
    timestamp: float
    control_dt: float
    sensor: dict
    gripper_pose: np.ndarray
    gripper_velocity: np.ndarray
    baseline_drive_proposal: np.ndarray
    baseline_pad_proposal: np.ndarray
    baseline_nominal_support: np.ndarray
    baseline_feedback: np.ndarray
    calibration: dict


@dataclass(frozen=True)
class ControllerOutput:
    executed_actuator_commands: np.ndarray
    executed_virtual_drive_wrenches: np.ndarray
    saturation_status: np.ndarray  # [left/right, force/torque], bool
    pad_saturation_status: np.ndarray
    contact_status: np.ndarray  # [left/right, upper/lower] counts at decision
    controller_mode: str
    assist_reference_executed: bool
    requested_total_object_wrench: np.ndarray | None = None
    allocated_gripper_wrenches: np.ndarray | None = (
        None  # world, at each root; gripper-on-panel target
    )
    # Pre-step controller cannot report a future measurement. Pipeline fills feedback after physics.
    realized_object_wrench: np.ndarray | None = None


class ObjectWrenchController(Protocol):
    def step(
        self, reference: AssistTorqueReference, observation: ControllerInput
    ) -> ControllerOutput: ...


class LearnedForcePolicy(Protocol):
    def predict(self, history) -> AssistTorqueReference: ...


def validate_input(reference, o):
    from phase1a.data.observation_builder import POLICY_SHAPES

    if not isinstance(reference, AssistTorqueReference) or not isinstance(
        o, ControllerInput
    ):
        raise TypeError("typed controller inputs required")
    if (
        not np.isfinite(o.timestamp)
        or not np.isfinite(o.control_dt)
        or o.control_dt <= 0
    ):
        raise ValueError("invalid controller clock")
    if reference.timestamp > o.timestamp + 1e-12:
        raise ValueError("future reference")
    finite_array(o.gripper_pose, (2, 7), "gripper_pose")
    finite_array(o.gripper_velocity, (2, 6), "gripper_velocity")
    finite_array(o.baseline_drive_proposal, (2, 6), "baseline drive")
    finite_array(o.baseline_pad_proposal, (4,), "pad commands")
    finite_array(o.baseline_nominal_support, (2, 6), "baseline nominal support")
    finite_array(o.baseline_feedback, (2, 6), "baseline feedback")
    if set(o.sensor) != set(POLICY_SHAPES):
        raise ValueError("controller sensor fields must match public allowlist")
    for key, shape in POLICY_SHAPES.items():
        finite_array(o.sensor[key], shape, key)
    if abs(np.linalg.norm(o.sensor["object_orientation"]) - 1) > 1e-8:
        raise ValueError("invalid object quaternion")
    if not np.allclose(
        np.linalg.norm(o.gripper_pose[:, 3:], axis=1), 1, atol=1e-8, rtol=0
    ):
        raise ValueError("invalid gripper quaternions")


def validate_output(out, o):
    if not isinstance(out, ControllerOutput):
        raise TypeError("ControllerOutput required")
    finite_array(out.executed_actuator_commands, (4,), "executed_actuator_commands")
    finite_array(
        out.executed_virtual_drive_wrenches, (2, 6), "executed_virtual_drive_wrenches"
    )
    finite_array(out.contact_status, (2, 2), "contact_status")
    if not np.array_equal(
        out.contact_status,
        np.stack([o.sensor["contact_state_left"], o.sensor["contact_state_right"]]),
    ):
        raise ValueError("contact status is not current observation")
    for a, shape in [
        (out.saturation_status, (2, 2)),
        (out.pad_saturation_status, (4,)),
    ]:
        if not isinstance(a, np.ndarray) or a.shape != shape or a.dtype != bool:
            raise ValueError("saturation flags must be bool arrays")
    if (
        type(out.assist_reference_executed) is not bool
        or not isinstance(out.controller_mode, str)
        or not out.controller_mode
    ):
        raise ValueError("invalid mode/execution flag")
    if out.controller_mode == "support_only" and out.assist_reference_executed:
        raise ValueError("support_only cannot execute assistance reference")
    for a, shape, n in [
        (out.requested_total_object_wrench, (6,), "requested total"),
        (out.allocated_gripper_wrenches, (2, 6), "allocated"),
    ]:
        if a is not None:
            finite_array(a, shape, n)
    if out.realized_object_wrench is not None:
        raise ValueError(
            "realized wrench is post-step simulator feedback, not controller prediction"
        )
    drives = out.executed_virtual_drive_wrenches
    cal = o.calibration
    if np.any(
        np.linalg.norm(drives[:, :3], axis=1) > cal["root_drive_force_limit_N"] + 1e-10
    ) or np.any(
        np.linalg.norm(drives[:, 3:], axis=1)
        > cal["root_drive_torque_limit_Nm"] + 1e-10
    ):
        raise ValueError(
            "drive limit exceeded; controller must limit and report saturation"
        )
    limits = np.array(cal["pad_actuator_limits_N"])
    if np.any(out.executed_actuator_commands < limits[:, 0] - 1e-10) or np.any(
        out.executed_actuator_commands > limits[:, 1] + 1e-10
    ):
        raise ValueError("pad actuator limits exceeded")


class SupportOnlyAdapter:
    """Consume existing v2 support proposals. Deliberately ignore assistance torque."""

    def step(self, reference, observation):
        validate_input(reference, observation)
        o = observation
        raw = o.baseline_nominal_support + o.baseline_feedback
        sat = np.column_stack(
            [
                np.linalg.norm(raw[:, :3], axis=1)
                > o.calibration["root_drive_force_limit_N"],
                np.linalg.norm(raw[:, 3:], axis=1)
                > o.calibration["root_drive_torque_limit_Nm"],
            ]
        )
        result = ControllerOutput(
            o.baseline_pad_proposal.copy(),
            o.baseline_drive_proposal.copy(),
            sat,
            np.zeros(4, bool),
            np.stack([o.sensor["contact_state_left"], o.sensor["contact_state_right"]]),
            "support_only",
            False,
        )
        validate_output(result, o)
        return result
