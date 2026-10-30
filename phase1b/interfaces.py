"""Position-actuator adapter and opt-in joint observations; no expert/controller claim."""

from dataclasses import dataclass, field, replace
from types import MappingProxyType
from typing import Literal, Protocol

import numpy as np


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
    source: Literal["scripted_test", "model_based_expert", "learned_policy"] = (
        "scripted_test"
    )
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


POLICY_SHAPES = {
    "wrist_ft_left": (6,),
    "wrist_ft_right": (6,),
    "tactile_left": (2, 3, 4, 4),
    "tactile_right": (2, 3, 4, 4),
    "object_position": (3,),
    "object_orientation": (4,),
    "object_linear_velocity": (3,),
    "object_angular_velocity": (3,),
    "contact_state_left": (2,),
    "contact_state_right": (2,),
}


@dataclass(frozen=True)
class PolicyHistory:
    timestamps: np.ndarray
    features: dict


# Sensing baseline; joint state logging is unconditional.
POLICY_PROFILES = {
    "baseline": tuple(POLICY_SHAPES),
    "with_joint_states": (*POLICY_SHAPES, "arm_joint_position", "arm_joint_velocity"),
}


def policy_observation(observation, profile="baseline"):
    if profile not in POLICY_PROFILES:
        raise ValueError("unknown observation profile")
    return MappingProxyType(
        {k: immutable(observation[k]) for k in POLICY_PROFILES[profile]}
    )


@dataclass(frozen=True)
class ArmControllerInput:
    timestamp: float
    control_dt: float
    observation: dict
    nominal_arm_position: np.ndarray
    nominal_pad_force: np.ndarray
    arm_position_limits: np.ndarray
    pad_force_limits: np.ndarray


@dataclass(frozen=True)
class ArmCommand:
    arm_position_target: np.ndarray  # (2,7), radians, Menagerie position servos
    pad_force_command: np.ndarray  # (4,), N, custom pad motors
    controller_mode: str = "position_hold_baseline"
    assist_reference_executed: bool = False


def validate_command(command, inp):
    if not isinstance(command, ArmCommand):
        raise TypeError("controller must return ArmCommand")
    for value, shape, limits in (
        (command.arm_position_target, (2, 7), inp.arm_position_limits),
        (command.pad_force_command, (4,), inp.pad_force_limits),
    ):
        if (
            not isinstance(value, np.ndarray)
            or value.shape != shape
            or not np.isfinite(value).all()
        ):
            raise ValueError("invalid command")
        if np.any(value < limits[..., 0]) or np.any(value > limits[..., 1]):
            raise ValueError("command limit exceeded")
    if (
        type(command.assist_reference_executed) is not bool
        or not isinstance(command.controller_mode, str)
        or not command.controller_mode
    ):
        raise ValueError("invalid execution metadata")
    if (
        command.controller_mode == "position_hold_baseline"
        and command.assist_reference_executed
    ):
        raise ValueError("hold baseline does not execute assistance")


Mode = Literal["ROTATE_POS", "ROTATE_NEG", "HOLD", "UNCERTAIN"]
Source = Literal["scripted_test", "rule_based", "oracle_expert", "learned_policy"]
MODES = ("ROTATE_POS", "ROTATE_NEG", "HOLD", "UNCERTAIN")
SOURCES = ("scripted_test", "rule_based", "oracle_expert", "learned_policy")


def number(value, name):
    if (
        isinstance(value, (bool, np.bool_))
        or not isinstance(value, (int, float, np.integer, np.floating))
        or not np.isfinite(value)
    ):
        raise ValueError(f"{name} must be a finite number")


def clock(timestamp):
    number(timestamp, "timestamp")
    if timestamp < 0:
        raise ValueError("negative timestamp")


def current(timestamp, now, max_age):
    clock(timestamp)
    clock(now)
    if timestamp > now + 1e-12 or now - timestamp > max_age + 1e-12:
        raise ValueError("future or stale timestamp")


def pose(value):
    finite_array(value, (7,), "world xyz + quaternion wxyz")
    if not np.isclose(np.linalg.norm(value[3:]), 1, atol=1e-8, rtol=0):
        raise ValueError("quaternion must be unit wxyz")


@dataclass(frozen=True)
class AssistanceDecision:
    timestamp: float
    mode: Mode
    assist_torque_y_nm: float
    confidence: float | None
    source: Source
    valid: bool

    def __post_init__(self):
        clock(self.timestamp)
        number(self.assist_torque_y_nm, "world +Y assistance about panel COM, Nm")
        if (
            self.mode not in MODES
            or self.source not in SOURCES
            or type(self.valid) is not bool
        ):
            raise ValueError("invalid mode, source or validity")
        if self.confidence is not None:
            number(self.confidence, "confidence")
            if not 0 <= self.confidence <= 1:
                raise ValueError("confidence outside [0,1]")
        if not self.valid and self.mode != "UNCERTAIN":
            raise ValueError("invalid decision must be UNCERTAIN")
        if self.mode == "UNCERTAIN" and self.assist_torque_y_nm != 0:
            raise ValueError("UNCERTAIN cannot request active assistance")
        # Intent direction and torque sign differ during braking/disturbance rejection.

    def to_legacy(self):
        if self.source == "rule_based":
            raise ValueError("legacy source enum cannot represent rule_based")
        return AssistTorqueReference(
            self.timestamp,
            self.assist_torque_y_nm if self.valid else None,
            source="model_based_expert"
            if self.source == "oracle_expert"
            else self.source,
            valid=self.valid,
        )

    @classmethod
    def from_legacy(cls, reference, mode):
        if not isinstance(reference, AssistTorqueReference):
            raise TypeError("AssistTorqueReference required")
        # A scalar cannot distinguish HOLD resistance from rotation intent.
        return cls(
            reference.timestamp,
            mode,
            reference.torque_nm if reference.valid else 0.0,
            None,
            "oracle_expert"
            if reference.source == "model_based_expert"
            else reference.source,
            reference.valid,
        )


@dataclass(frozen=True)
class ObjectCommand:
    timestamp: float
    mode: Mode
    pose_ref_world: np.ndarray  # xyz + quaternion wxyz, at panel COM
    twist_ref_world: np.ndarray  # angular xyz [rad/s], linear xyz [m/s]
    assist_wrench_world: np.ndarray  # force xyz [N], torque xyz [Nm]
    impedance_parameters: dict = field(default_factory=dict)
    reference_point: str = "panel_com"
    valid: bool = True
    frame_id: str = "world"

    def __post_init__(self):
        clock(self.timestamp)
        if self.mode not in MODES or type(self.valid) is not bool:
            raise ValueError("invalid mode or validity")
        if self.frame_id != "world" or self.reference_point != "panel_com":
            raise ValueError("world / instantaneous panel_com required")
        pose(self.pose_ref_world)
        finite_array(self.twist_ref_world, (6,), "twist angular then linear")
        finite_array(self.assist_wrench_world, (6,), "wrench force then torque")
        # MVP has exactly one assistance axis. Base support is not this wrench.
        AssistanceDecision(
            self.timestamp,
            self.mode,
            float(self.assist_wrench_world[4]),
            None,
            "scripted_test",
            self.valid,
        )
        if np.any(self.assist_wrench_world[[0, 1, 2, 3, 5]]):
            raise ValueError("MVP assistance is world Ty only")
        if self.mode == "UNCERTAIN" and np.any(self.twist_ref_world):
            raise ValueError("UNCERTAIN cannot request active motion")
        if not isinstance(self.impedance_parameters, (dict, MappingProxyType)) or set(
            self.impedance_parameters
        ) - {"stiffness", "damping"}:
            raise ValueError("impedance keys: stiffness, damping (angular then linear)")
        params = {}
        for key, value in self.impedance_parameters.items():
            finite_array(value, (6,), key)
            if np.any(value < 0):
                raise ValueError("negative impedance parameter")
            params[key] = immutable(value)
        if (
            self.assist_wrench_world[4] != 0
            and "stiffness" in params
            and params["stiffness"][1] != 0
        ):
            raise ValueError(
                "nonzero assistance cannot combine with pitch position stiffness"
            )
        for name in ("pose_ref_world", "twist_ref_world", "assist_wrench_world"):
            object.__setattr__(self, name, immutable(getattr(self, name)))
        object.__setattr__(self, "impedance_parameters", MappingProxyType(params))


# Explicit public boundary, not all keys provided by a caller or simulator.
PUBLIC_SHAPES = {
    **POLICY_SHAPES,
    "arm_joint_position": (2, 7),
    "arm_joint_velocity": (2, 7),
    "arm_actuator_torque": (2, 7),
    "arm_actuator_force": (2, 7),
    "gripper_joint_position": (2, 2),
    "gripper_joint_velocity": (2, 2),
    "gripper_actuator_force": (4,),
    "end_effector_pose": (2, 7),
    "end_effector_velocity": (2, 6),
    "wrist_sensor_left": (6,),
    "wrist_sensor_right": (6,),
    "object_contact_wrench": (6,),
    "wrist_ft_left_world_at_panel_com": (6,),
    "wrist_ft_right_world_at_panel_com": (6,),
}


def public_input(inp):
    if not isinstance(inp, ArmControllerInput):
        raise TypeError("ArmControllerInput required")
    clock(inp.timestamp)
    current(float(inp.observation["timestamp"]), inp.timestamp, 0)
    if not np.isclose(inp.control_dt, 0.002, atol=1e-12, rtol=0):
        raise ValueError("500 Hz control required")
    obs = {}
    for key, shape in PUBLIC_SHAPES.items():
        finite_array(inp.observation[key], shape, key)
        obs[key] = immutable(inp.observation[key])
    pose(np.r_[obs["object_position"], obs["object_orientation"]])
    obs["timestamp"] = inp.timestamp
    copies = {}
    for key, shape in (
        ("nominal_arm_position", (2, 7)),
        ("nominal_pad_force", (4,)),
        ("arm_position_limits", (2, 7, 2)),
        ("pad_force_limits", (4, 2)),
    ):
        finite_array(getattr(inp, key), shape, key)
        copies[key] = immutable(getattr(inp, key))
    return replace(inp, observation=MappingProxyType(obs), **copies)


def history_view(samples, now, profile="baseline"):
    times = np.array([s["timestamp"] for s in samples])
    if len(times) == 0 or np.any(np.diff(times) <= 0):
        raise ValueError("ordered nonempty history required")
    for t in times:
        current(t, now, now)
    current(times[-1], now, 0)
    selected = [policy_observation(s, profile) for s in samples]
    for sample in selected:
        for key, value in sample.items():
            finite_array(value, PUBLIC_SHAPES[key], key)
        pose(np.r_[sample["object_position"], sample["object_orientation"]])
    return PolicyHistory(
        immutable(times),
        MappingProxyType({k: immutable([s[k] for s in selected]) for k in selected[0]}),
    )


class AssistancePolicy(Protocol):
    def predict(self, history: PolicyHistory) -> AssistanceDecision: ...


class ObjectMotionController(Protocol):
    def step(
        self, decision: AssistanceDecision, observation: ArmControllerInput, dt: float
    ) -> ObjectCommand: ...


class DualArmController(Protocol):
    def step(
        self, command: ObjectCommand, observation: ArmControllerInput
    ) -> ArmCommand: ...


class ReferenceOnlyObjectAdapter:
    """Latch HOLD pose and encode requests; no rotation planner or impedance law."""

    def __init__(self):
        self.held_pose = None
        self.previous_mode = None

    def step(self, decision, observation, dt):
        observation = public_input(observation)
        current(decision.timestamp, observation.timestamp, 0)
        if not np.isclose(dt, 0.01, atol=1e-12, rtol=0):
            raise ValueError("100 Hz object reference required")
        measured = np.r_[
            observation.observation["object_position"],
            observation.observation["object_orientation"],
        ]
        if decision.mode == "HOLD" and self.previous_mode != "HOLD":
            self.held_pose = measured.copy()
        self.previous_mode = decision.mode
        return ObjectCommand(
            decision.timestamp,
            decision.mode,
            self.held_pose if decision.mode == "HOLD" else measured,
            np.zeros(6),
            np.array([0.0, 0.0, 0.0, 0.0, decision.assist_torque_y_nm, 0.0]),
            valid=decision.valid,
        )


class PositionHoldDualArmAdapter:
    """Reuse verified joint hold. Deliberately does not execute ObjectCommand."""

    def step(self, command, observation):
        observation = public_input(observation)
        current(command.timestamp, observation.timestamp, 0.008)
        return ArmCommand(
            observation.nominal_arm_position.copy(),
            observation.nominal_pad_force.copy(),
        )
