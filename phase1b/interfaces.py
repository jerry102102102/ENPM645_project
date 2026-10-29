"""Position-actuator adapter and opt-in joint observations; no expert/controller claim."""

from dataclasses import dataclass
from types import MappingProxyType

import numpy as np

from phase1a.data.observation_builder import POLICY_SHAPES
from phase1a.interfaces.controller_contract import AssistTorqueReference, immutable

# Backwards-compatible sensing baseline; joint state logging is unconditional.
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


class PositionHoldBaseline:
    """Keep nominal arm joint targets and preload; ignore assistance reference."""

    def step(self, reference: AssistTorqueReference, inp: ArmControllerInput):
        if reference.timestamp > inp.timestamp + 1e-12:
            raise ValueError("future reference")
        return ArmCommand(inp.nominal_arm_position.copy(), inp.nominal_pad_force.copy())
