"""Public force-policy/controller contract. No learned policy is implemented."""
from dataclasses import dataclass
from typing import Protocol
import numpy as np


@dataclass(frozen=True)
class AssistTorqueReference:
    timestamp: float
    # Additional cooperation torque, world +y, about current panel COM, Nm.
    # Excludes baseline gravity/stabilization. None means unavailable, never zero.
    torque_Nm: float | None
    available: bool
    reason: str


class LearnedForcePolicy(Protocol):
    def predict(self, observation_history: dict[str, np.ndarray]) -> AssistTorqueReference: ...


class ObjectWrenchController(Protocol):
    def step(self, reference: AssistTorqueReference, observation: dict) -> dict: ...


ASSIST_CONVENTION = {
    'frame': 'world', 'origin': 'instantaneous panel COM', 'axis': '+y', 'units': 'N m',
    'positive_direction': 'right-hand rule about world +y; panel +x edge moves down',
    'includes_gravity_compensation': False, 'includes_baseline_stabilization': False,
    'meaning': 'additional cooperative torque above independent baseline support',
    'total_reference': 'baseline object-level reference plus cooperative wrench; unavailable without allocator',
}
