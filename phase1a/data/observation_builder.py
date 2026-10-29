"""Strict causal policy boundary: only named current/past sensors may enter history."""

from dataclasses import dataclass
from types import MappingProxyType

import numpy as np

from phase1a.interfaces.controller_contract import finite_array, immutable

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
OBS_SHAPES = {
    **POLICY_SHAPES,
    "wrist_ft_left_world_at_panel_com": (6,),
    "wrist_ft_right_world_at_panel_com": (6,),
    "object_contact_wrench": (6,),
    "gripper_pose": (2, 7),
    "gripper_velocity": (2, 6),
}


@dataclass(frozen=True)
class SensorObservation:
    timestamp: float
    engine_timestamp: float
    physics_tick: int
    fields: dict

    def __post_init__(self):
        if set(self.fields) != set(OBS_SHAPES):
            raise ValueError(
                "Unexpected/missing observation fields; no privileged or future fields allowed"
            )
        if (
            not np.isfinite(self.timestamp)
            or not np.isfinite(self.engine_timestamp)
            or self.physics_tick < 0
        ):
            raise ValueError("invalid observation clock")
        for key, shape in OBS_SHAPES.items():
            finite_array(self.fields[key], shape, key)
        if abs(np.linalg.norm(self.fields["object_orientation"]) - 1) > 1e-8:
            raise ValueError("invalid quaternion")
        object.__setattr__(
            self,
            "fields",
            MappingProxyType({k: immutable(v) for k, v in self.fields.items()}),
        )


@dataclass(frozen=True)
class PolicyHistory:
    timestamps: np.ndarray
    features: dict


def policy_observation_builder(observations, decision_timestamp, history_length=25):
    if not observations or history_length < 1:
        raise ValueError("nonempty history required")
    if not np.isfinite(decision_timestamp):
        raise ValueError("invalid decision time")
    times = np.array([o.timestamp for o in observations])
    if np.any(np.diff(times) <= 0) or np.any(times > decision_timestamp + 1e-12):
        raise ValueError("noncausal or unordered history")
    selected = observations[-history_length:]
    return PolicyHistory(
        immutable([o.timestamp for o in selected]),
        MappingProxyType(
            {
                k: immutable(np.stack([o.fields[k] for o in selected]))
                for k in POLICY_SHAPES
            }
        ),
    )
