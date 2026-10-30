"""Causal E0/E1/E2 candidate references. No simulator, execution, or training."""

from dataclasses import asdict, dataclass, field

import numpy as np

from phase1b.interfaces import MODES, AssistanceDecision, clock, current, number, pose


def rotation(q):
    pose(np.r_[np.zeros(3), q])
    w, x, y, z = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def panel_inertia(mass, dimensions):
    number(mass, "mass")
    d = np.asarray(dimensions, dtype=float)
    if mass <= 0 or d.shape != (3,) or not np.isfinite(d).all() or np.any(d <= 0):
        raise ValueError("positive mass and xyz dimensions required")
    return np.diag(mass / 12 * (np.sum(d * d) - d * d))


def required_assistance_y(
    inertia_world,
    omega_world,
    alpha_world,
    human_torque_y,
    disturbance_torque_y,
    base_torque_y,
):
    """Rigid-body inverse dynamics about COM; not an actuator feasibility test."""
    inertia = np.asarray(inertia_world, dtype=float)
    omega, alpha = (
        np.asarray(omega_world, dtype=float),
        np.asarray(alpha_world, dtype=float),
    )
    if (
        inertia.shape != (3, 3)
        or not np.isfinite(inertia).all()
        or not np.allclose(inertia, inertia.T)
        or np.linalg.eigvalsh(inertia).min() <= 0
    ):
        raise ValueError("symmetric positive definite world inertia required")
    if (
        omega.shape != (3,)
        or alpha.shape != (3,)
        or not np.isfinite([omega, alpha]).all()
    ):
        raise ValueError("finite world omega/alpha required")
    for v in (human_torque_y, disturbance_torque_y, base_torque_y):
        number(v, "torque")
    return float(
        (inertia @ alpha + np.cross(omega, inertia @ omega))[1]
        - human_torque_y
        - disturbance_torque_y
        - base_torque_y
    )


def human_pitch_impedance(kh, dh, hand_offset_world):
    for v in (kh, dh):
        number(v, "hand impedance")
        if v < 0:
            raise ValueError("negative impedance")
    r = np.asarray(hand_offset_world, dtype=float)
    if r.shape != (3,) or not np.isfinite(r).all():
        raise ValueError("finite world offset required")
    jac = np.cross([0.0, 1.0, 0.0], r)
    return float(kh * (jac @ jac)), float(dh * (jac @ jac))


@dataclass(frozen=True)
class PublicSample:
    timestamp: float
    pitch: float
    omega_y: float
    robot_torque_left_y: float
    robot_torque_right_y: float
    off_axis_omega_norm: float = 0.0

    def __post_init__(self):
        clock(self.timestamp)
        for k, v in asdict(self).items():
            number(v, k)
        if self.off_axis_omega_norm < 0:
            raise ValueError("negative norm")

    @classmethod
    def from_observation(cls, observation):
        """Only public contact F/T + EE/object poses and object angular velocity."""
        xyz = np.asarray(observation["object_position"])
        quat = np.asarray(observation["object_orientation"])
        pose(np.r_[xyz, quat])
        R = rotation(quat)
        # atan2(-R20,R00) is the existing panel pitch definition, not arbitrary Euler-y rate.
        if np.hypot(R[2, 0], R[0, 0]) < 1e-8:
            raise ValueError("pitch projection singular")
        ee = np.asarray(observation["end_effector_pose"])
        omega = np.asarray(observation["object_angular_velocity"])
        if ee.shape != (2, 7) or omega.shape != (3,) or not np.isfinite(omega).all():
            raise ValueError("finite EE poses and world angular velocity required")
        torques = []
        for arm, side in enumerate(("left", "right")):
            pose(ee[arm])
            wrench = np.asarray(observation["wrist_ft_" + side])
            if wrench.shape != (6,) or not np.isfinite(wrench).all():
                raise ValueError("finite reconstructed F/T required")
            # panel-on-gripper local -> gripper-on-panel world, shifted to panel COM.
            rg = rotation(ee[arm, 3:])
            force = -rg @ wrench[:3]
            torque = -rg @ wrench[3:] + np.cross(ee[arm, :3] - xyz, force)
            torques.append(float(torque[1]))
        return cls(
            float(observation["timestamp"]),
            float(np.arctan2(-R[2, 0], R[0, 0])),
            float(omega[1]),
            *torques,
            float(np.linalg.norm(omega[[0, 2]])),
        )


@dataclass(frozen=True)
class TeacherGoal:
    """Privileged current goal, never an ObjectCommand or student observation."""

    timestamp: float
    mode: str
    angle: float

    def __post_init__(self):
        clock(self.timestamp)
        number(self.angle, "goal angle")
        if self.mode not in MODES:
            raise ValueError("unknown teacher mode")


@dataclass(frozen=True)
class DisturbanceTruth:
    timestamp: float
    torque_y: float

    def __post_init__(self):
        clock(self.timestamp)
        number(self.torque_y, "disturbance world Ty")


@dataclass(frozen=True)
class ExpertConfig:
    """All gains/limits are explicit; nothing here certifies physical reachability."""

    inertia_y: float
    force_gain: float
    lpf_time_constant: float
    kp_rotate: float
    kd_rotate: float
    kp_hold: float
    kd_hold: float
    torque_max: float
    torque_rate_max: float
    reference_rate_max: float
    gate_torque_on: float
    gate_torque_off: float
    gate_omega_on: float
    gate_omega_off: float
    gate_confirm_seconds: float
    gate_release_seconds: float
    disturbance_fraction: float
    limits_provenance: str
    policy_dt: float = 0.01

    def __post_init__(self):
        for k, v in asdict(self).items():
            if k == "limits_provenance":
                continue
            number(v, k)
            if v < 0:
                raise ValueError("negative " + k)
        for k in (
            "inertia_y",
            "lpf_time_constant",
            "torque_max",
            "torque_rate_max",
            "reference_rate_max",
            "gate_confirm_seconds",
            "gate_release_seconds",
        ):
            if getattr(self, k) <= 0:
                raise ValueError("positive " + k + " required")
        if not np.isclose(self.policy_dt, 0.01, atol=1e-12, rtol=0):
            raise ValueError("100 Hz reference required")
        if (
            self.gate_torque_on <= self.gate_torque_off
            or self.gate_omega_on < self.gate_omega_off
        ):
            raise ValueError(
                "on thresholds must exceed torque/off and not undercut omega/off"
            )
        if not 0 <= self.disturbance_fraction <= 1:
            raise ValueError("lambda_d outside [0,1]")
        if (
            not isinstance(self.limits_provenance, str)
            or not self.limits_provenance.strip()
        ):
            raise ValueError(
                "explicit limits provenance required; no assumed reachable limits"
            )


@dataclass(frozen=True)
class Candidate:
    decision: AssistanceDecision
    expert_id: str
    candidate_valid: bool
    intent_gate: bool
    gate_direction: int
    interaction_torque_estimate: float | None
    filtered_interaction_torque: float | None
    acceleration_estimate: float | None
    delta_pitch: float | None
    reference_angle: float
    reference_velocity: float
    reference_acceleration: float
    torque_before_limit: float
    torque_after_amplitude_limit: float
    torque_saturated: bool
    torque_rate_limited: bool
    rate_limit_overridden: bool
    label_invalid_reason: str
    execution_valid: bool = field(default=False, init=False)
    supervision_eligible: bool = field(default=False, init=False)

    def record(self):
        row = {k: v for k, v in asdict(self).items() if k != "decision"}
        row.update(
            timestamp=self.decision.timestamp,
            expert_mode=self.decision.mode,
            expert_mode_valid=self.candidate_valid,
            candidate_assist_torque_ref=self.decision.assist_torque_y_nm,
            torque_after_limit=self.decision.assist_torque_y_nm,
            source=self.decision.source,
            label_status="candidate_only" if self.candidate_valid else "unavailable",
        )
        for k in (
            "interaction_torque_estimate",
            "filtered_interaction_torque",
            "acceleration_estimate",
            "delta_pitch",
        ):
            row[k + "_valid"] = row[k] is not None
            if row[k] is None:
                row[k] = np.nan
        return row


class ReferenceGenerator:
    """Stateful analytic candidate generator. E0 rejects all privileged inputs."""

    def __init__(self, expert_id, config):
        if expert_id not in ("E0", "E1", "E2") or not isinstance(config, ExpertConfig):
            raise ValueError("E0/E1/E2 and ExpertConfig required")
        self.expert_id, self.config = expert_id, config
        self.previous = None
        self.filtered = None
        self.active_direction = self.pending_direction = 0
        self.confirm_time = self.release_time = 0.0
        self.reference_angle = self.reference_velocity = self.previous_torque = 0.0
        self.previous_mode = "UNCERTAIN"

    def _gate(self, tau, omega, dt):
        c = self.config
        direction = int(np.sign(tau))
        on = (
            direction != 0
            and abs(tau) >= c.gate_torque_on
            and direction * omega >= c.gate_omega_on
        )
        if self.active_direction:
            opposite = (
                abs(tau) >= c.gate_torque_on and direction != self.active_direction
            )
            keep = (
                self.active_direction * tau > c.gate_torque_off
                or self.active_direction * omega > c.gate_omega_off
            )
            self.release_time = 0.0 if keep else self.release_time + dt
            if opposite or self.release_time + 1e-12 >= c.gate_release_seconds:
                self.active_direction = self.pending_direction = 0
                self.confirm_time = self.release_time = 0.0
        if not self.active_direction:
            if not on:
                self.pending_direction, self.confirm_time = 0, 0.0
            else:
                self.confirm_time = (
                    self.confirm_time + dt
                    if direction == self.pending_direction
                    else dt
                )
                self.pending_direction = direction
                if self.confirm_time + 1e-12 >= c.gate_confirm_seconds:
                    self.active_direction = direction
        return self.active_direction

    def step(self, observation, *, goal=None, disturbance=None):
        if not isinstance(observation, PublicSample):
            raise TypeError("PublicSample required")
        if self.expert_id == "E0" and (goal is not None or disturbance is not None):
            raise ValueError("E0 may not receive privileged inputs")
        if self.expert_id != "E0":
            if not isinstance(goal, TeacherGoal):
                raise TypeError("current TeacherGoal required")
            current(goal.timestamp, observation.timestamp, 0)
        if self.expert_id == "E1" and disturbance is not None:
            raise ValueError("E1 may not receive disturbance truth")
        if self.expert_id == "E2":
            if not isinstance(disturbance, DisturbanceTruth):
                raise TypeError("current DisturbanceTruth required")
            current(disturbance.timestamp, observation.timestamp, 0)
        c, t = self.config, observation.timestamp
        dt = c.policy_dt
        acceleration = interaction = delta_pitch = None
        if self.previous is not None:
            if not np.isclose(t - self.previous.timestamp, dt, atol=1e-12, rtol=0):
                raise ValueError(
                    "nonconsecutive observation clock; reset generator between episodes"
                )
            acceleration = (observation.omega_y - self.previous.omega_y) / dt
            delta_pitch = float(
                np.arctan2(
                    np.sin(observation.pitch - self.previous.pitch),
                    np.cos(observation.pitch - self.previous.pitch),
                )
            )
            interaction = (
                c.inertia_y * acceleration
                - observation.robot_torque_left_y
                - observation.robot_torque_right_y
            )
            for name, value in (
                ("acceleration", acceleration),
                ("interaction", interaction),
                ("delta pitch", delta_pitch),
            ):
                number(value, name)
            alpha = -np.expm1(-dt / c.lpf_time_constant)
            self.filtered = (
                interaction
                if self.filtered is None
                else self.filtered + alpha * (interaction - self.filtered)
            )
        if self.filtered is not None:
            number(self.filtered, "filtered interaction")
        self.previous = observation
        raw, mode, gate, candidate_valid = 0.0, "UNCERTAIN", False, True
        reason = "execution_and_expert_quality_not_validated"
        ref_velocity = ref_acceleration = 0.0
        ref_angle = observation.pitch
        if self.expert_id == "E0":
            if self.filtered is None:
                candidate_valid, reason = False, "causal_acceleration_warmup"
            else:
                raw = c.force_gain * self.filtered
                mode = "HOLD" if raw == 0 else "ROTATE_POS" if raw > 0 else "ROTATE_NEG"
        elif goal.mode == "HOLD":
            self.active_direction = self.pending_direction = 0
            self.confirm_time = self.release_time = 0.0
            ref_angle = (
                self.reference_angle
                if self.previous_mode == "HOLD"
                else observation.pitch
            )
            mode = "HOLD"
            raw = (
                c.kp_hold * (ref_angle - observation.pitch)
                - c.kd_hold * observation.omega_y
            )
        elif goal.mode == "UNCERTAIN":
            self.active_direction = self.pending_direction = 0
            self.confirm_time = self.release_time = 0.0
            reason = "teacher_uncertain; execution_not_validated"
        else:
            direction = (
                self._gate(self.filtered, observation.omega_y, dt)
                if self.filtered is not None
                else 0
            )
            wanted = 1 if goal.mode == "ROTATE_POS" else -1
            gate = direction == wanted
            if gate:
                start = (
                    self.reference_angle
                    if self.previous_mode == goal.mode
                    else observation.pitch
                )
                ref_angle = start + float(
                    np.clip(
                        goal.angle - start,
                        -c.reference_rate_max * dt,
                        c.reference_rate_max * dt,
                    )
                )
                ref_velocity = (ref_angle - start) / dt
                ref_acceleration = (ref_velocity - self.reference_velocity) / dt
                raw = (
                    c.inertia_y * ref_acceleration
                    + c.kp_rotate * (ref_angle - observation.pitch)
                    + c.kd_rotate * (ref_velocity - observation.omega_y)
                )
                mode = goal.mode
            else:
                reason = "observable_rotation_gate_closed; execution_not_validated"
        # UNCERTAIN dominates oracle feedforward: no anticipatory assistance.
        if self.expert_id == "E2" and mode != "UNCERTAIN":
            raw -= c.disturbance_fraction * disturbance.torque_y
        for name, value in (
            ("raw torque", raw),
            ("reference angle", ref_angle),
            ("reference velocity", ref_velocity),
            ("reference acceleration", ref_acceleration),
        ):
            number(value, name)
        limited = float(np.clip(raw, -c.torque_max, c.torque_max))
        output = float(
            np.clip(
                limited,
                self.previous_torque - c.torque_rate_max * dt,
                self.previous_torque + c.torque_rate_max * dt,
            )
        )
        rate_limited = not np.isclose(output, limited, atol=1e-12, rtol=0)
        override = False
        if mode == "UNCERTAIN":
            # Safety cancellation beats slew limiting; explicitly record the exception.
            override = abs(self.previous_torque) > c.torque_rate_max * dt + 1e-12
            output, rate_limited = 0.0, False
        if self.expert_id == "E0" and mode != "UNCERTAIN":
            mode = (
                "HOLD" if output == 0 else "ROTATE_POS" if output > 0 else "ROTATE_NEG"
            )
        source = "rule_based" if self.expert_id == "E0" else "oracle_expert"
        result = Candidate(
            AssistanceDecision(t, mode, output, None, source, candidate_valid),
            self.expert_id,
            candidate_valid,
            gate,
            self.active_direction,
            interaction,
            self.filtered,
            acceleration,
            delta_pitch,
            ref_angle,
            ref_velocity,
            ref_acceleration,
            float(raw),
            limited,
            not np.isclose(limited, raw, atol=1e-12, rtol=0),
            rate_limited,
            override,
            reason,
        )
        self.reference_angle, self.reference_velocity = ref_angle, ref_velocity
        self.previous_torque, self.previous_mode = output, mode
        return result
