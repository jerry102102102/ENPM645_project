# Three-Layer Control Interface v1.1

One execution chain: `PolicyObservation → AssistanceDecision → ObjectCommand → ArmCommand → MuJoCo`. Scenario 3.0.0 / Hardware 2.0.0 remain unchanged. Types are in `phase1b/interfaces.py`, orchestration in `phase1b/control.py`, and logging in `phase1b/pipeline.py`.

| Status | Current scope |
|---|---|
| Interface Ready | Types, validation, causal isolation, and formal logging are connected; see results/phase1b/current for tests |
| Controller Implemented | Only a reference-only object adapter and a position-hold dual-arm adapter |
| Assistance Tracking Validated | **No**; no allocation / tracking algorithm |
| Expert Labels Validated | **No**; all formal supervision remains invalid |

## ① AssistanceDecision

| Field | Contract |
|---|---|
| timestamp | Episode seconds, finite and nonnegative; must equal this decision's observation time |
| mode | `ROTATE_POS / ROTATE_NEG / HOLD / UNCERTAIN` |
| assist_torque_y_nm | Finite scalar; World +Y, instantaneous panel COM, Nm; excludes basic support |
| confidence | `None` or `[0,1]`; use None without a calibrated estimate |
| source | `scripted_test / rule_based / oracle_expert / learned_policy` |
| valid | Actual bool; an invalid decision must be UNCERTAIN with torque=0 |

ROTATE_POS / ROTATE_NEG express intent direction. Torque may oppose that direction for braking; its sign does not directly determine intent. HOLD may have positive, negative, or nonzero torque. UNCERTAIN requires zero torque and must not introduce active rotation. `HOLD + disturbance` is not a fifth intent. Valid UNCERTAIN means a valid assessment of uncertainty; valid=false means the upstream result is unavailable.

One scalar data adapter remains: `AssistanceDecision.from_legacy(AssistTorqueReference(...), mode="HOLD")`. Mode must be explicit, not inferred from the torque sign. `to_legacy()` maps oracle_expert to model_based_expert; the old enum cannot represent rule_based, so that conversion is explicitly rejected. This is a data conversion within one module, not a retained legacy pipeline.

## ② ObjectCommand

| Field | Contract |
|---|---|
| timestamp / mode / valid | Same as the current AssistanceDecision |
| pose_ref_world | `(7,)`, world xyz + unit quaternion **wxyz**, positioned at the panel COM |
| twist_ref_world | `(6,)`, **angular xyz → linear xyz**, rad/s and m/s |
| assist_wrench_world | `(6,)`, **force xyz → torque xyz**, N and Nm; MVP is exactly `[0,0,0,0,decision.torque,0]` |
| impedance_parameters | Optional stiffness/damping, each `(6,)`, nonnegative, **angular→linear**. An empty dict means unspecified, not an implemented zero-stiffness controller |
| reference_point / frame_id | Only `panel_com` / `world` |

Rotational stiffness/damping units are Nm/rad and Nm·s/rad; translational units are N/m and N·s/m. Arrays are defensively copied and made read-only; the impedance mapping is also read-only.

**Responsibilities and priority:** Basic support, contact, and mechanical limits take priority over active assistance. Pose/twist provide nominal motion references. The teammate's single compliant pitch control law must integrate the additional wrench; do not combine competing high-stiffness position control and torque control about Y. The contract rejects nonzero Ty combined with explicitly nonzero pitch stiffness.

`ControlChain` test defaults are `max_assist_torque_nm=.02` and `max_rotation_rate_rad_s=.5`. Excess requests are rejected, not silently modified. These are configurable interface bounds, not a validated safe operating envelope. The full controller remains responsible for pose-path rate limiting, smoothing, admittance, contact constraints, and saturation handling.

The current `ReferenceOnlyObjectAdapter` latches the current pose on entry to HOLD. ROTATE only encodes the torque request, retaining the current pose and zero twist; **it does not generate a rotation trajectory**. UNCERTAIN uses the current pose with zero twist/wrench. The lower-level position-hold adapter ignores high-level motion/wrench and makes no tracking claim.

## ③ ArmCommand

Retained fields: `arm_position_target (2,7)` in rad, `pad_force_command (4,)` in N, `controller_mode`, and `assist_reference_executed`. Validation checks finiteness, shapes, joint/pad command bounds, mode, and bool types. Arm position servos do not accept Nm.

To avoid duplicate clocks, ArmCommand has no separate timestamp field. At dispatch, the single pipeline records `command_timestamp`, plus the command's source `decision_timestamp` and `object_command_timestamp`. Together with the command, these form a timestamped execution event.

`assist_reference_executed` only indicates that the controller used assistance; it does not establish successful torque tracking. UNCERTAIN must not report executed assistance. The pipeline records actual actuator torque, contact state, saturation, and post-step wrench rather than accepting fabricated controller measurements.

## Protocol and clock

```python
AssistancePolicy.predict(history: PolicyHistory) -> AssistanceDecision
ObjectMotionController.step(decision, observation: ArmControllerInput, dt: float) -> ObjectCommand
DualArmController.step(command, observation: ArmControllerInput) -> ArmCommand
```

At each 100 Hz tick, acquire the observation and call the policy and object controller once each. Five 500 Hz substeps retain the same ObjectCommand while acquiring fresh public observations. Maximum reference age is 0.008 s. Missing/out-of-order ticks, future/stale decisions, and invalid outputs abort execution; there is no silent controller fallback.

The policy receives allowlisted history, up to 50 rows by default; q/dq require explicit `with_joint_states`. Object/arm controllers receive more complete public robot/object sensing, nominal q, pad commands, and limits. An allowlisted copy is created before every callback. Simulator handles, human goals, scenario labels, ground-truth forces, and future commands are excluded. This is a module data boundary, not a sandbox for untrusted Python code.

See the new [Expert & Data Design](EXPERT_DATA_DESIGN.md): candidate references pass only AssistanceDecision downstream, never teacher-internal goals. The Expert update ran pure mathematical tests only.

## Tests and execution

```bash
uv run --locked python -m unittest phase1b.tests.test_contracts phase1b.tests.test_integration -v
uv run --locked python -m phase1b.scripts.collect --duration .12 --output results/phase1b/dev-contract
uv run --locked python -m phase1b.scripts.run_suite --output results/phase1b/reproduction --render
```

Tests cover modes/sign/confidence, frames/quaternions, allowlists and future-data rejection, HOLD latching, three-layer execution, clocks, invalid masks, the scalar adapter, unchanged physics, and rejection of corrupted data. The full suite also checks support negative controls, FK, replay, and A/B/C motion; see [Physical Results at 37b8c24](../results/phase1b/current/verification.json).

[Data Schema](DATA_SCHEMA_V2.md) · [Controller Handoff](CONTROL_HANDOFF_ARMS.md) · [Hardware](CURRENT_HARDWARE_HANDOFF.md)
