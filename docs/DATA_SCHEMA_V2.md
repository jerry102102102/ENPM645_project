# Data Schema — enpm645-three-layer-transition/1.0

The only formal writer/reader/validator is `phase1b.pipeline.run_episode / read_episode / validate_episode`. Hardware 2.0.0 / Scenario 3.0.0.

## Timing

Each transition: `o_t → decision_t → object_command_t → 5 arm commands/steps at 500 Hz → o_next`. Observations/references run at 100 Hz; physics/control at 500 Hz. The episode clock resets after 2 seconds of settling; engine time is recorded separately.

| File | Contents |
|---|---|
| observations.npz | Sensing/state at t before the decision; the previous command is still active |
| decisions.npz | Original AssistanceDecision request: mode, torque, source, valid; unavailable confidence uses NaN + confidence_present=false |
| object_commands.npz | World pose/twist/assistance wrench at t, mode, valid, frame/point; stiffness/damping each have a validity mask and use NaN when unavailable |
| controller.npz | Actual commands, source timestamps, torque, saturation, and post-step contact wrench for five substeps |
| next_observations.npz | t+0.01; exactly equals the next observations row except at the episode end; unavailable to the current policy |
| privileged.npz | Human targets, application points / ground-truth forces, and scenario; excluded from policy and lower-level controllers |
| supervision.npz | Formal Expert labels; currently torque=NaN and expert_label_valid=false, separate from candidate decisions |
| diagnostics.npz | 500 Hz contacts, penetration, joint limits, force balance, applied-force audit, and warnings |
| render_states.npz | qpos/qvel, for replay / FK / videos only |
| metadata.json | Schema/model/hardware/source hashes, component classes, allowlist profile, limits, and array shapes/dtypes |
| verification.json | Contract and physics checks for this episode; not proof of validated control capability |

## Observations

Shapes exclude N. Arm order is left/right, with joint1…7 per arm.

| Field | Shape / units |
|---|---|
| wrist_ft_left/right | Each `(6,)`; contact-reconstructed panel-on-gripper wrench; gripper-local, about the gripper origin; force xyz→torque xyz |
| wrist_sensor_left/right | Each `(6,)`; child→parent transmitted wrench; sensor at gripper-local `[0,.12,0]`; includes downstream weight and dynamics, uncompensated |
| tactile_left/right | Each `(2,3,4,4)`; upper/lower × sensor z/x/y × angular bins; N/bin |
| object_position/orientation | `(3,)` world xyz in m / `(4,)` quaternion wxyz |
| object_linear/angular_velocity | Each `(3,)`, world m/s and rad/s |
| contact_state_left/right | Each `(2,)`, upper/lower contact counts |
| arm_joint_position/velocity | Each `(2,7)`, rad and rad/s |
| arm_actuator_torque/force | Each `(2,7)`; current unit-gear hinges make both numerically Nm; not an external-force estimate |
| end_effector_pose/velocity | `(2,7)` xyz+wxyz / `(2,6)` angular→linear; gripper frame between the two pads |
| gripper_joint_position/velocity | Each `(2,2)`, m and m/s |
| gripper_actuator_force | `(4,)`, N |
| object_contact_wrench | `(6,)`; gripper-on-panel, world, about the instantaneous panel COM |

Combined gripper/object pose/velocity fields and separate left/right contact wrenches in world coordinates about the panel COM are also retained for diagnostics.

## Commands and diagnostics

- Decision/ObjectCommand each have N rows; their source times equal observation_t.
- `command_timestamp / decision_timestamp / object_command_timestamp`: each `(N,5)`; the first is t+[0,.002,.004,.006,.008], while the latter two remain t.
- `arm_position_target (N,5,2,7)` in rad; `pad_force_command (N,5,4)` in N.
- `arm_requested_servo_torque`, `arm_actuator_torque_at_actuation`, and `arm_actuator_torque_after_step`: respectively before clipping, after command application but before stepping, and after stepping; not interchangeable.
- `realized_timestamp`=command_timestamp+.002; `realized_object_contact_wrench (N,5,6)` is the total contact wrench, not isolated assistance.
- `requested_object_wrench (N,5,6)` denotes the total demand including support. It is not yet computed: NaN + requested_object_wrench_valid=false. The known additional assistance request is in object_commands.assist_wrench_world.
- `allocated_gripper_wrenches (N,5,2,6)` has no allocator yet: NaN + allocation_valid=false.
- `tracking_error_Nm (N,)` is NaN with tracking_error_valid=false; total Ty must not substitute for assistance tracking.
- `controller_mode / assist_reference_executed` are logged per substep; contact_status corresponds to diagnostics.contact_counts, and saturation to controller.arm_force_saturated.

## Learning boundary

By default, the policy receives the two contact F/T streams, tactile, object pose/velocity, and contact state listed above (10 fields). `with_joint_states` additionally enables q/dq. Raw wrist sensors, torques, commands, truth, and metadata are not automatically included.

Setting source to oracle_expert does not make supervision valid. Candidate decisions remain separate from formal labels. No valid Expert labels are produced before allocation/tracking validation.

Scripted motion demonstrations use separate render/diagnostic records containing privileged truth. **These are not the formal training-data format.** Old schemas and experiments remain only in historical Git revisions; the current reader does not maintain multiple-version branches.

## Expert shadow annotation

The [Expert Data Contract](EXPERT_DATA_DESIGN.md) defines the `enpm645-expert-shadow/1.0` sidecar: derived sensing, separate E0/E1/E2 candidates, detailed metadata, and teacher-internal state. It adds no second physical pipeline and does not overwrite this schema's original files. Even numerically valid shadow candidates keep formal torque NaN and execution_valid/supervision_eligible=false.
