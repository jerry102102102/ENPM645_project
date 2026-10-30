# Controller Handoff — Implement Once, Test in Scenes

**Assignment:** replace the reference-only and position-hold adapters with one working object/dual-arm controller stack. Test references, Expert, and future Student all use it. This document specifies the next delivery; controller algorithms and benchmark results are not already complete.

## 1. Existing APIs: keep these boundaries

[interfaces.py](../phase1b/interfaces.py) defines the types and allowlists; [control.py](../phase1b/control.py) schedules them. Implement two classes in one control module; no new policy-to-actuator interface is needed.

```python
ObjectMotionController.step(decision: AssistanceDecision,
                            observation: ArmControllerInput, dt: float) -> ObjectCommand
DualArmController.step(command: ObjectCommand,
                       observation: ArmControllerInput) -> ArmCommand
```

Object control runs at **100 Hz** with dt=.01. Dual-arm control runs at **500 Hz** with control_dt=.002, using a fresh observation and the latest ObjectCommand for five substeps. Maximum reference age is .008 s. Missing/stale/future data or invalid outputs abort; there is no silent fallback. Timestamps are episode seconds, after 2 s initial settling.

**Upstream input:** AssistanceDecision has timestamp, mode, assist_torque_y_nm, confidence, source, valid. The torque is **additional Nm about World +Y at the instantaneous panel COM**, excluding basic support. Modes are ROTATE_POS/ROTATE_NEG/HOLD/UNCERTAIN. ROTATE may brake with opposite-signed torque; HOLD may have nonzero torque; UNCERTAIN must be zero. Invalid means UNCERTAIN/0. Confidence may be None. Source identifies provenance, not a different control law.

**Internal output:** ObjectCommand carries timestamp/mode/valid, pose_ref_world `(7,)` = world COM xyz + unit quaternion **wxyz**, twist_ref_world `(6,)` = **angular→linear** in rad/s and m/s, and assist_wrench_world `(6,)` = **force→torque** in N/Nm. Current MVP requires `[0,0,0,0,decision.assist_torque_y_nm,0]`, frame_id=`world`, reference_point=`panel_com`. Preserve the requested scalar; report unrealizable execution separately, not by silently rewriting the request.

Optional stiffness/damping each have six nonnegative angular→linear entries (Nm/rad, Nm·s/rad; N/m, N·s/m). Empty means unspecified. The current contract rejects nonzero assistance combined with explicitly nonzero pitch stiffness. Coordinate motion and assistance through one compliant law; do not create competing high-stiffness pitch and torque loops. UNCERTAIN must use current measured pose, zero twist/wrench, and no executed-assistance flag. Support/damping remains a lower-level responsibility.

**Actuator output:** ArmCommand contains `arm_position_target (2,7)` **rad**, `pad_force_command (4,)` **N**, controller_mode, and assist_reference_executed. These are position servos: **Nm cannot be assigned to joint-position commands**. Validate command bounds. The pipeline timestamps each command; ArmCommand has no duplicate timestamp. A true execution flag means the request was used, not that tracking passed.

## 2. What the controller receives

Both callbacks receive `ArmControllerInput`. Its observation is a public, read-only snapshot. This is richer than Student inputs; choosing the Student profile does not remove joint states from the controller.

| Input | Shape / meaning |
|---|---|
| timestamp, control_dt | Current episode time and .002 s physics step; ObjectController also receives dt=.01 |
| nominal_arm_position, nominal_pad_force | `(2,7)` rad, `(4,)` N |
| arm_position_limits, pad_force_limits | `(2,7,2)` rad, `(4,2)` N; lower/upper command bounds |
| observation.arm_joint_position / velocity | Each `(2,7)`, left/right × joints 1…7, rad / rad/s |
| observation.end_effector_pose / velocity | `(2,7)` world xyz+wxyz / `(2,6)` world angular→linear; EE between pads |
| observation.object_position / orientation | `(3,)` world m / `(4,)` quaternion wxyz |
| observation.object_linear_velocity / angular_velocity | Each `(3,)`, world m/s / rad/s |
| observation.wrist_ft_left / right | Each `(6,)`, contact-reconstructed panel-on-gripper; gripper-local about gripper origin; force→torque |
| observation.wrist_sensor_left / right | Each `(6,)`, raw child→parent transmitted wrench at gripper-local `[0,.12,0]`; includes downstream weight/dynamics |
| observation.tactile_left / right | Each `(2,3,4,4)`: upper/lower × sensor z/x/y × angular bins; N/bin, not Pa/images |
| observation.contact_state_left / right | Each `(2,)`, upper/lower contact counts |
| observation.gripper_joint_position / velocity | Each `(2,2)`, m / m/s |
| observation.arm_actuator_torque / force | Each `(2,7)`; current unit-gear hinge values in Nm; not external-force estimates |
| observation.gripper_actuator_force | `(4,)`, N |
| observation.object_contact_wrench | `(6,)`, total gripper-on-panel wrench, world, about instantaneous COM |
| observation.wrist_ft_left_world_at_panel_com / right | Each `(6,)`, gripper-on-panel contact wrench expressed in world about panel COM; these two contributions sum to object_contact_wrench |

Canonical names/shapes are `PUBLIC_SHAPES`; the table abbreviates shared prefixes. Raw and contact-reconstructed F/T are not interchangeable. To transform local panel-on-gripper data into robot-on-panel torque, negate/rotate it and shift the moment to COM. Contact counters are not full collision geometry; the runner separately audits collisions, penetration, and numerical warnings.

### Static model access and calculations

Use the frozen [MJCF](../phase1b/models/dual_panda_v1.xml), its vendor assets, and [hardware contract](../phase1b/config/hardware_v2.json) for robot geometry, inertias, joints, actuator parameters, and limits. These are known model data, available equally to Expert and Student execution. If needed, the controller may load its **own isolated model/data** and calculate FK, Jacobians, mass matrix, gravity/bias terms, and grasp geometry from the public robot/object/gripper state. It must map joint indices and free-body velocity frames correctly and verify its FK against the provided EE poses.

**Those matrices are not currently supplied in ArmControllerInput, and no ready-made controller model helper is provided.** Implement the required computations as part of the controller, rather than assuming observation contains them. Do not pass the live PandaWorld/MjData into either controller or use private-model contact solutions as measured plant truth. Obtain measured feedback from observation only.

The scenario JSON mixes physical parameters with private human/disturbance schedules. Do not expose that whole object to controllers. Only explicitly selected static calibration, such as runtime friction, may be configured alongside the model and recorded by hash. Human goals, exact schedules, human-force truth, disturbance truth, scenario labels, future commands/observations, and Expert internal reference angles are forbidden controller inputs.

### Frozen physical setup

Scenario **3.0.0**, Hardware **2.0.0**: 1 kg free panel, .60×.20×.02 m; nominal COM `[0,0,.80]` m with pitch approximately +.336°. World Z up, gravity `[0,0,-9.81]` m/s². Panel local origin is COM; X long edge, Y short edge, Z thickness.

Two 7-DoF Panda bases are fixed at world `[-.45,.65,.25]` and `[.45,.65,.25]` m, yaw −90°. Nominal panel-local grasps: L `[-.20,.10,0]`, R `[.20,.10,0]`; human `[.27,-.10,0]` m on the opposite edge. Four custom sliding pads, nominal commands `[-40,+40,-40,+40]` N; commands are not guaranteed contact forces. Only the panel has a freejoint; no weld or root-wrench support. Retain model collision exclusions and account for their limitations.

Runtime first friction coefficient: `1.0482960672343025`. Arm servo torque bounds: ±87 Nm for joints 1–4, ±12 Nm for joints 5–7. Human spring-damper is approximately 98.21 N/m and 7.30 N·s/m, with 50 N force cap and .35 m/s target-speed cap. Exact nominal state/calibration is in the versioned model/config, not a new hardware design. Actuator changes require a new hardware version and physical revalidation.

## 3. Controller responsibility and reuse test

Implement compliant object motion, dual-arm coordination/IK/Jacobians, baseline weight support, contact/internal-force management, joint/actuator constraints, and safe handling of saturation/contact loss. Translation, roll/yaw, and vertical support still matter even though assistance has only a Y torque component. Do not hardcode a task schedule or Panda joint trajectory to pass a test.

Use the same code, configuration, reset rules, and input permissions for scripted tests, E0/E1/E2, and Student. With identical decision values/public state/history and reset, changing only `decision.source` must not change commands. Add this source-invariance contract test. E1/E2 teacher knowledge stays upstream; replaying a fixed human force across strategies is not a fair closed-loop comparison.

[ControlChain](../phase1b/control.py) currently bounds requests at .02 Nm and rotation reference speed at .5 rad/s. These are interface-test defaults, not certified capabilities. Calibrate and explicitly configure real limits; preserve errors for requests outside them. Gain/limit provenance, controller version, and reset behavior belong in the delivery.

## 4. Scene acceptance: T0–T3 first

**Fixtures and the benchmark runner are planned, not implemented.** Jerry supplies repeatable setup, human/disturbance schedules and reference injection; the teammate integrates its controller and demonstrates execution. Both freeze thresholds before held-out evaluation. Do not call the existing A/B/C demo suite a benchmark pass.

For T0–T3, reuse one scene:

1. Load the frozen model through Python and perform initial settling with zero assistance. Disable external disturbance.
2. Latch the post-settle human interaction point as a fixed **world target**, target velocity zero, while keeping the spring-damper active. Allow the target change to settle again under predeclared stability/contact criteria. Human force need not be zero; do not move its target with the panel.
3. Save/reproduce the full initial simulator and controller state S0, including q/dq, controls/actuator/solver state as applicable, fixed target, filter/reference state, seed and clocks. Validate reset equivalence; nominal q/.80 m alone is not a settled snapshot.
4. Inject a 100 Hz `AssistanceDecision` with mode=HOLD, confidence=None, source=scripted_test, valid=True and torque=u(t). Keep mode/reference policy and control law constant across all four tests. HOLD permits nonzero torque here to isolate control from intent transitions. Assistance must come through arm/pad commands, never direct panel torque injection.

`PandaWorld("B")` currently keeps a config-derived target instead of latching post-settle position; default ScriptedContractPolicy cycles modes/torques. The new runner needs explicit fixture/reset/reference hooks. `collect --scenario B` alone is not T0.

| Case | Input from identical S0 | Required result |
|---|---|---|
| **T0** | u=0, fixed human target, no disturbance | Stable support, four-pad contact, bounded pitch/height drift, limits/collisions/warnings checked; no active assistance |
| **T1** | 0 → smooth +A → hold → smooth 0 | Credible positive **incremental** Y contact-torque response; gain/lag/overshoot/saturation measured |
| **T2** | Sign mirror of T1, same amplitude/timing | Negative response; report bias/asymmetry rather than retuning after evaluation |
| **T3** | `0 → +A → 0 → −A → 0`, smooth ramps, no reset between pulses | Continuous tracking and return of extra torque to zero, controlled slew/overshoot, contact and load stability |

A is **TBD after calibration**. Development timing example: 1 s initial zero, .5 s ramp, 1 s plateau, .5 s return, 1 s final zero = 4 s for T1/T2; insert 1 s middle zero and a mirrored negative pulse for a 7 s T3. Run matched zero-only baselines for the same durations. These are examples, not locked acceptance values. A cubic ramp `s(v)=3v²−2v³` has peak slew `1.5*abs(Δu)/r`; lengthen r to respect calibrated limits. Stop or use a documented fallback on constraint failures, retaining failure records.

Positive Y rotation lowers the nominal +X panel side. Near horizontal, an ideal incremental left +δ / right −δ vertical force couple gives Δτ_y≈.40 m×δ with no net vertical-force increment; this explains sign, not a prescribed feasible allocator. Torque tests do not require a specified angle. **Removing extra torque is not the same as returning the panel to its initial pose.**

### Evidence and thresholds

Compare each active run with zero assistance from equivalent S0, human goal/impedance, and disturbance conditions. Log trajectory/state divergence: torque differences long after trajectories diverge are not same-state assistance measurements. Total contact Ty includes baseline support and is not the desired extra torque.

Deliver requested assistance, baseline wrench proposal, requested total wrench, allocated left/right wrenches, actual commands, and measured total contact response. State how the incremental estimate/internal-force estimate is obtained and when valid. If baseline separation is not credible, report **tracking_unverified**.

The existing [pipeline](../phase1b/pipeline.py) hardcodes total-demand/allocation/tracking fields to NaN/false; baseline_wrench_proposal is absent. A genuine controller-diagnostics path and updated writer/validators are required for this delivery. Keep ArmCommand's actuation boundary; do not promote a flag or fabricated zero into tracking evidence. Proposed diagnostics must be specified and tested when implemented.

Use development runs to calibrate amplitudes/rates and noise, then lock criteria for signed response, error/gain/lag, pitch/height deviation, settling, per-pad contact, penetration/collision, joint margin, saturation and warnings. Record thresholds/version/hash. Evaluate held-out seeds/onsets/conditions without relaxing limits after observing results. Missing thresholds/results mean pending, not pass. The current 20 N disturbance demo and ±.02 Nm interface signals are not certified test envelopes.

## 5. Integration scenes after control passes

The same controller now receives Expert decisions. Human target motion creates real spring-damper force; it does not teleport the panel or prescribe arm trajectories. All strategies share human goals/impedance/disturbances while actual human force changes with feedback.

| Case | Human / environment | Check |
|---|---|---|
| **T4** | HOLD → candidate +10° relative goal; varied onset, no disturbance | Actual observable guidance triggers assistance; no hidden-goal/fixed-time anticipation |
| **T5** | Mirrored −10° goal, varied onset | Both directions, with limits/contact retained |
| **T6** | Hold current angle at several starting poses, small corrections | No false rotation; drift and human effort measured |
| **T7** | Human holds; independent off-COM force ramps in and persists | Less pitch error and RMS/peak human force than support_only while preserving height/contact; start well below 20 N |
| **T8 — optional** | Rotate, pause at intermediate target, resume or reverse; varied timings | Stop/resume/reverse delay, continued unwanted effort, overshoot and contact |

T9 is not defined. ±10° are candidate targets subject to reachability checks. A paused human target still resists panel deviation. Off-COM vertical disturbance also creates vertical load/other torques; Y assistance alone cannot satisfy all constraints.

**Control GO:** T0–T3 pass locked physical/tracking criteria. **Expert GO:** T4–T7 show closed-loop cooperation benefits versus support_only (and E0 where useful). **Learning-Data GO:** execution/teacher quality, causal synchronized labels and observation–label consistency pass separately. Human `∫||F_h||²dt` measures force burden, not metabolic energy. Retain failures; no demonstrations or training claims from scripted trajectories/shadow annotations.

## 6. Logging, delivery and entry points

Record `observation_t → decision_t → object_command_t → five arm/physics steps → observation_t+.01`. At each substep save command time, decision/object source time and post-step realized time. Current files are observations, next_observations, decisions, object_commands, controller, diagnostics, privileged, supervision, render_states, and metadata. Unavailable quantities use NaN + false masks. Physical warnings/contact/collision audits remain runner-owned, not fabricated controller reports.

Each result needs task_id, seed/reset state, human/force onset, versions and source/model/config/threshold hashes; synchronized plots of request/response, object pose/velocity, human effort and physical constraints; verdict and failure reason; exact reproduction command. Keep goals, forces, scenario, teacher references and future data outside controller/Student observations. Formal labels remain invalid until separately validated. Existing E0/E1/E2 tools are in [expert/](../phase1b/expert); they are mathematical candidates, not validated execution.

```bash
# Existing injection path; classes need no-argument constructors for CLI use.
# MODULE:ObjectController and MODULE:DualArmController are placeholders for the teammate's implementation.
uv run --locked python -m phase1b.scripts.collect \
  --policy TEST_MODULE:Policy --object-controller MODULE:ObjectController \
  --dual-arm-controller MODULE:DualArmController --output results/phase1b/dev-controller
# Existing contract/physics checks; not T0–T8 acceptance:
uv run --locked python -m unittest phase1b.tests.test_contracts phase1b.tests.test_integration -v
uv run --locked python -m phase1b.scripts.launch_viewer --scenario B
```

For constructor parameters/limits, use `phase1b.pipeline.run_episode(output, scenario="B", duration=4, policy=..., object_controller=..., dual_arm_controller=..., max_assist_torque_nm=..., max_rotation_rate_rad_s=...)`. These interfaces already exist; the benchmark runner must still integrate the fixture hooks and diagnostics. Existing results at `37b8c24` require matching source hashes for strict historical validation; generate new outputs for new code, never rewrite provenance.

**Handoff checklist:** controller code/config and model calculations; source-invariance test; T0–T3 runner plus fixtures/locked thresholds; real diagnostic logging; results with failures and reproduction instructions. No new policy interface, separate Student controller, learned model, hardware redesign, or fake Expert labels is requested.
