# Swathi's controller implementation and tests

Swathi, your part is to build the controller that turns a requested panel assistance torque into motion and contact forces from the two Panda arms. The Expert and the future Student will both use your controller. Keep the existing object and arm interfaces so Jerry can change the policy without changing your implementation.

Start with **T0 through T3**. The repository now has a runner, a repeatable initial state, a fixed human target, and scripted torque inputs for these tests. You can run the full input path before writing your controller. The default adapters hold the arms and leave assistance unexecuted, so a completed run does not mean torque tracking passed.

| Owner | Work for this milestone |
|---|---|
| You | Object control, dual-arm coordination, required model calculations, contact maintenance, limits, and diagnostics that explain how each request was executed |
| Jerry | Scene fixtures, human and disturbance inputs, scripted requests, Expert decisions, and the boundary between observations and labels |
| Both | Connect your controller to the runner, calibrate limits, agree on acceptance thresholds, and review the T0 through T3 results before starting the Expert comparisons |

The interfaces, sensor logging, and frozen hardware are available. Your control algorithm and its allocation/tracking diagnostics still need implementation. T4 through T8 also have runnable human and disturbance inputs. Their Expert integration and cooperation acceptance tests are still pending. You do not need to train a policy or redesign the hardware for this delivery.

[Run the fixtures](#6-run-it-and-hand-back-the-results), then use the API and test requirements below to implement your controller.

## 1. Implement these two interfaces

[interfaces.py](../phase1b/interfaces.py) defines the types and allowlists; [control.py](../phase1b/control.py) schedules them. Implement the two classes in one control module and keep these signatures.

```python
ObjectMotionController.step(decision: AssistanceDecision,
                            observation: ArmControllerInput, dt: float) -> ObjectCommand
DualArmController.step(command: ObjectCommand,
                       observation: ArmControllerInput) -> ArmCommand
```

Object control runs at 100 Hz with dt=.01. Dual-arm control runs at 500 Hz with control_dt=.002. Each arm step gets a fresh observation and uses the current ObjectCommand for up to five substeps. The maximum reference age is .008 s. The pipeline rejects missing, stale, or future data and invalid outputs. Timestamps start at zero after settling. The benchmark performs the existing 2 s settle, latches the human target, and waits for its fixture stability checks before starting the episode clock.

### What comes from the policy

AssistanceDecision has timestamp, mode, assist_torque_y_nm, confidence, source, valid. The torque is **additional Nm about World +Y at the instantaneous panel COM**, excluding basic support. Modes are ROTATE_POS/ROTATE_NEG/HOLD/UNCERTAIN. ROTATE may brake with opposite-signed torque; HOLD may have nonzero torque; UNCERTAIN must be zero. Invalid means UNCERTAIN/0. Confidence may be None. Source identifies provenance, not a different control law.

### What you send to arm control

ObjectCommand carries timestamp/mode/valid, pose_ref_world `(7,)` = world COM xyz + unit quaternion **wxyz**, twist_ref_world `(6,)` = **angular→linear** in rad/s and m/s, and assist_wrench_world `(6,)` = **force→torque** in N/Nm. Current MVP requires `[0,0,0,0,decision.assist_torque_y_nm,0]`, frame_id=`world`, reference_point=`panel_com`. Preserve the requested scalar; report unrealizable execution separately, not by silently rewriting the request.

Optional stiffness/damping each have six nonnegative angular→linear entries (Nm/rad, Nm·s/rad; N/m, N·s/m). Empty means unspecified. The current contract rejects nonzero assistance combined with explicitly nonzero pitch stiffness. Coordinate motion and assistance through one compliant law; do not create competing high-stiffness pitch and torque loops. UNCERTAIN must use current measured pose, zero twist/wrench, and no executed-assistance flag. Support/damping remains a lower-level responsibility.

### What you send to MuJoCo

ArmCommand contains `arm_position_target (2,7)` **rad**, `pad_force_command (4,)` **N**, controller_mode, and assist_reference_executed. These are position servos: **Nm cannot be assigned to joint-position commands**. Validate command bounds. The pipeline timestamps each command; ArmCommand has no duplicate timestamp. A true execution flag means the request was used, not that tracking passed.

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

You will need to implement the model calculations your controller uses. ArmControllerInput does not supply these matrices, and the repository does not yet have a controller model helper. Do not pass the live PandaWorld/MjData into either controller or use private-model contact solutions as measured plant truth. Obtain measured feedback from observation only.

The scenario JSON mixes physical parameters with private human/disturbance schedules. Do not expose that whole object to controllers. Only explicitly selected static calibration, such as runtime friction, may be configured alongside the model and recorded by hash. Human goals, exact schedules, human-force truth, disturbance truth, scenario labels, future commands/observations, and Expert internal reference angles are forbidden controller inputs.

### Frozen physical setup

Scenario **3.0.0**, Hardware **2.0.0**: 1 kg free panel, .60×.20×.02 m; nominal COM `[0,0,.80]` m with pitch approximately +.336°. World Z up, gravity `[0,0,-9.81]` m/s². Panel local origin is COM; X long edge, Y short edge, Z thickness.

Two 7-DoF Panda bases are fixed at world `[-.45,.65,.25]` and `[.45,.65,.25]` m, yaw −90°. Nominal panel-local grasps: L `[-.20,.10,0]`, R `[.20,.10,0]`; human `[.27,-.10,0]` m on the opposite edge. Four custom sliding pads, nominal commands `[-40,+40,-40,+40]` N; commands are not guaranteed contact forces. Only the panel has a freejoint; no weld or root-wrench support. Retain model collision exclusions and account for their limitations.

Runtime first friction coefficient: `1.0482960672343025`. Arm servo torque bounds: ±87 Nm for joints 1-4, ±12 Nm for joints 5-7. Human spring-damper is approximately 98.21 N/m and 7.30 N·s/m, with 50 N force cap and .35 m/s target-speed cap. Exact nominal state/calibration is in the versioned model/config, not a new hardware design. Actuator changes require a new hardware version and physical revalidation.

## 3. What your controller needs to do

Choose a compliant object-control law and implement the coordination, IK/Jacobian calculations, and contact-force management it needs. The arms must support the panel weight, respect joint and actuator limits, and handle saturation or lost contact safely. Translation, roll/yaw, and vertical support still matter even though assistance has only a Y torque component. Do not hardcode a task schedule or Panda joint trajectory to pass a test.

Use the same code, configuration, reset rules, and input permissions for scripted tests, E0/E1/E2, and Student. With identical decision values/public state/history and reset, changing only `decision.source` must not change commands. Add this source-invariance contract test. E1/E2 teacher knowledge stays upstream; replaying a fixed human force across strategies is not a fair closed-loop comparison.

[ControlChain](../phase1b/control.py) currently bounds requests at .02 Nm and rotation reference speed at .5 rad/s. These are interface-test defaults, not certified capabilities. Calibrate and explicitly configure real limits; preserve errors for requests outside them. Gain/limit provenance, controller version, and reset behavior belong in the delivery.

## 4. Run T0 through T3 first

Use [benchmark.py](../phase1b/benchmark.py) to run these cases through the existing [pipeline](../phase1b/pipeline.py). The runner prepares the scene, injects each request at 100 Hz, calls arm control at 500 Hz, and writes synchronized transitions. Supply your two controller classes through the CLI. Each run creates fresh controller instances, so initialize your filters and reference state in the constructors.

The runner checks physical failures at every physics step. It stops on lost pad contact, excessive penetration, unexpected penetrating contact, joint-limit violations, non-finite observations, or MuJoCo warnings. It saves the failure state and any completed transitions as partial data. Those files are not valid demonstrations. Saturation is logged for review.

The fixture and input checks are ready; calibrated controller acceptance thresholds remain our joint task. The existing A/B/C demos are separate regression evidence.

All four cases use the same prepared scene:

1. Load the frozen model through Python and perform initial settling with zero assistance. Disable external disturbance.
2. Latch the post-settle human interaction point as a fixed **world target**, target velocity zero, while keeping the spring-damper active. Allow the target change to settle again under predeclared stability/contact criteria. Human force need not be zero; do not move its target with the panel.
3. Save S0 in `initial_state.npz` using MuJoCo's integration-state mask, including time, controls, activation, applied inputs, and warm-start state. Save the fixed target and nominal arm positions alongside it. Restore S0 before every run and construct fresh controllers. The runner checks that every initial observation is identical; the tests also check deterministic trajectory replay. These fixtures use no random sampling.
4. Inject a 100 Hz `AssistanceDecision` with mode=HOLD, confidence=None, source=scripted_test, valid=True and torque=u(t). Keep mode/reference policy and control law constant across all four tests. HOLD permits nonzero torque here to isolate control from intent transitions. Assistance must come through arm/pad commands, never direct panel torque injection.

`BenchmarkWorld` supplies the fixed target through a runner-owned environment hook. It keeps the human spring-damper active and sets external disturbance to zero. The public controller input has no task ID, human target, or future schedule. `PandaWorld("B")` and the ordinary collect command retain their existing behavior; use the benchmark entry point for T0 through T3.

| Case | Input from identical S0 | Required result |
|---|---|---|
| **T0** | u=0, fixed human target, no disturbance | Stable support, four-pad contact, bounded pitch/height drift, limits/collisions/warnings checked; no active assistance |
| **T1** | 0 → smooth +A → hold → smooth 0 | Credible positive **incremental** Y contact-torque response; gain/lag/overshoot/saturation measured |
| **T2** | Sign mirror of T1, same amplitude/timing | Negative response; report bias/asymmetry rather than retuning after evaluation |
| **T3** | `0 → +A → 0 → −A → 0`, smooth ramps, no reset between pulses | Continuous tracking and return of extra torque to zero, controlled slew/overshoot, contact and load stability |

The runnable development input uses A = .01 Nm, bounded by .02 Nm and .05 Nm/s. These values exercise the interface; they are not certified capabilities. Calibrate A before acceptance testing. Default timing: 1 s initial zero, .5 s ramp, 1 s plateau, .5 s return, 1 s final zero = 4 s for T1/T2; insert 1 s middle zero and a mirrored negative pulse for a 7 s T3. The runner also runs matched zero-only baselines for the same durations, including a separate `T0_for_T3` run. The CLI exposes these timings and rejects invalid durations or ramps that exceed the configured slew bound. They remain development settings until we lock acceptance values. A cubic ramp `s(v)=3v²−2v³` has peak slew `1.5*abs(Δu)/r`; lengthen r to respect calibrated limits. Stop or use a documented fallback on constraint failures, retaining failure records.

Positive Y rotation lowers the nominal +X panel side. Near horizontal, an ideal incremental left +δ / right −δ vertical force couple gives Δτ_y≈.40 m×δ with no net vertical-force increment; this explains sign, not a prescribed feasible allocator. Torque tests do not require a specified angle. **Removing extra torque is not the same as returning the panel to its initial pose.**

### Show what the controller actually delivered

Compare each active run with zero assistance from equivalent S0, human goal/impedance, and disturbance conditions. Log trajectory/state divergence: torque differences long after trajectories diverge are not same-state assistance measurements. Total contact Ty includes baseline support and is not the desired extra torque.

Deliver requested assistance, baseline wrench proposal, requested total wrench, allocated left/right wrenches, actual commands, and measured total contact response. State how the incremental estimate/internal-force estimate is obtained and when valid. If baseline separation is not credible, report **tracking_unverified**.

The pipeline currently writes total-demand/allocation/tracking fields as NaN/false; baseline_wrench_proposal is absent. Your delivery needs a real diagnostics path from the controller, with matching writer and validator updates. Work with Jerry on that schema when your algorithm defines the quantities. Keep ArmCommand's actuation boundary; do not promote a flag or fabricated zero into tracking evidence. Proposed diagnostics must be specified and tested when implemented.

Use development runs to calibrate amplitudes/rates and noise, then lock criteria for signed response, error/gain/lag, pitch/height deviation, settling, per-pad contact, penetration/collision, joint margin, saturation and warnings. Record thresholds/version/hash. Evaluate held-out seeds/onsets/conditions without relaxing limits after observing results. Missing thresholds/results mean pending, not pass. The current 20 N disturbance demo and ±.02 Nm interface signals are not certified test envelopes.

## 5. Then test cooperation with Jerry

You can run these fixtures now with zero assistance to check their inputs. After T0 through T3 pass, use `--policy MODULE:Policy` to connect Expert decisions to the same controller. That option applies only to T4 through T8; T0 through T3 always use the scripted torque tests. Human target motion creates real spring-damper force; it does not teleport the panel or prescribe arm trajectories. All strategies share human goals/impedance/disturbances while actual human force changes with feedback.

| Case | Human / environment | Check |
|---|---|---|
| **T4** | Ramp the human target to +10° relative to its initial position and hold; no disturbance | Actual observable guidance triggers assistance; no hidden-goal/fixed-time anticipation |
| **T5** | Mirror T4 with a -10° human target | Both directions, with limits/contact retained |
| **T6** | A +2° human-target correction, hold, then return to the original target | No false rotation; drift and human effort measured |
| **T7** | Fixed human target; world -Z external force ramps to 2 N at the configured off-COM point and persists | Less pitch error and RMS/peak human force than support_only while preserving height/contact; start well below 20 N |
| **T8 (optional)** | Human target moves to +5°, pauses, then resumes to +10° or reverses to 0° | Stop/resume/reverse delay, continued unwanted effort, overshoot and contact |

T9 is not defined. These are human-target angles, not guaranteed panel angles. The runner rotates the target around the saved initial panel COM without moving the panel directly. Default onset is 1 s, each ramp takes .5 s, the T6/T8 pause lasts 1 s, and the episode ends at 4 s. `--reverse-after-pause` selects T8 reversal. The external force in T7 acts at panel-local `[0.06369127617716655, -0.015330968965703077, 0]` m and remains active through the end.

Use `--human-angle-deg`, `--correction-angle-deg`, `--disturbance-force-n`, and the shared timing options to vary the inputs. Requests that exceed the frozen human target-speed limit stop the run. All current fixtures start from the same nominal S0. Testing T6 at other starting poses still needs a reachable, validated reset state. ±10° are development targets subject to controller reachability checks. A paused human target still resists panel deviation. Off-COM vertical disturbance also creates vertical load/other torques; Y assistance alone cannot satisfy all constraints.

**Control GO:** T0-T3 pass locked physical/tracking criteria. **Expert GO:** T4-T7 show closed-loop cooperation benefits versus support_only (and E0 where useful). **Learning-Data GO:** execution/teacher quality, causal synchronized labels and observation-label consistency pass separately. Human `∫||F_h||²dt` measures force burden, not metabolic energy. Retain failures; no demonstrations or training claims from scripted trajectories/shadow annotations.

## 6. Run it and hand back the results

[Fixture verification](../results/phase1b/controller_fixtures/verification.json) records 31 passing tests and a full T0 through T8 input run plus the extra T3 zero baseline: 10 episodes and 23,000 physics steps. Contact and basic physical checks passed with the default adapters. The existing A/B/C support, replay, and scripted motion regressions also passed. This verifies the test setup and data path; it does not certify assistance tracking or Expert cooperation. T8 reversal has an input unit test; the full recorded fixture run uses resume.

Record `observation_t → decision_t → object_command_t → five arm/physics steps → observation_t+.01`. At each substep save command time, decision/object source time and post-step realized time. Current files are observations, next_observations, decisions, object_commands, controller, diagnostics, privileged, supervision, render_states, and metadata. Unavailable quantities use NaN + false masks. Physical warnings/contact/collision audits remain runner-owned, not fabricated controller reports.

Each result needs task_id, seed/reset state, human/force onset, versions and source/model/config/threshold hashes; synchronized plots of request/response, object pose/velocity, human effort and physical constraints; verdict and failure reason; exact reproduction command. Keep goals, forces, scenario, teacher references and future data outside controller/Student observations. Formal labels remain invalid until separately validated. Existing E0/E1/E2 tools are in [expert/](../phase1b/expert); they are mathematical candidates, not validated execution.

```bash
uv sync --locked

# Ready now: default adapters exercise the inputs and logging, not torque tracking.
uv run --locked python -m phase1b.benchmark \
  --output results/phase1b/dev-controller-inputs

# Replace MODULE and class names with your implementation.
uv run --locked python -m phase1b.benchmark \
  --object-controller MODULE:ObjectController \
  --dual-arm-controller MODULE:DualArmController \
  --output results/phase1b/dev-swathi-controller

# Human/disturbance fixtures, initially with zero assistance.
uv run --locked python -m phase1b.benchmark --cases T4 T5 T6 T7 T8 \
  --output results/phase1b/dev-human-inputs

# Later, add --policy MODULE:Policy and the same two controller arguments.
# Fixture, interface, physical regression, and Expert arithmetic checks.
uv run --locked python -m unittest phase1b.tests.test_benchmark \
  phase1b.tests.test_contracts phase1b.tests.test_integration \
  phase1b.tests.test_expert -v
```

Use a fresh output directory each time. The CLI expects no-argument controller constructors. `--cases T1 T2`, `--amplitude-nm`, `--onset-seconds`, `--ramp-seconds`, `--hold-seconds`, `--zero-seconds`, `--max-assist-torque-nm`, and `--max-slew-nm-s` let you select cases and configure the development inputs. The runner automatically adds zero baselines for T1 through T3. With a custom policy, it also adds a zero-assistance run for each T4 through T8 case, using the same controller and human/disturbance inputs. Actual human force may differ as trajectories diverge.

`report.json` records the input specification, initial-state hash, reset checks, controller source hashes, metrics, and errors. Each case directory contains the existing transition files and source/model/config provenance. `metadata.json.experiment` records its task/run IDs, fixture version, input settings, checkpoint hash, fixed target, and injected component source hashes. The private `scenario_label` records T0 through T8; metadata `scenario="B"` identifies the underlying simulator setup. This metadata describes the experiment and is not a policy input. The older Expert shadow-data annotator still assumes the A/B/C schedules; it needs a fixture-aware metadata update before use with these benchmark episodes. The comparison NPZ files contain total contact-Ty differences and position/quaternion divergence from the matched zero baseline. Their `tracking_valid` masks stay false: a difference between two diverging trajectories is not a validated incremental-assistance measurement.

For Python use, `BenchmarkWorld.prepare()` returns S0 and `restore(S0)` resets it. Pass the prepared world to `phase1b.pipeline.run_episode(..., world=world, step_guard=physical_guard, policy=TorqueInput(case, spec), object_controller=..., dual_arm_controller=...)`. The world stays in the runner; controller callbacks still receive only `ArmControllerInput`. Historical results at `37b8c24` require their producing source hashes. Keep new outputs separate.

Before handing your work back, include:

- [ ] Both controller classes, their configuration, model calculations, and documented reset behavior.
- [ ] A test showing that changing only `decision.source` leaves commands unchanged for identical values, history, and initial state.
- [ ] Real allocation/execution diagnostics with validity masks, plus writer and validator tests. Explain the baseline separation used for incremental torque estimates.
- [ ] T0 through T3 results with calibrated, frozen thresholds. Report failures, saturation, tracking error/gain/lag where valid, and physical constraints.
- [ ] Exact commands and plots for reproducing those results. State which checks passed and which remain unverified.

Interfaces and fixture execution can pass while assistance tracking remains unverified. We will validate Expert quality and learning labels separately after your controller is ready.
