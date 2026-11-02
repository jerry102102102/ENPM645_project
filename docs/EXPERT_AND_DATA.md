# Expert and data contract

This is the project's design document for Expert decisions, Student inputs, data collection, and evaluation. It records what we have agreed, what we are assuming for the pilot, and what still needs a decision. Controller implementation details belong in [the controller handoff](CONTROLLER.md).

We want a Student to infer useful assistance from physical interaction history. Its primary action is an additional torque about World +Y at the instantaneous panel center of mass (COM), in Nm. It does not predict the Panda joint commands. Expert and Student use the same downstream controller.

![Shared policy and controller architecture](architecture.svg)

## 1. Decisions and assumptions

| Topic | Decision | Current implementation or limitation |
|---|---|---|
| Control ownership | Expert/Policy decides additional assistance. The controller uses public state, basic support, and a consistent compliant response to realize it. | Interfaces exist; full control and assistance tracking remain unvalidated. No downstream loop may pursue a hidden human goal or duplicate the Expert's pitch feedback. |
| Human behavior | For the first pilot, assume scripted Virtual Human behavior approximates the interaction distribution we want to study. | This is a working assumption, to compare with manual trials. It does not certify Expert labels. |
| HOLD | A person may request a stop during motion at any reachable pose, before the original goal is reached. | Adapters latch pose on entry to HOLD. Observable stop recognition and physical stopping control are still missing. |
| Panel information | Assume panel position, orientation, and velocity are available to the policy and controller. | Read from MuJoCo for now. Camera estimation, noise, and latency are later sensing work. Human goals and future states remain private. |
| History | Use at least two elapsed seconds for the primary Student design. | Default online buffer is 201 samples at 100 Hz. Two seconds is an initial choice, not proof of sufficient memory. |
| Student output | Torque regression is the primary learning target. A neural mode classifier is not required by the research question. | The existing execution interface still requires mode. Torque-only deployment needs an explicit public-state supervisor or an interface revision; neither has been selected. |
| Sensors and model assumptions | Standardize F/T sources/frames, separate joint and tactile ablations, and define the valid domain of the single-axis model. | Some profiles exist; the unified profiles and model-domain acceptance thresholds are unfinished. |
| Evaluation | Combine repeatable scripted tests with manual interaction trials. | Fixtures exist. Manual input/annotation tooling, calibrated tracking, and human-interaction evaluation remain to be built. |
| RL | Consider later refinement after controller, initial policy, and evaluation are established. | No RL algorithm, reward, or training run has been selected. |

The Virtual Human produces human behavior and physical forces. The Expert produces robot-assistance decisions. Treating the former as representative does not imply that the latter is correct, executable, or learnable.

## 2. What the Expert computes

Near single-axis pitch motion, with the other degrees of freedom controlled:

$$
J_y\ddot\theta=\tau_h+\tau_d+\tau_b+\tau_a.
$$

Here $\tau_h$ is human torque, $\tau_d$ disturbance torque, $\tau_b$ the base controller's contribution, and $\tau_a$ additional assistance. All torques refer to the same world axis and instantaneous COM. Desired assistance excludes basic support.

The ideal inverse-dynamics relationship is:

$$
\tau_a^*=J_y\ddot\theta_{des}-\tau_h-\tau_d-\tau_b.
$$

It explains why a candidate assistance torque can be calculated when desired motion and the other contributions are known. It does not prove that position servos and frictional contacts can realize it. The full rigid-body relationship also contains inertia-frame and gyroscopic coupling; a finite single-axis result is not enough to establish physical validity.

E1 is the primary candidate teacher, E2 an oracle reference, and E0 the reactive comparison. These are decision strategies above one shared controller.

| Strategy | Information permission | Candidate rule and purpose |
|---|---|---|
| E0: force following | Public object state and reconstructed robot contact F/T, including the public transforms needed to express it at COM | Estimate combined external interaction, filter it, and provide proportional assistance. It cannot separate human intent from disturbance merely by knowing the net torque. |
| E1: goal aware | Public state plus the current teacher-only human goal/mode | Wait for observable rotation guidance, update an internal bounded reference, and provide goal-related assistance. |
| E2: disturbance aware | E1 information plus current disturbance torque truth | Add partial disturbance compensation to E1. It is an oracle comparison, not a guaranteed optimal policy or automatically suitable supervision. |

E0 uses a causal acceleration estimate:

$$
\hat\tau_{int}=J_y\hat{\ddot\theta}-\tau_{r,y}^{measured},\qquad
u_0=k_f\operatorname{LPF}(\hat\tau_{int}).
$$

E1's rotation request, after the observable gate opens, is:

$$
u_1=J_y\ddot\theta_{ref}+K_{p,R}(\theta_{ref}-\theta)
+K_{d,R}(\dot\theta_{ref}-\omega_y).
$$

Its reference advances toward the teacher goal at a bounded rate. In HOLD:

$$
u_{1,H}=K_{p,H}(\theta_{hold}-\theta)-K_{d,H}\omega_y.
$$

This is additional impedance assistance, not the complete inverse-dynamics cancellation above. Base pitch support and damping must be coordinated with these gains.

E2 adds feedforward before output limiting:

$$
u_2=u_1-\lambda_d\tau_{d,y},\qquad 0\leq\lambda_d\leq1.
$$

Each strategy applies torque amplitude and slew limits. Limits must come from controller validation; current test constants are not certified operating limits. In the current generator, UNCERTAIN forces zero assistance, even when this overrides the slew limit; that exception is logged and still needs physical evaluation. E2 does not add disturbance feedforward while UNCERTAIN.

The [reference generators](../phase1b/expert/reference.py) implement these candidate rules. They do not implement a complete online policy adapter or validate cooperation. The rotation gate reduces premature action but does not make all E1 labels observable: hidden mode changes, final goals, and long-lived teacher state can still matter.

## 3. Human intent, teacher state, and robot mode

These are three different records:

| Record | Meaning | Access |
|---|---|---|
| Human intent and goal | What the person wants to accomplish, including an intermediate stop or reversal | Teacher/evaluation only |
| Human target trajectory | The hand target position and velocity used by the spring-damper to generate force | Simulation/teacher only |
| Assistance decision | What assistance is justified now, given the strategy and evidence | Passed to the shared controller |

A moving hand target is not the same as a final goal angle. A task ID is not a moment-by-moment intent label. Declining human force alone does not prove a stop request: the person may simply need less force because assistance is working.

The existing teacher generator requires timestamp-aligned `TeacherGoal(timestamp, mode, angle)` and, for E2, `DisturbanceTruth(timestamp, torque_y)`. Offline annotation requires explicit `ground_truth_intent`, `human_goal_angle`, and `external_disturbance_wrench` arrays. The human-input pilot now exports these fields from explicit operator/script events, separately from the moving hand target. The T0-T8 fixture bridge is still pending. Pilot annotations describe a declared command; they do not measure unexpressed human intent or establish label eligibility.

The execution modes currently mean:

| Mode | Robot behavior | Torque semantics |
|---|---|---|
| ROTATE_POS / ROTATE_NEG | Allow bounded assistance in a confirmed rotation interaction | Torque may oppose the intended direction to brake. |
| HOLD | Maintain an accepted stopping reference | Additional torque may be nonzero, including during sustained disturbance. |
| UNCERTAIN | Cancel high-level active assistance; retain bounded base support/stability | Additional torque is zero. |

`ground_truth_intent=ROTATE_POS` and `expert_mode=UNCERTAIN` can both be correct when guidance has not become identifiable. C/HOLD-with-disturbance is not another intent class. T0-T3 intentionally use HOLD with scripted nonzero torque to isolate control testing; those decisions are not human-intent demonstrations.

### Stopping and latency

The agreed requirement is that stopping is available throughout the motion, within physical constraints. The current implementation choice is to latch measured pose on accepted HOLD entry, keep that reference, and permit bounded braking while settling. This is not a claim that the accepted pose exactly matches where the person first wanted to stop. How to infer that intended stopping pose from physical cues remains open. Do not continuously relatch a drifting pose or resume toward an old hidden goal after a stop.

Evaluation needs the intended stop time/pose, observable detection time, command application time, and settling time/pose. Report both detection delay and physical stopping displacement. Manual annotation timestamps are observations of an operator's report, not exact measurements of an unexpressed mental event; record annotation uncertainty.

Acquisition time, observation availability, decision time, and actuator time must also be distinguished when sensor/processing delays are introduced. The current 100/500 Hz scheduling contract does not establish camera or human-interaction latency. The human-input pilot records keyboard receipt and application times, but sensor availability, controller detection delay, and the criteria for accepted start/stop/reversal cues are still pending.

### Does the Student need mode?

The learning target may be simply $O_t\mapsto\tau_a$. Mode can remain useful for Expert design, annotations, and evaluation without becoming a neural classification output.

However, the current `AssistanceDecision` contract still uses mode for HOLD latching and UNCERTAIN behavior. Before torque-only deployment, choose either a causal supervisor using public history or a deliberate execution-interface change. Do not infer mode from torque sign. Supplying hidden teacher mode is permissible only as an explicitly named oracle-mode ablation, not as an end-to-end Student result. This choice is unresolved; the existing interface has not been removed.

## 4. Student observations and history

Record full observations, then select inputs through an allowlist. Panel state is public under the pilot's known-state assumption. Hidden human goals, exact disturbance truth, scenario/task labels, teacher references, future commands, and future observations are excluded.

| Signal | Shape per sample | Units / convention |
|---|---|---|
| Object position / orientation | `(3,)` / `(4,)` | World m / quaternion wxyz |
| Object linear / angular velocity | `(3,)` each | World m/s / rad/s |
| Left and right F/T | `(6,)` each | Force then torque, N/Nm; declared source, frame, and moment center |
| Left and right tactile | `(2,3,4,4)` each | Upper/lower pads, three channels, 4x4 bins; N/bin, not pressure Pa |
| Joint q / dq | `(2,7)` each | Left/right Panda, rad / rad/s; optional Student features |
| Contact state | `(2,)` per gripper | Current contact counts; not contact force |

Joint states remain available to the controller regardless of the Student profile. Static panel geometry, mass, and inertia are known model information; exact human goals are not.

Fix one F/T source for each comparison. Contact-reconstructed local F/T is panel-on-gripper. Transforming it to robot-on-panel at COM requires sign reversal, rotation, and a moment shift. Raw wrist transmitted wrench also contains downstream weight/dynamics. Keep both in the record, but do not mix their semantics. Teacher reference angles and velocities belong to private Expert traces, not Student features.

The primary history is at least two elapsed seconds. At 100 Hz, 201 samples cover `[t-2.0, t]`. The online buffer currently supplies only the available prefix during startup. It does not pad with invented observations or implement a learned-policy warmup strategy. Define startup eligibility/behavior before training and deployment. A rolling history does not require a two-second action delay, and two seconds may still be too short to reconstruct a much older HOLD reference.

The planned ablations are F/T+object, +joints, +tactile, and +both. Current offline tools implement `ft_object`, `ft_object_joints`, and `ft_object_joints_tactile`; the independent +tactile profile is missing. The online profile named `baseline` already includes tactile and contact counts. Unify these definitions before comparing models. Any derived F/T transforms must use declared public inputs consistently; E0's helper currently uses end-effector poses that are not in the default Student feature profile.

Set a valid domain for the pitch approximation before accepting labels. Current code records off-axis angular velocity but does not automatically reject a candidate when coupling becomes significant. General 3D angular velocity Y is not always the derivative of the selected pitch parameter. Thresholds and the response outside that domain remain to be defined.

## 5. Data contract and label eligibility

Each 100 Hz transition preserves this order:

```text
Observation_t
  -> AssistanceDecision_t
  -> ObjectCommand_t
  -> five ArmCommands / physics steps at 500 Hz
  -> Observation_t+0.01
```

Each arm substep records its command time, decision/object-reference source time, and post-step measurement time. A label at time t may use only information available by t under that strategy's permissions. The subsequent observation is an outcome, never a feature for the preceding action.

| Stored block | Required meaning |
|---|---|
| observations / next_observations | Public measurements before and after the transition |
| decisions | Timestamp, mode, requested additional torque, source, validity, optional confidence |
| object_commands | World COM pose reference (xyz+wxyz), twist (angular then linear), assistance wrench (force then torque), impedance settings |
| controller | Joint position targets in rad, pad commands in N, actuator response, command/source/realized timestamps |
| diagnostics | Physical failures, contact, saturation, limits, and validated execution diagnostics when available |
| privileged | Human goals/targets, applied human/disturbance forces and wrenches, labels and scenario context |
| supervision | Formal teacher labels and eligibility masks |
| render_states | Simulator state needed to inspect the recorded motion |
| metadata | Identity, versions, model/config/source hashes, timing, component configuration and provenance |

Full episode provenance should include panel properties, grasps, human impedance/limits, disturbance protocol, Expert gains and bounds, controller identity, observation profile, reset state, and quality/failure reasons. Exact task schedules and manual annotations are private experiment context. Never feed arbitrary metadata or episode time as a shortcut for a fixed schedule. Not all provenance fields are populated uniformly yet; fixture metadata and the older Expert sidecar metadata need reconciliation.

The current [pipeline](../phase1b/pipeline.py) logs these transition blocks. Its `metadata.experiment` stores benchmark context; that is not a Student input. Missing diagnostic quantities use NaN with false validity, not fabricated zeros.

The [Expert sidecar](../phase1b/expert/data.py) keeps private candidate traces separately from formal supervision. A candidate trace records Expert ID, mode, gate/direction, internal reference angle/velocity/acceleration, raw and limited torque, saturation/slew flags, and invalidity reasons. Confidence is `None` unless a calibrated estimate exists.

| Eligibility | What it establishes |
|---|---|
| candidate_valid | A candidate was produced under the mathematical/interface rules; not a physical or teacher-quality guarantee. |
| execution_valid | Required execution and physical checks passed; criteria and promotion path are not implemented yet. |
| supervision_eligible | Execution and teacher-quality requirements were met for formal supervision; this is not proof of learnability. |

`AssistanceDecision.valid` is not `expert_label_valid`. Current candidates keep execution and supervision eligibility false. Formal `expert_assist_torque_ref` stays NaN and formal validity masks stay false. Candidate values remain available in private traces for diagnostics. An actual zero assistance request is different from an unavailable label.

Requested extra torque, allocated contact wrench, executed joint command, and measured total contact wrench must remain distinct. The measured wrench includes base support. A difference from a zero-assistance run can be informative, but after trajectories diverge it is not a same-state measurement of extra assistance. The baseline decomposition, estimator, and valid tracking windows must be specified before reporting assistance RMSE. Current allocation/tracking fields are still unavailable.

## 6. Collection and evaluation

Start with shadow candidates on recorded interactions. E0/E1/E2 can inspect the same public trajectory under their respective information permissions, but their recommendations did not cause that trajectory. Shadow annotation is useful for checking signs, gating, limiting, and label conflicts; it does not demonstrate closed-loop benefits.

After controller tracking is validated, run each strategy in closed loop with the same initial conditions, human goals/impedance, and disturbance conditions. Let actual human force change with panel response. Replaying an identical human-force trace across strategies would remove the interaction effect we want to measure.

Manual pilot trials should control the virtual hand target through its spring-damper, not teleport the panel or prescribe Panda trajectories. Record the input device/mapping, raw input, requested intent/goal, event timestamps, and annotation uncertainty alongside the standard data. This can reveal how scripted and manual behavior differ and how a person uses the assistance. It does not directly label the optimal robot torque. The keyboard pilot and event logger below implement this first step. They do not provide force feedback to the operator or establish that keyboard interaction matches physical human manipulation.

Before training collection, specify ranges and sampling rules for direction, angle, speed, onset, intermediate stops, pause length, reversals, human impedance, and disturbance timing/magnitude. Keep the frozen hardware and grasp geometry. Choose held-out conditions and manual sessions in advance; split by episode and condition/session, not overlapping windows. Fit normalization on training data only. Sampling ranges, dataset sizes, and thresholds are still pending; repeating the fixed benchmark scripts is not sufficient distribution coverage.

| Evaluation | Evidence needed |
|---|---|
| Physical execution | Validated extra-torque tracking where identifiable, contact stability, limits, saturation and numerical health; T0-T3 are the starting fixtures. |
| Cooperation | Task accuracy, human force burden, start/stop/reversal delay, stopping drift/overshoot, false activation and failures; T4-T8 plus manual trials. Compare with support_only and E0. |
| Learning | Prediction error against eligible labels on held-out episodes, observable label consistency, and closed-loop cooperation after replacing the Expert with the Student. |

Human force burden can use `integral ||F_h||^2 dt`, RMS, and peak force; these are not metabolic energy. Report intended-stop error as well as error around the later accepted HOLD reference. Manual labels of intent and preference complement physical measurements; they cannot replace assistance tracking validation.

Before training, examine whether similar public histories require conflicting teacher torques. The [learnability tool](../phase1b/expert/learnability.py) finds cross-episode neighborhoods and reports coverage, label variation, and sign conflicts. It is a finite-sample diagnostic, not a conditional-variance estimate or proof of learnability. Report coverage when adding tactile dimensions, exclude same-episode leakage, and keep E1/E2 and different teacher configurations separate. E2 truth access and E1 hidden goals/internal state can create ambiguity even when execution succeeds.

RL refinement remains a future option. Define cooperation metrics and verify a controller and initial policy first. Any later reward must address task accuracy and stopping behavior as well as human effort. No reward, learning algorithm, or training implementation is agreed here.

### Human-side work before the shared controller is ready

Use [`phase1b.human`](../phase1b/human.py) to operate the existing Virtual Human Hand. It reuses the frozen hardware, settled fixture, spring-damper force law, physical guards, and standard transition logger. It never assigns panel poses or robot joint trajectories.

```text
Operator key / scripted event (private)
    -> bounded hand target position and velocity (private)
    -> existing spring-damper force at the human grasp
    -> panel + gripper contacts
    -> public tactile / F/T / object / joint observations
```

The first pilot controls one World Y rotation arc around the initial settled panel COM. The pivot stays fixed during the episode; it is a hand-input mapping, not a moving-COM torque definition. The v2 default caps speed at 15 deg/s and acceleration/deceleration at 60 deg/s^2; the known-input following session defaults to a +/-60 deg target bound. Sessions without `--follow-input` retain +/-10 deg. These are controller-specific development bounds, not global robot limits. The existing 0.35 m/s target-speed and 50 N force limits remain in force.

| Key | Human target operation |
|---|---|
| L | Request a goal 5 deg above the current target offset, clipped to the session upper bound |
| J | Request a goal 5 deg below the current target offset, clipped to the session lower bound |
| U / O | Request a smaller -1 / +1 deg goal for fine adjustment |
| K | Cancel the pending goal, brake with bounded acceleration, then hold the resulting stopping target |
| Q / close window | Finish the current 100 Hz transition, save, and exit |

These are key-press commands, not press-and-release velocity control. Repeated key events can extend the goal. Each command replans from the current target position **and velocity**. Longer moves accelerate, cruise and decelerate; short moves use a triangular speed profile and may never reach the speed cap. The target starts decelerating before its requested endpoint. Reversals brake to zero before moving the other way. Position and velocity are continuous; acceleration is bounded but jerk is not. This is a motion hypothesis for the pilot, not an empirically fitted human behavior model.

K starts braking immediately. Its target stopping distance is $\omega^2/(2\alpha_{max})$, or 1.875 deg from 15 deg/s at 60 deg/s^2; target braking time is 0.25 s. The real panel can take longer and move farther because of controller tracking lag. The event records `requested_stop_offset_deg` (target position when K was received) separately from `goal_offset_deg` (feasible planned stopping position). HOLD remains the declared intent while braking. Neither quantity is automatically the person's exact intended pose or the robot's measured pose. Arrival at a rotation goal does not manufacture a new HOLD annotation.

Human force remains $F_h=\operatorname{clip}_{50N}[K_h(x_h^{ref}-x_h)+D_h(v_h^{ref}-v_h)]$, using the frozen stiffness/damping. Acceleration shapes the target trajectory; it is not an independently prescribed force pulse. Constant angular speed does not imply constant hand force: centripetal acceleration, contact loading, gravity/support sharing and robot response still matter. The pilot records reference angular velocity/acceleration and Cartesian target acceleration in the private block. It does not add these private targets to Student inputs.

Without `--follow-input`, the default robot decision is always `UNCERTAIN`, zero additional torque. Human mode is not copied into the robot decision. The current arm adapter holds nominal joints, so expect small panel motion and changing contact force, not cooperative following. The same object/arm controller import options are available when Swathi's implementation is ready. No Expert or Student is running here.

Run from the repository root with a fresh output directory:

```bash
# Headless, reproducible start / intermediate stop / reversal / hold:
uv run --locked python -m phase1b.human --mode demo --duration 5 --output results/phase1b/dev-human-demo

# macOS native viewer with keyboard input; maximum 60 simulated seconds:
uv run --locked python -m phase1b.scripts.launch_viewer --human --mode manual --viewer --duration 60 --output results/phase1b/dev-human-manual

# Replay the saved input events and initial state with recomputed physical forces:
uv run --locked python -m phase1b.human --mode replay --replay results/phase1b/dev-human-manual --output results/phase1b/dev-human-replay

uv run --locked python -m unittest phase1b.tests.test_human -v
```

On Linux, run the manual command directly with `python -m phase1b.human`. The cyan marker is the desired hand target; red marks the panel's human grasp and the human force arrow (0.03 m of display length per N). The display uses a separate model/data copy so mouse interaction cannot change the recorded physics. The native debug sidebars are hidden by default. The bottom overlay shows all four pad normal-force grids (0-40 N/bin) and signed changes from the settled starting sample (-5 to +5 N/bin, red increase / blue decrease). Color values clip at these fixed bounds; printed values do not. Both rows use the same current physics sample as the scene. These are normal forces per bin, not pressure in Pa; shear channels remain recorded but are not shown in this overlay. Physics remains 500 Hz in simulation time, observations 100 Hz, and display updates 25 Hz. Wall-clock pacing is best effort; a slow computer can run below real time.

Each session saves:

- `episode/`: the existing synchronized sensor, command, private-truth, diagnostic, and render-state files. Formal Expert labels remain NaN/false.
- `events.json`: applied simulation timestamps, declared mode and goal offset; manual events also retain key receipt/application wall times.
- `session.json`: input mapping, limits, provenance, raw handled key events, verification and actual duration. Manual timing is an input-callback delay measurement, not a measurement of mental intention.
- `initial_state.npz`: the settled checkpoint used by both the original run and replay.
- `teacher_inputs.npz`: timestamp-aligned `ground_truth_intent`, `human_goal_angle`, and disturbance wrench for private shadow analysis. The legacy field name `ground_truth_intent` means **reported operator intent / scripted instruction** in this pilot, not an inferred human mental state.

The v2 contract (`virtual-human-input/2.0`) rejects v1 event replay because the stopping/velocity semantics changed. Prior results are retained; the pre-change local source snapshot is in `results/phase1b/dev-human-smooth-audit/source_before/`. Override speed/acceleration for a new session with `--speed-deg-s` / `--accel-deg-s2`; replay preserves its recorded parameters, including the angle bound. `--limit-deg` can reduce the new-session bound (maximum 60 with `--follow-input`, otherwise 10). Existing v2 recordings retain their original bounds.

Private transition fields additionally distinguish `human_goal_angle` (declared goal), `human_target_angle` / `human_target_trajectory` / `human_target_velocity` (moving reference), `human_event_timestamp`, and `human_input_source`. New fields include `human_target_angular_velocity_y` (rad/s), `human_target_angular_acceleration_y` (rad/s^2), `human_target_acceleration` (world m/s^2), and `human_stop_requested_angle` with a validity mask (NaN when not a HOLD event). Angles are radians; goals are initial measured pitch plus the requested World Y offset, a single-axis pilot convention. Existing `human_goal` retains the instantaneous-target meaning. Neither goal nor target is the measured panel pose. Events are applied only when their simulation timestamp arrives and never enter the public observation allowlist.

Replay checks saved event/checkpoint hashes and model/config identity. With the same code, controller and configuration, tests require identical recorded arrays. Changing the controller deliberately changes the physical response and human force; replay is of the input targets, not a fixed force trace. Physical failures preserve events and partial evidence and do not export a successful teacher stream.

The v2 implementation passed 42 tests across input profiles, continuity, speed/acceleration bounds, stop/reverse, exact replay, keyboard callback flow, causal interfaces, physics, Expert arithmetic and benchmark regressions. This is not a human usability study. The human pilot's timestamp-only test policies now use a one-sample online history; the complete 100 Hz observation stream remains logged, and the shared `ControlChain` default for future learned policies remains 201 samples. A profiled three-second v1 comparison dropped from 9.595 s to 4.275 s after removing unused pilot-history work and optimizing fixed-size contact cross products; every recorded array remained exactly equal. This timing includes startup/I/O and profiling overhead, so it is not a real-time guarantee. Fonts are cached for the live overlay.

Before the controller is available, use these sessions to check sensor responses, input/target timing, stop semantics, logging, and private candidate generation. Do not use them to claim assistance tracking, reduced effort, representative human data, or eligible training demonstrations. Once the controller is ready, compare the same input protocol under support-only and assistance, then evaluate manual sessions separately.

### Known-input following baseline

Add `--follow-input` to the human runner to make both arms follow the current hand target. This is an explicitly privileged input-following test, not intent inference, an Expert, or a torque-tracking controller. The ordinary policy/sensor allowlists are unchanged.

The test policy reports the current input mode. A known-input object-reference adapter rotates the initial panel pose about World Y using the **current** hand-target offset, never the future goal or schedule. A separate arm adapter receives only `ObjectCommand` and public observations. It rotates both nominal gripper poses together about the initial COM, solves IK on its own model at 100 Hz, and sends position-servo commands at 500 Hz with a 0.5 rad/s per-joint command-rate limit. It reuses the existing IK solver and fixed pad commands. No execution qpos writes, panel welds, or virtual arm-root forces are introduced.

K brakes the hand target and corresponding object reference to a planned stop. The real panel follows and settles after its tracking lag. J/L reverse or extend the target from its current position. Contact/joint/numerical guards remain active. This position-following baseline is useful for operating the virtual hand; it does not provide the compliant force allocation required by the eventual shared controller. It rejects nonzero assistance-wrench requests rather than treating Nm as joint positions. `assist_reference_executed` and all formal label/tracking validity flags remain false.

```bash
# Visible human/robot following, with the same keyboard and live tactile overlay:
uv run --locked python -m phase1b.scripts.launch_viewer --human --mode manual --viewer --follow-input --duration 180 --output results/phase1b/dev-human-follow-manual

# Reproducible positive rotation, intermediate stop, reversal and final hold:
uv run --locked python -m phase1b.human --mode demo --follow-input --duration 6 --output results/phase1b/dev-human-follow-demo

uv run --locked python -m unittest phase1b.tests.test_human_follow -v
```

Replay inherits the source session's `follow_input` setting. Use `--no-follow-input` explicitly for a matched fixed-joint comparison. The runner rejects mixing this baseline with external object/arm controller imports. Session metadata records `known_input_position_follow`, and transition metadata marks `known_human_input_baseline=true`; this input-access privilege must remain visible in any comparison.

The v2 six-second check brakes at intermediate targets +7.5/-7.5 deg. Actual panel rotation settled at +7.491/-7.490 deg; peak moving-reference pitch error was 1.589 deg and peak simulated hand force was 0.779 N. All four pads retained contact, maximum penetration was 0.354 mm, with no unexpected penetrating contacts or numerical warnings. The faster setting has more tracking lag than the earlier 5 deg/s pilot; target acceleration bounds alone do not make the robot a perfectly tracking or biomechanical system.

### Pitch range: three different limits

All angles below are World Y rotation **offsets from the settled start**, with the initial COM fixed and the existing two-grasp geometry. This is pitch-like reorientation, not yaw about vertical Z. These limits do not describe arbitrary arm poses or translations.

| Evidence | Result | Interpretation |
|---|---|---|
| Manual operating bound | +/-60 deg with known-input following; +/-10 deg otherwise | 15 deg/s and 60 deg/s^2; the selected bound is recorded and shown in the viewer |
| Earlier sampled local IK path, 0.5 deg increments | Last pass +110.0 deg; right-arm IK fails at +110.5 deg. Last pass -140.5 deg; arm-arm collisions appear at -141.0 deg | One seeded branch and enabled MJCF collision pairs, not a global workspace or continuous collision-free proof. The solver uses a 0.01 rad joint margin. |
| Two independent physical probes | +90 and -90 deg targets both passed; actual offsets +90.074/-89.884 deg | Each starts from nominal and holds its endpoint. Not a validated +90-to-90 reversal or a maximum physical limit. |

At the positive IK boundary, `right_joint6` reaches the solver's joint margin. At the negative collision boundary, `left_link4`/`left_link5` intersect `right_link6`. For the physical +90 probe, the closest observed joint margin is only 0.00846 rad (about 0.49 deg); this is not unique to +90 deg: the newer +/-60 deg round-trip also reaches a 0.00860 rad minimum margin (about 0.49 deg). The +/-60 deg UI is an experimental simulation envelope with runtime guards, not a claim of comfortable joint clearance. Joint velocity, servo response, friction/contact constraints, collision geometry and the chosen controller all affect the usable range. A different IK branch or base placement would require a new analysis.

Reproduce with fresh directories:

```bash
uv run --locked python -m phase1b.human_follow --extent-deg 180 --output results/phase1b/dev-pitch-geometry
uv run --locked python -m phase1b.human_follow --physics --extent-deg 90 --output results/phase1b/dev-pitch-physics
```

Local evidence is in `results/phase1b/dev-human-smooth-final/validation_summary.json` and `motion_profile.png`, with detailed source-bound episodes in `dev-human-pitch-physical-final/` and sampled configurations in `dev-human-pitch-scan-final/`. Physical probes preserve failures and partial evidence instead of advancing the accepted limit. No geometry, actuation mode, force-allocation algorithm or formal label eligibility was changed.

## 7. Implementation status and decisions still needed

| Item | Status |
|---|---|
| E0/E1/E2 reference math, private candidate traces, causal-window diagnostics | Implemented for candidate analysis |
| Three-layer transport, joint/sensor logging, T0-T8 fixture inputs | Implemented; default adapters do not execute assistance |
| Two-second default history | Implemented at `d73de9e`; its 32-test run covered history and existing contract/integration tests |
| Teacher intent/goal/trajectory bridge | Implemented for the human-input pilot; T0-T8 bridge pending |
| Online E1/E2 adapter | Pending; `ReferenceGenerator.step` is not a ready `predict(history)` policy |
| Manual input and timing | Keyboard target control, event replay, explicit reported intent, receipt/application clocks implemented; human-intent detection and end-to-end sensor/control latency remain pending |
| Known-input position-follow pilot | Implemented for manual input testing; explicitly privileged, not a torque controller |
| Full shared controller, allocation, incremental torque tracking | Pending validation/implementation |
| Formal label promotion, Student training, RL | Not implemented |

Next design decisions are: the teacher goal/intent/event mapping; start/stop/reversal cues and their delays; the torque-only execution behavior and startup/memory policy; unified sensor profiles and single-axis validity criteria; and the sampling plan, tracking estimator, and acceptance thresholds. HOLD-at-any-reachable-pose and the other accepted requirements above do not by themselves resolve these implementation choices.

This document clarifies the design. It does not promote existing candidates, certify controller performance, or claim that scripted behavior matches measured human behavior.

The expanded-range check runs 25 simulated seconds: 0 -> +60 (hold) -> -60 (hold), then brakes during positive rotation at -30, reverses again during motion, and returns to 0. Measured offsets at the check points were +60.054, -59.935, -29.955 and +0.00018 deg. All four pads retained at least two contacts; peak penetration was 0.528 mm, with no unexpected penetration, servo saturation or numerical warnings. The minimum joint margin was only 0.493 deg (`right_joint7`). These results pass the existing hard-limit/contact checks but **fail a separate proposed 5-degree clearance criterion**. A planning-only 5-degree reserve was tried and failed IK; it was not adopted. Hardware joint ranges, bases, grasps and the existing following algorithm remain unchanged.

Evidence and the reproducible input sequence are in `results/phase1b/dev-human-wide-20261102/`; `acceptance.json` separates the passing physical checks from the failed extra-clearance criterion. The failed reserve experiment retains partial data in `dev-human-wide-reserve-20261102/`. These are simulation development trials, not assistance tracking validation or formal demonstrations.

The expanded-envelope revision passed 43 automated tests, including the 25-second stop/reverse trajectory, controller-specific angle validation and replay-bound preservation. Reproduce the physical check with a fresh output directory:

```bash
uv run --locked python -m results.phase1b.dev-human-wide-20261102.reproduce results/phase1b/dev-human-wide-repeat
uv run --locked python -m unittest phase1b.tests.test_human phase1b.tests.test_human_follow phase1b.tests.test_contracts phase1b.tests.test_integration phase1b.tests.test_expert phase1b.tests.test_benchmark -v
```

The first command uses the locally retained validation script; the unit-test entry is part of the maintained source tree. The proposed five-degree clearance check remains separately reported as failed.

### Candidate base placement for larger rotations (not the active scene)

The placement study keeps the panel, all three grasp locations, gripper geometry, tactile sensors, actuator gains and hardware joint limits unchanged. It changes the fixed base transforms, matching pedestal cylinders and the initial IK solution. The candidate is stored separately; the manual runner still uses Hardware 2.0.0 / Scenario 3.0.0 and its +/-60 deg experimental envelope.

| Setting | Active scene | Recommended candidate |
|---|---|---|
| Left base XYZ, world m | (-0.45, 0.65, 0.25) | (-0.25, 0.45, 0.05) |
| Right base XYZ, world m | (+0.45, 0.65, 0.25) | (+0.25, 0.45, 0.05) |
| Base yaw, world Z | -90 deg on both | Left -60 deg; right -120 deg |
| Base spacing | 0.90 m | 0.50 m |
| Panel starting height | approximately 0.80 m | unchanged |
| IK planning reserve | 0.01 rad solver margin | additional 5 deg inside each physical joint limit |

The final coarse search compared 73 placements, including the current base coordinates, with multiple deterministic nominal IK seeds and a 5-degree planning reserve. A promising nominal pose alone was insufficient: some candidates lost reachability or collided during rotation. Candidate 10 was selected for a finer 2-degree scan and physical trials. This is a sampled search over local IK branches, not a global optimum or continuous collision-clearance proof.

Base relocation alone did not solve the margin issue. With the original unrestricted IK, the candidate completed a +/-90 deg trial but its minimum joint margin fell to 0.639 deg. The successful candidate trial therefore also reserves 5 deg **in the planner's model only**. This uses the existing IK algorithm and position-following adapter; the execution model's actual joint limits are unchanged. It is a combined layout/initial-posture/planning-constraint result, not an isolated causal claim about base position.

The combined candidate completed a 38-second physical trial at 15 deg/s and 60 deg/s^2: +100, -100, an intermediate braking hold at -70, reversal during motion, and return to zero. Recorded endpoint offsets were +100.066, -99.938, -70.027 and +0.006 deg. Across 19,000 physics steps, the minimum joint margin was **5.630 deg**, pad contact counts never fell below [2, 2, 2, 3], maximum pad penetration was 0.530 mm, and the physical guard detected no unexpected penetrating contacts, hard-limit violations or numerical warnings. The refined geometric path also passed. These checks establish this simulation trajectory, not unrestricted human operation or assistance-torque tracking.

Candidate artifacts use provisional `+base-study` identities and their actual XML/hardware hashes; they are not relabeled as frozen production inputs. The capture is a dedicated diagnostic trace, not the formal learning pipeline or an eligible Expert demonstration. It saves the input events, initial checkpoint, joint states, commands, tactile data, contact counts, model snapshots and pass/failure report. The current formal data pipeline is unchanged.

Reproduce with fresh output directories:

```bash
uv run --locked python -m phase1b.base_placement --wide --reserve-deg 5 --extent-deg 90 --output results/phase1b/base-search
uv run --locked python -m phase1b.base_placement --search-result results/phase1b/base-search/search.json --candidate-id 10 --extent-deg 100 --step-deg 2 --reserve-deg 5 --output results/phase1b/base-candidate
uv run --locked python -m unittest phase1b.tests.test_base_placement -v
```

The export regression passed and the physical runner independently asserts that all compiled hardware arrays are unchanged except the two base transforms and pedestal positions/sizes. Local evidence: `results/phase1b/dev-base-candidate10-reserved100-20261102/` contains `scene.xml`, `hardware.json`, `report.json`, `trace.npz`, `poses.png` and `tracking_and_margin.png`. The candidate scene must be driven through the supplied runner to reproduce the human forces and motion; opening the XML alone does not reproduce the experiment. Earlier failed-clearance trials are retained separately.

Recommendation: use this configuration for the next scene revision, with a conservative initial interactive bound of +/-90 deg and the tested 5-degree planning reserve. The extra +/-100 deg trial provides evidence beyond that proposed UI range. Adoption still requires versioning the canonical hardware/config snapshots and checking the manual/replay pipeline against the new geometry; that migration was not performed during the placement study.

A subsequent inter-arm clearance audit covered all 3,800 recorded poses (100 Hz) and 192 enabled cross-arm collision-geometry pairs. Bounding spheres and explicit mesh/box vertex support projections establish a conservative **0.11794 m separation lower bound** for those samples. This is not an exact minimum-distance measurement, a swept-volume proof, or a clearance guarantee for arbitrary manual input. The pair producing the weakest bound is `left_link2` / `right_link5`, near the +100 deg hold. This supports keeping a proposed 0.10 m sampled inter-arm clearance criterion alongside the separate 5-degree joint-limit reserve.

The first raw `mj_geomDistance` result was inconsistent with its witness points and is explicitly marked invalid in `arm_clearance.json`; a cross-version query also had inconsistent witnesses. A related issue is tracked in the [MuJoCo repository](https://github.com/google-deepmind/mujoco/issues/3383). The accepted audit uses separating-plane lower bounds from compiled geometry vertices, not the native scalar distance. Its method, source/trajectory hashes and limits are in `arm_clearance_projection_bounds.json`, with `clearance_check.py` retained beside the trace. The temporary distance cross-check environment did not change the project's MuJoCo version or lockfile.

### Trying the candidate with the manual controller

The candidate is now an opt-in configuration of the **same** human runner and transition pipeline, with Hardware `2.1.0-preview.1` / Scenario `3.1.0-preview.1`. The original configuration remains the default so historical experiments are not silently rebound to new geometry. Start the candidate on macOS with:

```bash
uv run --locked python -m phase1b.scripts.launch_viewer --human --mode manual --viewer --follow-input --scene-config phase1b/config/dual_panda_base_candidate.json --duration 600 --output results/phase1b/candidate-manual
```

The header displays `BASE CANDIDATE`, and the reference stays within +/-90 deg. J/L request -/+5 deg, U/O request -/+1 deg, K brakes to a hold, and Q saves and closes. Maximum speed remains 15 deg/s with 60 deg/s^2 acceleration/deceleration. This is the known-input position-following baseline, not the shared compliant assistance controller.

The planner reserves 5 deg inside the actual joint bounds; hardware limits remain unchanged. In addition to existing 500 Hz physical guards, the runner checks actual joint clearance at 500 Hz and certifies the 10 cm inter-arm separation bound at 100 Hz. If a check fails or clearance cannot be certified, the runner stops and preserves failure/partial evidence. These sampled development guards do not certify continuous clearance or arbitrary real-world operation. The separation calculation uses explicit convex vertex projections; the unreliable native scalar distance is not used as a clearance value.

Sessions record the selected scene configuration and following settings. The formal transition schema now includes `config_path` and `hardware_path`; model/config/hardware hashes bind to the selected scene. Validation reconstructs that scene instead of incorrectly checking every run against the original hardware. Replay inherits the scene and recorded motion parameters; it rejects a scene override. Expert supervision and assistance-tracking validity remain false.

The candidate manual model was checked for exact compiled-hardware equality with the successful +/-100 deg study model. Its runtime separation checker also passed all 3,800 recorded study poses. A new six-second run checks actual dynamics through the normal pipeline; regression tests require exact replay arrays, correct scene identities, unchanged physical joint bounds, and rejection of overlapping arms by the clearance checker.

Manual-preview integration validation: **46 tests passed**, covering the new candidate identity/replay/clearance checks and existing human, following, interface, physics, Expert arithmetic and benchmark regressions. The native viewer also received L, J and K successfully; the applied events were recorded in `dev-candidate-live-20261102-01`. Verification output is retained in `dev-candidate-manual-preflight-20261102/tests.log`.

### Configurable follower delay

The known-input follower now accepts `--follow-delay-ms 200`. This is an explicit **200 ms delay in sampled human-reference availability**, measured in simulation time. It is not a calibrated human reaction model, camera latency, motor transport delay, or a promise about wall-clock frame rate. The ordinary default remains 0 ms, and replay inherits the recorded value. Delay must be nonnegative and a multiple of the 10 ms reference period.

The virtual human's target and spring-damper force respond immediately. At 100 Hz, an immutable sample of that target's angle, velocity and reported mode enters a FIFO. The known-input policy and object-reference adapter use a sample only after its configured delay has elapsed. The arm controller still receives current robot observations and current command timestamps. During startup, before a delayed sample is available, it holds the initial reference; source timestamp/age are NaN with `reference_available=false`. K's intended braking/hold reference and reversals are delayed just like rotation. Q/close and physical/joint/clearance checks are not queued.

`follower_timing.npz` records current input/decision time, source sample and input-event times, availability, reference age, human versus follower target angles/velocities/modes, the actual five arm-command timestamps per transition, and their source-reference ages. This private diagnostic sidecar is not a policy feature or formal label. It is row-aligned with the standard transitions; the episode context and session metadata record the configured delay. Failure keeps `partial_follower_timing.npz` rather than claiming a complete alignment.

For 200 ms, valid high-level reference age is exactly 200 ms; the five 500 Hz command substeps use ages 200–208 ms. An input event between reference ticks first waits until the next 100 Hz sample: for example, an event at 0.102 s is sampled at 0.110 s and delivered at 0.310 s. Physical panel tracking adds its own lag. The two-second Student history requirement is unrelated to this injected delay.

```bash
uv run --locked python -m phase1b.scripts.launch_viewer --human --mode manual --viewer --follow-input --scene-config phase1b/config/dual_panda_base_candidate.json --follow-delay-ms 200 --duration 600 --output results/phase1b/candidate-delay200-manual
uv run --locked python -m phase1b.human --mode replay --replay results/phase1b/candidate-delay200-manual --output results/phase1b/candidate-delay200-replay
```

The viewer header shows the configured delay. The tactile overlay shows immediate hand/panel angular speeds, the follower angle reference and its age. An unavailable startup age is shown as `nan`, not a fabricated measurement. The speed and acceleration limits are unchanged.

Validation includes zero-delay equivalence at the reference boundary, causal FIFO sampling, startup masks, a between-tick input, delayed stop/reverse modes, 200–208 ms actuator-source ages and exact recorded-array replay. A separate 36-second +/-90 deg delayed physical trial passed the standard physical checks plus the 5-degree joint-margin and 10 cm sampled arm-clearance guards. Minimum joint margin was approximately 5.64 deg, peak pad penetration 0.487 mm, and peak simulated human force 2.77 N, with no unexpected penetrating contacts or numerical warnings. This validates the test trajectory, not all possible manual inputs or assistance tracking. Evidence is retained in `dev-follow-delay200-20261102/` (including `delay_response.png`) and `dev-follow-delay200-range-20261102/`.

This revision passed 2 delay-specific tests and 26 selected regression tests. The live header was verified as `follow delay 200 ms`. Because the overlay is drawn after a physics step and just before the next 100 Hz reference refresh, it can show a source age of 210 ms; this is distinct from the configured 200 ms FIFO delay and the recorded 200–208 ms command-source ages.

### Matched stop/reversal comparison: 0 vs 200 ms

`phase1b.demos.compare_follow_delay` runs both delays from the same saved integration state, using the candidate scene and identical immediate human target trajectories. Only follower-reference delay changes; the spring–damper human force is recomputed from each physical response. The input starts positive rotation at 1.00 s, requests braking HOLD at 1.80 s, resumes at 3.00 s, and requests negative rotation at 3.80 s while still moving positively. Speed/acceleration limits remain 15 deg/s and 60 deg/s².

| Measurement | 0 ms | 200 ms |
|---|---:|---:|
| Stop response after the 1.80 s request | 0.50 s | 0.70 s |
| Continued positive motion after stop request | 3.45 deg | 6.41 deg |
| Peak human force in stop window | 0.72 N | 2.05 N |
| Negative-motion response after the 3.80 s request | 0.37 s | 0.57 s |
| Continued positive motion after reversal request | 3.15 deg | 6.10 deg |
| Peak human force in reversal window | 0.81 N | 2.12 N |

Response detection uses 100 Hz samples: stop requires angular speed magnitude ≤0.5 deg/s for 0.20 s; reversal requires speed ≤−0.5 deg/s for 0.10 s. Windows are 1.80–2.90 s and 3.80–4.90 s. Both windows have a maximum same-time angle gap of approximately 2.97 deg. Both runs settle near 12 deg after the first stop: the gap is transient, not a final angle error. Continued positive motion is measured from each run's angle at the input event, not from the final target.

The maximum absolute per-bin tactile difference is 8.79 N/bin during stopping and 13.81 N/bin during reversal. At 4.00 s, right-upper pad totals are almost equal (34.42 vs 34.39 N), while individual bins differ by +13.77 and −11.78 N/bin. This shows spatial redistribution that a pad-total scalar hides. These are coarse 4×4 sensor-local normal-force bins, not pressure in Pa; contact crossing a bin boundary can create abrupt changes. Pad-normal sums measure clamping load, not net object force or executed assistance torque.

Both 8.5-second episodes pass the existing pipeline/physical checks, 500 Hz joint-margin checks and 100 Hz inter-arm clearance certification. Each contains 850 transitions / 4,250 physics steps, with at least four contacts per pad, maximum pad penetration below 0.257 mm, minimum joint margin above 20.5 deg, no unexpected penetration, no actuator saturation and no numerical warnings. Exact initial q/qdot and human target arrays are asserted equal across conditions. This is one deterministic known-input baseline comparison; assistance tracking and Expert-label validity remain false.

```bash
uv run --locked python -m phase1b.demos.compare_follow_delay --output results/phase1b/delay-comparison
# Rebuild the visuals from retained episodes without running physics again:
uv run --locked python -m phase1b.demos.compare_follow_delay --render-only --output results/phase1b/delay-comparison
```

The first command requires a fresh output directory. Local evidence is in `results/phase1b/dev-delay-comparison-20261102/`: standard episodes and timing sidecars, `comparison.json`, `comparison.csv`, overview/heatmap PNGs, and self-contained `index.html`. The HTML synchronizes both traces and all four tactile maps, with playback, event shortcuts and a shared time slider. It can be opened offline; no external libraries or network service are required.

The maintained [controller quickstart](CONTROLLER.md#start-here-human-control-and-delayed-follow-reference) now includes this reference and custom-controller entry points on the same candidate scene. Custom manual controllers start with a ±10-degree human target bound and `UNCERTAIN` / zero assistance; the 200 ms option belongs only to the known-input follower. T0–T3 can select that same scene through `--scene-config`. Both runners share the physical/joint/clearance guard. New session and episode metadata record in-repo scene paths relative to the repository; replay preserves controller imports as well as scene/input settings. These changes make the pilot transferable without changing policy permissions or promoting demonstration labels.
