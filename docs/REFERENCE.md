# Technical Reference

Implementation details for the [project plan, status, and T0–T8 acceptance benchmark](../README.md). One implementation: `phase1b/`. Contract v1.1; Scenario 3.0.0 / Hardware 2.0.0. Benchmark changes described as planned below are not implemented capabilities.

## Frozen hardware

| Item | Definition |
|---|---|
| Panel | Free rigid cuboid, 0.60×0.20×0.02 m, 1 kg; initial COM `[0,0,.80]` m, pitch approximately +0.336° |
| Frames | Panel origin = COM; local X long dimension, Y short dimension, Z thickness. World Z up; gravity `[0,0,-9.81]` m/s² |
| Arms | Two Menagerie Franka Pandas, 7 joints each; fixed world bases L `[-.45,.65,.25]`, R `[.45,.65,.25]` m, both yaw −90° |
| Nominal grasps | Panel-local L `[-.20,.10,0]`, R `[.20,.10,0]` m on one long edge; human `[.27,-.10,0]` m on the opposite edge |
| Pads | Upper/lower per arm; each 0.09×0.14×0.02 m, 0.10 kg, slides along gripper-local Z; custom pads, not stock Panda fingers |
| Connection | Flange at link7-local Z +0.107 m; gripper offset from flange Z +0.12 m, X rotation −90°; 0.08 kg adapter per arm |
| Commands | Four pad motor commands `[-40,+40,-40,+40]` N (L upper/lower, R upper/lower), not guaranteed contact forces; 14 arm position targets in rad |
| Arm torque bounds | Per arm: joints 1–4 ±87 Nm; joints 5–7 ±12 Nm. No direct joint-torque adapter |
| Human | One virtual spring-damper point, not articulated human anatomy; approximately 98.21 N/m, 7.30 N·s/m; force cap 50 N, target-speed cap .35 m/s |
| Disturbance | Independent force at panel-local `[.063691,-.015331,0]` m; exact schedule/force truth remains privileged |
| Runtime | Physics/control 500 Hz, observations/references 100 Hz, 2 s settling; runtime first friction coefficient `1.0482960672343025` |

Only the panel has a freejoint. No panel weld or gripper-root virtual wrench supports it; loads pass through arm joints and pad contact. Nominal grasps are panel-local definitions; actual contacts are solved by physics. Panda inertias/limits/collision geometry are retained, subject to model collision exclusions. Object pose is simulator state, not a visual estimate; no VLM/RGB-D/vision estimator is integrated.

Exact values: [model XML](../phase1b/models/dual_panda_v1.xml), [scenario / initial q / human parameters](../phase1b/config/dual_panda_v1.json), [compiled hardware / limits](../phase1b/config/hardware_v2.json). Load through Python to apply runtime configuration. Geometry, actuation, or contact changes require versioning and affected physical tests.

## Interface contract

Types/validation: [interfaces.py](../phase1b/interfaces.py). Orchestration: [control.py](../phase1b/control.py). All timestamps are finite, nonnegative episode seconds. Arrays are finite, shape-checked, and defensively copied; ObjectCommand arrays and impedance mapping are read-only.

### AssistanceDecision

| Field | Contract |
|---|---|
| timestamp | Exactly the current decision observation time |
| mode | `ROTATE_POS`, `ROTATE_NEG`, `HOLD`, `UNCERTAIN` |
| assist_torque_y_nm | Scalar Nm, World +Y about instantaneous panel COM; additional assistance, excluding basic support |
| confidence | `None` without a calibrated estimate; otherwise `[0,1]` |
| source | `scripted_test`, `rule_based`, `oracle_expert`, `learned_policy` |
| valid | Actual bool; false requires UNCERTAIN and zero torque |

ROTATE intent and torque sign may differ during braking. HOLD permits nonzero resisting torque. UNCERTAIN must have zero assistance; valid UNCERTAIN means uncertainty was assessed, invalid means the result is unavailable. HOLD + disturbance is not another intent class.

Benchmark notation `desired_assist_torque_y` refers to this same scalar; the actual API field is `assist_torque_y_nm`, and the existing controller log alias is `desired_assist_torque`. Do not introduce a second semantic definition. `from_legacy(AssistTorqueReference(...), mode=...)` requires an explicit mode; `to_legacy()` maps oracle_expert to model_based_expert and rejects unrepresentable rule_based. This is data conversion, not a second runtime.

### ObjectCommand

| Field | Contract |
|---|---|
| timestamp / mode / valid | Same as AssistanceDecision |
| pose_ref_world | `(7,)` world xyz at panel COM + unit quaternion **wxyz**; m |
| twist_ref_world | `(6,)` **angular xyz → linear xyz**; rad/s, m/s |
| assist_wrench_world | `(6,)` **force xyz → torque xyz**; N, Nm; exactly `[0,0,0,0,assist_torque_y_nm,0]` in the current MVP |
| impedance_parameters | Optional stiffness/damping, each nonnegative `(6,)`, angular→linear; empty means unspecified, not implemented zero stiffness |
| reference_point / frame_id | `panel_com` / `world` only |

Rotational stiffness/damping: Nm/rad, Nm·s/rad; translational: N/m, N·s/m. Support/contact/mechanical limits take priority. One compliant pitch law must coordinate motion and assistance: do not add competing high-stiffness pitch control and extra torque. The current type rejects nonzero Ty with explicitly nonzero pitch stiffness.

The reference-only adapter latches measured pose on entry to HOLD. ROTATE retains current pose/zero twist while forwarding torque; it creates no trajectory. UNCERTAIN uses current pose and zero twist/wrench. ControlChain defaults `.02` Nm and `.5` rad/s are interface-test bounds, not a reachable envelope. Full smoothing, admittance, contact constraints, and saturation handling belong to the implemented controller.

### ArmCommand and timing

`arm_position_target (2,7)` rad; `pad_force_command (4,)` N; `controller_mode` string; `assist_reference_executed` bool. Validate shapes, finite values, and joint/pad bounds. The position-hold adapter returns nominal q/preload and does not execute assistance. A true execution flag alone is not tracking evidence; UNCERTAIN cannot report executed assistance.

```python
AssistancePolicy.predict(history: PolicyHistory) -> AssistanceDecision
ObjectMotionController.step(decision, observation: ArmControllerInput, dt: float) -> ObjectCommand
DualArmController.step(command, observation: ArmControllerInput) -> ArmCommand
```

At 100 Hz: observe → decide → object command → five 500 Hz arm/physics substeps → post-observation. Each substep gets fresh public sensing but retains the high-level reference; maximum age .008 s. Missing, repeated, future/stale ticks and invalid outputs abort without a silent fallback. ArmCommand has no duplicate clock: the pipeline logs command and source timestamps at dispatch.

PolicyHistory uses the sensor allowlist (default maximum 50 rows); q/dq require `with_joint_states`. Lower controllers also receive public EE states, actuator data, nominal q/preload, and limits. No goals, scenario, truth forces, future data, or simulator handles cross these boundaries. This is API isolation, not a security sandbox. Teacher-only calls must be integrated upstream without widening student/controller inputs.

## Sensor and data contract

The only formal writer/reader/validator is [pipeline.py](../phase1b/pipeline.py): `run_episode / read_episode / validate_episode`. Schema: `enpm645-three-layer-transition/1.0`. Shapes below exclude time; arms are left/right and joints 1…7.

| Observation | Shape / frame / units |
|---|---|
| wrist_ft_left/right | Each `(6,)`; contact-reconstructed panel-on-gripper, gripper-local, about gripper origin; force→torque, N/Nm |
| wrist_sensor_left/right | Each `(6,)`; child→parent transmitted wrench, sensor at gripper-local `[0,.12,0]`; includes downstream weight/dynamics, uncompensated |
| tactile_left/right | Each `(2,3,4,4)`; upper/lower × sensor-local z/x/y × 4×4 angular bins; N/bin, not Pa or images |
| object_position/orientation | `(3,)` world m / `(4,)` quaternion wxyz |
| object_linear/angular_velocity | Each `(3,)`, world m/s or rad/s |
| contact_state_left/right | Each `(2,)`, upper/lower contact counts |
| arm_joint_position/velocity | Each `(2,7)`, rad / rad/s |
| arm_actuator_torque/force | Each `(2,7)`; current unit-gear hinges make both numerically Nm, not external-force estimates |
| end_effector_pose/velocity | `(2,7)` world xyz+wxyz / `(2,6)` angular→linear; gripper frame between pads, not link7 |
| gripper_joint_position/velocity | Each `(2,2)`, m / m/s; gripper_actuator_force `(4,)`, N |
| object_contact_wrench | `(6,)`; gripper-on-panel, world, about instantaneous COM |

Tactile uses `mujoco.sensor.touch_grid`; heatmaps show normal force and logs retain all three channels. The default policy receives contact-reconstructed F/T, tactile, object pose/velocity, and contact state (10 fields). Raw wrist signals, actuator torques, commands, metadata, and truth are not automatically inputs. Contact-reconstructed and raw wrist F/T are not interchangeable. Combined pose/velocity and separate world-at-panel-COM left/right wrenches remain diagnostics.

| Episode file | Meaning |
|---|---|
| observations.npz / next_observations.npz | Pre-decision t / post-step t+.01 sensing; previous command still acts at t. Next observations equal the next row except at episode end; unavailable to the current decision |
| decisions.npz | Mode/torque/source/valid; unavailable confidence = NaN + confidence_present=false |
| object_commands.npz | Pose/twist/assistance/impedance requests, frame/point, validity; unspecified stiffness/damping use NaN + false masks |
| controller.npz | Five substep commands, source times, actuator torque, saturation, post-step contact wrench, and unavailable tracking/allocation fields |
| privileged.npz | Human targets/forces, disturbance/application points, scenario; teacher/evaluation only |
| supervision.npz | Formal torque NaN, expert_label_valid=false; separate from candidate decisions |
| diagnostics.npz / render_states.npz | Contact, penetration, joint limits, force balance, warnings / qpos/qvel for replay, FK and rendering |
| metadata.json / verification.json | Versions/hashes/component classes/profiles/limits/array layout / episode contract and physical checks, not Expert certification |

Each decision/object command has N rows. `command_timestamp`, `decision_timestamp`, `object_command_timestamp` are `(N,5)`; command time is t+[0,.002,.004,.006,.008], source times remain t. `realized_timestamp` is command time+.002. Engine time is recorded separately from the episode clock reset after settling.

Commands: `arm_position_target (N,5,2,7)` and `pad_force_command (N,5,4)`. Servo torques distinguish requested before clipping, at actuation before stepping, and after stepping. `realized_object_contact_wrench (N,5,6)` is total contact wrench. Contact status maps to diagnostics.contact_counts; saturation to controller.arm_force_saturated.

**Current limitations:** `requested_object_wrench (N,5,6)` means total demand including support; `allocated_gripper_wrenches (N,5,2,6)` means per-arm allocation. Both remain NaN with false validity masks, as does `tracking_error_Nm (N,)`. Known additional demand is in object_commands.assist_wrench_world. The pipeline currently writes and validates these unavailable placeholders; Track B integration must add a genuine diagnostics path and adjust validators. `baseline_wrench_proposal` is a **planned benchmark field**, not an existing measurement. Never replace missing quantities with fake zeroes or total contact Ty.

## Expert references and shadow labels

[reference.py](../phase1b/expert/reference.py) contains pure mathematical generators, not a validated controller. Full rigid-body dynamics about COM are `I_W α + ω×(I_W ω) = τ_h + τ_d + τ_r`, with `I_W=R I_body Rᵀ`. `required_assistance_y()` subtracts human, disturbance, and base contributions from the required Y torque, retaining gyroscopic coupling. Uniform gravity has no direct COM torque; edge-support contacts do. This requirement does not guarantee actuator/contact feasibility.

`panel_inertia()` gives `J_y=.03003333 kg·m²` for this cuboid. The reduced model is `J_y θ̈=τ_h+τ_d+τ_b+τ_a`. Ideal exact inverse dynamics with positive PD gains yields `ë+k_d ė+k_p e=0`, not a dual-Panda stability guarantee. Human pitch impedance uses `J_h=e_y×r_world`, `K_θ=k_h‖J_h‖²`, `D_θ=d_h‖J_h‖²`; human and disturbance contributions need not be uniquely identifiable.

| Generator | Input permissions | Raw candidate torque |
|---|---|---|
| E0 | Public F/T, EE pose, object pose/ω; rejects privileged inputs | `k_f LPF(J_y α_hat−τ_L−τ_R)` |
| E1 | Public sample + current TeacherGoal; rejects disturbance truth | ROTATE: `J_y α_ref+Kp_R(θ_ref−θ)+Kd_R(ω_ref−ω)`; HOLD: `Kp_H(θ_hold−θ)−Kd_H ω` |
| E2 | E1 inputs + current DisturbanceTruth | `raw_E1−λ_d τ_d`, `0≤λ_d≤1`; UNCERTAIN still overrides to zero |

Create a generator per episode/expert. It requires consecutive 100 Hz samples and exactly current teacher timestamps. E0 uses backward-difference acceleration and LPF coefficient `1−exp(−dt/T)`; first sample is invalid UNCERTAIN/0 with NaN/false estimate masks. E0 mode is reactive direction, not recognized human intent. Convert local panel-on-gripper F/T by negating/rotating and adding `(p_gripper−p_COM)×F`. Its EE-pose requirement is additional public kinematic information compared with a student using local F/T + object alone.

E1 opens its gate after direction-consistent torque/velocity thresholds persist, matching teacher mode. Reference starts at measured angle and advances at a bounded rate toward the current goal. Either continuing torque or velocity can sustain it; both below off thresholds for release_seconds close it. Opposing torque cancels the old direction and reversal needs confirmation. HOLD latches measured angle on entry. The gate can still be fooled by disturbance; it is not proof of human-intent identification.

Teacher reference angles/state stay private: only `candidate.decision` crosses downstream. The teacher callback is not yet connected to the physical runner. E1 does not fully cancel external forces; E2 must not duplicate compensation already handled by the base controller. A simplified steady-state deviation `(1−λ_d)τ_d/K_sum` assumes positive total stiffness, equilibrium, and no saturation.

`ExpertConfig` requires all gains/thresholds/limits and limits_provenance explicitly. Kp: Nm/rad; Kd: Nm·s/rad; k_f/λ dimensionless; timing in s. Apply amplitude then slew limits, starting at zero. UNCERTAIN immediately zeros output, possibly overriding slew and recording `rate_limit_overridden`; physical smoothing remains a controller responsibility. Test-fixture numbers are not operating limits. The scalar pitch/ω approximation assumes small angles near Y without crossing ±π; off_axis_omega_norm is diagnostic, not a validity certificate.

[data.py](../phase1b/expert/data.py) writes new `enpm645-expert-shadow/1.0` sidecars without altering source episodes:

- `derived_features.npz`: causal θ/ω/Δθ/acceleration, interaction estimate, left/right COM torques, off-axis norm, and validity masks.
- `expert_internal/E0.npz`, E1, E2: candidate mode/torque, gate/direction, reference angle/velocity/acceleration, raw/amplitude/final output, saturation/slew/override, source, status/reason. Entire traces are teacher-internal.
- `supervision/E*.npz`: formal torque remains NaN; mode/torque/label validity, execution_valid, and supervision_eligible remain false. candidate_valid is numerical availability only; internal mode validity is not formal supervision.
- `metadata.json`: identity/seed, commit/dirty state, schema/MuJoCo/scenario/hardware versions, model/config/hardware/source/code/output hashes; panel/grasp/human parameters, controller components, full expert config, timing, frames, and quality.
- `privileged_metadata.json`: teacher source/hash, exact targets/schedules/disturbance point/time/magnitude and scenario. Unknown source/controller versions or ranges stay null; episode_valid=null because source physics was not revalidated.

Explicit teacher NPZ requires aligned `timestamp (N,)`, `ground_truth_intent (N,)` four-mode strings, `human_goal_angle (N,)` rad, and `external_disturbance_wrench (N,6)` world force→torque about instantaneous COM. Existing old logs lack explicit intent; do not derive it from A/B/C. Fixed-config metadata is not a substitute for actual randomized per-episode parameters. `validate_sidecar()` checks hashes, clocks, bounds, slew exceptions and label isolation, not physical quality; recorded absolute source paths must remain resolvable.

[learnability.py](../phase1b/expert/learnability.py) uses causal windows with profiles `ft_object`, `ft_object_joints`, and `ft_object_joints_tactile`. Choose one F/T source, contact_reconstructed or wrist_transmitted. Exclude metadata/IDs/time, hidden targets, truth forces, references and future data from features. E1/E2 targets are analyzed separately. Scaling uses reference-split std; constant dimensions use one native unit. Quaternions are sign-canonicalized but distance is not SO(3) geodesic.

Same-source observation hashes are excluded as neighbors; also split by seed/protocol to avoid near-duplicate runs. Report coverage with local label variation, neighbor-mean error and sign conflicts. Neighbor counts are windows, not independent episodes; equal radii across feature dimensions do not ensure fairness. No matches means `insufficient_matched_evidence`, metrics=null. This is not conditional variance, a Bayes-error estimate, proof of tactile benefit, or training. The pilot caps each set at 500 windows; adjust stride before expanding it.

## T0–T3 scene recipe

**Planned fixture specification; not an implemented runner or validated operating point.** One frozen hardware model is reused for all four cases. No ML, intent inference, or oracle input is needed. Track A owns fixtures/reference injection/recording; Track B owns controller initialization, object/arm execution, and genuine controller diagnostics. Both freeze acceptance settings after calibration.

### 1. Prepare and reset one starting state

1. Load Scenario 3.0.0 / Hardware 2.0.0 through the existing Python path, with nominal q, runtime friction, pad preload, and human impedance unchanged. Begin with the existing 2 s settling procedure and zero additional assistance.
2. Disable the independent disturbance. After initial settling, latch the **world position of the human interaction point** as the fixed human target and set target velocity to zero. Keep the spring-damper active; do not force its measured output to zero. Do not continuously move the target with the panel, which would remove the intended reaction.
3. Allow this target change to settle under the same zero-assistance controller. The additional duration and drift/contact criteria must be calibrated and recorded, not assumed satisfied after 2 s. Confirm four-pad contact, acceptable pitch/height drift, joint margin, collision/penetration/saturation limits, and no numerical warnings.
4. Save starting state S0: all simulation state required for reproducible continuation (including qpos/qvel, actuator/control and solver state as applicable), fixed human target, controller/filter/internal reference state, random state/seed, model/config/controller versions, and clocks. Record actual panel pose and human force; do not substitute the nominal .80 m height or assume zero residual load.
5. Restart each T-case and its zero-assistance counterpart from equivalent S0 with identical controller initialization and human settings. Reset episode/reference clocks consistently; restore or deterministically reproduce solver/history state rather than only copying q/dq. Record reset mismatch and reject it if it exceeds the pre-locked tolerance.

**Current implementation gap:** `PandaWorld("B")` uses a fixed config-derived human target; it does not latch the post-settle interaction point. The default `ScriptedContractPolicy` cycles test modes/torques; it is not T0. Therefore `collect --scenario B` alone does not implement this recipe. Add the explicit fixture reset/human-target hook and deterministic test-reference source, with a versioned fixture configuration; do not silently relabel existing B logs as T0.

### 2. Inject requests, not physical torques

At every 100 Hz observation timestamp, emit `AssistanceDecision(timestamp=t, mode="HOLD", assist_torque_y_nm=u(t), confidence=None, source="scripted_test", valid=True)`. Route it through the same Object Controller and Dual-Arm Controller, with five 500 Hz substeps per reference. Do not apply u(t) directly to the panel with `mj_applyFT` or a root wrench. Only ordinary arm/pad actuation may realize assistance. Human/disturbance forces remain separately owned by the fixture.

Keep the same nominal object-reference policy and compliant pitch law in zero/active runs. Nonzero HOLD torque is permitted, but a rigid pitch lock competing with the extra wrench is not. The current ObjectCommand rejects nonzero assistance combined with explicitly nonzero pitch stiffness; a changed control contract must be reviewed explicitly. The teammate chooses the physical control design and reports how assistance changes commands without losing support.

For sign intuition near the nominal horizontal pose, `τ_y = r_z F_x − r_x F_z`. An ideal incremental couple with left ΔF_z=+δ and right ΔF_z=−δ has zero net vertical increment and Δτ_y≈0.40 m × δ. Positive Y rotation lowers the panel's +X side and raises its −X side. This explains the sign, **not a required allocation algorithm or a claim that these forces are achievable**. Actual allocation must use current world geometry and contact constraints; both arms continue carrying baseline load.

### 3. Shared command schedule

Let A>0 be the **calibrated** torque amplitude in Nm; let b, r, h, z, f be baseline, ramp, plateau, intermediate-zero, and final-zero durations in seconds. Keep all event times on the 100 Hz grid. A, rate bounds, motion envelope, test duration, and abort thresholds are TBD until calibration. Never reuse .02 Nm as a proven amplitude.

| Case | Sequence from identical S0 | Required zero-assistance counterpart |
|---|---|---|
| T0 | u=0 for the declared observation interval | Establish baseline response and repeatability |
| T1 | 0 for b → ramp to +A over r → hold for h → ramp to 0 over r → zero for f | Same total duration, controller, fixed human target, and initial state, with u=0 throughout |
| T2 | Exact sign mirror of T1, using the same A and durations | Same as T1; do not retune the negative case after seeing its result |
| T3 | 0 for b → +A pulse → zero for z → −A pulse → zero for f; each pulse has ramp r / plateau h / ramp r | Same full duration with u=0; no reset between positive and negative pulses |

A concrete **development-only timing sketch** is b=1 s, r=.5 s, h=1 s, z=1 s, f=1 s: T1/T2 last 4 s, T3 lasts 7 s. Run T0 for the matching 4 s and 7 s baseline windows. These times are illustrative, not acceptance thresholds; lengthen them if settling, response, or slew limits require it and lock the final values before held-out tests.

For smooth ramps, interpolate endpoints using `s(v)=3v²−2v³`, `v∈[0,1]`. Its peak slew is `1.5*abs(Δu)/r`; select r to respect the calibrated rate bound. A smooth requested curve does not prove smooth realized torque. On contact loss, unacceptable joint margin/penetration/collision/saturation, or warnings, record the failure and stop/enter the predeclared fallback; do not continue to claim a successful plateau.

### 4. What the teammate returns

For every case, provide one self-contained result containing fixture/controller/threshold hashes, seed/reset checks, reproduction command, all failures, and these synchronized plots/metrics:

- **Command and response:** requested extra torque, baseline and total wrench proposals, allocated left/right wrenches, actual joint/pad commands, total measured contact Ty, and the paired incremental estimate with its valid time window. Show state divergence beside the estimate; long diverged trajectories are not a same-state measurement.
- **Object and human:** pitch/height/angular velocity, human force/wrench, drift/overshoot/settling. Human truth is evaluation-only; a fixed target may develop resisting force as the panel moves.
- **Physical constraints:** four-pad contacts/normal loads (tactile supports contact diagnosis here), joint-limit margins, actuator torque/saturation, unexpected collisions/penetration, numerical warnings, and left/right load/internal-force changes with the estimation method stated.
- **Verdict:** locked criteria, measured values, pass/fail/unverified and reason. Missing force allocation or an untrustworthy baseline separation means tracking_unverified. A changing tactile map, moving arm, or true assist_reference_executed flag alone is insufficient.

T0 should support without extra assistance. T1/T2 should show opposite signed **incremental torque responses**, not necessarily opposite final angles or identical gains. T3 should track changes and return additional torque to zero without losing contact; zero torque does not promise return to the starting angle. Explicitly separate torque settling from pose recovery. Successful T0 alone is not Control GO; all T0–T3 must pass their locked acceptance tests.

## Integration and commands

Use `uv sync --locked`. Default runs exercise interfaces/position hold, not the planned benchmark. New output paths are mandatory; viewer closes without recording; rendering needs ffmpeg. `simulation.py` and `sensing.py` own physics/measurements; `geometry.py` supports shared geometry/demo IK; `demos/` is scripted visualization, not Expert data.

```bash
# Inject controllers (CLI classes need no-argument constructors)
uv run --locked python -m phase1b.scripts.collect \
  --policy my_policy:Policy --object-controller my_control:ObjectController \
  --dual-arm-controller my_control:DualArmController \
  --output results/phase1b/dev-controller
# Existing contract smoke, tests, and viewer
uv run --locked python -m phase1b.scripts.collect --duration .12 --output results/phase1b/dev-contract
uv run --locked python -m unittest phase1b.tests.test_contracts phase1b.tests.test_integration -v
uv run --locked python -m phase1b.scripts.launch_viewer --scenario B
```

For constructor arguments, call `phase1b.pipeline.run_episode(output, scenario="B", duration=4, policy=..., object_controller=..., dual_arm_controller=..., max_assist_torque_nm=..., max_rotation_rate_rad_s=...)`. Bounds need explicit engineering justification, not automatic increases. Import/type/clock/bound errors abort.

```python
from phase1b.expert.reference import ExpertConfig, PublicSample, TeacherGoal, ReferenceGenerator

expert = ReferenceGenerator("E1", ExpertConfig(**explicit_configuration))
public = PublicSample.from_observation(observation_t)
candidate = expert.step(public, goal=TeacherGoal(public.timestamp, mode_t, goal_angle_t))
decision = candidate.decision  # Only this goes downstream; no hidden reference angle.
```

```bash
# Offline tools; placeholders require real inputs, not guessed teacher truth/limits
uv run --locked python -m phase1b.expert.data \
  --input PATH_TO_EPISODE --teacher-inputs PATH_TO_TEACHER_NPZ \
  --config PATH_TO_EXPLICIT_EXPERT_CONFIG_JSON \
  --episode-id UNIQUE_EPISODE_ID --output NEW_SHADOW_DIRECTORY
uv run --locked python -m phase1b.expert.learnability \
  --reference REF_SIDECAR_1 REF_SIDECAR_2 --query HELD_OUT_SIDECAR \
  --expert E1 --profile ft_object --history 20 --stride 10 \
  --radius 0.1 --torque-tolerance-nm 0.05 --min-neighbors 3 --output NEW_REPORT_JSON
```

Diagnostic CLI numbers are examples, not validated settings. Existing physical evidence belongs to `37b8c24`; strict episode validation requires its matching source hashes. Use that historical revision for old evidence or generate new outputs from current code; never rewrite old provenance. Scripted `motion/*/samples.npz` mixes truth/render state and is not a policy dataset. Historical documentation is available at `8fe6467`; current status and next acceptance criteria live only in the [README](../README.md).
