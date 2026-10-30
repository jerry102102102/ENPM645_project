# Expert & Data Design v1.0

This version implements **E0/E1/E2 mathematical reference generators, offline shadow annotation, and sensor-history/label consistency diagnostics**. It uses the single `phase1b` implementation, Scenario 3.0.0 / Hardware 2.0.0. No physical simulation, demonstration collection, or model training was performed for this update.

| Status | Conclusion |
|---|---|
| Reference / Interface Ready | Yes; pure Python/NumPy, producing `AssistanceDecision` |
| Controller Implemented | Still only reference-only / position-hold adapters |
| Assistance Tracking Validated | No; formula outputs do not establish dual-arm execution capability |
| Expert Labels Validated | No; execution_valid and supervision_eligible are both false |
| Learnability Validated | No; diagnostic tools exist, but experimental data have not been evaluated |

## Physics and units

All torques are in **Nm about World +Y at the instantaneous Panel COM**. Angles are in rad, angular velocities in rad/s, angular accelerations in rad/s², and time in episode seconds.

The full rigid-body relation is `I_W α + ω × (I_W ω) = τ_h + τ_d + τ_r`, with `I_W = R I_body Rᵀ`. `required_assistance_y()` retains gyroscopic coupling and subtracts human, disturbance, and base-controller torque to obtain the additional demand. This is an inverse-dynamics requirement, not proof that contacts, friction, joints, or position servos can realize it. Uniform gravity produces no direct torque about the COM; torques from support contacts at the edges must still be included.

`panel_inertia()` uses cuboid inertia. For a 1 kg, 0.6×0.2×0.02 m panel, `J_y = 0.03003333 kg·m²`. The single-axis approximation is `J_y θ̈ = τ_h + τ_d + τ_b + τ_a`. Ideal exact inverse dynamics with positive PD gains gives `ë + k_d ė + k_p e = 0`; this is not a stability guarantee for the current dual-Panda system.

`human_pitch_impedance(k_h,d_h,r_world)` uses `J_h = e_y × r_world` to obtain `K_θ = k_h ‖J_h‖²` and `D_θ = d_h ‖J_h‖²`. Hidden human goals affect forces and motion through impedance, but external disturbances may produce the same net force. Unique recovery of the goal is not guaranteed.

The reference generators use a single-axis approximation: pitch=`atan2(-R20,R00)` and ω=`ω_world,y`. These are not an exact derivative pair under arbitrary 3D motion. `off_axis_omega_norm` is diagnostic, not automatic certification of the approximation. This version assumes small-angle operation near the Y axis without crossing ±π. Full 3D and contact validation require later closed-loop tests.

## Three reference generators

Code: [`phase1b/expert/reference.py`](../phase1b/expert/reference.py). Create a separate `ReferenceGenerator` for every episode and expert. Inputs must be consecutive at 100 Hz; missing/repeated timestamps and future teacher inputs are rejected. `confidence=None`.

| Strategy | Available information | Output before limiting |
|---|---|---|
| E0: reactive baseline | Public F/T, EE pose, object pose/ω; rejects goal/disturbance truth | `k_f LPF(J_y α_hat − τ_L − τ_R)` |
| E1: primary candidate teacher | Same public state + current teacher mode/goal; rejects disturbance truth | ROTATE: `J_y α_ref + Kp_R(θ_ref−θ) + Kd_R(ω_ref−ω)`; HOLD: `Kp_H(θ_hold−θ)−Kd_H ω` |
| E2: oracle reference | E1 information + current disturbance truth | `raw_E1 − λ_d τ_d`, with `0≤λ_d≤1`; UNCERTAIN takes priority and sets output to zero |

E0 estimates α_hat using a backward difference. Its LPF coefficient is `1−exp(−dt/T)`, initialized with the first valid estimate. The first sample has no previous velocity: it produces invalid UNCERTAIN/0, with NaN + false masks for unavailable estimates. E0's mode is a reactive direction classification, not true intent recognition.

Public contact-reconstructed F/T is **panel-on-gripper, gripper-local, about the gripper origin**. Negate and rotate it to robot-on-panel, then add `(p_gripper−p_COM)×F`; sum the two arms. Raw wrist transmitted wrench cannot be inserted directly into this formula because it also includes downstream weight/dynamics. E0 needs public EE pose for frame conversion. This is additional robot-kinematics information and must be disclosed when comparing against a student receiving only local F/T + object state.

E1 enables ROTATE only when public estimated torque reaches the on threshold, angular velocity is directionally consistent, this persists for confirm_seconds, and the direction matches the teacher mode. On activation, its internal reference starts at the measured angle and approaches the current goal by at most `reference_rate_max*dt` per update. Torque or angular velocity above the off threshold in the original direction can sustain activation. If both are insufficient for release_seconds, the gate releases. Opposing torque reaching the on threshold immediately cancels the old direction; reversal requires renewed confirmation. This is only an engineering gate: external forces may still trigger it, so it does not establish separation of human input from disturbances.

HOLD latches the measured angle on entry and retains it. UNCERTAIN does not advance the reference and immediately sets assistance torque to zero. E1/E2 internal references are not passed to the Object Controller. E1 does not fully cancel human and disturbance forces. E2's λ_d must also be coordinated with the lower layer to avoid duplicate compensation. In a simplified static model, displacement is `(1−λ_d)τ_d/K_sum`, valid only under assumptions including positive total stiffness, no saturation, and equilibrium.

### Limits and modes

`ExpertConfig` requires explicit gains, gate settings, torque_max, torque_rate_max, reference_rate_max, and `limits_provenance`. No default numbers are presented as physically achievable. Kp is in Nm/rad and Kd in Nm·s/rad; k_f and λ_d are dimensionless; LPF/confirm/release times are in s; torque and angular-velocity thresholds are in Nm and rad/s.

Order: raw → amplitude clip → slew clip. Initial previous torque is zero. **UNCERTAIN / gate cancellation takes priority and immediately zeros output**, overriding the slew bound if necessary and logging `rate_limit_overridden=true`. The lower layer must still handle physical smoothing and contact safety. This flag must not be interpreted as passing the slew constraint.

**Interface v1.1 semantic correction: ROTATE_POS may output negative torque for braking, and ROTATE_NEG may output positive torque.** Mode expresses intent; torque sign expresses the applied direction. HOLD may be nonzero; UNCERTAIN must be zero. The existing type is updated without retaining a separate old branch.

## API and downstream responsibilities

```python
from phase1b.expert.reference import ExpertConfig, PublicSample, TeacherGoal, ReferenceGenerator

config = ExpertConfig(**explicit_configuration)  # All fields are in the dataclass; include limit provenance.
expert = ReferenceGenerator("E1", config)
public = PublicSample.from_observation(observation_t)
candidate = expert.step(public, goal=TeacherGoal(public.timestamp, mode_t, goal_angle_t))
# Once the controller is integrated, only this crosses the downstream boundary:
decision = candidate.decision
```

TeacherGoal / DisturbanceTruth are separate privileged streams, timestamped exactly with the current observation, not PolicyHistory fields. E0 accepts neither; E1 does not accept disturbance truth. The current ControlChain student policy remains allowlisted; oracle access has not widened its permissions. This version does not connect a teacher callback to the physical runner. The next stage needs an explicit teacher-only upstream call site while Object/Arm Controllers continue to receive only public inputs and the decision.

E0/E1/E2 and the future student must share the same Object Controller, Dual-Arm Controller, limits, and public observations. Never pass `reference_angle`, goals, scenarios, or future commands downstream through a hidden channel. Candidate torque exceeding ControlChain's current .02 Nm **test bound** is still rejected. Increasing that constant is not tracking validation. Position servos accept rad, not Nm.

## Data contract: original episode + shadow sidecar

The only physical writer remains `phase1b.pipeline`. This version's `phase1b.expert.data` reads original files and writes a separate new directory, refusing an existing output path. It does not rewrite existing observations, commands, privileged data, or supervision.

| File | Fields / purpose |
|---|---|
| metadata.json | episode_id, dataset version, seed, annotation commit/dirty flag, source commit (null if unknown), schema/MuJoCo/scenario/hardware versions, model/config/hardware/source/code/output hashes |
| metadata.json: physics | Panel mass/principal inertia/dimensions/friction; three panel-local grasps; human stiffness/damping/force/speed limits; controller components and position-servo type; dt/duration/settle; frames |
| privileged_metadata.json | Teacher-stream path/hash; scenario; exact human target/ramp and disturbance point/magnitude/start/end; excluded from the student |
| derived_features.npz | timestamp, theta_y, omega_y, delta_theta_y, acceleration_y, interaction_torque_estimate, tau_left_y, tau_right_y, off_axis_omega_norm; masks for unavailable estimates |
| expert_internal/E0.npz, E1.npz, E2.npz | Candidate torque/mode, expert_id/source, gate/direction, reference angle/velocity/acceleration, raw/amplitude/output torque, saturation/slew/override, label status/reason, and three validity levels |
| supervision/E*.npz | Formal expert_assist_torque_ref is **all NaN**; expert_mode is retained for analysis but mode_valid/torque_valid/label_valid are **false**; candidate_valid is separate; execution_valid/supervision_eligible are **false** |

`candidate_valid` means only that the numerical computation is available. It does not guarantee observability, executability, stability, or learnability. The internal trace's `expert_mode_valid` also applies only to the candidate and must not replace a supervision mask. Treat the entire expert_internal directory as teacher-internal data. Diagnostics extract only the target being analyzed, not the remaining fields as input features.

Metadata comes from fixed model/config/hardware files whose hashes match the source. Unknown controller version, source commit, and disturbance magnitude_range remain null. episode_valid=null and failure_reason states that the episode was not revalidated. Protocol values in the fixed configuration describe the source; the explicitly supplied teacher stream is authoritative for the exact annotation. Once a collector supports randomized conditions, record actual per-episode parameters rather than substituting the fixed configuration.

Observations, arm commands, and post-step feedback remain linked by the original episode timestamps; the sidecar binds their source files by hash. Requested assistance is in object_commands. Requested total wrench / allocation / assistance tracking are currently unavailable. Actual total contact wrench is not additional assistance.

### Offline usage (not run on experimental data in this update)

A separate teacher-input NPZ is required, containing timestamp `(N,)`, ground_truth_intent `(N,)` (strings from the four modes), human_goal_angle `(N,)` in rad, and external_disturbance_wrench `(N,6)` in world force→torque order about the instantaneous panel COM. These must align exactly with observations. Never infer intent automatically from A/B/C. Existing records lack explicit ground_truth_intent, so the tool refuses to use them directly as complete teacher input.

```bash
uv run --locked python -m phase1b.expert.data \
  --input PATH_TO_EPISODE --teacher-inputs PATH_TO_TEACHER_NPZ \
  --config PATH_TO_EXPLICIT_EXPERT_CONFIG_JSON \
  --episode-id UNIQUE_EPISODE_ID --output NEW_SHADOW_DIRECTORY
```

Required Config JSON fields are fully defined in `ExpertConfig`. The test file includes a named fixture **for arithmetic testing only, without validated hardware reachability**. Each annotation saves the full configuration and limits_provenance. No default configuration is supplied that could be mistaken for an experimental setting.

`validate_sidecar()` checks source/teacher/code/output hashes, clocks, finiteness/bounds, UNCERTAIN, slew exceptions, and supervision isolation. This checks data integrity, not physical quality. Moving data requires preserving or resolving the recorded absolute source paths.

## Label learnability: diagnostics only

`phase1b.expert.learnability` builds causal sensor windows `[t−H+1,t]` and extracts the current candidate target. Three profiles are available:

1. `ft_object`: left/right F/T + object position/quaternion/linear/angular velocity.
2. `ft_object_joints`: adds q/dq.
3. `ft_object_joints_tactile`: additionally includes tactile from all four pads.

The F/T source must be consistent: contact_reconstructed by default, or explicitly wrist_transmitted. Metadata, timestamps, IDs, goals, ground-truth forces, expert references, and future observations/commands are not features. Quaternion signs are canonicalized; their Euclidean distance remains a simplifying approximation, not an SO(3) geodesic.

Scaling uses per-dimension standard deviations from the reference split; constant dimensions use one native unit. Neighbors satisfy standardized RMS distance <= radius, with the same episode excluded. The sidecar CLI uses the original observation hash as the group, so renaming cannot create an independent sample. Experimenters must still split by seed, human conditions, and disturbance protocol to prevent leakage from nearly duplicate runs.

Reports include coverage, local label std/range, error relative to the neighbor mean, and inconsistency/sign-conflict rates above a torque tolerance. The minimum neighbor count counts windows, **not independent episodes**. Increase independent episodes first, then compare history/radius/profile while reporting coverage. The same radius in different dimensions does not ensure comparability; a lower std alone does not establish a tactile benefit.

Insufficient matching returns `insufficient_matched_evidence` and metrics=null, not zero. Finite-neighborhood variation is a diagnostic proxy, **not** true `Var(Y|O)`, Bayes irreducible error, or proof of learnability. Analyze E1/E2 separately. Even correct E2 oracle reactions may compensate before the student has any observable signal.

```bash
uv run --locked python -m phase1b.expert.learnability \
  --reference REF_SIDECAR_1 REF_SIDECAR_2 --query HELD_OUT_SIDECAR \
  --expert E1 --profile ft_object --history 20 --stride 10 \
  --radius 0.1 --torque-tolerance-nm 0.05 --min-neighbors 3 \
  --output NEW_REPORT_JSON
```

These window/radius/tolerance values only demonstrate the CLI; they are not validated on data. Each set is limited to 500 windows to bound exact neighbor scanning. Adjust stride first, then replace the index if later data scale requires it. This tool does not fit or predict with a learned model.

## Next stage and verification for this update

Choose gains/limits with documented provenance. The teammate must then implement requested → allocated → executed → realized assistance tracking, pitch-control responsibility, and contact validation. Subsequently compare E0/E1/E2 in closed loop under the same human goal/impedance/disturbance conditions. Recompute human force from actual panel state rather than replaying fixed human forces.

Formal quality gates include false starts, drift after stopping, pose deviation, contact/joint limits, saturation, tracking error, and `∫‖F_h‖²dt` as a proxy for human force burden, not energy expenditure. Begin with a pilot of approximately 10–20 valid episodes. Controllability, observability, and learnability must be tested separately; none substitutes for another.

```bash
# Only this arithmetic/data test was run for this update; it does not load MuJoCo.
uv run --locked python -m unittest phase1b.tests.test_expert -v
```

All 9 tests passed. Coverage includes rigid-body/hand equations, F/T frames, causal differences/LPF, gating/pause/reversal, braking, HOLD latching, E2 λ, bounds/safety cancellation, shadow metadata/validity/hashes/overwrite rejection, three sensor profiles, and cross-episode diagnostics. Tests use fixed inputs that can be checked by hand, without time-integration simulation. See [Execution Evidence](../results/phase1b/expert_math/verification.json).

Model, hardware/config, and original experimental arrays are unchanged. Existing physical results belong to `37b8c24`. Because this update changed interface source code, use that revision for the old episodes' strict source-hash validator; do not fabricate updated hashes. Physical regression is deferred until a later task permits simulation.
