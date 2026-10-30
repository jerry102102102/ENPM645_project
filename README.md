# ENPM 645 — Intent-Aware Human–Robot Co-Manipulation

**Next milestone: demonstrate that the dual-arm controller can execute a small, known additional panel torque before collecting Expert demonstrations.**

A human and two Franka Panda arms jointly hold a panel. We want the robot to assist rotation, maintain a pose, or resist a disturbance without spoken commands. The first scope is small-angle rotation about World Y in MuJoCo, not a full flip or real painting/welding.

**Read this page for design, status, ownership, and the next acceptance tests.** Use the [Technical Reference](docs/REFERENCE.md) only when implementing: it contains the frozen hardware, exact interfaces, data fields, and commands. These are the two maintained team documents; literature and result files are supporting evidence.

## Design we are implementing

```text
Public sensor history ──► Expert / future learned policy
                                  │ AssistanceDecision: mode + additional torque
                                  ▼
                         Object-Level Controller
                                  │ ObjectCommand: pose / twist / assistance wrench
                                  ▼
                           Dual-Arm Controller
                                  │ ArmCommand: joint positions + pad forces
                                  ▼
                         MuJoCo ──► sensor feedback
```

- **Track A decides what assistance is wanted.** E0 follows estimated interaction torque; E1 uses the human goal after an observable initiation gate; E2 adds privileged disturbance compensation. E1 is the primary candidate teacher; E0 is a baseline and E2 an oracle reference. A student would later predict mode/torque from public sensor history.
- **Track B makes that request physically happen.** Object control defines compliant motion; dual-arm control coordinates joints, contact, support, and wrench allocation. Both layers must be implemented and validated, not just connected by types.
- **The shared contract is additional torque in Nm about World +Y at the instantaneous panel COM.** It excludes basic support. Current arm actuators accept joint-position targets in rad, not Nm. Translation, roll/yaw, weight support, and grasp stability remain lower-level responsibilities.
- Teacher-only goals/disturbance truth stay upstream. All experts and the future student use the same downstream controllers with public observations. Mode expresses intent: ROTATE may use opposing torque to brake; HOLD may use nonzero torque; UNCERTAIN requires zero assistance.

## Where we are now

Status checked against implementation revision `8fe6467`. This documentation update adds no controller or experiment result.

| Component | Available now | Still missing |
|---|---|---|
| Hardware | Two physically coupled Panda arms, four custom pads, one virtual hand, free rigid panel; Scenario 3.0.0 / Hardware 2.0.0 | No hardware redesign planned |
| Motion evidence | Contact support and scripted A/B/C rotation/lifting/load demonstrations | These do not validate intent recognition or assistance tracking |
| Interfaces and logging | Typed three-layer contracts; 100 Hz observations/references and 500 Hz arm/physics records | Benchmark-specific fixtures and acceptance reporting |
| Controllers | Reference-only object adapter + nominal position-hold arm adapter | Object motion/force control, allocation, verified extra-torque tracking |
| Expert references | E0/E1/E2 formulas, causal gates/limits, offline shadow labels; 9 arithmetic/data tests passed | Teacher callback in the physical runner; closed-loop Expert quality |
| Learning | Allowlisted inputs and neighborhood-consistency diagnostic tools | Valid demonstrations, measured learnability, trained policy |

Evidence: [physical tests and videos at 37b8c24](results/phase1b/current/README.md), [Expert arithmetic/data checks](results/phase1b/expert_math/verification.json). The retained physical report records 15 passed tests; the Expert update did not rerun physics. All formal supervision remains invalid. **Control GO, Expert GO, and Learning-Data GO are not yet established.** Not yet validated is not a failed benchmark run.

## Work split and delivery order

| Order / owner | Work | Deliverable / completion condition |
|---|---|---|
| **Now — Track B (controller teammate)** | Implement object-level and dual-arm control using the existing interfaces; expose support/assistance decomposition | Runnable T0–T3, numerical plots/report, contact/limit diagnostics, exact commands and commit; calibrated torque envelope |
| **In parallel — Track A (Jerry)** | Define benchmark fixtures/metrics, explicit teacher intent streams, and label-validity rules; prepare T4–T7 human/disturbance schedules | Task and acceptance definitions paired by task_id, development/held-out split, reproducible inputs; no label promotion |
| **Then — both** | Connect E1/E2 upstream of the same controllers; compare support_only / E0 / E1 / E2 under matched conditions | T4–T7 closed-loop evidence and honest failure reports; optional T8 |
| **Only after the gates — Track A** | Collect a small pilot (approximately 10–20 valid episodes), diagnose observation/label consistency | Learning-Data GO before offline learning and joint/tactile ablations |

**Track B is not responsible for building the learning model or collecting a large ML dataset.** Its immediate handoff is T0–T3 control capability. No MLP/GRU training, CG-DAgger, ViT/VLM integration, 90° flips, arbitrary 6D wrench learning, or real manufacturing tasks in this milestone.

## Next acceptance benchmark — v1.0, planned

Adapted from the team's Task & Test Benchmark v1.0. **All T-cases below are pending, not passed results.** Each needs both a task specification (initial state, human goal/schedule, disturbance, command, seed, permissions) and an acceptance test (measurements, baseline, locked tolerances, failure reasons, reproduction evidence).

T0–T3 bypass intent/Expert logic and inject deterministic test references. T4–T8 use actual spring-damper human forces and rule/Expert decisions. No prerecorded Panda joint trajectories or virtual root wrenches may stand in for control. Deterministic inputs support reproducibility, not exact realization of arbitrary torque.

| Case / priority | Known task input | Required evidence |
|---|---|---|
| **T0 — required** Support | HOLD, extra torque 0, human holds current position, no disturbance; run for several seconds | All four pads in contact; pitch/height drift, baseline load, joint/actuator limits, collisions and numerical warnings; no assistance execution |
| **T1 — required** Positive torque | Same initial state as T0; small +τ_test, ramp/hold/return to zero | Positive incremental pitch response, amplitude/gain, lag and saturation; total Ty is not assistance |
| **T2 — required** Negative torque | Symmetric −τ_test | Correct sign, positive/negative asymmetry and bias, maintained contact |
| **T3 — required** Command sequence | Smooth `0 → +τ → 0 → −τ → 0`, reproducible | Slew, overshoot, settling/return to zero, left/right loading and internal forces, continuous tracking and synchronized command records |
| **T4 — integration** Positive rotation | Human switches HOLD → candidate +10° goal; vary onset, no disturbance | Starts from actual human guidance, reasonable achieved angle and human effort; no fixed-time self-start |
| **T5 — integration** Negative rotation | Candidate −10° goal; vary onset | Both directions work without memorizing one direction or time schedule |
| **T6 — integration** HOLD | Hold current angle at several initial poses; small corrections, no large disturbance | No false rotation, final drift and effort measured, no unnecessary active motion |
| **T7 — integration** HOLD + sustained disturbance | Fixed human goal; varying ramp onset; begin below the old 20 N demo load | Compare pitch error and RMS/peak human force against support_only; maintain height, contact, and limits |
| **T8 — optional** Pause / resume / reverse | Interrupt, reverse, and resume human guidance at varying times | Response delay, unwanted continued effort, overshoot, and contact stability |

All amplitudes, rates, angles, durations, and acceptance tolerances require calibration. **±10° are candidate integration targets; ±0.02 Nm was only an interface-test bound, never a validated tracking envelope.** The prior world −Z 20 N demo force is off-COM and produces vertical load plus pitch/other torques. Scalar Y assistance cannot handle all of that; 20 N is a later stress test, not the starting requirement for T7.

### What counts as extra-torque tracking

1. Log the command decomposition: requested assistance, baseline wrench proposal, total requested wrench, allocated left/right wrenches, and actual arm commands. Missing computations stay unavailable. The current pipeline still hardcodes allocation/total-demand/tracking fields as unavailable; this needs a real diagnostic path and updated validation, not just a controller flag.
2. Pair short assist=0 and assist=±τ runs with matched or explicitly controlled q/dq, panel pose/velocity, human input, and disturbance conditions. Measure the incremental panel-contact torque response **and state divergence**. Long-run differences after trajectories diverge are not same-state assistance measurements.
3. Report sign, gain, lag, overshoot, saturation, and internal-force changes. `assist_reference_executed=True` means the request was used, not that it was tracked. If separation is not credible, report `tracking_unverified`; do not compare requested extra torque directly with total contact Ty and call it a pass.

### Lock acceptance before evaluation

- Use a small **development/calibration split** for T0–T3 to establish the reachable torque range, baseline noise, and safe contact envelope.
- Freeze test amplitudes and limits for sign consistency, tracking/incremental error, delay, motion error, contact, penetration, joint margin, collisions, saturation, and warnings **before held-out runs**. Record the engineering basis and threshold version/hash; missing thresholds mean not ready, not pass.
- Evaluate held-out seeds, onset times, and disturbance conditions without relaxing thresholds after viewing outcomes. T4–T8 need absolute physical constraints and measured improvement over the same-task support_only baseline (E0 where useful).
- Compare the same human goal, impedance, and disturbance configuration, while allowing actual human force to change with closed-loop feedback. Report angle achievement, false starts, pitch/height error, overshoot/recovery/pause drift, human RMS/peak force, and `∫ ||F_h(t)||² dt` as a force-burden proxy, **not metabolic energy**. Retain failed episodes and reasons.

| Gate | Evidence required | Current state |
|---|---|---|
| Control GO | T0–T3 pass pre-locked reference-tracking, contact, and physical-limit tests | Pending |
| Expert GO | T4–T7 show executable teacher behavior and measurable cooperation benefits | Pending |
| Learning-Data GO | Valid teacher execution/quality, causal synchronized data, initial observation–label consistency | Pending |

If small extra torque cannot be tracked with position servos, report Control NO-GO with the smallest justified controller/actuator change. Changing actuator type or contact hardware requires a versioned model and new physical acceptance, not an unreported workaround.

### Concrete next coding deliverable

Build a T0–T3 runner using the existing pipeline, a machine-readable benchmark configuration pairing tasks/tests by `task_id`, and a versioned locked-threshold artifact. Keep task schedules and metrics together rather than duplicating them across documents. These artifacts **do not exist yet**; this page is their shared specification.

Every case must save its seed, initial q/dq, settled panel pose, human/force onset, actual controller/expert versions, code/model hashes, source data, thresholds, metrics, pass/fail (or unverified), and failure reasons. Later integration must add explicit `ground_truth_intent` in the teacher-only stream; do not infer it from A/B/C. Formal demonstrations require Expert + Controller closed-loop execution; annotating a scripted/support-only log does not qualify.

## Start here in the repository

```bash
uv sync --locked
# Pure Expert arithmetic/data checks; no MuJoCo stepping
uv run --locked python -m unittest phase1b.tests.test_expert -v
# Existing physical regression/demo suite; this is NOT a T0–T8 benchmark runner
uv run --locked python -m phase1b.scripts.run_suite --output results/phase1b/reproduction --render
```

Python 3.12.11, MuJoCo 3.3.5, local CPU, no Docker. Use a new output path; rendering requires ffmpeg and `--render` can be omitted. Controller injection, viewer, and offline tools are in the [Technical Reference](docs/REFERENCE.md#integration-and-commands).

## Working agreement

Maintain one implementation in `phase1b/` and keep these two documents in English. Update this page when a gate or ownership changes; update the technical reference only when a contract changes. Use branches/PRs, run relevant checks, and commit `pyproject.toml`/`uv.lock` together for dependency changes. Keep experiment evidence intact; record exact producing commits rather than relabeling old results. Earlier documents remain recoverable in Git at `8fe6467`; pre-cleanup experiments at `996a524`.

[Optional literature](01_literature/README.md) is background, not required reading before T0–T3. Only actual readers fill in reading scope/date; AI summaries are not completed team readings.
