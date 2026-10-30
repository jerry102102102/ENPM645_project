# Track A / Track B — One Three-Layer Interface

Start with the [Three-Layer Contract](THREE_LAYER_INTERFACE.md). Import all current types from `phase1b.interfaces`; the only collection entry point is `phase1b.scripts.collect`.

| Development task | Input → output |
|---|---|
| Track A Policy / candidate Expert | `PolicyHistory → AssistanceDecision` |
| Object-Level Controller | `AssistanceDecision + ArmControllerInput + dt=.01 → ObjectCommand` |
| Track B Dual-Arm Controller | `ObjectCommand + latest ArmControllerInput → ArmCommand` |

Controller observations are read-only copies of public sensing: q/dq, EE pose, F/T, tactile, and object state. Nominal q, pad preload, and actuator command limits are also available. Human goals, scenario labels, disturbance truth, future states, and live MuJoCo handles are excluded.

## Integration

Custom classes must provide the predict/step methods above. The CLI uses constructors without arguments. You may replace just the layer under development and retain the reference-only / position-hold adapters for the others:

```bash
uv run --locked python -m phase1b.scripts.collect \
  --policy my_policy:Policy \
  --object-controller my_control:ObjectController \
  --dual-arm-controller my_control:DualArmController \
  --output results/phase1b/dev-controller
```

Use Python when constructor arguments are needed:

```python
from phase1b.pipeline import run_episode
run_episode("results/phase1b/dev-controller", scenario="B", duration=4,
            policy=my_policy, object_controller=my_object_controller,
            dual_arm_controller=my_arm_controller,
            max_assist_torque_nm=.02, max_rotation_rate_rad_s=.5)
```

These two bounds are interface test settings, not safety certification. Import, type, clock, and bound errors abort execution without silently falling back to a baseline.

## Existing ArmCommand

```python
from phase1b.interfaces import ArmCommand

class DualArmController:
    def step(self, command, observation):
        # Only a hold adapter; does not execute assistance or object motion.
        return ArmCommand(
            arm_position_target=observation.nominal_arm_position.copy(),
            pad_force_command=observation.nominal_pad_force.copy(),
            controller_mode="position_hold_baseline",
            assist_reference_executed=False,
        )
```

Arm commands are `(2,7)` in **rad**; pad commands are `(4,)` in **N**. Do not put object-level Nm into position targets. Assistance about World +Y / panel COM is separate from basic support; nonzero Ty must not be added to an independent high-stiffness pitch position loop. Direct torque actuation requires a new hardware/controller version.

The current reference-only adapter latches the current pose only on entry to HOLD; ROTATE produces no motion. The dual-arm adapter only holds nominal q. Full admittance, IK/Jacobian execution, wrench allocation, contact control, bounds/smoothing, and assistance tracking remain unimplemented. Scalar AssistTorqueReference retains only explicit data conversion; the old step(reference, inp) execution branch was removed to maintain one version.

After changing controllers, validate support versus assistance responsibilities, requested → allocated → commands → realized, contact/collision/limits/saturation, and tracking error. `assist_reference_executed=True` is only a flag that the request was used; it does not establish tracking or Expert quality.

[Hardware](CURRENT_HARDWARE_HANDOFF.md) · [Data Schema and Timing Events](DATA_SCHEMA_V2.md) · [Test Results](../results/phase1b/current/verification.json)

## Integrating Expert candidates

The [E0/E1/E2 Design and API](EXPERT_DATA_DESIGN.md) can compute `candidate.decision`, but are not yet connected to the physical runner. TeacherGoal/DisturbanceTruth remain in a separate upstream channel. Teammates still receive only AssistanceDecision, not the expert reference angle. Opposing torque in ROTATE may be braking and must not be rejected solely for its sign; UNCERTAIN still requires zero. All three experts and the student must share the same downstream logic.

ExpertConfig torque_max/rate_max must come from controller validation. There are currently no certified values, and adding generators does not make tracking or labels valid.
