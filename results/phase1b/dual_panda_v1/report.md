# Dual Panda integration report

- Arm Model Integration: **PASS**. Two fixed Panda bases, 14 arm hinge joints, four custom pad slides; the only freejoint belongs to the panel. Initial IK is within 1 micrometer and 1e-5 rad; no initial penetrations beyond 1e-5 m.
- Physical Coupling: **PASS for these smoke episodes**. Arms are driven by upstream bounded position servos; pad motors preload the object. No root wrench drive, panel weld, kinematic hand following or qpos overwrite during stepping.
- Data Pipeline: **PASS**. 1,600 transitions at 100 Hz / 8,000 physics steps; same-time joint states, sensing and command/realized events; all four episodes replay exactly. q/dq logging is enabled, default policy use is disabled.
- Control Readiness: **joint-position adapter ready; force allocation, assistance tracking, direct joint-torque adapter and Expert not implemented**. Signed desired-assist references deliberately leave the hold baseline unchanged.

## Physical measurements

| Episode | Minimum contacts [L upper,L lower,R upper,R lower] | Max pad penetration mm | Minimum arm joint-limit margin rad | Saturated arm steps |
|---|---|---|---|---|
| A | [3, 4, 3, 4] | 0.476 | 0.1482 | 0 |
| B | [4, 4, 4, 4] | 0.340 | 0.1482 | 0 |
| C | [2, 2, 3, 4] | 0.523 | 0.1482 | 0 |
| reference_test | [4, 4, 4, 4] | 0.340 | 0.1482 | 0 |

All reported episodes have no unexpected penetrations beyond 1e-5 m, no warnings, and finite states. Panel force-balance residuals and exact replay counts are in `verification.json`. Arm velocities/torques respond to loads, but the high-gain hold baseline intentionally allows little motion; this is not a cooperative flip controller. Checks cover the sampled four-second episodes, not arbitrary trajectories or real hardware.

## Coupling controls

- Pad contacts removed: panel drops 0.322 m in 0.30 s; both tactile maps and reconstructed contact F/T become zero.
- Arm actuators disabled: panel drops 0.323 m in 0.30 s; arm joints move and actuator torque is zero.
- Human force removed while joint support remains: robot vertical contact force is 9.8100 N for a 9.81 N panel weight.

Negative controls intentionally alter fresh test instances and are not nominal datasets. Test output is in `tests.txt`; old Phase 1A source/models/results are retained. The mount adds 0.08 kg per custom gripper and changes its attachment; this is Hardware v2 / Scenario v3, not the previous dynamics.

## Reproduce

```bash
uv sync --locked
uv run --locked python -m phase1b.scripts.run_suite --output results/phase1b/reproduction --render
```

See `phase1b/README.md`, `docs/DATA_SCHEMA_V2.md`, and `docs/CONTROL_HANDOFF_ARMS.md` for frame conventions and integration. Rendering needs local ffmpeg. The vendored Menagerie revision and Apache-2.0 license are preserved.
