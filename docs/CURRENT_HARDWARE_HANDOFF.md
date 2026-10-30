# Current Hardware — Dual-Panda Panel Co-Manipulation

Reference for planning data collection, training, and evaluation. **The current system is simulation only; there is no physical robot deployment or trained model.**

Scenario `3.0.0` / Hardware `2.0.0`; MuJoCo `3.3.5`, Python `3.12.11`, and uv; no Docker.

## Geometry and physical coupling

| Component | Fixed configuration |
|---|---|
| Arms | Two MuJoCo Menagerie Franka Panda arms, 7 joints each, with bases fixed to the world |
| Panel | Rigid cuboid, 0.60 × 0.20 × 0.02 m, 1 kg; initial COM `[0,0,0.80]` m, approximately +0.336° about Y |
| Panel local frame | Origin at the panel COM; X along the long dimension, Y along the short dimension, Z through the thickness |
| World frame | Z up; gravity `[0,0,-9.81]` m/s² |
| Left/right bases | World `[-0.45,+0.65,+0.25]` / `[+0.45,+0.65,+0.25]` m; both yaw −90° |
| Left/right nominal grasps | Panel-local `[-0.20,+0.10,0]` / `[+0.20,+0.10,0]` m; on the same long edge, 0.40 m apart |
| Human interaction point | Panel-local `[+0.27,-0.10,0]` m; on the opposite long edge |
| Custom grippers | One upper and one lower pad per arm, 4 pads total; not the stock Panda fingers |
| Each pad | 0.09 × 0.14 × 0.02 m, 0.10 kg; slides along gripper-local Z |
| Pad preload | Left upper/lower, right upper/lower: `[-40,+40,-40,+40]` N; motor commands, not fixed contact forces |
| End-effector connection | Flange at link7-local Z +0.107 m; gripper offset from flange by Z +0.12 m and rotation X −90°; each arm has an additional 0.08 kg adapter |

The panel has a freejoint; the arms and grippers do not. No weld fixes the panel, and no virtual wrench at a gripper root supports its weight. Loads pass through arm actuators and pad contacts. Nominal grasps are fixed in the panel local frame; the simulator solves actual contact locations.

Panda inertias, joint limits, and collision geometry are retained. Collision checks are subject to the model's exclusion settings. The first runtime friction coefficient is `1.0482960672343025`; load the scene through the Python entry points.

## Human hand and external force

The virtual human is a spring-damper at a single interaction point, **without a full human arm or articulated fingers**. Force depends on target versus actual position and velocity: stiffness approximately 98.21 N/m, damping approximately 7.30 N·s/m, force limit 50 N, and target-speed limit 0.35 m/s.

An independent external force acts on the panel at the current panel-local point `[+0.063691,-0.015331,0]` m. Human and disturbance forces, application points, and wrenches about the panel COM are logged separately. Their sources and targets are simulation ground truth.

## Available sensors and states

Shapes below exclude time. Arm order is left/right; joint order is 1…7.

| Data | Shape / definition |
|---|---|
| Left/right tactile | Each `(2,3,4,4)`: upper/lower × sensor-local z/x/y force × 4×4 angular bins; N/bin, not Pa or camera images |
| `wrist_ft_left/right` | Each `(6,)`; panel-on-gripper wrench reconstructed from panel-pad contacts; gripper-local, about the gripper origin |
| `wrist_sensor_left/right` | Each `(6,)`; MuJoCo wrist transmitted wrench, including downstream weight and dynamics; uncompensated and not interchangeable with the row above |
| Object contact wrench | `(6,)`; gripper-on-panel, world frame, about the instantaneous panel COM |
| Arm q / dq / actuator torque | Each `(2,7)`; rad / rad/s / Nm; actuator torque is not external contact torque |
| End-effector pose / velocity | `(2,7)` world xyz+wxyz / `(2,6)` world angular xyz→linear xyz; the EE is between the two pads, not link7 |
| Gripper position / velocity | Each `(2,2)`; m / m/s; 4 pad actuator forces are also recorded in N |
| Object state | World position, quaternion wxyz, and linear/angular velocity; read directly from simulation, not estimated by vision |
| Contact state | Each side `(2,)`; upper/lower contact information |

Wrenches use `[Fx,Fy,Fz,Tx,Ty,Tz]`. Tactile sensing uses `mujoco.sensor.touch_grid`; heatmaps display the normal channel, while raw data retain all three channels. Synchronized videos can be rendered. There is no VLM, RGB-D system, or visual pose estimator yet.

## Control and logging

- Physics/control: 500 Hz; high-level reference/observation: 100 Hz; 2-second settling period before each episode.
- Formal transition: `o_t → reference → 5 control/physics substeps → o_next`.
- Commands: `arm_position_target (2,7)` in rad; `pad_force_command (4,)` in N. Torque bounds per arm are ±87 Nm for joints 1–4 and ±12 Nm for joints 5–7. This is a position-servo model, without a direct joint-torque adapter.
- High-level interface: `AssistanceDecision → ObjectCommand → ArmCommand`. Additional assistance is a scalar in Nm about World +Y at the instantaneous panel COM. **Only reference-only / position-hold adapters exist. Assistance tracking and allocation are not implemented; the new [E0/E1/E2](EXPERT_DATA_DESIGN.md) generators compute candidate references without validated execution.** See the [Three-Layer Contract](THREE_LAYER_INTERFACE.md).
- q/dq are logged but excluded from policy inputs by default; `with_joint_states` enables them. Privileged truth, commands, and render states do not automatically become policy inputs. Expert labels remain NaN / invalid.

## Validated motions

A: rotate approximately +20°, lift 3.05 cm, then hold. B: lift 7.98 cm, then hold. C: rotate −11.94°, then withstand a world −Z force of 20 N during 4.3–8 seconds. Each demonstration lasts 8 seconds and maintains contact at all four pads, without unexpected penetration beyond the threshold, numerical warnings, or actuator saturation; 168 arrays replay exactly.

**These are physical demonstrations of predefined joint trajectories, not intention-recognition results.** The specified trajectories were validated; arbitrary poses, 90° flips, and physical robots were not. Demo `samples.npz` files mix in ground truth and must not be used directly as formal policy datasets.

## Exact specifications and entry points

[Scenario / Initial q / Human Parameters](../phase1b/config/dual_panda_v1.json) · [Full Hardware and Limits](../phase1b/config/hardware_v2.json) · [Data Schema](DATA_SCHEMA_V2.md) · [Controller Interface](CONTROL_HANDOFF_ARMS.md) · [Motion and Validation](../results/phase1b/current/README.md) · [Run Instructions](../phase1b/README.md)
