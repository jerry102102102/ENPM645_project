# Articulated Data Schema — enpm645-articulated-transition/1.0

這是新的資料格式，適用 Scenario 3.0.0 / Hardware 2.0.0。舊 Phase 1A schema、檔案與結果不修改；不得用零填補舊資料不存在的 Joint States。入口為 `phase1b.pipeline.read_episode / validate_episode`。

## 時間與命令順序

每筆 transition 是：`o_t → reference → 5 個 500 Hz commands/physics steps → o_next`。settle 2 秒後 episode time 歸零，engine timestamp 保留原時鐘。`physics_tick` 與 index 同步，100 Hz 的相鄰 samples 相隔五個 physics ticks。

- `observations.npz`：決策前、上一個命令仍作用的狀態，timestamp = t。
- `controller.npz`：五個 ordered 子步。`command_timestamp` 是該子步決策時刻；`*_at_actuation` 是新命令設定及 forward 後、step 前；`realized_timestamp` 與 `*_after_step` 是 step 後。
- `next_observations.npz`：t + 0.01 s；除末筆外，與下一筆 observations 精確相同。不得用於當下 policy。
- `privileged.npz`：t 時刻的人手目標、來源、外力與作用點；供評估和影片標註，不提供 controller/policy。
- `supervision.npz`：expert torque 全為 NaN，valid 全 false。
- `diagnostics.npz`：500 Hz 接觸數、穿模、關節界限、panel 力平衡、警告及外加力稽核。
- `render_states.npz`：t 的完整 MuJoCo qpos/qvel，僅供 native FK 畫面與資料驗證，不是 policy input。

## Observation 新增欄位

以下 shape 不含最前面的 N；arm 順序固定 `[left, right]`，每臂 joints 1…7。`metadata.json` 記錄精確 joint names；不要假設 qpos index 等於 joint id。

| 欄位 | Shape | 單位／定義 |
|---|---|---|
| `arm_joint_position` | (2,7) | rad；qpos 經 named joint qposadr 取得 |
| `arm_joint_velocity` | (2,7) | rad/s；qvel 經 named joint dofadr 取得 |
| `arm_actuator_torque` | (2,7) | Nm；qfrc_actuator 在 arm DOFs 的 generalized torque，不含其他 generalized force |
| `arm_actuator_force` | (2,7) | 原始 actuator_force；本版本 unit gear hinge transmission，數值等於上述 Nm，但保留不同語意 |
| `gripper_joint_position` | (2,2) | m；每個 upper/lower pad slide 位移 |
| `gripper_joint_velocity` | (2,2) | m/s |
| `gripper_actuator_force` | (4,) | N；L upper/lower、R upper/lower |
| `end_effector_pose` | (2,7) | custom gripper frame 的 world xyz + wxyz；等於本版 gripper_pose，不是 Panda link7 frame |
| `end_effector_velocity` | (2,6) | world angular xyz、linear xyz；rad/s、m/s |
| `wrist_sensor_left/right` | (6,) | MuJoCo wrist site force/torque，N/Nm，下面另述 |

原本 object_position/orientation、linear/angular velocity、左右 tactile、contact state 與 contact-wrench F/T 保留。另保存 object_pose/velocity 的合併形式供記錄／視覺共用。

## 兩種 F/T 必須區分

- `wrist_ft_left/right`：沿用 Phase 1A 的 **panel-on-gripper 接觸 wrench 重建**，gripper-local、about gripper origin，`[Fx,Fy,Fz,Tx,Ty,Tz]`。只納入 panel 與對應 pads 的接觸。
- `wrist_sensor_left/right`：新加入的 MuJoCo force/torque sensor，位於 custom gripper local `[0,+0.12,0]`，sensor axes 與 gripper 一致。它是 gripper child 對 parent tool flange 的傳遞 wrench（官方 child→parent convention），包含下游手部動力學，未做重量／慣性補償；不可直接替換上述 contact wrench。
- `wrist_ft_*_world_at_panel_com` 與 `object_contact_wrench`：gripper-on-panel、world、about instantaneous panel COM。

[MuJoCo force/torque 官方定義](https://mujoco.readthedocs.io/en/3.3.5/XMLreference.html#sensor-force)。tactile 仍為各 pad 的 sensor-frame z/x/y、4×4 angular bins，N/bin；不是 Pa。

## Controller 記錄

| 欄位 | 每 transition shape | 語意 |
|---|---|---|
| `desired_assist_torque` | scalar | Nm；World +Y、Panel COM、額外 assistance，source=scripted_test |
| `arm_position_target` | (5,2,7) | 實際送入 position servo 的 rad 命令 |
| `pad_force_command` | (5,4) | 實際送入 pad motors 的 N 命令 |
| `arm_requested_servo_torque` | (5,2,7) | 模型 gain/bias 計算出的 clipping 前 servo torque，Nm |
| `arm_actuator_torque_at_actuation` | (5,2,7) | 新命令作用後、step 前的實際 generalized actuator torque，Nm |
| `arm_actuator_torque_after_step` | (5,2,7) | step 完成並 refresh 後的 torque，Nm |
| `arm_force_saturated` | (5,2,7) | torque 經模型 force limit clipping 的 flags |
| `realized_object_contact_wrench` | (5,6) | 每子步完成後的總 panel 接觸 wrench，不等於額外 assistance |
| `assist_reference_executed` | (5,) | hold baseline 全 false；不等於 tracking 成功旗標 |
| `tracking_error_Nm / tracking_error_valid` | scalar | NaN / false，尚無驗證過的 assistance tracking evaluator |

沒有已實作的 allocator，所以不產生偽造 allocated wrench 或有效 expert label。

## Policy whitelist

`phase1b.interfaces.POLICY_PROFILES`：

- `baseline`（預設）：精確沿用 Phase 1A 的 10 項 sensor allowlist。q/dq、actuator torques、raw wrist sensor、commands、truth 與 metadata 不加入。
- `with_joint_states`（opt-in）：baseline + `arm_joint_position`、`arm_joint_velocity`。此開關是未來消融介面，目前沒有學習模型。

`policy_observation` 只複製 whitelist 欄位，回傳唯讀 arrays。控制器可讀當下全部公開 observation（含 joints），但不取得 privileged section；controller 能讀 joints 不代表 learner 已使用 joints。

讀回驗證包含 clocks、五子步對齊、前後 observation 連續性、q/dq 與 raw MuJoCo state 的索引對照、command/torque bounds、unit gear torque 關係、NaN/valid masks，以及無 arm/root 外加 wrench。完整 suite 另驗證 FK 對照與 exact replay。
