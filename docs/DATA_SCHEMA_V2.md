# Data Schema — enpm645-three-layer-transition/1.0

唯一 writer/reader/validator：`phase1b.pipeline.run_episode / read_episode / validate_episode`。Hardware 2.0.0 / Scenario 3.0.0。

## 時間

每筆 transition：`o_t → decision_t → object_command_t → 5 個 500 Hz arm commands/steps → o_next`。Observation/reference 100 Hz，physics/control 500 Hz；settle 2 秒後 episode clock 歸零，engine clock 另存。

| 檔案 | 內容 |
|---|---|
| observations.npz | 決策前 t 的 sensing/state；前一個命令仍作用 |
| decisions.npz | AssistanceDecision 原始請求，含 mode、torque、source、valid；confidence 不可用時 NaN + confidence_present=false |
| object_commands.npz | t 的 world pose/twist/assist wrench、mode、valid、frame/point；stiffness/damping 各有 valid mask，不可用填 NaN |
| controller.npz | 五個子步的實際命令、來源時間、torque、saturation 與 post-step contact wrench |
| next_observations.npz | t+0.01；除末筆外，精確等於下一筆 observations；不供當下 policy |
| privileged.npz | 人手目標、作用點／真值外力、scenario；不供 policy 或下層 controller |
| supervision.npz | 正式 Expert Labels；目前 torque=NaN、expert_label_valid=false，與 candidate decision 分開 |
| diagnostics.npz | 500 Hz 接觸、穿透、joint limits、force balance、外加力稽核與 warnings |
| render_states.npz | qpos/qvel，僅供 replay／FK／影片 |
| metadata.json | Schema/model/hardware/source hashes、component 類別、white-list profile、limits、array shapes/dtypes |
| verification.json | 此 episode 的契約與物理檢查結果，不代表控制能力已驗證 |

## Observation

所有 shapes 不含 N；arm 順序 left/right，各 joint1…7。

| 欄位 | Shape／單位 |
|---|---|
| wrist_ft_left/right | 各 `(6,)`，接觸重建的 panel-on-gripper wrench；gripper-local、about gripper origin；force xyz→torque xyz |
| wrist_sensor_left/right | 各 `(6,)`，child→parent transmitted wrench；sensor 在 gripper local `[0,.12,0]`，包含下游重量與動力學，未補償 |
| tactile_left/right | 各 `(2,3,4,4)`；upper/lower × sensor z/x/y × angular bins；N/bin |
| object_position/orientation | `(3,)` world xyz m / `(4,)` quaternion wxyz |
| object_linear/angular_velocity | 各 `(3,)` world m/s、rad/s |
| contact_state_left/right | 各 `(2,)` upper/lower 接觸點數 |
| arm_joint_position/velocity | 各 `(2,7)`，rad、rad/s |
| arm_actuator_torque/force | 各 `(2,7)`；目前 unit-gear hinge，數值皆為 Nm；不是外力估計 |
| end_effector_pose/velocity | `(2,7)` xyz+wxyz / `(2,6)` angular→linear；兩 pad 中間的 gripper frame |
| gripper_joint_position/velocity | 各 `(2,2)`，m、m/s |
| gripper_actuator_force | `(4,)` N |
| object_contact_wrench | `(6,)`，gripper-on-panel、world、about instantaneous panel COM |

另保留 gripper/object pose/velocity 合併欄位與左右 world-at-panel-COM contact wrenches，供診斷。

## Commands 與診斷

- Decision/ObjectCommand 各 N 筆；來源時間與 observation_t 相同。
- `command_timestamp / decision_timestamp / object_command_timestamp`：各 `(N,5)`；前者 t+[0,.002,.004,.006,.008]，後兩者保持 t。
- `arm_position_target (N,5,2,7)` rad；`pad_force_command (N,5,4)` N。
- `arm_requested_servo_torque`、`arm_actuator_torque_at_actuation`、`arm_actuator_torque_after_step`：分別為 clipping 前、作用後 step 前、step 後；不可互換。
- `realized_timestamp`=command_timestamp+.002；`realized_object_contact_wrench (N,5,6)` 是總接觸 wrench，不是已分離的 assistance。
- `requested_object_wrench (N,5,6)` 定義為含支撐的總需求，尚未計算，NaN + requested_object_wrench_valid=false。已知的額外 assistance request 存於 object_commands.assist_wrench_world。
- `allocated_gripper_wrenches (N,5,2,6)` 尚無 allocator，NaN + allocation_valid=false。
- `tracking_error_Nm (N,)` 為 NaN、tracking_error_valid=false；不可用總 Ty 代替。
- `controller_mode / assist_reference_executed` 每子步記錄；contact_status 對應 diagnostics.contact_counts，saturation 對應 controller.arm_force_saturated。

## 學習邊界

預設 policy 只有上述兩側 contact F/T、tactile、object pose/velocity、contact state（10 fields）。`with_joint_states` 再開啟 q/dq。Raw wrist sensors、torques、commands、truth、metadata 不自動加入。

Source 寫 oracle_expert 也不會讓 supervision 自動有效。所有候選決策與正式標籤分開；沒有配力／追蹤驗證前，仍不產生有效 Expert Labels。

Scripted motion 展示沿用獨立的 render/diagnostic record，含 privileged truth，**不是正式訓練資料格式**。舊 schema 與實驗只留在歷史 Git revision；現行 reader 不保留多版本分支。

## Expert shadow annotation

[Expert Data Contract](EXPERT_DATA_DESIGN.md) 定義 `enpm645-expert-shadow/1.0` sidecar：衍生感測、分開的 E0/E1/E2 candidates、詳細 metadata 與教師內部 state。不新增第二個物理 pipeline，也不覆寫本 schema 的原始檔。Shadow candidates 即使數值有效，正式 torque 仍 NaN、execution_valid/supervision_eligible=false。
