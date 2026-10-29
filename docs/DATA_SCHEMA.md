# Phase 1A data schema

Version：`enpm645-transition/1.0`。NPZ 以 `allow_pickle=False` 讀取；JSON 保存 metadata、frames、time semantics。完整 array 欄位、shape 和 dtype 列於每個 `metadata.json` 的 `array_layout` 及輸出根目錄的 `schema.json`。N 是 transition 數；K=5 是每筆 transition 內的物理步數。

## 一筆資料的事件與時鐘

```text
cached o_t (old command held) → policy history ≤ t → reference u_t
  → controller(o_t, u_t) → apply a_0 → physics step → observe
  → controller(current observation, held u_t) → apply a_1 → physics → observe
  → ... a_2, a_3, a_4 ... → o_next → save transition
```

Reference / logging=100 Hz；low-level controller / physics=500 Hz。`a_t` 是 **五筆實際命令序列**，不是把第一筆命令假裝保持 10 ms。Low-level controller 可用中間的即時回饋；本次 100 Hz decision 的 policy history 不可使用它們。

Episode clock = settle 2 s 結束後的 simulated seconds；`timestamp = physics_tick × 0.002`。另外保存真實 MuJoCo `engine_timestamp` 與 metadata 的 `engine_time_origin`，交叉驗證引擎時間，沒有依賴 index 相同就宣稱同步。

`o_t` 在決策之前、舊命令持續作用時取得。施加新命令後 `mj_forward` 可能在相同 simulation time 改變 contact force；這個 action 後量測存在 controller 的 `contact_wrench_at_actuation`，絕不回填 o_t。執行 `mj_step` 後重新量測，保存 post-step wrench 與時間。

驗證包括 `t_observation ≤ t_decision < t_next`、reference time=decision time、五個 controller/physics ticks、realized times=(ticks+1)×dt、實際 engine clock、相鄰 `o_next[i] == o_t[i+1]`、所有儲存 section 長度、quaternion、validity masks、command limits、contact retention 和力平衡。

## 分檔與欄位

| 檔案 | 內容 / 存取邊界 |
|---|---|
| `observations.npz` | 當下 o_t，policy reader 的唯一輸入來源 |
| `next_observations.npz` | o_next，與 observation 同 schema；不可供同一 transition 的 policy 決策 |
| `supervision.npz` | 正式 expert target，本階段全部 unavailable |
| `controller.npz` | reference、baseline proposal、實際命令序列、action 後與 post-step 診斷 |
| `privileged.npz` | hidden human goal、施力、擾動、scenario、seed；僅供生成/分析/未來 teacher |
| `metadata.json` | scenario/model/config versions、runtime versions、clock、controller factory/modes、array layout |

### Observation

每檔都有 `(N,)` 的 `timestamp`、`engine_timestamp`、`physics_tick`。Policy allowlist 只包含下表前十個 sensor keys；timestamp 另外用於 causal indexing，不把 scenario metadata 混進 feature vector。

| Key | 每筆形狀 | Frame / unit |
|---|---|---|
| `wrist_ft_left`, `wrist_ft_right` | 各 (6,) | panel-on-gripper，各自 root-local、about root；F xyz N、T xyz Nm |
| `tactile_left`, `tactile_right` | 各 (2,3,4,4) | upper/lower × force z/x/y × angular bins；pad sensor-local，N/bin；沿用 v2 plugin |
| `object_position` | (3,) | world，m；本場景 body origin=COM |
| `object_orientation` | (4,) | panel→world，wxyz |
| `object_linear_velocity` | (3,) | world，m/s |
| `object_angular_velocity` | (3,) | world，rad/s |
| `contact_state_left`, `contact_state_right` | 各 (2,) | upper/lower contact counts |
| `wrist_ft_left_world_at_panel_com`, `wrist_ft_right_world_at_panel_com` | 各 (6,) | 診斷副本：作用方向反號並移至 COM 的 gripper-on-panel world wrench |
| `object_contact_wrench` | (6,) | 左右接觸合計，world about instantaneous COM；N/Nm |
| `gripper_pose` | (2,7) | left/right，world xyz + wxyz |
| `gripper_velocity` | (2,6) | world angular xyz、linear xyz；rad/s、m/s |

最後四列是公開感測/kinematic diagnostics，但尚未納入目前 policy feature allowlist。Object state 暫由 MuJoCo ground truth 提供，是本規格允許的輸入；它不代表已實作相機感知。

### Supervision

`timestamp (N,)`；`expert_assist_torque_ref (N,)` **全 NaN**；`expert_label_valid (N,)` **全 false**；`expert_source (N,)` **全 unavailable**。Scripted reference 不會被複製成 expert label。未來要正式收集 teacher data，必須另外建立驗證過的 expert 和 collection mode，不能只翻 validity flag。

### Controller

| 欄位群 | 形狀 / 語意 |
|---|---|
| `timestamp`, `decision_timestamp`, `reference_timestamp` | (N,)，明確事件時刻 |
| `desired_assist_torque_ref`, `reference_source`, `reference_valid`, `reference_frame_id`, `reference_point` | (N,)，world +Y、panel COM、Nm 的額外 torque |
| `requested_assistance_wrench` | (N,6)，原始需求 `[0,0,0,0,tau,0]`；不是總 target 或量測 |
| `requested_total_object_wrench` / `_valid` | (N,K,6) / (N,K)，baseline+assist 的總接觸目標；目前 NaN/false |
| `allocated_gripper_wrenches` / `_valid` | (N,K,2,6) / (N,K)，world、about each root 的 gripper-on-panel targets；目前 NaN/false |
| `executed_actuator_commands` | (N,K,4)，實際 pad motor commands，N |
| `executed_virtual_drive_wrenches` | (N,K,2,6)，實際施加在各 root 的 world wrench，N/Nm |
| `baseline_virtual_drive_proposal`, `baseline_nominal_support`, `baseline_feedback` | 各 (N,K,2,6)，原 v2 proposal、clipping 前 nominal 與 feedback；未宣稱是量測 contact contribution |
| `controller_call_timestamps`, `physics_step_ticks`, `realized_substep_timestamps` | 各 (N,K)，各子步的 apply time、step tick、post-step measurement time |
| `realized_contact_wrench_substep` | (N,K,6)，各步之後的實際總 contact wrench，world COM |
| `realized_object_contact_wrench` | (N,6)，最後子步的 contact wrench，等於 o_next 的對應 sensor |
| `controller_mode`, `assist_reference_executed` | 各 (N,K)，目前 support_only / false |
| `saturation_status`, `pad_saturation_status` | (N,K,2,2) root force/torque clipping；(N,K,4) pad clipping，bool |
| `contact_status` | (N,K,2,2)，每次控制前的 current contact counts |
| `history_latest_timestamp`, `history_length` | (N,)，用於檢查當次 high-level history 範圍；最多 25 個 frames |
| `tracking_error_Nm` / `_valid` | (N,) / (N,)，**NaN / false**；尚無 baseline/assistance 分離評估 |
| `contact_wrench_at_actuation` | (N,K,6)，apply 後、physics step 前，同 timestamp 的 contact response |
| `actual_pad_actuator_force` | (N,K,4)，engine actuator force，驗證與 command 相符 |
| `panel_acceleration` | (N,K,3)，apply 後 world linear acceleration，m/s² |
| `linear_balance_residual_N`, `contact_generalized_error`, `penetration_m` | (N,K)，力平衡 residual、接觸 generalized-force reconstruction 最大誤差、penetration |

`contact_generalized_error` 是不同 generalized DOFs 的最大絕對誤差，對應 N 或 Nm，不能解讀成單一純力單位。所有 invalid numeric fields 必須是 NaN；runtime Optional arrays 用 None。Validator 拒絕 mask=false 但內容為 0 的假 unavailable 欄位。

`realized_object_contact_wrench` 是 **總接觸 wrench**。資料沒有 `realized_assistance_torque`，也沒有把 desired torque 當 measured torque。

### Privileged

`human_goal (N,)`：目標 pitch rad；`human_goal_type (N,)`：ROTATE 或 HOLD，C 仍為 HOLD。`human_target_trajectory (N,3)`：當下 world target position m。

`human_applied_force / human_applied_torque` 與 `external_disturbance_force / external_disturbance_torque` 各 (N,3)，對應控制前 timestamp 的 world force / moment about COM；`human_application_point / external_application_point` 各 (N,3)，world m。

`applied_human_wrench_substeps / applied_external_wrench_substeps` 各 (N,K,6)，實際各步使用的施力；`applied_wrench_timestamps (N,K)` 對齊 controller call times。另有 `scenario_label (N,)`、`random_seed (N,)`。這些欄位都不能進入 policy history。

## 讀取與隔離

```python
from phase1a.data.episode_io import DatasetReader, validate_dataset

validate_dataset('results/phase1a/smoke/A')
reader = DatasetReader('results/phase1a/smoke/A')
history = reader.policy_history(index=100, history_length=25)
# history.timestamps: only observations at or before transition 100
# history.features: immutable arrays of the ten allowlisted sensor keys
```

不要直接把 `reader.sections`、metadata 或所有 NPZ 欄位送進訓練。底層 reader 可讀分析欄位，但 `policy_history` 固定只使用 `observations`，正式模型入口是 `policy_observation_builder`。它拒絕額外 observation fields、不遞增 timestamps、未來 frames 和非有限 sensor data；輸出為唯讀 arrays。

已測試：毒化未來 observation、next_observation、controller、supervision、privileged arrays 不改變當下 history；錯誤 clocks、有效 expert flag、假零 tracking error 被 validator 拒絕。這是程式 API 邊界與 regression tests，不是對任意惡意 Python controller 的 sandbox。

## 驗證範圍

預設 `validate_dataset(path)` 同時要求本次 support-only smoke 的狀態。替代 controller 可以用 `require_support_only=False` 驗證資料/物理完整性與 availability masks；reference 仍是 scripted_test、expert 仍 invalid、tracking evaluator 仍 unavailable。替代控制器若失去接觸、違反 command bounds 或力平衡，驗證仍會失敗。

請使用一般 Python（不可加 `-O`；validator 會拒絕關閉 assertions 的環境）。確定性只對本次記錄的 Python/NumPy/MuJoCo 平台組合做 exact replay 驗證；跨版本/硬體需重新檢查。
