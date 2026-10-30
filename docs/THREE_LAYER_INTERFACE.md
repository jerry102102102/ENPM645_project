# Three-Layer Control Interface v1.0

唯一執行鏈：`PolicyObservation → AssistanceDecision → ObjectCommand → ArmCommand → MuJoCo`。Scenario 3.0.0 / Hardware 2.0.0 不變。型別在 `phase1b/interfaces.py`，串接在 `phase1b/control.py`，記錄在 `phase1b/pipeline.py`。

| 狀態 | 目前範圍 |
|---|---|
| Interface Ready | 型別、驗證、因果隔離與正式記錄已接通；測試見 results/phase1b/current |
| Controller Implemented | 只有 reference-only object adapter 與 position-hold dual-arm adapter |
| Assistance Tracking Validated | **否**；沒有 allocation / tracking algorithm |
| Expert Labels Validated | **否**；所有正式 supervision 保持 invalid |

## ① AssistanceDecision

| 欄位 | 契約 |
|---|---|
| timestamp | Episode seconds，有限且非負；必須等於本次決策 observation 時間 |
| mode | `ROTATE_POS / ROTATE_NEG / HOLD / UNCERTAIN` |
| assist_torque_y_nm | 有限 scalar；World +Y、instantaneous panel COM、Nm；不含基本支撐 |
| confidence | `None` 或 `[0,1]`；無校準估計時用 None |
| source | `scripted_test / rule_based / oracle_expert / learned_policy` |
| valid | 真正 bool；無效 decision 必須是 UNCERTAIN、torque=0 |

ROTATE_POS torque ≥0，ROTATE_NEG torque ≤0；HOLD 可正、可負、可非零。UNCERTAIN torque 必須為零，不得新增主動旋轉。`HOLD + disturbance` 不是第五種意圖。有效 UNCERTAIN 表示有效的「無法判定」結果；valid=false 表示上游結果不可用。

保留一個 scalar 資料轉接器：`AssistanceDecision.from_legacy(AssistTorqueReference(...), mode="HOLD")`；mode 必須明示，不能從 torque 正負推定意圖。`to_legacy()` 將 oracle_expert 對應 model_based_expert；舊 enum 無法表達 rule_based，會明確拒絕。這只是單一模組內的資料轉換，不保留舊 pipeline。

## ② ObjectCommand

| 欄位 | 契約 |
|---|---|
| timestamp / mode / valid | 與本次 AssistanceDecision 相同 |
| pose_ref_world | `(7,)`，world xyz + unit quaternion **wxyz**，位置在 panel COM |
| twist_ref_world | `(6,)`，**angular xyz → linear xyz**，rad/s、m/s |
| assist_wrench_world | `(6,)`，**force xyz → torque xyz**，N、Nm；MVP 精確為 `[0,0,0,0,decision.torque,0]` |
| impedance_parameters | 可省略的 stiffness/damping；各 `(6,)`、非負，**angular→linear**。空 dict 表示未指定，不代表零剛性控制器已實作 |
| reference_point / frame_id | 僅允許 `panel_com` / `world` |

Rotational stiffness/damping 單位 Nm/rad、Nm·s/rad；translational 部分 N/m、N·s/m。Arrays 防禦性複製為唯讀；impedance mapping 亦唯讀。

**責任與優先順序：** 基本支撐／接觸與機械限制優先，然後才是主動協助。Pose/twist 是名義運動參考；額外 wrench 應由組員的單一 compliant pitch control law 整合，不可在 Y 旋轉方向疊上互相競爭的高剛性位置控制與力矩控制。非零 Ty 與明示非零 pitch stiffness 的組合會被契約拒絕。

`ControlChain` 的測試預設 bound 為 `max_assist_torque_nm=.02`、`max_rotation_rate_rad_s=.5`；超限拒絕，不偷偷改值。這些是可設定的介面限額，不是驗證過的安全工作範圍。完整 pose-path rate limiting、smoothing、admittance、contact constraints 與飽和處置仍由正式控制器負責。

目前 `ReferenceOnlyObjectAdapter` 在進入 HOLD 時記住當下 pose；ROTATE 只編碼力矩請求，保留當下 pose、zero twist，**不生成旋轉軌跡**。UNCERTAIN 使用當下 pose、zero twist/wrench。下層 position hold 忽略高層 motion/wrench，不宣稱追蹤。

## ③ ArmCommand

沿用既有欄位：`arm_position_target (2,7)` rad、`pad_force_command (4,)` N、`controller_mode`、`assist_reference_executed`。驗證 finite、shape、joint/pad command bounds、模式與 bool。Arm position servos 不接受 Nm。

為避免重複 clock，ArmCommand 不另增 timestamp 欄位；唯一 pipeline 在發送當刻記錄 `command_timestamp`，以及此命令使用的 `decision_timestamp`、`object_command_timestamp`。它們與命令一起構成有時間戳的執行事件。

`assist_reference_executed` 只表示控制器有使用 assistance；不表示力矩追蹤成功。UNCERTAIN 禁止回報已執行 assistance。實際 actuator torque、contact state、saturation、post-step wrench 由 pipeline 記錄，不由 controller 偽造。

## Protocol 與 clock

```python
AssistancePolicy.predict(history: PolicyHistory) -> AssistanceDecision
ObjectMotionController.step(decision, observation: ArmControllerInput, dt: float) -> ObjectCommand
DualArmController.step(command, observation: ArmControllerInput) -> ArmCommand
```

每 100 Hz：先取得 observation，policy 與 object controller 各呼叫一次。五個 500 Hz 子步保留同一 ObjectCommand，但每次取得新的公開 observation；最大 reference age 為 0.008 s。缺步、亂序、future/stale decision、錯誤輸出均中止；不靜默改用其他 controller。

Policy 收到既有白名單的歷史，預設最多 50 筆；q/dq 用 `with_joint_states` 明確開啟。Object/arm controllers 取得更完整的公開 robot/object sensing、名義 q、pad commands 及 limits。每個 callback 前都建立白名單副本；不傳入 simulator handle、human goal、scenario、ground-truth forces 或 future commands。這是模組資料邊界，不是對不受信任 Python 程式的 sandbox。

## 測試與執行

```bash
uv run --locked python -m unittest phase1b.tests.test_contracts phase1b.tests.test_integration -v
uv run --locked python -m phase1b.scripts.collect --duration .12 --output results/phase1b/dev-contract
uv run --locked python -m phase1b.scripts.run_suite --output results/phase1b/reproduction --render
```

測試涵蓋 modes/sign/confidence、frame/quaternion、白名單與 future rejection、HOLD latch、三層執行、時鐘、invalid masks、scalar adapter、物理無變動及資料損壞拒絕。完整 suite 另檢查支撐負對照、FK、重播與 A/B/C 動作；[本次結果](../results/phase1b/current/verification.json)。

[資料格式](DATA_SCHEMA_V2.md) · [組員接入](CONTROL_HANDOFF_ARMS.md) · [硬體](CURRENT_HARDWARE_HANDOFF.md)
