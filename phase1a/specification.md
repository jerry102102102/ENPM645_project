# ENPM 645 — Phase 1A：Frozen Scenario、Desired Torque Interface 與 Data Pipeline

## 0. 任務背景

我們正在開發一個 Simulation-Based Physical Human-Robot Collaboration 專案。

目前已完成：

- Phase 0：單一簡化 Gripper 的 Tactile Observability Pilot。
- Phase 0 v2：Dual-Gripper MuJoCo Simulation。
- v2 已驗證雙 Grippers 具有獨立物理接觸、F/T、Tactile 與物體動力學。
- A/B/C 各 10 episodes，共 12,000 筆同步樣本。
- 物理驗證結果為 Physical GO。
- Learning-Data 尚未 GO，因為目前沒有經過驗證的 Expert Assistance Torque Labels。

**這次不要重新建立物理模擬，也不要重新設計 Tactile。請以既有 Phase 0 v2 為基礎。**

我們目前已經決定將工作拆成兩條線：

**Track A — Data & Learning（我們負責）**

固定場景、設計資料介面、收集同步的 Observation / Desired Torque / Executed Control / Privileged Ground Truth，後續才研究 Expert Supervision 與 Offline Learning。

**Track B — Model-Based Control（另一位組員負責）**

實作 Object-Level Desired Torque 到左右 Gripper Wrenches 的力分配、基本支撐、內力管理與實際力矩追蹤。

這次 Codex 只實作 Track A，以及 Track A / Track B 之間必要的介面契約。

**不要自行實作完整 Force Allocation Controller 或 Model-Based Expert。**

---

## 1. Phase 1A 的核心目標

本次需要完成四件事：

1. 凍結 Phase 0 v2 的場景幾何、抓取位置及 A/B/C 定義。
2. 建立統一的 Desired Object Assistance Torque Interface。
3. 建立時間對齊且沒有 Privileged Information Leakage 的資料收集管線。
4. 執行資料管線 Smoke Test，驗證所有欄位、時間戳與儲存結果正確。

這不是訓練任務。

本次成功標準是：

**固定場景 → 取得 Observation → 接收 Desired Torque Reference → 交給 Controller Interface → 執行模擬 → 記錄實際結果。**

即使組員的 Controller 尚未完成，仍要能使用既有 support-only baseline 執行資料管線測試。

但不得假裝新的 Desired Torque 已經被實際執行。

---

## 2. 先檢查現有 Repository

在修改之前，請先檢查：

- Phase 0 v2 的模型 MJCF。
- 左右 Gripper 的結構及 Contact Geometry。
- Virtual Human Hand 的施力實作。
- External Disturbance 的實作。
- 現有 Observation / Controller / Privileged 資料結構。
- `LearnedForcePolicy.predict(history)` 介面。
- `ObjectWrenchController.step(reference, observation)` 介面。
- 目前的 Simulation Loop。
- README、測試及既有結果。

優先重用現有模組，不要重新建立平行但不相容的程式架構。

保留原始 Phase 0 / Phase 0 v2 的程式與報告。

---

# Part A — Frozen Simulation Scenario

## 3. 固定幾何與抓取配置

請建立一份唯一的場景設定，例如：

`config/scenario_v1.yaml`

但不要自行猜測握持位置座標。

**請從現有 Phase 0 v2 模型讀取並記錄實際值。**

需要固定：

### 3.1 Object

- Panel dimensions
- Mass
- Inertia
- Collision geometry
- Initial pose
- Contact friction

目前板材參考尺寸為：

- 0.6 × 0.2 × 0.02 m
- 1.0 kg

若現有 v2 數值不同，以已驗證的 v2 模型為準。

### 3.2 Robot Grippers

固定：

- Left Gripper 的名義握持位置
- Right Gripper 的名義握持位置
- 每個 Gripper 相對於板材的名義握持姿態
- Pad geometry
- Pad preload
- Contact friction
- Sensor frames
- Root drive limits

**所有握持位置必須以 Panel Local Frame 記錄。**

不能只記錄世界座標。

注意：

「固定握持位置」是指兩個 Grippers 在板材上的名義抓取位置固定，並不是把 Gripper root 鎖死在 World Frame。

板材和 Grippers 仍然必須允許物理運動與接觸。

實際 contact points 可因軟接觸和微滑移產生變化，不能將其人工固定。

### 3.3 Virtual Human Hand

本專案只有一個 Virtual Human Hand。

固定：

- Human grasp point（Panel Local Frame）
- Human attachment / interaction model
- Spring-damper 計算方式
- Nominal stiffness / damping
- Force limits

人類可以改變手部目標位置，但名義握持點在板材上的位置應固定。

不要加入第二隻 Virtual Human Hand。

### 3.4 External Disturbance

保留獨立的 Environment Disturbance。

固定名義擾動配置，包括：

- Application location
- Direction convention
- Force limits
- Activation protocol

在後續資料收集時，可以按照明確定義的實驗參數改變擾動大小和時間，但不要任意修改抓取幾何。

### 3.5 版本控制

請建立：

- `scenario_id`
- `scenario_version`
- `model_hash`
- `config_hash`

讓每個 Episode 都能追溯使用的 MJCF 與場景設定。

若後續需要改變幾何或接觸參數，應建立新的 Scenario Version，而不是靜默覆寫 v1。

---

## 4. 固定 A/B/C 任務定義

繼續使用三種情境。

| Scenario | Human Goal | Disturbance | 預期協作 |
|---|---|---|---|
| A — ROTATE | 改變板材角度 | 無顯著額外擾動 | 協助旋轉 |
| B — HOLD | 保持指定角度 | 無顯著擾動 | 維持姿態 |
| C — RESIST | 保持指定角度 | 持續外部擾動 | 分擔抵抗力 |

注意：

**C 的 Human Goal 仍然是 HOLD。**

`scenario_label` 只用於資料生成與評估。

未來學生模型不得直接讀取 `scenario_label`。

本階段使用固定的基準情境及 seeds，不必進行大量 Domain Randomization。

---

# Part B — Desired Torque Interface

## 5. 定義模型的最終輸出

我們已確定 Learning Model 不直接輸出 Joint Commands。

模型輸出為：

\[
\tau^*_{\mathrm{assist}}\in\mathbb R
\]

這是一個帶正負號的物體層級協助力矩。

請統一使用：

| 項目 | 定義 |
|---|---|
| Quantity | Desired Assistance Torque |
| Reference point | Panel COM |
| Coordinate frame | World Frame |
| Axis | +Y |
| Unit | Nm |
| Sign | 依右手定則 |
| Meaning | 基本承重／穩定控制之外的額外協助力矩 |

請確認現有 Phase 0 v2 的 Pitch 軸與 World +Y 一致。

如果有不一致，建立明確的座標轉換，不要靜默更改原始物理模型。

## 6. 建立控制參考資料結構

建立明確的 `AssistTorqueReference` 資料型別。

至少包含：

```python
AssistTorqueReference(
    timestamp,
    torque_nm,
    frame_id,
    reference_point,
    source,
    valid,
)
```

`source` 必須能區分：

- `scripted_test`
- `model_based_expert`
- `learned_policy`

本次只需要實際使用 `scripted_test`。

保留其他兩個來源供未來使用。

**Scripted Test Torque 不等於 Expert Label。**

不能將測試指令直接存成有效的 `expert_assist_torque_ref`。

---

## 7. 建立 Controller Interface

請沿用或調整現有：

```python
ObjectWrenchController.step(
    reference,
    observation,
)
```

並明確定義輸入與輸出。

### Input

- Desired Assistance Torque
- Object pose / velocity
- Left/right gripper state
- Left/right F/T
- Contact state
- Control timestep

### Output

介面應支援：

```python
requested_object_wrench
allocated_gripper_wrenches
executed_actuator_commands
realized_object_wrench
saturation_status
contact_status
controller_mode
```

請清楚區分：

1. Requested Reference
2. Allocated Wrenches
3. Executed Commands
4. Realized Contact Wrench

目前尚未實作的欄位應回傳明確的 unavailable 狀態與 validity mask。

不得將未計算的欄位用 0 假裝成有效數值。

也不能將既有 root drive 直接改名為 force allocation。

### Controller Stub / Baseline Adapter

為了讓我們可以獨立完成資料管線：

請建立或重用既有的 `support_only` adapter。

此 adapter 可以維持現有 Phase 0 v2 的基本支撐控制。

但如果它沒有追蹤新的 Desired Assistance Torque，必須明確記錄：

```text
controller_mode = support_only
assist_reference_executed = false
```

不可宣稱 Desired Torque Tracking 成功。

**完整的 Desired Torque → Dual-Gripper Force Allocation 將由另一位組員實作。**

請提供組員能直接實作的介面契約與接入位置。

---

# Part C — UMI-Style Data Pipeline

## 8. 建立統一的 Episode Schema

我們要使用 UMI-style 的資料設計概念，但不是直接複製 UMI 原始格式。

資料必須與特定機械臂關節配置解耦。

請將資料分成以下四類。

### 8.1 Observation

這是未來 Learned Policy 可以使用的輸入。

```text
timestamp

wrist_ft_left
wrist_ft_right

tactile_left
tactile_right

object_position
object_orientation
object_linear_velocity
object_angular_velocity

contact_state_left
contact_state_right
```

請保留感測座標系資訊。

例如：

- F/T 原始測量所屬 local frame
- 轉換後的 world-frame wrench
- Tactile map 所屬 pad frame
- Object orientation 的 quaternion ordering

不可混用不同 Frame 的力矩。

初期允許直接使用 MuJoCo Ground Truth Object State。

### 8.2 Supervision

```text
expert_assist_torque_ref
expert_label_valid
expert_source
```

**本次尚無 Model-Based Expert。**

因此正式 Expert Label 應維持 invalid。

不可把 `scripted_test` torque 寫成有效的 expert supervision。

### 8.3 Controller Data

```text
desired_assist_torque_ref
reference_source

requested_total_object_wrench
allocated_gripper_wrenches

executed_actuator_commands
executed_virtual_drive_wrenches

realized_object_contact_wrench

controller_mode
saturation_status
contact_status
```

如果可以取得 baseline controller 的貢獻，也請單獨保存。

注意：

`realized_object_contact_wrench` 是實際總接觸 wrench。

它不一定等於額外的 assistance torque。

在尚未驗證如何分離 baseline 與 assistance 的情況下，不要產生假的 `realized_assistance_torque`。

### 8.4 Privileged Ground Truth

單獨存放：

```text
human_goal

human_target_trajectory
human_applied_force
human_applied_torque

external_disturbance_force
external_disturbance_torque

scenario_label
random_seed
```

這些只供：

- 教師策略
- Ground Truth 評估
- 資料分析

未來 Learned Policy 不可以直接使用。

---

## 9. 嚴格定義時間同步

這是本次最重要的技術要求之一。

請建立以下控制與資料收集順序：

```text
Simulation State at t
        |
        v
Observe o_t
        |
        v
Build Observation History
        |
        v
Generate / Receive Torque Reference u_t
        |
        v
Controller.step(u_t, o_t)
        |
        v
Apply Controller Commands
        |
        v
MuJoCo Physics Step
        |
        v
Observe o_(t+1)
        |
        v
Log Transition
```

每筆資料必須明確對應：

\[
(o_t,u_t,a_t,o_{t+1})
\]

其中：

- \(o_t\)：控制前觀測
- \(u_t\)：期望協助力矩
- \(a_t\)：實際送到模擬器的控制命令
- \(o_{t+1}\)：控制後觀測

如果物理模擬與控制頻率不同，請正確記錄每個控制週期實際涵蓋的 physics steps。

可以沿用：

- Physics timestep：約 0.002 s
- Logging frequency：100 Hz

但以目前 repository 的有效設定為準。

不得依賴陣列索引碰巧相同就宣稱資料已經同步。

### 防止資料洩漏

未來模型在時間 \(t\) 的輸入只能包含：

\[
o_{\leq t}
\]

不能包含：

- 執行控制之後才得到的 \(o_{t+1}\)
- Future Force
- Future Pose
- Human Goal
- Scenario Label
- Expert Internal State

請建立正式的 `policy_observation_builder` 白名單。

不要讓訓練程式直接讀取所有 NPZ 欄位。

---

# Part D — Smoke Test

## 10. 建立測試用 Torque Reference Generator

在 Controller 尚未完成之前，我們先使用明確的測試訊號來驗證資料流。

例如：

| 時間 | 測試 Torque |
|---|---:|
| 0–1 s | 0 Nm |
| 1–2 s | +0.2 Nm |
| 2–3 s | 0 Nm |
| 3–4 s | −0.2 Nm |
| 4–5 s | 0 Nm |

實際力矩幅度必須符合現有測試配置，不可直接假設 +0.2 Nm 一定安全或可達。

如果需要，請使用更小的初始值。

這個 Generator 的目的僅是產生可追蹤的 Reference。

不需要建立 Expert，也不需要讓模型辨識 A/B/C。

### 重要限制

如果目前 Controller Adapter 尚未使用這個 Reference，請保持原有 support-only 動力學，並明確記錄 Reference 尚未被執行。

不能為了讓圖表吻合而直接修改板材姿態、接觸力或 measured wrench。

---

## 11. 執行資料 Pipeline Smoke Test

至少執行：

1. A 情境一個 episode。
2. B 情境一個 episode。
3. C 情境一個 episode。
4. 一個 Scripted Torque Reference Test。

請驗證：

- 所有資料有正確 timestamps。
- Observation 與 reference 同步。
- Controller output 不與 desired reference 混淆。
- Invalid expert labels 確實保持 invalid。
- Privileged data 沒有洩漏到 policy observation。
- Contact / F/T / Tactile 資料正常保存。
- Dataset 可以正確讀回。
- 相同 seed 可以確定性重現。

### 必須特別驗證

請檢查：

\[
t_{\mathrm{observation}}
\leq
t_{\mathrm{decision}}
<
t_{\mathrm{next\ observation}}
\]

以及：

\[
\text{Expert Validity}=\text{False}
\]

在真正的 Expert 尚未存在時，不能出現有效的 Expert Labels。

如果沒有實際執行 Desired Torque，Tracking Evaluation 應標示為 unavailable，而不是顯示 0 誤差。

---

# Part E — 給組員的 Controller Handoff

## 12. 產生獨立的 Controller Interface 文件

請建立：

`docs/CONTROL_HANDOFF.md`

這份文件要讓另一位組員不用閱讀整個資料收集系統，也能理解她需要實作什麼。

必須包含：

### 12.1 她會收到什麼？

- Desired Assistance Torque（Nm）
- Object Pose / Velocity
- Left/right Gripper State
- Left/right F/T
- Contact State

### 12.2 她需要輸出什麼？

- 左右 Gripper 的 Control Commands
- Requested / Allocated / Executed Wrenches
- Saturation and Contact Diagnostics

### 12.3 她負責什麼？

將：

\[
\tau_{\mathrm{assist}}^*
\]

轉成物理上可實現的左右 Gripper 施力。

她可以使用：

- Grasp Matrix
- Model-Based Force Allocation
- Impedance / Admittance Control
- Internal Force Regulation

但本次 Codex 不要實作這些控制算法。

### 12.4 座標與單位

必須明確說明：

- World Frame
- Panel Frame
- Left Gripper Frame
- Right Gripper Frame
- Torque 正負號
- COM Reference
- 單位

### 12.5 Controller 實作限制

- 不可直接改寫板材 qpos。
- 不可依賴 Human Goal / Scenario Label 作弊。
- 不可把 desired torque 直接當成 realized torque。
- 必須處理 saturation。
- 必須保留 baseline support 和額外 assistance 的區分。
- 回傳完整的執行診斷資訊。

### 12.6 最小整合測試

未來她的 Controller 應能在相同場景下，追蹤有限的正、負小力矩參考，並提供量化的 requested / realized 比較。

請在文件中指出她應實作或替換的 class 與函式位置。

---

# Part F — 程式結構與交付

## 13. 建議模組

優先整合到既有 Repository。

若現有結構適合，可以新增：

```text
config/
    scenario_v1.yaml

simulation/
    scenario_loader.py

interfaces/
    assist_torque_reference.py
    controller_contract.py

data/
    episode_schema.py
    observation_builder.py
    episode_logger.py
    dataset_reader.py

tests/
    test_scenario_freeze.py
    test_controller_contract.py
    test_data_alignment.py
    test_privileged_leakage.py
    test_reproducibility.py

scripts/
    run_pipeline_smoke.py
    validate_dataset.py

docs/
    SCENARIO_V1.md
    DATA_SCHEMA.md
    CONTROL_HANDOFF.md

results/
    phase1a/
        smoke/
        validation.json
        report.md
```

不要求逐字採用這個目錄，應避免破壞原有程式架構。

### Data Format

優先使用：

- NPZ：數值資料
- JSON / YAML：Metadata 與 Schema
- PNG：驗證圖表
- Markdown：報告

不要為了這個階段導入複雜的資料庫或大型資料框架。

---

## 14. 最終報告要求

請建立 `report.md`，回答以下問題：

**Q1.** 是否成功從 Phase 0 v2 凍結場景配置？

**Q2.** 三個握持位置是否已用 Panel Local Frame 明確定義？

**Q3.** Desired Assistance Torque 的座標、單位與正負方向是否一致？

**Q4.** Observation、Supervision、Controller 與 Privileged Ground Truth 是否已完全分離？

**Q5.** 時間同步與資料讀寫是否通過驗證？

**Q6.** Scripted Torque Reference 是否可以透過統一介面傳入 Controller Adapter？

**Q7.** 現有 support-only baseline 是否能在不破壞物理模型的情況下繼續執行？

**Q8.** 組員是否能僅依照 CONTROL_HANDOFF.md 開發新的 Controller？

**Q9.** 現在距離正式 Expert Demonstration Collection 還缺少哪些必要模組？

請明確區分：

- 已實作
- 已驗證
- 僅建立介面
- 尚未實作

不要把 interface ready 當作 force controller ready。

---

## 15. 驗收標準

本階段有兩個獨立的判定。

### Data Pipeline GO

需要同時滿足：

- Frozen Scenario 可重現。
- Data Schema 正確。
- 時間同步驗證通過。
- Privileged Information 隔離通過。
- A/B/C Smoke Tests 通過。
- Desired Torque Reference 可接入 Controller Interface。
- Dataset 可儲存、讀取並驗證。
- 新資料不會覆寫 Phase 0 v2。

### Controller Integration Ready

需要滿足：

- 介面定義完整。
- 座標／單位／方向清楚。
- Controller Inputs / Outputs 具備型別與有效性檢查。
- 有可執行的 Stub / Baseline Adapter。
- 新 Controller 可以不修改 Data Pipeline 的核心程式就接入。

注意：

**Controller Integration Ready 不等於 Desired Torque Tracking GO。**

本次不要求 Expert Label GO 或 Offline Learning GO。

---

## 16. 執行原則

請直接開始實作，不要只寫規劃文件。

先檢查現有 repository，再重用既有資料結構與模擬環境。

優先完成小而可靠的系統，不要自行擴大範圍。

本次不要：

- 訓練 ML Model
- 收集正式 Expert Demonstrations
- 實作 CG-DAgger
- 實作 ViT
- 重新設計 Tactile
- 建立完整雙臂 IK
- 改用 Isaac Sim
- 加入 ROS 2
- 實作組員負責的完整雙臂力分配控制器

如果有工程問題，請採用最小合理修正，並清楚記錄。

最後必須實際執行 Smoke Test、Dataset Validation 和 Regression Tests，提供結果檔案位置、執行指令與未完成項目。

**最終目標：完成一套場景已固定、Desired Torque 介面清楚、資料同步可信、能獨立接入 Model-Based Controller 的 Phase 1A 系統。**