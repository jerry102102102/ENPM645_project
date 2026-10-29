# ENPM 645 — Phase 0 v2：Dual-Gripper Observability & Force Demonstration Pilot

## 0. 專案背景與最新決策

我們正在開發一個 Human-Robot Collaboration 課程專案。

**研究目標：讓機器人根據物理互動訊號，學習在不同協作情境中應該提供多少力、朝哪個方向施力，而不是直接學習機械臂的關節動作。**

最終場景：

- 兩支機器人手臂共同支撐一塊剛性板材。
- 人類使用一隻手引導板材，另一隻手保留給加工操作。
- 機器人依據 Force/Torque、Tactile 和 Object State，決定適當的協助力量。
- 底層由 Model-Based Controller 負責雙臂協調、力分配和實際執行。

專案以可完成的模擬 Demo 為優先，不要求新的研究突破。

本次請根據既有 Phase 0 程式與報告，建立 **Phase 0 v2**。

**重要：不要直接進入模型訓練、完整雙臂控制或 CG-DAgger。先重新驗證新的物理配置與資料收集介面。**

---

## 1. 必須保留的 Phase 0 結論

原始 Phase 0 已完成：

- A/B/C 各 10 episodes，共 30 episodes。
- 4 秒／episode、100 Hz，共 12,000 筆同步樣本。
- 已驗證真實 MuJoCo 接觸、力學一致性及重現性。
- 使用非 weld 的 pad 接觸，取得 reconstructed F/T、touch_grid 與 object state。

**原始結論為 NO-GO：尚未證明 tactile 相對於 F/T + Object State 有額外的意圖辨識資訊。**

原因包括：

1. 自然 A/C 情境下的 F/T 與物體運動差異過大，沒有合格的匹配資料。
2. 相同合外力矩與相同接觸邊界條件下，tactile 也沒有可用差異。
3. 原始模型的接觸資訊相對稀疏。

這個 NO-GO 不代表整個 HRI 專案失敗，也不代表 tactile 永遠沒有價值。

但不能因為換成兩個夾爪，就預設 tactile 一定有新的資訊。

請保留舊版實驗、報告及結果，不要覆寫。

---

## 2. 新的系統架構

我們正式採用以下分層架構：

```text
Virtual Human + External Disturbance
                |
                v
        Shared Rigid Panel
                |
        +-------+-------+
        |               |
   Robot Hand L     Robot Hand R
   F/T + Tactile    F/T + Tactile
        |               |
        +-------+-------+
                |
          Object State
                |
                v
       Observation History
                |
                v
       Learned Force Policy
       (未來才進行訓練)
                |
                v
      Desired Object Wrench
                |
                v
       Model-Based Controller
       Force Allocation + Feedback
                |
                v
        Robot Hands L/R
```

### 2.1 學習器負責什麼？

未來模型的輸入：

\[
o_{t-H:t}=
\{W_L,W_R,T_L,T_R,x_{\mathrm{obj}},\dot{x}_{\mathrm{obj}}\}_{t-H:t}
\]

其中：

- \(W_L,W_R\)：左右機器人 F/T
- \(T_L,T_R\)：左右 tactile features
- \(x_{\mathrm{obj}}\)：物體姿態
- \(\dot{x}_{\mathrm{obj}}\)：物體速度

模型預測：

\[
\pi_\phi(o_{t-H:t})
\rightarrow \tau^*_{\mathrm{assist}}
\]

初期只有一個主要旋轉軸，因此輸出為**帶有正負方向的物體層級協助力矩（Nm）**。

未來可擴充為 6D desired object wrench。

模型不直接輸出：

- Joint positions
- Joint torques
- Robot trajectories
- 左右手的獨立動作序列

### 2.2 Model-Based Controller 負責什麼？

底層控制器接收 desired object wrench，負責：

- 左右機器人握持端的力分配
- 必要的重力補償
- 力回饋與順應控制
- 保持有效的接觸和握持力
- 遵守物理與安全限制

可以用 Grasp Matrix 表示：

\[
w_{\mathrm{obj}}=Gf_c
\]

學習器預測物體需要的協助力，控制器解決左右接觸力如何實現它。

**請將 Learned Force Policy 和 Model-Based Controller 設計成兩個獨立模組。**

Phase 0 v2 不要求完整控制器，但必須建立清楚的介面，並確認未來的 force reference 具備一致的座標系、單位和方向定義。

---

## 3. Phase 0 v2 的新物理配置

### 3.1 雙機器人握持端

將原本的單一夾爪配置，改為：

**Robot Gripper L + Robot Gripper R**

注意：

原始 Phase 0 的上、下兩個 pad 屬於同一組夾爪，不能將它們重新命名成左右手，就宣稱已經完成雙手機器人。

新版必須具有：

- 兩個空間上分離的握持位置。
- 每個握持端有自己的接觸面與預壓。
- 每個握持端有獨立的 F/T 訊號。
- 每個握持端有獨立的 tactile contact map。
- 兩個握持端能透過各自的順應性或受限驅動共同支撐板材。

每個 gripper 可先使用簡化的雙 pad 結構，不需要完整機械臂。

請明確區分各握持端的接觸力、控制力與支架反力。

不要用隱藏的 weld 或固定支架吸收全部外力，再將其宣稱為雙手協作。

若受限支架仍然必要，請量測並報告其負載貢獻與限制。

### 3.2 單一 Virtual Human Hand

只有一個人類虛擬握持點。

人類透過 spring-damper controller 向板材施力：

\[
F_h=K_h(p_h^*-p_h)+D_h(v_h^*-v_h)
\]

保留真實的剛體受力和動力學更新。

不要直接改寫板材 qpos。

### 3.3 External Disturbance

環境擾動與人類施力是兩個獨立來源。

可以指定：

- 外力大小
- 方向
- 作用位置
- 開始與結束時間

保留真實的外力 ground truth，但不得作為學生模型的觀測輸入。

### 3.4 任務維度

先維持單一主要旋轉軸（pitch）。

不用實作 6-DoF 自由操作，也不需要在本階段加入完整雙臂 IK。

---

## 4. 保留三種 HRI 情境

### A — Human-Guided Rotation

人類目標是旋轉板材，例如從 0° 到 +20°。

環境不施加額外持續擾動。

機器人理想上應提供與預期旋轉一致的協助力矩。

### B — Hold

人類目標是保持目前板材姿態。

環境沒有明顯擾動。

機器人應提供必要的承重、穩定與接觸維持。

### C — Hold Under Sustained Disturbance

人類目標仍然是保持目前姿態。

環境額外施加一股持續外力。

Virtual Human 會根據姿態誤差產生抵抗力。

機器人理想上應分擔抵抗擾動所需的力量，減少人類負擔。

**重要：B 和 C 的 Human Intent 都是 HOLD。**

因此，未來模型的主要輸出不再是 A/B/C 分類，而是適當的協助力矩。

A/B/C 仍然作為資料生成與評估情境標籤。

---

## 5. UMI-Style Force Demonstration Dataset

我們希望借用 UMI / UMI-FT 的核心設計概念：

- 同步記錄多模態感測資訊。
- 明確定義 Observation 與 Action／Force Reference。
- 讓資料不依賴特定的機械臂關節配置。
- 將上層學習與底層控制解耦。

本專案不需要重現 UMI 的硬體、Diffusion Policy 或影像訓練。

### 5.1 Observation

每個時間點至少記錄：

```text
timestamp
episode_id

wrist_ft_left
wrist_ft_right

tactile_left
tactile_right

object_pose
object_velocity

contact_state_left
contact_state_right
```

保留必要的座標轉換與 calibration metadata。

### 5.2 Expert Supervision

未來學習的目標是：

```text
expert_assist_torque_ref
```

這是教師希望機器人在物體主要旋轉軸提供的**協助力矩參考值**。

必須明確定義：

- 使用 Object Frame 還是 World Frame
- 正負方向
- 單位 Nm
- 是否包含重力補償
- 是否為基本穩定控制以外的額外協助力矩

建議將它定義為**基本承重與穩定控制之上的協作力矩參考**，並另外記錄總目標力矩。

不要將 measured F/T 或已執行的控制力直接改名為 expert label。

### 5.3 Controller Diagnostics

另行記錄：

```text
requested_object_wrench
allocated_gripper_wrenches
executed_actuator_commands
realized_object_wrench
controller_mode
```

若某欄位在 Phase 0 尚無實作，不要填入假的零值；應標記 unavailable 或清楚標示缺失。

### 5.4 Privileged Ground Truth

單獨保存：

```text
human_goal
human_target_trajectory
human_applied_wrench
external_disturbance_wrench
scenario_label
expert_internal_state
```

這些資料可供產生 expert reference、訓練監督與評估。

但未來學生模型部署時不可直接讀取。

---

## 6. Model-Based Expert：本階段先設計介面與最小驗證

我們計畫使用 Model-Based Expert 產生期望協助力矩。

Expert 可以使用：

- 已知的人類目標姿態
- 物體動力學參數
- 模擬中的人類施力
- 外界擾動
- 接觸與控制限制

目標是在完成使用者意圖的同時，合理分擔負載，避免不必要的機器人施力。

可以考慮姿態誤差、人類努力、機器人施力與控制平滑度的加權目標函數。

但**不要只依照情境 A/B/C 手動指定固定的力矩標籤**。

同一個 C 情境，不同外力大小、方向及人類抵抗程度，應可能需要不同的機器人協助力矩。

請先：

1. 定義 Expert 的輸入、輸出與必要的物理假設。
2. 定義 `expert_assist_torque_ref` 的確切意義。
3. 建立可儲存 Expert Reference 的資料管線。
4. 如可行，實作一個簡單、具物理意義且有限幅的 Model-Based Expert，進行小規模測試。

如果這需要完整雙臂控制器才能正確實現，Phase 0 v2 可以先只完成介面及資料契約，不要產生不可信的 expert force labels。

**本階段不要求收集大量完整 expert demonstrations。**

---

## 7. Phase 0 v2 的核心實驗

本階段最重要的問題仍然是：

**兩個獨立握持端的 F/T 與 tactile，是否比原本單一夾爪提供更多可用的物理互動資訊？**

以及：

**這些資訊是否有機會支持我們未來想學習的協助力矩，而不是只能辨識固定腳本？**

### Experiment 0 — 原版回歸驗證

先重跑原始 Phase 0 的代表性 A/B/C episodes。

確認：

- 原始模型仍可執行。
- 原始數值驗證仍通過。
- 不會覆寫舊報告與資料。

接著再測試新版雙握持端模型。

### Experiment 1 — Dual-Gripper Physical Validation

確認：

- 左右握持端確實有獨立的接觸。
- 兩側各自產生有效的 F/T 和 tactile 訊號。
- 板材受到人類施力後，物理反應合理。
- 支架與 actuator 的負載貢獻可以區分。
- 沒有不合理的接觸穿透、滑落或數值不穩定。

請驗證物體受力平衡、接觸廣義力重建與時間步長敏感性。

### Experiment 2 — A/B/C Natural Conditions

先各執行 10 episodes。

記錄：

- 左右 F/T
- 左右 tactile maps
- 物體角度與角速度
- 接觸中心
- 左右負載分配
- 虛擬人類施力
- 獨立外界擾動

畫出同時間軸比較圖。

### Experiment 3 — Sensor Information Ablation

依序比較三個觀測層級。

**S1 — Aggregated Wrench + Object State**

只使用左右機器人感測的合成 object-level wrench，以及物體姿態／速度。

**S2 — Independent Left/Right F/T + Object State**

保留兩個獨立的 wrist wrench，不先相加。

**S3 — Left/Right F/T + Object State + Tactile**

額外加入左右夾爪的空間接觸資訊。

目的是區分：

- 第二個機器人握持端本身增加了多少資訊？
- Tactile 在已有雙手 F/T 時，還有沒有額外資訊？

不要把 S1→S2 的改善錯誤歸因於 tactile。

本階段以分布、匹配與可觀測性分析為主，不強制訓練 classifier。

### Experiment 4 — Hard Negative / Matched Conditions

延續上一版 Phase 0 的嚴格匹配精神。

嘗試建立或搜尋 A/C 在感測器總受力與物體運動接近的時間窗，然後依序檢查：

1. S1 是否可區分？
2. S2 是否仍有左右負載差異？
3. S3 是否存在 S2 無法解釋的 tactile 空間差異？

匹配時必須保留整段時間序列，避免只匹配平均力矩。

請記錄每個分量的 RMSE、匹配容差與合格樣本數。

如果沒有合格匹配，請直接報告 **insufficient matched evidence**，不能以未匹配的 tactile heatmap 宣稱有資訊增益。

另外保留等效 wrench 的負對照：

當理想剛體具有相同合外 wrench 歷史、相同初始狀態及接觸／控制邊界條件時，不能期待僅改變力的等效表示就產生新的 tactile 來源資訊。

### Experiment 5 — Force Reference Feasibility

如果已實作可信的簡化 Expert，請選少量 A/B/C episodes，檢查：

- Expert 在不同條件下的協助力矩方向及大小是否合理。
- Expert 的參考力矩是否符合現有接觸與 actuator 限制。
- Observation 與 Expert Force Reference 是否正確同步。
- 是否存在感測歷史非常相似，但 Expert 卻要求明顯不同協助力矩的案例。

最後一點尤其重要。

如果相同或近似的可觀測歷史對應互相衝突的 expert actions，學生即使使用 DAgger，也未必能可靠學會。

若 Expert 尚未具備物理可信度，請將此實驗標示為未完成，不要產生任意 teacher labels。

---

## 8. 後續 Offline Learning 與 CG-DAgger 的規劃

請在專案文件中記錄以下決策，但**本次不要實作這兩個階段**。

### Offline Force Learning

未來使用同步的 UMI-style demonstrations：

\[
(o_{t-H:t},\tau^*_{\mathrm{assist},t})
\]

訓練小型 MLP 或 GRU 預測協助力矩。

比較 S1、S2、S3 的輸入效果。

學生模型不使用 hidden human goal 或 disturbance ground truth。

### Simplified CG-DAgger

完成 Offline Behavior Cloning 後，讓學生在模擬中自行 rollout。

利用程式偵測：

- 協助方向錯誤
- 姿態追蹤失敗
- C 情境抵抗不足
- 不合理的大力量
- 接觸失效

觸發 Scripted Expert 介入，收集修正動作及相關狀態，再將資料聚合，進行下一輪離線訓練。

這是 Code-Gated DAgger 的簡化應用。

**CG-DAgger 不是 Phase 0 的任務，也不能用來取代感測可觀測性的驗證。**

---

## 9. 執行環境與範圍限制

目前先在 macOS 上使用標準 CPU MuJoCo。

- 優先 Headless Simulation。
- 可另外提供 Viewer。
- 不需要 CUDA 或 GPU 模擬。
- 不需要 Isaac Sim。
- 不需要 ROS 2。
- 不需要完整雙臂機器人。
- 不需要訓練 ViT、VLA 或 RL policy。

如果需要提高資料收集速度，先量測實際瓶頸，再決定是否評估 GPU 平行模擬。

---

## 10. 實作與交付要求

請先檢查現有 repository，理解原始 Phase 0 的程式、模型與資料格式，再開始修改。

優先重用既有程式，避免平行建立兩套無關系統。

建議保留以下邏輯結構：

```text
phase0_v2/
├── models/
│   └── dual_gripper_pilot.xml
├── simulation/
│   ├── virtual_human.py
│   ├── disturbance.py
│   └── dual_gripper.py
├── sensing/
│   ├── wrench.py
│   └── tactile.py
├── control/
│   ├── interface.py
│   └── expert_reference.py
├── data/
│   └── episode_schema.py
├── scripts/
│   ├── run_pilot.py
│   └── analyze_pilot.py
├── results/
│   ├── data/
│   └── figures/
├── README.md
└── report.md
```

不要求逐字使用這些檔名，應以既有專案架構為準。

請實際執行驗證，而不是只建立檔案。

### 最終交付

1. 可執行的 Dual-Gripper MuJoCo 模型。
2. A/B/C 三種情境的同步感測資料。
3. 左右 F/T 與 Tactile 分布圖。
4. S1/S2/S3 的觀測資訊比較。
5. 嚴格匹配與負對照結果。
6. UMI-style Force Demonstration 資料格式。
7. Model-Based Expert / Controller 的介面定義。
8. 更新後的 README 與 Phase 0 v2 report。
9. 一鍵重現指令與必要的驗證測試。

### 最終報告必須回答

**Q1.** 新的兩個 Robot Grippers 是否真的有獨立的物理接觸、受力與支撐能力？

**Q2.** 相對於單一握持端，雙握持端增加了哪些可觀測資訊？

**Q3.** 在左右 F/T 已知的情況下，Tactile 是否仍有額外資訊？

**Q4.** A/C 在受力與物體狀態接近時，是否能更合理地區分？

**Q5.** 我們是否已具備可靠的 Observation → Expert Assistance Torque 資料介面？

**Q6.** 如果加入一個 Model-Based Expert，它的 Force Reference 是否能由目前的觀測歷史合理預測？

**Q7.** 下一階段是否適合正式建立 Expert Controller 並收集 Offline Demonstrations？

### 最後決策

請區分兩個不同的 GO/NO-GO：

**Physical GO/NO-GO**

雙握持端物理模型、獨立感測與資料記錄是否可信。

**Learning-Data GO/NO-GO**

觀測與教師目標是否有足夠且一致的關係，值得繼續收集力矩示範資料。

可以出現：

- Physical GO / Learning-Data GO
- Physical GO / Learning-Data CONDITIONAL GO
- Physical GO / Learning-Data NO-GO
- Physical NO-GO

不能僅憑成功產生 tactile heatmap 或同步資料，就宣稱 Learning-Data GO。

---

## 11. 執行原則

**請先完成可重現的物理驗證，再決定是否擴展資料與模型。**

不要為了證明 tactile 有效而刻意設計必然分開的標籤或人工接觸圖案。

不要讓教師模型的 privileged goal 洩漏到學生觀測。

不要將 teacher desired wrench、機器人實際施力和感測器量到的 wrench 混為一談。

如果遇到重要的物理或感測限制，請保留負面結果並清楚說明，不要自行擴大到完整雙臂 IK、Isaac Sim、ViT 或 RL。

**現在開始實作 Phase 0 v2，完成實際測試後，以量化證據回報是否值得進入 Model-Based Expert Demonstration Collection。**