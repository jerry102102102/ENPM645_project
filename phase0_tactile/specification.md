# ENPM 645 — Phase 0：Tactile Observability Pilot

## 1. 任務目標

請在目前工作環境中，使用 Python + MuJoCo 建立一個最小物理實驗，驗證以下研究假說：

**在 Human-Robot Co-Manipulation 中，相較於只使用 Force/Torque 與 Object Pose，加入空間 Tactile Contact Information，是否能提供額外資訊，幫助區分不同來源與目的的物理作用力？**

這是 ENPM 645 Human-Robot Interaction 課程專案的第一階段。

我們最終希望讓人和機器人共同操作一塊板材，機器人可以辨識人類究竟希望：

- A：主動翻轉板材（ROTATE）
- B：維持目前姿態（HOLD）
- C：在持續外力作用下，與機器人一起抵抗擾動（HOLD + DISTURBANCE）

但這次**只驗證感測資訊是否足夠，不實作完整人機協作控制器**。

請直接實作、執行實驗、分析結果並產生報告。不要只交付設計文件。

---

## 2. 硬體與技術限制

使用者的 GPU 為 NVIDIA RTX 4070 12 GB。

本階段優先使用標準 CPU MuJoCo。

技術要求：

- Python
- MuJoCo
- NumPy
- Matplotlib
- Pandas（必要時）
- 可使用 SciPy 進行基本數值分析

不要使用 Isaac Sim、ROS 2、RL、ViT、Transformer 或大型學習模型。

程式必須支援 Headless Simulation，不能依賴 GUI 才能執行實驗。

請先檢查目前工作目錄、Python 與 MuJoCo 版本，以及現有專案結構。若已有相關程式，優先重用；否則建立獨立的 Phase 0 實驗目錄。

---

## 3. 最小物理場景

請建立下列元件：

### 3.1 Rigid Panel

一塊具有實際質量、慣性、摩擦與碰撞幾何的長方形剛性板材。

初始尺寸可以參考：

- 長度：0.6 m
- 寬度：0.2 m
- 厚度：0.02 m
- 質量：1.0 kg

這些不是強制規格，可以為了模擬穩定性調整，但必須記錄修改原因。

### 3.2 Simplified Robot Gripper

第一階段不需要完整 6-DoF 機械臂。

請建立簡化的雙側夾爪，從板材的一端夾住板材。

要求：

1. 兩側夾爪和板材之間必須存在真實的 MuJoCo 接觸與摩擦。
2. 不可以直接把板材 weld 到夾爪上，再假裝產生 tactile 訊號。
3. 夾爪應具有合理的預壓力，避免板材立即滑落。
4. 可使用簡化的單軸可旋轉 wrist fixture，讓板材能在限制的旋轉方向上運動。
5. 若使用固定支架，必須說明支架承受哪些負載，不能把它承受的力量錯誤視為機器人觸覺資訊。

目標是建立真正的 Panel-Gripper Contact Interaction。

不要求完整 90° 翻轉；先完成小角度的旋轉與負載測試即可。

### 3.3 Virtual Human Hand

在板材另一端設置 Virtual Human Hand。

不需要人體模型或手指模型。

使用 Spring-Damper Impedance Model 產生人類施力：

\[
F_h=K_h(p_h^*-p_h)+D_h(v_h^*-v_h)
\]

其中：

- \(p_h^*\)：人類希望到達的握持點位置
- \(p_h\)：板材實際握持點位置
- \(K_h\)：人手等效剛性
- \(D_h\)：人手等效阻尼

請設定合理的力與速度限制。

人類透過改變虛擬握持點的目標位置，對板材施加真實物理外力。

可以使用 MuJoCo `mj_applyFT` 或等效機制。

重要：**不要直接修改板材的 qpos 來模擬人類翻轉。**

### 3.4 External Disturbance

另行建立一個獨立的外部施力來源。

要求：

- 能在板材指定位置施加持續外力。
- 能設定施力方向、大小、位置與開始／結束時間。
- 與 Virtual Human Hand 的力完全獨立。
- 可以產生不同於人類握持點的負載路徑。

C 情境中，Virtual Human 仍然想保持板材原本姿態，只是環境另外施加持續擾動。

---

## 4. 感測器設計

請建立三組不同的觀測資訊。

### Sensor 1 — Force/Torque

優先實作模擬 wrist F/T sensor。

記錄：

- Fx, Fy, Fz
- Tx, Ty, Tz

如果簡化的機械結構不適合直接設置 wrist sensor，可以從夾爪接觸力重建等效 wrench。

但必須清楚標註：

- `measured_wrist_ft`
- 或 `reconstructed_contact_wrench`

不能將兩者混為一談。

### Sensor 2 — Tactile Contact Distribution

這是本實驗的重點。

請先確認目前 MuJoCo 版本是否支援 `mujoco.sensor.touch_grid`。

優先使用官方 Plugin。若無法使用，則使用 `mj_contactForce` 取得真實 contact forces，結合 contact positions，在夾爪局部座標系中建立簡化的 spatial contact map。

初始可考慮每個夾爪接觸面 4×4 的接觸網格。

如果物理引擎只產生少數 contact points，請保留真正的稀疏接觸資料，不要用插值製造不存在的空間資訊。

希望記錄：

- Contact location
- Normal force distribution
- Tangential force components
- Center of pressure（在可計算且定義合理時）
- Contact count
- Slip / relative contact motion（若可取得）

請明確說明使用的是接觸力網格、接觸深度，還是其他 tactile proxy，不要將它們都稱為真實壓力分布。

### Sensor 3 — Object State

直接使用 MuJoCo Ground Truth：

- Object orientation
- Angular velocity
- Object position（必要時）
- Linear velocity（必要時）

本階段不需要 camera 或 computer vision。

---

## 5. 三種實驗情境

### A — Human-Initiated Rotation

Virtual Human 希望旋轉板材。

例如：

- 初始角度 0°
- 目標角度 +15° 或 +30°
- 人類手部目標逐漸改變
- 沒有額外環境擾動

人類的力量必須透過 spring-damper 物理模型產生。

記錄人類施力、物體實際運動、wrist F/T 與 tactile distribution。

同時測試正向與反向旋轉。

### B — Hold

Virtual Human 希望維持目前角度。

例如：

- 初始角度 0°
- 目標角度 0°
- 沒有明顯外部擾動
- 可以存在小幅正常的手部修正

預期板材維持在初始姿態附近。

此情境用來建立正常握持狀態下的感測基準。

### C — Hold Under Sustained Disturbance

Virtual Human 希望維持板材角度，但環境施加持續外力。

例如：

- 初始角度 0°
- 人類目標角度 0°
- 外部擾動在模擬開始後某個時間點啟動
- 擾動持續約 1–2 秒
- 人類透過 spring-damper controller 嘗試抵抗

請觀察不同擾動位置及方向對 wrist F/T、tactile distribution 和 object motion 的影響。

注意：C 的 Human Intent 是 HOLD，不是 ROTATE。

---

## 6. 最重要的對照實驗：A vs. C

我們真正想驗證的是：

**當 A 和 C 產生相近的 Wrist F/T 與 Object State 時，Tactile 是否仍存在可用來區分兩者的空間資訊？**

請設計兩類比較：

### Experiment 1 — Natural Conditions

直接比較 A/B/C 三種情境的原始感測訊號。

觀察不同情境下：

- F/T 的大小與方向
- Tactile 分布
- Contact center
- Shear distribution
- Object orientation

### Experiment 2 — Similar-Wrench Conditions

這是必要實驗。

請嘗試調整 A/C 的人類施力、擾動大小與施力位置，或從已產生的資料中尋找可比較的時間窗，讓兩種情境的：

- Wrist F/T
- Object angle
- Angular velocity

盡可能接近。

然後比較它們的 spatial tactile contact distribution。

必須量化匹配品質，不可以只宣稱兩組力矩「差不多」。

**重要物理限制：**

如果兩種情境對理想剛體產生完全相同的合外力矩、運動歷史與夾爪接觸邊界條件，tactile 也可能無法區分它們。

因此，請不要假設 tactile 一定有效。

我們的目標是測試能否觀測到差異，而不是刻意製造一個有利於 tactile 的結果。

---

## 7. 資料收集

請先建立小型 Pilot Dataset，不需要大量生成資料。

最低要求：

- A：10 個 episodes
- B：10 個 episodes
- C：10 個 episodes

每個 episode 可先執行約 3–5 秒，依穩定性調整。

建議物理步長約 0.002 秒，感測資料可用 100 Hz 記錄。

固定 random seeds，以確保結果可重現。

適度隨機化：

- Human stiffness / damping
- Human movement speed
- External force magnitude
- External force application point
- Initial object angle
- Contact friction（小幅變化）

請保存每個 episode 的完整 metadata，包括 scenario、參數、seed 與 MuJoCo 版本。

資料分為兩組：

**Observation：**

- wrist_ft
- object_pose
- object_velocity
- tactile_map
- contact_count

**Privileged Ground Truth：**

- human_goal
- human_applied_force
- external_disturbance_force
- force_application_points
- scenario_label

之後做意圖辨識時，不能直接使用 Privileged Ground Truth 當作模型輸入。

---

## 8. 分析與視覺化

請至少輸出：

1. A/B/C 的 Wrist F/T 時間序列比較。
2. A/B/C 的 Object Angle 與 Angular Velocity。
3. A/B/C 的 Tactile Heatmaps。
4. Contact Center 或 Contact Force Distribution 的變化。
5. A/C Similar-Wrench Conditions 的對照圖。

請明確分析：

- Tactile 是否真的隨施力條件改變？
- Tactile 是否有不同於總 F/T 的空間特徵？
- 這些差異是否在匹配 F/T 與 object state 後仍存在？
- 觀察到的差異會不會只是物體角度、夾爪預壓或接觸點數量不同造成？
- 現在的模擬接觸模型是否足夠合理？

若資料品質足夠，可以額外嘗試簡單的 Logistic Regression，比較 F/T、F/T + Object Pose、F/T + Object Pose + Tactile，但本階段不強制要求訓練模型。

不要只依靠一個分類 accuracy 就宣布 tactile 有用。

---

## 9. 物理驗證要求

請特別檢查以下問題：

1. 施加人類外力時，物體是否真的透過物理模擬產生反應？
2. Wrist F/T 是否能反映經由板材傳遞的負載？
3. Tactile 數據是否來自真正的接觸，而不是人工產生的數值？
4. 是否出現穿模、不合理振盪、爆炸性加速度或接觸遺失？
5. A/B/C 的差異是否只是程式直接輸入了不同標籤，而不是感測到了真正不同的物理現象？

如果發現接觸模型沒有產生有效的空間資訊，請先嘗試合理的幾何、摩擦或接觸參數調整。

但不要為了讓實驗有正面結果，而加入人為的 tactile pattern。

---

## 10. 交付成果

請建立一個可重現的 Phase 0 實驗。

建議檔案：

```text
phase0_tactile/
├── models/
│   └── tactile_pilot.xml
├── scripts/
│   ├── run_experiment.py
│   └── analyze_results.py
├── results/
│   ├── data/
│   └── figures/
├── README.md
└── report.md
```

實際架構可按照現有 repository 調整。

README 必須包含：

- 環境與版本
- 安裝與執行指令
- A/B/C 情境設定
- 感測器實作方式
- 如何重現實驗

請提供一個能執行全部情境的 Headless 指令。

---

## 11. 最終報告

請在 `report.md` 回答以下問題：

**Q1.** 我們是否成功建立真正的 Panel-Gripper Contact Simulation？

**Q2.** Wrist F/T 能夠觀測到哪些交互作用資訊？

**Q3.** Spatial Tactile 是否提供額外的可觀測資訊？

**Q4.** 在 A/C 受力與物體狀態接近時，是否仍能觀察到 tactile 差異？

**Q5.** 現在的物理模型和資料是否足以進入 Intent Recognition 階段？

**Q6.** 下一階段應優先加入完整機械臂、改善 tactile 模型，還是考慮其他感測來源？

請明確給出以下其中一個判斷：

- **GO：** 發現可信的 tactile 額外資訊，可以繼續。
- **CONDITIONAL GO：** 有初步證據，但需要改善接觸模型或驗證方式。
- **NO-GO：** 尚未觀察到足夠資訊，不應直接進入意圖辨識訓練。

如果結果為 NO-GO，請解釋原因，不要自行擴大專案去做 ViT、完整機械臂或 RL。

---

## 12. 執行原則

這是 Class Project 的可行性驗證，不是追求新的研究突破。

優先順序：

1. 物理正確
2. 感測資料可信
3. 實驗可以重現
4. 結果可視化
5. 清楚說明限制

請實際完成程式、執行 Pilot，並根據真實輸出的資料撰寫結論。

不要只提出計畫，也不要把尚未執行的實驗寫成已完成。

如果某個技術部分不可行，請採用最小合理替代方案，清楚記錄限制，並繼續完成可執行的部分。

**現在開始實作 Phase 0，最後回報實際執行結果、圖表位置，以及是否建議進入下一階段。**