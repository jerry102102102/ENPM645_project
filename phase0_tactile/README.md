# Phase 0 — Tactile Observability Pilot

已在本機 CPU MuJoCo 完成 **A/B/C 各 10 episodes，共 12,000 筆同步樣本**。本次結論為 **NO-GO：尚未證明 tactile 相對 F/T + object state 有額外的意圖辨識資訊**，不是模擬無法執行。先讀 [實驗報告](report.md)；原始要求保存在 [specification.md](specification.md)。

## 環境與重現

實際驗證環境：macOS arm64、Python **3.12.11**、MuJoCo **3.3.5**、NumPy **2.2.6**、Matplotlib **3.10.6**。本機原有另一個專案安裝 MuJoCo；為避免改動其依賴，本專案另建 `.venv`，沿用同一版 MuJoCo。沒有使用 Docker、GPU、ROS 或大型模型。輸入規格提及 RTX 4070，但本次實際執行的主機是 Mac，物理計算使用 CPU。

從 repository 根目錄安裝：

```bash
python3.12 -m venv .venv
.venv/bin/python -m pip install -r phase0_tactile/requirements.txt
```

一個指令執行全部 30 episodes、物理對照、重播檢查及分析，完全 headless：

```bash
.venv/bin/python phase0_tactile/scripts/run_all.py --output phase0_tactile/results/reproduction
```

輸出目錄必須尚不存在，以免覆寫既有證據。物理模擬不使用 viewer 或 renderer；只有額外指定 `--render` 才建立 3 張場景截圖，可能需要系統圖形函式庫。Matplotlib 圖表固定使用 Agg，不需要 GUI。

已執行的正式資料在 `results/pilot/`。只重新分析既有資料：

```bash
.venv/bin/python phase0_tactile/scripts/validate_physics.py --output phase0_tactile/results/pilot
.venv/bin/python phase0_tactile/scripts/analyze_results.py --output phase0_tactile/results/pilot
```

`run_all.py` 最後保存 source/artifact SHA-256 到 `provenance.json`。若重新分析已保存的結果，原有 provenance 雜湊不會自動更新；請保留原始結果並在新資料夾重跑，避免把新程式的輸出當成原始證據。不同平台或 MuJoCo 版本可能有浮點差異；精確重播僅在本次環境驗證。

## 場景與物理

- 板材：0.6 × 0.2 × 0.02 m，1 kg，free joint；尺寸沒有縮減。均質 box 慣性由 MuJoCo 計算。
- 夾爪：上下各一個 0.09 × 0.14 m 接觸面，兩個滑動關節分別施加 −40/+40 N 的恆定 motor 力。實際接觸法向力會隨重量和運動變化，不能直接當成 40 N。
- Wrist fixture：世界位置 `(−0.25, 0, 0.8)` m，繞世界初始 y 軸旋轉，角度範圍 ±0.7 rad；彈簧 1 N m/rad、阻尼 0.8 N m s/rad。支架約束三軸平移、roll/yaw，承受相應反力與力矩；pitch 亦受彈簧、阻尼及限位影響。這不是自由漂浮夾爪，也不是完整機械臂控制器。
- 板材與夾爪只有真實碰撞／摩擦，**沒有 weld/equality**。`condim=3`，elliptic friction cone；`solref=(0.01,1)`，`solimp=(0.95,0.99,0.001)`。這些是數值接觸參數，不代表校正過的橡膠材料。
- 人手握點在板材 local `(0.27,0,0)` m，用 `F = Kh(target−point) + Dh(target_velocity−point_velocity)`，經 `mj_applyFT` 作用於板材。力範數上限 50 N，目標速度上限 0.35 m/s；固定垂直目標偏移提供約半個板材重量的支撐。
- 除每回合初始放置外不修改 `qpos`；運動皆由 `mj_step` 積分。先 settle 1.5 s，再記錄 4 s；dt=0.002 s，100 Hz 取樣，每 episode 400 筆。
- 觀測在當步施力與 `mj_forward` 後、`mj_step` 前記錄，避免力與位姿錯開一個 integration step。

## A/B/C 與隨機化

| 情境 | 人手目標 | 獨立外力 | Human intent |
| --- | --- | --- | --- |
| A | 0.7 s 起平滑改變握點，目標轉角大小 15–25°；5 正向、5 反向 | 無 | ROTATE |
| B | 維持初始目標 | 無 | HOLD |
| C | 維持初始目標，以 spring-damper 抵抗 | 0.7 s 開始，2.7 s 開始移除，各 0.15 s 平滑過渡；向上／下 7–22 N | HOLD |

10 個 nuisance blocks，seed `64500+i`；同 block 的 A/B/C 共用 `Kh∈[65,110] N/m`、`Dh∈[4,8] N s/m`、摩擦係數 `[0.85,1.05]`、初始角 `±0.015 rad` 和預壓。人手轉動時間隨機，並延長到滿足速度限制。C 施力位置 local x∈[−0.08,0.18] m、y∈[−0.035,0.035] m、z=0，與人手握點分開；方向由 block 正負號指定。開始／結束時間、大小與位置都保存在 metadata，可修改 `parameters()` 後用新輸出目錄測試。

穩態重力和 fixture 彈簧造成 settle 後 pitch 約 −1.76° 至 −0.49°，所以「保持」以 settle 後基線衡量，不能宣稱每回合精確從 0° 開始。C 有限剛性下仍會偏轉，不代表其 intent 變成 ROTATE。

## 感測定義

1. **F/T = `reconstructed_contact_wrench`**：從 `mj_contactForce` 重建板材作用在兩個 pad 上的合力／力矩，以 wrist body frame 表示，力矩參考 wrist origin。順序 `[Fx,Fy,Fz,Tx,Ty,Tz]`，單位 N、N m。不是直接 wrist sensor；不包含夾爪自身重量／慣性造成的完整支架反力。
2. **`tactile_map`**：確認本機官方 `mujoco.sensor.touch_grid` 可用，每側 `(3,4,4)`，channel 是 sensor frame `[z,x,y]`。它是角度分箱後的力，並非均勻平面壓力像素。上下 site 朝向相反，所以不同 pad 的 shear 通道不能直接視為同一 world 軸。配置 `fov=(110,85)` deg、`gamma=0`，site 往各 pad 背面退 0.08 m，以涵蓋接觸區。[官方 3.3.5 plugin 說明](https://github.com/google-deepmind/mujoco/blob/3.3.5/plugin/sensor/README.md)
3. **`planar_contact_map`**：保留同一批真實接觸，將 local x/y 分成 4×4，只累加每格的 normal magnitude、pad-x shear、pad-y shear（N）。沒有插值，沒有除面積。邊界點分配到最外側 bin；實際接觸位置另外保存在 raw contacts。本次每側僅 4 個接觸點，不能稱為 16 個獨立 taxels。
4. **`contact_center`**：每側接觸位置 local x/y，以正法向接觸力加權，單位 m；是離散接觸中心／CoP proxy，沒有連續壓力場的宣稱。
5. **`slip_speed`**：所有接觸點中最大的板材相對 pad 切向速度範數（m/s），由 Jacobian 和 qvel 計算；不是「已掉落」標籤。
6. **Object state**：模擬 ground truth，`object_pose=[world xyz, quaternion wxyz]`；`object_velocity=[world angular xyz, linear xyz]`。角度分析採 pitch，角速度採 world y。這是 oracle state，不是實際相機估計。

MuJoCo 接觸座標與軟接觸限制參考 [3.3.5 computation](https://mujoco.readthedocs.io/en/3.3.5/computation/)。這個 pilot 的 tactile 與 F/T 都由同一組物理接觸衍生，不能當成互相獨立的量測來源。

## 檔案與資料邊界

```text
models/tactile_pilot.xml
scripts/run_experiment.py    # 30 episodes；真實接觸與觀測
scripts/validate_physics.py  # 合力矩等價、無接觸、dt 減半、資料及重播檢查
scripts/analyze_results.py   # 同 block 時間窗匹配、CSV、6 張圖
scripts/run_all.py           # 一個指令完成全部步驟
results/pilot/
  data/{A,B,C}_000...009/
    observations.npz        # 感測候選輸入，含 object-state oracle
    privileged.npz          # 人手目標／力、環境力、施力點、intent、scenario
    diagnostics.npz         # 接觸明細、穿透、力平衡、fixture angle
    metadata.json           # seed、完整參數、版本、座標約定、驗證摘要
  manifest.json
  physics_controls.json
  analysis.json
  episode_summary.csv
  matched_pairs.csv
  figures/01...06*.png
  provenance.json
```

`diagnostics.npz/raw_contacts` 每列依序為：`sample, pad, local_xyz(3), world_xyz(3), pad_force_xyz(3), normal_N, slip_m_s, distance_m`，共 14 欄。完整欄名也在 metadata。`pad=0/1` 為 upper/lower。

未來若做分類，只能載入 observations 中預先選定的欄位；不能把 scenario 檔名、metadata、privileged 或 seed 當成模型輸入。這次不訓練分類器。分析只用 scenario／seed 分組和預先指定的 active window 範圍，不把人手目標或外力值拿來建構 tactile。

正式 pilot 的資料、摘要及圖表一併保存供兩人協作檢查。開發試跑與預設 reproduction 目錄不納入 Git。
