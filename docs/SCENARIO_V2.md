# Fixed edge-grasp layout — scenario v2

這是 **Phase 1A 的場景幾何修正版**（scenario 2.0.0），不是重寫 Phase 0 v2。使用者已選擇：Robot L/R 在同一條長邊，人手在對側長邊。新設定為 [scenario_v2.json](../phase1a/config/scenario_v2.json)，實際碰撞模型為 [edge_grasp_v2.xml](../phase1a/models/edge_grasp_v2.xml)。原 scenario_v1、原 MJCF、舊結果均保留。

![Fixed local geometry and cross-section](../results/phase1a/edge_v2/geometry/fixed_edge_layout.png)

硬體已固化為 [hardware v1](HARDWARE_V1.md)：兩端各 0.50 kg 的雙 pad 夾爪、既有 sensor frames 和一個固定 spring-damper 人手。每個新版 episode 另外綁定 hardware_id/version/hash。

## 三個位置與「固定」的定義

Panel local frame 原點在 COM。板材長 0.60 m（X）、寬 0.20 m（Y）、厚 0.02 m（Z），邊界為 X=±0.30、Y=±0.10、Z=±0.01 m。

| Actor | 名義位置 [X,Y,Z] m | 幾何意義 |
|---|---|---|
| Robot L | **[-0.20,+0.10,0]** | +Y 長邊上的 between-jaw grasp frame |
| Robot R | **[+0.20,+0.10,0]** | +Y 長邊上的 between-jaw grasp frame |
| Human | **[+0.27,-0.10,0]** | -Y 側面上的虛擬施力 site |

機器人兩端相距 0.40 m，相對 panel 的名義 quaternion 都是 [1,0,0,0] wxyz。人手保留 X=0.27 的偏心力臂，讓作用力能產生 Y 軸旋轉力矩；沒有移到 X=0 的中心位置。

這些 local nominal positions 跨 A/B/C、seed 與資料蒐集固定。人手 site 依附 panel，local 座標在動態中保持不變。Robot grasp frames 是預先定義的名義位置，**不是 weld，也不是強制指定實際 contact points**；free roots、pad slides、接觸與微滑移仍正常解算。新版測試中 robot grasp frame 的相對偏移最多約 1.05 cm，尚未建立零滑移或固定相對姿態的 controller 保證。

## Pad 幾何與 contact semantics

每個 gripper 的兩片 pads 尺寸保持 0.09 × 0.14 × 0.02 m。Upper/lower pad 中心在 local Z=+0.02/-0.02；夾爪 frame 中心 Z=0 是兩片 pad 的中間，不是 pad 穿進板材。

| 幾何量 | Robot L | Robot R |
|---|---|---|
| Pad X 範圍 | [-0.245,-0.155] m | [+0.155,+0.245] m |
| Pad Y 範圍 | [0.03,0.17] m | [0.03,0.17] m |
| 與板材的名義重疊 Y 範圍 | [0.03,0.10] m | [0.03,0.10] m |
| Upper 名義接觸表面 | Z=+0.01 m | Z=+0.01 m |
| Lower 名義接觸表面 | Z=-0.01 m | Z=-0.01 m |

因此這是**從長邊伸入、上下夾住板材**，不是沿板材側面水平推擠。名義 contact patch 是面積區域；MuJoCo 的實際 contact manifold 是其中受當下姿態/接觸形變影響的離散點。`grasp_semantics` 明確存放 nominal anchor、pad bounds、contact patch bounds、edge、jaw axis 與 human model。

Human 仍是單一 point spring-damper，沒有新增手指碰撞幾何。紅色球是 site 標記，不代表已模擬真實人手摩擦抓取。兩個 robot 仍是簡化 grippers，沒有新增完整手臂、IK 或可達性證明。

## 原場景到底哪裡有問題

![Before and after](../results/phase1a/edge_v2/geometry/before_after_top.png)

舊 robot anchors=[±0.20,0,0] 的確位於板材中面內部，但那是兩片 jaws 中間的虛擬座標原點；**實體 pads 接觸的是板材上下表面**，名義 pad Y 範圍為 [-0.07,+0.07]。所以原本是內側面夾持，不是邊緣握持，也不是 pad 幾何穿模。

舊 human site=[0.27,-0.08,0] 則的確位於板材體積內部，是理想化的內部施力點。新版把它移到 Y=-0.10 側面，並同步修正 spring-damper 的 target geometry；不能只移 site 卻繼續用舊 HAND 常數計算目標。

## 變更範圍與證據

MJCF 只有模型名稱、left/right root 的 Y placement、人手 site 的 Y placement 改變。Panel mass/inertia、pad geometry、sensor frames、friction、preload、motor limits、baseline gains、task/seeds 皆不變。初始化與 human target 改由新 variant 從實際模型讀取，支撐、擾動與 sensing 重用原模組。

[結果報告](../results/phase1a/edge_v2/report.md) 包含 4 個 smoke episodes、17 項 tests、新舊 geometry audit、372-array exact replay、每步 Newton–Euler 檢查、獨立接觸移除對照與舊版回歸。所有驗證僅限本次 MuJoCo 場景與測試時段，不宣稱 force closure、真實硬體安全或額外力矩追蹤。

```bash
# 新版完整重現；選擇未使用的輸出位置
uv run --locked python -m phase1a.scripts.run_edge_scene --output results/phase1a/reproduction-edge

# 僅收集新版；可搭配 --controller module:Class
uv run --locked python -m phase1a.scripts.run_pipeline_smoke \
  --scenario-config phase1a/config/scenario_v2.json \
  --output results/phase1a/dev-edge-smoke
```

歷史 `run_all` 與未指定 `--scenario-config` 的 smoke 預設仍是 v1，以免靜默改變舊實驗。新的 controller 工作請明確指定 v2。每個 episode 的 scenario_version / model_hash / config_hash 可辨別兩者；reader 依 registry 解析正確版本。
