# Phase 1B — Dual Franka Panda physical integration

兩支固定基座 Panda、14 個 arm joints 與原本四片 tactile pads 已整合。這個版本使用 **joint-position hold baseline** 驗證支撐與資料管線；不包含物體 wrench allocation、assistance tracking、Expert 或模型訓練。

- [四項驗收與物理測試](../results/phase1b/dual_panda_v1/report.md)
- [完整機構圖](../results/phase1b/dual_panda_v1/visuals/dual_panda_scene.png)
- [場景／關節／tactile 同步影片](../results/phase1b/dual_panda_v1/visuals/dual_panda_sync.mp4)
- [新版資料格式](../docs/DATA_SCHEMA_V2.md)
- [組員 Controller Handoff](../docs/CONTROL_HANDOFF_ARMS.md)

## 重現

```bash
uv sync --locked
uv run --locked python -m phase1b.scripts.run_suite --output results/phase1b/reproduction --render
```

已 vendoring 所需 Menagerie 檔案，平常不必連網或重新下載。`--render` 需要本機 `ffmpeg`；省略時仍執行資料收集、正反物理檢查、全部新舊單元測試及 exact replay。輸出不得覆寫，請使用新路徑。`dev*` / `reproduction*` 不加入 Git。

只收集一次短 episode：

```bash
uv run --locked python -m phase1b.scripts.collect --scenario A --output results/phase1b/dev-A
```

macOS 開啟原生 MuJoCo viewer：

```bash
uv run --locked python -m phase1b.scripts.launch_viewer --scenario B
```

Viewer 跑同一個 position-hold baseline，未另存資料；關閉視窗即可停止。啟動器會在 macOS 為 uv 的 base Python 設定子程序專用 library path，解決原始 mjpython 找不到 libpython 的問題。本次已驗證原生 viewer 啟動、1,000 physics steps 與正常關閉，零 warnings；未驗證滑鼠／鍵盤的互動操作。

## 固定幾何與來源

- **Scenario 3.0.0 / Hardware 2.0.0**：`config/dual_panda_v1.json`、`config/hardware_v2.json`，`dual_panda_v1` 表示第一個 Panda 整合版本，不是舊 scenario v1。
- Base L = `[-0.45, +0.65, +0.25] m`，Base R = `[+0.45, +0.65, +0.25] m`；world Z 軸 yaw 都為 −90°。基座固定在 world，無 freejoint。
- 原 panel 0.60 × 0.20 × 0.02 m、1 kg；名義 gripper frames 為 panel-local `[-0.20,+0.10,0]` / `[+0.20,+0.10,0]`；virtual human `[+0.27,-0.10,0]`。A/B/C、起始角與人手／外力參數沿用 Phase 1A v2。
- upstream Panda hand/fingers/tendon/equality 不載入執行場景；保留七軸手臂與慣性／碰撞網格，接上原自訂夾爪。這不是原廠 Panda hand。
- Panda link7 後沿 local Z +0.107 m 設 tool flange；custom gripper 相對 flange 沿 Z +0.12 m，旋轉 X −90°。增加 **0.08 kg** mount adapter，連到既有 flange block；原 gripper root freejoint 移除，flange block 啟用碰撞。四片 pad 的尺寸、質量、slide ranges、preload 與 touch-grid 設計保留。
- Arm collisions 啟用 self / other arm / panel / floor / custom gripper，遵循 MuJoCo 相鄰剛體過濾及 upstream link0-link1 exclusion。檢查不涵蓋被模型明訂排除的碰撞對；沒有為了通過測試新增穿透遮蔽。
- 全場只有 panel freejoint，沒有 equality constraints。IK 僅用於初始配置；正式 stepping 不寫 qpos。模型也提供 `nominal_grasp` keyframe。
- Menagerie revision `f3b6013629c7b94f8ee93fd977b161fa8350f9bc`；來源、每檔 SHA256 與 Apache-2.0 license 在 `vendor/franka_emika_panda/`。啟動時驗證模型、vendor 與 compiled hardware。正式 runtime 依 config 將所有 geoms 的第一摩擦係數設為沿用的 1.0482960672343025；重現以 Python 入口為準，不只單獨開啟 XML。

## 本版本控制邊界

14 個 arm actuators 沿用 upstream **position servos**：raw ctrl 單位 rad，內部產生 bounded torque；前四軸每軸 ±87 Nm、後三軸 ±12 Nm 是該模擬模型的 actuator bounds。四個自訂 pad motors 的 ctrl 單位 N，名義 `[-40,+40,-40,+40]`。底層的實際 actuator generalized torque 與命令各自記錄。

`desired_assist_torque` 沿用 World +Y / instantaneous Panel COM / Nm。hold baseline 忽略此 reference，±0.02 Nm 測試已確認不改變 physics；這是介面隔離測試，不是追蹤成功。若組員需要直接 joint-torque control，需另行建立 torque actuator/adapter 版本並重做支撐、限制與資料驗證，不能把 Nm 塞進目前 position targets。

Joint States 無條件記錄，policy 預設排除；`with_joint_states` profile 僅額外開啟 q/dq，供未來消融，尚未訓練任何模型。
