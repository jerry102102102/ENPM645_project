# Dual Franka Panda — 整合驗收報告

**整體狀態：完成（Complete），範圍限於本次雙臂物理整合與資料記錄。** Scenario 3.0.0 / Hardware 2.0.0 已加入兩支七軸 Panda，透過關節致動器、末端自訂夾爪與接觸支撐板材。尚無完整 Model-Based Controller、assistance tracking、Expert 或 AI 訓練。

## 四項驗收

| 項目 | 狀態 | 具體結果與證據 |
|---|---|---|
| Arm Model Integration | 完成（Complete） | 固定世界基座 ×2、七軸手臂 ×2；名義握點 IK 位置誤差 L=2.50e-7 m、R=5.76e-10 m；14 joints 均在 limits 內；native MuJoCo 載入與 render 成功 |
| Physical Coupling | 完成（Complete），限已測 episodes | A/B/C 各 4 s 保持四片 pad 接觸；沒有 arm/root virtual wrench、panel weld 或 stepping 中改寫 qpos。移除接觸、關閉 arm actuators、移除 human force 的 controls 確認支撐路徑 |
| Data Pipeline | 完成（Complete） | 4 episodes、1,600 transitions、8,000 子步；q/dq、commands、actuator torques、EE/gripper states 與其他 sensing 對齊；424 arrays exact replay；1,200 frames 的 EE pose 與 raw qpos FK 精確相同 |
| Control Readiness | 部分完成（Partial） | object desired-assist torque 與 joint-position adapter 可接入；wrench allocation、direct joint-torque adapter、assistance tracking/evaluator 與 Expert 尚未具備，均不屬本次實作 |

## 關鍵變更

- Menagerie Panda revision 固定 `f3b6013629c7b94f8ee93fd977b161fa8350f9bc`，保留來源、檔案 SHA256、Apache-2.0 license 與 upstream position actuators。
- 移除 upstream 原廠 hand/fingers，接回既有自訂雙 pad 夾爪及 touch_grid。夾爪 root freejoint 移除；增加 0.08 kg adapter，flange block 啟用碰撞。這是新硬體動力學，不能與舊資料視為同一版本。
- 基座 L/R = `[-.45,.65,.25] / [.45,.65,.25] m`，yaw −90°。名義 panel-local grasp L/R = `[-.2,+.1,0] / [.2,+.1,0]`；human = `[.27,-.1,0]`，A/B/C 與人手／外力參數沿用。
- q/dq 無條件記錄；policy 預設維持原 10 項 whitelist。只有明確選擇 `with_joint_states` 才納入 q/dq，未訓練學習模型。
- 區分舊 contact-reconstructed F/T 與新增 wrist transmitted wrench；兩者的參考點、重量／慣性內容不同，不能直接互換。

## 驗證結果

完整 [machine report](../results/phase1b/dual_panda_v1/verification.json)、[實驗報告](../results/phase1b/dual_panda_v1/report.md) 與 [24 項測試輸出](../results/phase1b/dual_panda_v1/tests.txt)。

| 檢查 | 結果 | 證明範圍 |
|---|---|---|
| 四片 pads 接觸 | A 最少 [3,4,3,4]；B [4,4,4,4]；C [2,2,3,4] | 每個 500 Hz 子步的作用前狀態；短 episode 內接觸保持 |
| 最大 pad 接觸穿透 | 0.523 mm | 小於本次 1 mm 門檻；這是 soft-contact 模擬，不宣稱零穿透 |
| 非預期穿模／警告 | 0 個超過 0.01 mm 的非預期穿透；0 MuJoCo warnings | 已啟用碰撞對與已採樣狀態；不涵蓋排除對或任意路徑 |
| Arm limits / saturation | 最小 joint-limit margin 約 0.148 rad；0 飽和子步 | 本次 nominal/model limits，非真實硬體安全認證 |
| Panel 線性力平衡 | 最大 residual < 4.7e-14 N | 機器人接觸＋human＋external 與 panel acceleration/gravity 一致 |
| 移除 pad 接觸 | 0.3 s 下降 0.322 m；左右 tactile/contact F/T 歸零 | 排除板材被隱藏固定或由幾何外觀支撐 |
| 停用 arm actuators | 0.3 s 下降 0.323 m；joint change 最大 0.591 rad | 支撐依賴手臂致動器，而非 free-root virtual drive |
| 關閉 human force | 機器人垂直接觸力 9.81000065 N，panel weight 9.81 N | 兩臂可自行提供基本重力支撐 |
| Signed reference 對照 | ±0.02 Nm 與零 reference 的 sensing/state 相同 | baseline 確實未執行 assistance；不是 tracking pass |
| 新舊測試 | 24/24 通過 | 7 項新整合測試＋17 項原 Phase 1A 測試；舊檔案保留 |

## 畫面與可重現入口

[完整雙臂場景](../results/phase1b/dual_panda_v1/visuals/dual_panda_scene.png) · [同步 MP4](../results/phase1b/dual_panda_v1/visuals/dual_panda_sync.mp4) · [同步 GIF](../results/phase1b/dual_panda_v1/visuals/dual_panda_sync.gif)。影片從記錄的 qpos/qvel 做 native MuJoCo FK，tactile、Joint States 和施力標註使用同一 sample；不是用 IK 生成替代關節量測。

```bash
uv sync --locked
uv run --locked python -m phase1b.scripts.run_suite --output results/phase1b/reproduction --render
```

本機 uv，無 Docker。Headless render 已驗證；原生 viewer 經 uv 啟動器已跑完 1,000 physics steps、正常關閉且零 warnings；滑鼠／鍵盤互動未驗證。正式 reference 不覆寫，重跑用新目錄。

## 限制與組員下一步

現在的 upstream position servos 偏硬，板材只小幅變動，不能把這個持板 baseline 當作人機協作翻轉策略。關節命令為 **rad**，實際 actuator torque 才是 **Nm**；不具備直接輸入 joint torque 的 adapter。

建議組員先讀 [Controller Handoff](CONTROL_HANDOFF_ARMS.md)，明確選定 position 或新 torque-actuator 路線，再驗證小幅正負 desired-assist torque 的 requested → allocated → commands → realized 鏈條，降低命令單位混淆與不可達力矩的風險。完成條件應包含 baseline 分離、接觸保持、飽和與量化 tracking error；通過前不產生有效 Expert Labels。

[資料格式修改](DATA_SCHEMA_V2.md) 已列出欄位、shape、單位、正負號、時間事件與 whitelist。真實機器人、90° 翻轉與跨姿態泛化均未驗證。
