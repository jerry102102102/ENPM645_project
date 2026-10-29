# 目前硬體 — 雙 Panda 協作板材

供後續討論資料收集、訓練與驗證使用。**目前只有模擬；沒有實體機器人或已訓練模型。**

Scenario `3.0.0` / Hardware `2.0.0`；MuJoCo `3.3.5`、Python `3.12.11`、uv，無 Docker。

## 幾何與物理連接

| 元件 | 固定設定 |
|---|---|
| 機械臂 | MuJoCo Menagerie Franka Panda ×2，每臂 7 軸，base 固定於 world |
| 板材 | 剛性長方體，0.60 × 0.20 × 0.02 m，1 kg；初始 COM `[0,0,0.80]` m，繞 Y 約 +0.336° |
| Panel local frame | 原點為板材 COM；X 長邊、Y 短邊、Z 厚度方向 |
| World frame | Z 向上，重力 `[0,0,-9.81]` m/s² |
| 左／右 base | World `[-0.45,+0.65,+0.25]` / `[+0.45,+0.65,+0.25]` m，yaw 均 −90° |
| 左／右名義握點 | Panel-local `[-0.20,+0.10,0]` / `[+0.20,+0.10,0]` m；同側長邊，相距 0.40 m |
| 人手作用點 | Panel-local `[+0.27,-0.10,0]` m；對側長邊 |
| 自訂夾爪 | 每臂 upper/lower 各一片 pad，共 4 片；不是原廠 Panda fingers |
| 單片 pad | 0.09 × 0.14 × 0.02 m，0.10 kg，沿 gripper local Z 滑動 |
| Pad preload | L upper/lower、R upper/lower：`[-40,+40,-40,+40]` N；是 motor commands，不是固定接觸力 |
| 末端連接 | link7 local Z +0.107 m 為 flange；夾爪相對 flange 沿 Z +0.12 m、繞 X −90°；每臂增加 0.08 kg adapter |

板材有 freejoint；手臂與夾爪沒有 freejoint。沒有 weld 固定板材，也不靠 gripper-root virtual wrench 承重；負載經由手臂致動器與 pad 接觸傳遞。名義握點固定於 panel local frame，實際接觸點由模擬解算。

保留 Panda 慣性、關節限制與碰撞幾何；碰撞檢查受模型 exclusion 設定限制。Runtime 第一摩擦係數為 `1.0482960672343025`，應透過 Python 入口載入。

## 人手與外力

Virtual human 是單一作用點的 spring-damper 模型，**沒有完整人類手臂／手指關節**。力由 target 與實際位置／速度差計算：stiffness 約 98.21 N/m、damping 約 7.30 N·s/m、力上限 50 N、target speed 上限 0.35 m/s。

外力獨立作用於板材，當前 panel-local 作用點 `[+0.063691,-0.015331,0]` m。人手與外力的 force、作用點及 about-panel-COM wrench 分別記錄；它們的來源與目標屬模擬 truth。

## 可取得的感測與狀態

以下 shape 不含時間軸；arm 順序固定 left/right，joint 順序為 1…7。

| 資料 | Shape／定義 |
|---|---|
| 左右 tactile | 各 `(2,3,4,4)`：upper/lower × sensor-local z/x/y force × 4×4 angular bins；N/bin，不是 Pa 或相機影像 |
| `wrist_ft_left/right` | 各 `(6,)`，panel-pad 接觸重建的 panel-on-gripper wrench；gripper-local、about gripper origin |
| `wrist_sensor_left/right` | 各 `(6,)`，MuJoCo wrist transmitted wrench；包含下游重量與動力學，未做補償，不能直接與上列互換 |
| Object contact wrench | `(6,)`，gripper-on-panel、world frame、about instantaneous panel COM |
| Arm q / dq / actuator torque | 各 `(2,7)`，rad / rad/s / Nm；actuator torque 不是外部接觸 torque |
| End-effector pose / velocity | `(2,7)` world xyz+wxyz / `(2,6)` world angular xyz→linear xyz；EE 是兩 pad 中間，不是 link7 |
| Gripper position / velocity | 各 `(2,2)`，m / m/s；另記錄 4 個 pad actuator forces，N |
| Object state | World position、quaternion wxyz、linear/angular velocity；直接讀模擬器，非視覺估計 |
| Contact state | 每側 `(2,)`，upper/lower 接觸資訊 |

Wrench 排列統一 `[Fx,Fy,Fz,Tx,Ty,Tz]`。Tactile 使用 `mujoco.sensor.touch_grid`，熱圖顯示 normal channel，原始資料保留三個 channels。可渲染同步影片；尚無 VLM、RGB-D 或視覺 pose estimator。

## 控制與記錄

- Physics/control：500 Hz；高層 reference/observation：100 Hz；episode 前 settle 2 秒。
- 正式 transition：`o_t → reference → 5 個 control/physics 子步 → o_next`。
- 現有命令：`arm_position_target (2,7)`，rad；`pad_force_command (4,)`，N。每臂 joint 1–4 torque bounds ±87 Nm、5–7 ±12 Nm；這是 position-servo 模型，沒有直接 joint-torque adapter。
- 高層介面：`desired_assist_torque`，scalar、World +Y、instantaneous Panel COM、Nm，表示額外協助力矩。**目前尚無 assistance tracking、allocator 或 Expert。**
- q/dq 已記錄，policy 預設不使用；`with_joint_states` 可開啟。Privileged truth、commands、render states 不自動成為 policy inputs；Expert labels 仍為 NaN / invalid。

## 已驗證的動作

A：旋轉約 +20°、抬升 3.05 cm 後停留。B：抬升 7.98 cm 後停留。C：旋轉 −11.94° 後承受 world −Z 20 N（4.3–8 秒）。三段各 8 秒，維持四片 pads 接觸，無非預期穿透超標、數值警告或 actuator saturation；168 arrays exact replay。

**這是預設 joint trajectories 的物理展示，不是意圖辨識結果。** 指定軌跡已驗證；任意姿態、90° 翻轉與實體機器人未驗證。展示檔 `samples.npz` 混有 truth，不能直接當正式 policy dataset。

## 精確規格與入口

[場景／初始 q／human 參數](../phase1b/config/dual_panda_v1.json) · [完整硬體與限制](../phase1b/config/hardware_v2.json) · [資料格式](DATA_SCHEMA_V2.md) · [Controller 介面](CONTROL_HANDOFF_ARMS.md) · [動作與驗證](../results/phase1b/motion_examples_v1/README.md) · [執行方式](../phase1b/README.md)
