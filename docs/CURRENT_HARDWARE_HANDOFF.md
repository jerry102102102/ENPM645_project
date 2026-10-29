# ENPM 645 — 目前已建立的模擬硬體與介面

本文件供另一個 ChatGPT 或組員理解目前環境，作為後續討論資料收集、訓練與效果驗證的背景。只描述已建立的系統，不提出訓練方案。

- 基準程式版本：`3c9514b`。
- Scenario：`3.0.0`；Hardware：`2.0.0`。
- Repository：https://github.com/jerry102102102/ENPM645_project
- 全部為 MuJoCo 模擬，尚無實體機器人驗證。

## 1. 場景與座標

兩支完整 Franka Panda 位於板材同側，各自用一個自訂夾爪握持板材；單一 virtual human hand 位於對側。板材可以平移、旋轉，承重由手臂關節與夾爪接觸傳遞。

| 元件 | 固定設定 |
|---|---|
| 板材 | 剛性長方體，0.60 × 0.20 × 0.02 m，1 kg |
| Panel local frame | 原點在板材 COM；X 為 0.60 m 長邊，Y 為 0.20 m 短邊，Z 為厚度方向 |
| World frame | Z 向上；重力 `[0,0,-9.81]` m/s² |
| 板材初始 COM | World `[0,0,0.80]` m |
| 初始姿態 | 繞 world Y 約 +0.336°；實際 settled pose 可能有微小差異 |
| Robot L 名義握點 | Panel-local `[-0.20,+0.10,0]` m |
| Robot R 名義握點 | Panel-local `[+0.20,+0.10,0]` m |
| 人手作用點 | Panel-local `[+0.27,-0.10,0]` m |
| 左臂 base | World `[-0.45,+0.65,+0.25]` m，yaw −90° |
| 右臂 base | World `[+0.45,+0.65,+0.25]` m，yaw −90° |

左右名義握點相距 0.40 m，在同一條長邊；人手在對側長邊且偏 +X 端。Panel-local 名義位置固定，world 位置會隨板材移動。真實接觸點由 MuJoCo 接觸解算，不能把名義握點當成逐時刻量測接觸點。

## 2. 兩支 Panda 機械臂

採用 MuJoCo Menagerie 的 `franka_emika_panda` MJCF，保留每臂七軸運動鏈、link inertia、joint limits 與碰撞幾何。來源 revision：`f3b6013629c7b94f8ee93fd977b161fa8350f9bc`，模型與來源 manifest 已存入 repo。

- 兩支 base 固定於 world；手臂與夾爪沒有獨立 freejoint。
- 全場只有板材具有 freejoint；沒有 weld/equality constraints 固定板材。
- 手臂共有 14 個 revolute joints；自訂夾爪另有 4 個 slide joints。
- 正式物理執行透過 actuators 驅動，不以逐步覆寫 qpos 或 gripper-root virtual wrench 搬動板材。
- 啟用手臂、板材、地面及自訂夾爪相關碰撞；受 MuJoCo 相鄰剛體過濾與 upstream exclusions 限制，不代表所有幾何配對皆被檢查。

目前 arm actuators 是 position servos，命令為 rad。模型內部產生 torque；每臂 joint 1–4 的 actuator torque limits 為 ±87 Nm，joint 5–7 為 ±12 Nm。這些是目前模擬模型設定，不是實體硬體認證。

## 3. 自訂夾爪

沒有使用原廠 Panda fingers。沿用此前已建立的上下兩片 pad 夾爪，固定連接至每支 Panda 的末端 flange。

| 項目 | 設定 |
|---|---|
| 夾爪數量 | 2，每臂 1 個 |
| Pads | 每個夾爪 upper/lower 各 1 片，共 4 片 |
| 單片 pad 尺寸 | 0.09 × 0.14 × 0.02 m，為完整尺寸 |
| 單片 pad 質量 | 0.10 kg |
| Pad 運動 | 沿 custom gripper local Z 的 slide joint |
| Upper slide range | `[-0.006,+0.015]` m |
| Lower slide range | `[-0.015,+0.006]` m |
| 名義 preload commands | `[−40,+40,−40,+40]` N，順序為 L upper/lower、R upper/lower |
| Pad command bounds | Upper `[-80,0]` N；lower `[0,80]` N |
| Mount adapter | 每臂新增 0.08 kg |

40 N 是各 pad 的 motor preload command，不代表每片 pad 的量測接觸力恆定為 40 N。板材載重、姿態與動態會改變接觸力。

Flange 位於 Panda link7 local Z +0.107 m；custom gripper 相對 flange 沿 Z +0.12 m，繞 X −90°。End-effector pose 定義在兩片 pad 中間的 custom gripper origin，不是 Panda link7 frame。

正式 Python runtime 將 geom 第一摩擦係數設為既有固定參數 `1.0482960672343025`。重現應走 Python 入口，不能只單獨打開 XML 就假設設定完全相同。

## 4. Virtual Human Hand 與外部擾動

人手目前是單一作用點與 spring-damper interaction model，沒有完整人類手臂／手指關節或人體碰撞模型。畫面中的手部標記用來呈現作用點與力量。

人手力由 target position/velocity 與實際作用點 position/velocity 的差值計算，施加於板材。當前參數：

- Stiffness：98.2109 N/m。
- Damping：7.3012 N·s/m。
- Force norm limit：50 N。
- Target speed limit：0.35 m/s。
- 作用點固定為 panel-local `[+0.27,-0.10,0]` m。

外部擾動是另一個獨立的板材外力來源；目前作用點為 panel-local `[+0.063691,-0.015331,0]` m。人手力與外力各自記錄 force、application point 及 about-panel-COM wrench。

人手目標與外力來源屬模擬 truth，並非機器人實際感測器輸出。

## 5. 已建立的感測與狀態記錄

### Tactile

使用 `mujoco.sensor.touch_grid`，四片 pad 各有一個 touch grid。

- 每片 pad：3 channels × 4 × 4 angular bins。
- 每側夾爪：`(2,3,4,4)`，2 表示 upper/lower。
- Channels 為 sensor-local z/x/y force components。
- 單位為 N/bin，不是壓力 Pa，也不是相機式 tactile image。
- 展示熱圖使用 normal channel；原始記錄保留三個 channels。
- 四片 pad 的 sensor frame 各自定義，上下 pad 不可未經轉換就當成同一 world 方向。

### F/T

目前有兩種不同來源：

| 欄位 | 定義 |
|---|---|
| `wrist_ft_left/right` | 由 panel-pad 接觸重建的 panel-on-gripper wrench；gripper-local，about gripper origin |
| `wrist_sensor_left/right` | MuJoCo wrist force/torque sensor；量測 child→parent transmitted wrench，包含下游重量與動力學 |

兩者都是 6 維 `[Fx,Fy,Fz,Tx,Ty,Tz]`，但物理意義、參考點不同。Raw wrist sensor 位於 custom gripper local `[0,+0.12,0]`，尚未做重量／慣性補償。

另記錄 world frame、about instantaneous panel COM 的 gripper-on-panel contact wrenches 與總 `object_contact_wrench`。

### Robot / object states

以下 shape 不含時間軸；arm 順序固定 left/right，每臂 joint1…joint7。

| 欄位 | Shape | 單位／意義 |
|---|---|---|
| `arm_joint_position` | `(2,7)` | rad |
| `arm_joint_velocity` | `(2,7)` | rad/s |
| `arm_actuator_torque` | `(2,7)` | Nm，arm DOFs 的 qfrc_actuator |
| `end_effector_pose` | `(2,7)` | World xyz + quaternion wxyz |
| `end_effector_velocity` | `(2,6)` | World angular xyz → linear xyz |
| `gripper_joint_position` | `(2,2)` | m，upper/lower slide |
| `gripper_joint_velocity` | `(2,2)` | m/s |
| `gripper_actuator_force` | `(4,)` | N |
| `object_position/orientation` | `(3,)` / `(4,)` | World position / quaternion wxyz |
| `object_linear/angular_velocity` | 各 `(3,)` | World m/s、rad/s |
| `contact_state_left/right` | 各 `(2,)` | Upper/lower pad 接觸資訊 |

Object state 目前直接取自模擬器，不是視覺估計結果。Actuator torque 不是外力 torque，也不是已完成重力補償的外部接觸估計。

目前可以產生同步 MuJoCo 影片，但尚未接入 VLM、RGB-D 感知或視覺 pose estimator；影片不代表已有視覺模型輸入管線。

## 6. 控制與資料介面

- Physics timestep：0.002 s，500 Hz。
- Controller 更新：500 Hz。
- 高層 reference / observation：100 Hz。
- 正式 transition：`o_t → reference → 5 個 control/physics 子步 → o_next`。
- Episode 開始前 settle 2 秒。

Controller 目前輸出：

```text
arm_position_target: (2,7), rad
pad_force_command:   (4,), N
```

保留的高層介面：

```text
desired_assist_torque: scalar, Nm
axis: world +Y
reference point: instantaneous panel COM
meaning: additional assistance torque
```

目前 position-hold baseline 不執行 assistance tracking；動作 demo 則執行預先規劃的 joint-position trajectories。尚無完整 object-wrench allocator、經驗證的 assistance tracking controller 或 Expert。

Joint states 已無條件記錄。Policy default whitelist 目前只包括左右 contact-reconstructed F/T、tactile、object pose/velocity 與 contact state；`with_joint_states` 可另外加入 q/dq。這只是資料介面，尚無訓練模型。

正式 pipeline 將 observations、next observations、controller commands、supervision、privileged truth、diagnostics 與 render states 分開保存。Expert supervision 仍是 NaN / invalid。動作 demo 的 `samples.npz` 則混有真值與 render state，屬診斷資料，不能直接視為正式 policy observations。

## 7. 已經能執行的動作

新增 `scripted-motion/1.0` 展示；舊版 A/B/C position-hold 驗證資料完整保留。下表是新動作展示，不可與舊測試名稱混為一談。

| 展示 | 實際觀察 |
|---|---|
| A | 板材旋轉約 +20°、抬升約 3.05 cm，然後停留 |
| B | 板材抬升約 7.98 cm，然後停留 |
| C | 板材旋轉約 −11.94°，之後承受 world −Z 20 N 固定外力 |

每段 8 秒；人手 target 在 0.6 秒開始移動，預設 arm trajectory 在 1.4 秒開始、3.8 秒完成。C 的外力於 4.0–4.3 秒漸增，4.3 秒後保持 20 N 至結束。

三段均由真正的 arm actuation 與接觸推動板材；每段 4,000 physics steps、800 observation samples。指定測試中維持四片 pads 接觸，無非預期穿透超標、數值警告或 actuator saturation。重複執行的 168 arrays 完全一致。

這證明這些指定軌跡可執行，沒有證明任意姿態、90° 翻轉或意圖辨識已完成。人手與手臂時間表預先設定，機器人不是看到人手力後自行決定動作。

## 8. 對照文件與重現

- [場景模型](../phase1b/models/dual_panda_v1.xml)
- [固定場景參數與初始關節配置](../phase1b/config/dual_panda_v1.json)
- [完整 compiled hardware contract](../phase1b/config/hardware_v2.json)
- [資料格式與座標定義](DATA_SCHEMA_V2.md)
- [Controller 接入文件](CONTROL_HANDOFF_ARMS.md)
- [動作展示、影片、原始資料與驗證](../results/phase1b/motion_examples_v1/README.md)

環境：MuJoCo 3.3.5、Python 3.12.11、uv；不使用 Docker。Repo root 執行下列命令，輸出路徑須尚未存在：

```bash
uv sync --locked
uv run --locked python -m phase1b.demos.motion_examples --output results/phase1b/reproduction-motion
uv run --locked python -m phase1b.demos.render_examples --input results/phase1b/reproduction-motion --output results/phase1b/reproduction-motion/visuals
```

渲染需要本機 ffmpeg。上述環境、幾何、感測與介面是目前已建立的基礎；後續資料收集策略、模型訓練方式與效果驗證設計尚待討論。
