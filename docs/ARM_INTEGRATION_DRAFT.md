# 雙臂整合與關節資料草案

狀態：使用者已要求加入真正的手臂與關節資料。手臂型號待澄清：口述「Frame Capture」可能是 Franka Panda，也可能是影格擷取；尚未選定或安裝模型。本文為介面草案，不代表雙臂模擬、力追蹤或新資料集已完成。現有 scenario v2 / hardware v1 與結果保留。

## 目前實作與必要改動

現有 `phase1a` 的兩個 gripper roots 是 freejoint，`executed_virtual_drive_wrenches` 直接在 root 施加 world-frame wrench。模型沒有 arm joints，所以目前無法提供真正的手臂 q / dq 或關節致動力矩。現有 F/T 是逐夾爪的 panel 接觸 wrench 重建，不是含手部重力、慣性等效應的商用腕部感測器讀值。

新場景必須有固定基座、連桿、關節、慣性、碰撞幾何與關節致動器。夾爪要接在末端的運動鏈上，板材透過真實模擬接觸受力；不能仍由 floating-root wrench 驅動夾爪，再用 IK 讓手臂外觀跟隨，並把這種角度冒充物理關節量測。

保留 Robot L/R 在板材同側、人手在對側的名義握持配置。名義 grasp frames 沿用 v2 作為新模型的初始幾何目標，是否可達須用新手臂 FK/IK、joint limits 與碰撞檢查確認；不事先宣稱整段翻轉軌跡可達。固定的是基座與名義初始握點，不是把板材或手臂末端焊死。

## 三層責任

| 層級 | 輸入 | 輸出 | 不能混淆的量 |
|---|---|---|---|
| 意圖／協助目標 | 過去與當下的 F/T、tactile、q、dq、物體狀態；視覺日後另接 | `desired_assist_torque_ref`：panel COM、world +Y、額外協助 Nm | 不是直接輸出每個關節角度或 motor torque |
| 分配與手臂控制 | 目標、雙臂狀態與幾何、限制、接觸回饋 | 左右目標接觸 wrench、關節／夾爪命令及飽和狀態 | 目標接觸 wrench 不等於關節致動力矩 |
| 模擬與實現回饋 | 實際送出的命令、MuJoCo dynamics/contact | q/dq、實際 actuator generalized force、腕部量測與 panel 接觸 wrench | 命令不等於實現；實現的總接觸力矩不等於額外 assistance |

`J(q)^T w` 可作為 wrench 映射至關節力矩的一部分，但必須對齊 wrench/Jacobian 的座標、參考點、速度排列與作用方向，並處理手臂重力／慣性、接觸及限制。它本身不是已驗證的 force controller。高層 reference 可以先用 scripted source，控制器與 learner 仍能分開開發。

## 新版資料契約草案

以下欄位須放在新 schema；不可把零值塞進舊 episode 當成未量測的關節資料。每臂的長度由固定 `joint_names` 順序決定，尚不硬編碼未確認的自由度。

| 分區 | 欄位 | 單位／語意 |
|---|---|---|
| observations | `arm_joint_position_left/right` | rad，實際 MuJoCo joint state；由 joint qpos address 取值 |
| observations | `arm_joint_velocity_left/right` | rad/s，由 dof address 取值，與 q 的 named-joint 順序對齊 |
| observations | gripper opening / pad joint state | m、m/s；與 revolute arm joints 分開 |
| observations | EE pose/twist、現有 tactile、定義清楚的 F/T、物體 pose/twist | 同一 timestamp；pose frame、quaternion 與 twist 排列明訂 |
| controller | raw requested command、bounded applied command、command type | 區分 position target、velocity target、torque command；不能一律標成 Nm |
| controller | per-arm actuator generalized torque | Nm，依傳動映射後作用於 arm dofs 的 `qfrc_actuator`；與 raw `actuator_force` 分開 |
| controller | requested assistance、requested total、allocated contact wrenches | 保留高層需求至接觸分配的鏈條；未實作為 unavailable |
| controller | realized contact wrench、tracking evaluation、saturation | 物理步進後寫入，不回填成決策前 observations |
| metadata | joint/actuator names、dof mapping、base poses、limits、model/source hashes | 固定順序與版本、時間原點、單位、座標、感測器定義 |
| privileged | human target、external source/point/force、scenario label | 僅供評估／影片標註，不餵給 learner 或 controller |

q、dq 應是可供 learner 使用的因果輸入；實際使用哪些關節力矩量測、是否加噪聲，需明訂感測來源後才加入 allowlist。不能把即將輸出的命令或未來 realized torque 放進當下輸入。

沿用 500 Hz physics/control、100 Hz transition 的時間結構作為整合起點。影格擷取只是顯示與記錄：RGB frame index 必須連到 simulation timestamp / physics tick，q/dq 應直接讀模型狀態，不能由畫面反推或由影片播放時鐘標記。

## 整合順序與交付證據

1. 確認手臂型號；固定來源 revision、模型檔案與 license。確認所選模型的 actuator 類型，不能把預設 position actuator 當成 torque motor。
2. 建立兩個固定基座與末端夾爪的完整模型，保留名義握持語意；核對連接、關節 limits、FK、碰撞與握點可達性。先交付帶關節名稱／角度的 MuJoCo 畫面。
3. 用明確標為 baseline 的關節控制完成短時間持板，確認重力、接觸、致動器飽和、數值穩定與不靠 root 外力支撐。這不等於 assistance tracking 通過。
4. 記錄 q/dq、命令、實現力矩、F/T、tactile 與同時刻場景；驗證讀回、時間對齊與確定性重播。舊資料與新資料不得混成同一 hardware version。
5. 再讓 Track B 接入 desired torque，驗證 requested → allocated → joint command → realized contact wrench；通過後才建立有效 expert labels。

## 候選模型來源（尚未選定）

[MuJoCo Menagerie 的 Franka Emika Panda](https://github.com/google-deepmind/mujoco_menagerie/tree/main/franka_emika_panda) 提供 MJCF、visual/collision meshes 與 Apache-2.0 license。其 README 明確寫出 arm 採用 position-controlled actuators。若採用，需以來源 revision 鎖定檔案，並獨立定義我們的 tactile 夾爪與控制版本，不能把客製夾爪宣稱為原廠 Panda 手指。
