# Track B — Controller handoff

Track A 已完成固定場景、typed interface、reference generator、同步記錄與讀回驗證。**Track B 要做的是把額外物體力矩變成可實現的左右夾爪施力**。目前可執行的 `SupportOnlyAdapter` 只保持原 v2 的基本支撐；沒有 allocation，也沒有 assistance tracking。

## 接入位置與最短執行路徑

介面定義：[controller_contract.py](../phase1a/interfaces/controller_contract.py)。實作任意可 import、無必要建構參數的 class：

```python
from phase1a.interfaces.controller_contract import (
    AssistTorqueReference, ControllerInput, ControllerOutput,
)

class MyController:
    def step(self, reference: AssistTorqueReference,
             observation: ControllerInput) -> ControllerOutput:
        # Your allocation/control implementation goes here.
        # Return actual bounded commands; None means unavailable diagnostics.
        raise NotImplementedError
```

把 class 放在例如 `my_control/controller.py`（自行建立），從 repository root 執行：

```bash
uv run --locked python -m phase1a.scripts.run_pipeline_smoke \
  --controller my_control.controller:MyController \
  --output results/phase1a/dev-my-controller
```

核心 pipeline 不需修改。import / constructor / input validation / output validation 的錯誤會直接中止，不會悄悄退回 baseline。可先閱讀 `SupportOnlyAdapter.step()` 作為 **support passthrough** 範例；不要把其 root commands 改名成 allocated contact wrenches。替代 class 的接入和短 episode 讀回已由 `test_new_controller_import_and_dataset_poisoning` 驗證，該測試 class 也只是委派原 baseline。

## 你會收到什麼

`step(reference, observation)` 每 **2 ms / 500 Hz** 呼叫一次。reference 每 **10 ms / 100 Hz** 更新一次，期間五次呼叫持有相同 reference timestamp；不應假設 reference 每次都是新的。

| 輸入 | 形狀 / 語意 |
|---|---|
| `reference.timestamp` | Episode simulated time，settle 完成為 0；不得晚於 observation.timestamp |
| `reference.torque_nm` | 額外 assistance scalar，world +Y、瞬時 panel COM、Nm；invalid 時為 None |
| `frame_id / reference_point` | 必須為 `world / panel_com` |
| `source / valid` | scripted_test、model_based_expert、learned_policy / bool；本階段只使用 scripted_test |
| `observation.timestamp / control_dt` | 目前觀測時間 / 0.002 s |
| `sensor.object_position / object_orientation` | (3,) m / (4,) wxyz，panel frame → world；暫用 MuJoCo state |
| `sensor.object_linear_velocity / object_angular_velocity` | (3,) m/s / (3,) rad/s，world |
| `sensor.wrist_ft_left / right` | (6,) `[Fx,Fy,Fz,Tx,Ty,Tz]`，**panel 對 gripper**、gripper-local、about root origin |
| `sensor.tactile_left / right` | (2,3,4,4)，pad sensor frame 的現有 angular tactile map，N/bin |
| `sensor.contact_state_left / right` | (2,) upper/lower contact counts |
| `gripper_pose` | (2,7)，left/right，world xyz + wxyz |
| `gripper_velocity` | (2,6)，world **angular xyz then linear xyz**，rad/s、m/s |
| `baseline_drive_proposal` | (2,6) 既有 v2 clipped support root-drive proposal；world about root |
| `baseline_pad_proposal` | (4,) N，left upper/lower、right upper/lower |
| `baseline_nominal_support / baseline_feedback` | (2,6) clipping 前兩部分；不是已實現的 panel contact wrench |
| `calibration` | 固定 panel、gripper geometry、sensor frames、gravity、command limits |

`sensor` 是 dict，例如 `observation.sensor['object_position']`。數值 sensor arrays 為唯讀副本。Controller 不會收到 simulator handle、human goal、scenario label、未來觀測或 expert state。禁止修改輸入 calibration；它是公開物理設定，不是可變的控制輸出。

## 你需要回傳什麼

| `ControllerOutput` 欄位 | 型別 / 語意 |
|---|---|
| `executed_actuator_commands` | finite ndarray (4,)，實際送入四個 pad motors 的力命令，N |
| `executed_virtual_drive_wrenches` | finite ndarray (2,6)，實際施加在 left/right **root origin** 的 world wrench，N/Nm |
| `requested_total_object_wrench` | finite ndarray (6,) 或 None；baseline 與額外 assistance 合成後的 **總 panel contact target**，world about COM |
| `allocated_gripper_wrenches` | finite ndarray (2,6) 或 None；目標 **gripper 對 panel** 接觸 wrench，world、about respective root origin |
| `saturation_status` | bool ndarray (2,2)：left/right × force/torque clipping |
| `pad_saturation_status` | bool ndarray (4,) |
| `contact_status` | ndarray (2,2)，本次觀測的 left/right × upper/lower contact counts；必須與輸入一致 |
| `controller_mode` | 非空字串；支撐 adapter 為 `support_only`，未來自訂描述性名稱 |
| `assist_reference_executed` | bool，controller 是否真的使用 reference 產生 assistance 命令；**不是追蹤成功旗標** |
| `realized_object_wrench` | 回傳時必須 None；這是物理步進後的量測，由 pipeline 填入資料，不接受 controller 自稱的 future measurement |

未計算的 requested / allocated 欄位回傳 None，logger 轉成 NaN + false validity mask；不允許以假 0 表示 unavailable。`requested_assistance_wrench=[0,0,0,0,tau,0]` 是 pipeline 記錄的原始需求，與 requested **total** 分開。

目前 actuator 邊界：root force norm ≤ **35 N**、torque norm ≤ **3 Nm**；pad bounds 依序 **[-80,0], [0,80], [-80,0], [0,80] N**。你必須先做飽和處理並回報 flags；pipeline 會拒絕越界命令，不替你隱藏 clipping。Basic baseline 是 `baseline_drive_proposal` 與 `baseline_pad_proposal`。如何合成 baseline 與 assistance、內力如何限制，由 Track B 負責。

## 座標、正負號與 wrench transport

- World：MuJoCo world，Z 向上；gravity = [0,0,-9.81] m/s²。
- Panel：剛體 local frame，原點等於本場景 COM；名義三個握持點與 pad frames 見 [SCENARIO_V1.md](SCENARIO_V1.md)。所有 quaternion 為 wxyz。
- Left/right gripper frame：各自 free root body frame；F/T 的作用方向是 panel-on-gripper。
- Desired torque 正 Y 使用右手定則：+X 側向 -Z 轉動是正 pitch。既有 `rotate_y` 與此一致。
- 所有 wrench vector 為 `[force xyz, torque xyz]`。不要與 `gripper_velocity` 的 angular/linear 順序混淆。

將 local F/T 轉成 panel-on-gripper world wrench，先旋轉 force 和 torque，再將作用方向反號，才得到 gripper-on-panel。把它從 root 搬到 COM：

```text
F_panel_world = -R_world_gripper @ F_local
T_panel_world_at_root = -R_world_gripper @ T_local
T_panel_world_at_COM = T_panel_world_at_root
                       + (root_position - panel_COM) × F_panel_world
```

`allocated_gripper_wrenches` 按同一 reference-point convention；把兩個目標移到 panel COM 後再相加比較 requested total。**root actuator drive ≠ panel contact wrench**：其中還有 gripper 自身動力學，不能直接拿 drives 當已分配接觸力。

## 執行順序與回饋

100 Hz transition 是 `(o_t, u_t, [a_0,…,a_4], o_next)`。每個 2 ms 子步都以當下 sensor 更新 controller，再施加命令、物理步進、取得下一個 sensor。100 Hz policy 只看到 decision 時間以前的 observations。

`o_t` 是上一個命令持續作用時的 sensor snapshot。新命令施加後、同一 simulation timestamp 的 contact force response 另存在 `controller.contact_wrench_at_actuation`；它是 action 之後的診斷，不能回填成 o_t。

每子步的實際量測由 pipeline 存 `realized_contact_wrench_substep`（world about instantaneous COM），最後一個另存 `realized_object_contact_wrench`。下一次 `step()` 也會收到上一步完成後的 F/T。這是 **總接觸 wrench**，不等於額外 assistance；目前沒有 validated baseline subtraction，所以 `tracking_error_Nm` 保持 NaN / invalid。

## Track B 的責任與最小整合驗收

可自行採用 grasp matrix、model-based allocation、impedance/admittance、internal-force regulation。不得改寫 panel qpos、接觸量測、reference 對應的 measured wrench；不得讀取 scenario/human goal 作弊。不能為了整合而更改 frozen geometry；若確需修改，另開 scenario version。

1. 在相同 frozen scene 以零 reference 穩定支撐，確認 actual command bounds、contacts 與 dynamics。
2. 從小幅正負 reference 起，建立 requested total → allocated contacts → executed drives/pads → realized contacts 的量化對照；±0.02 Nm 只是本次資料流訊號，尚未證明可達。
3. 報告 world-COM 下各 wrench 的 residual、方向、暫態、飽和、接觸狀態和失敗片段；明確處理 baseline，不能以 total Ty 直接冒充額外 assistance。
4. 在來源仍為 scripted_test、expert labels 仍 invalid 的情況下通過 pipeline validation；另建立 tracking evaluator 和成功門檻。現有 smoke CLI 會對替代 controller 檢查資料/物理一致性，**不會自動認證 tracking**。
5. Controller 通過驗證後，才與 Track A 討論 expert 策略、有效 teacher labels 及正式 collection mode。本階段 writer 刻意不產生有效 expert labels。
