# Track A / Track B — 單一三層介面

先讀 [三層契約](THREE_LAYER_INTERFACE.md)。現行所有型別從 `phase1b.interfaces` 匯入；唯一收集入口是 `phase1b.scripts.collect`。

| 開發項目 | 輸入 → 輸出 |
|---|---|
| Track A Policy / candidate Expert | `PolicyHistory → AssistanceDecision` |
| Object-Level Controller | `AssistanceDecision + ArmControllerInput + dt=.01 → ObjectCommand` |
| Track B Dual-Arm Controller | `ObjectCommand + 最新 ArmControllerInput → ArmCommand` |

Controller observation 是公開感測的唯讀副本，包含 q/dq、EE pose、F/T、tactile、object state。另有名義 q、pad preload 與 actuator command limits。沒有 human goal、scenario label、外力真值、future state 或 live MuJoCo handle。

## 接入

自訂類別需提供上述 predict/step 方法；CLI 使用無參數 constructor。可只替換正在開發的一層，其餘使用 reference-only / position-hold adapters：

```bash
uv run --locked python -m phase1b.scripts.collect \
  --policy my_policy:Policy \
  --object-controller my_control:ObjectController \
  --dual-arm-controller my_control:DualArmController \
  --output results/phase1b/dev-controller
```

需要 constructor 參數時用 Python：

```python
from phase1b.pipeline import run_episode
run_episode("results/phase1b/dev-controller", scenario="B", duration=4,
            policy=my_policy, object_controller=my_object_controller,
            dual_arm_controller=my_arm_controller,
            max_assist_torque_nm=.02, max_rotation_rate_rad_s=.5)
```

這兩個限額是介面測試設定，不是安全認證。Import、型別、時鐘、超限錯誤會直接中止，不靜默退回 baseline。

## 現有 ArmCommand

```python
from phase1b.interfaces import ArmCommand

class DualArmController:
    def step(self, command, observation):
        # Only a hold adapter; does not execute assistance or object motion.
        return ArmCommand(
            arm_position_target=observation.nominal_arm_position.copy(),
            pad_force_command=observation.nominal_pad_force.copy(),
            controller_mode="position_hold_baseline",
            assist_reference_executed=False,
        )
```

Arm commands 為 `(2,7)` **rad**，pad commands 為 `(4,)` **N**；不能把 object Nm 放進 position targets。World +Y / panel COM 的 assistance 與基本支撐分開；非零 Ty 不可與獨立高剛性 pitch position loop 疊加。直接 torque actuation 需要另建 hardware/controller version。

目前 reference-only adapter 只在 HOLD 入口 latch 當下姿態，ROTATE 不產生動作；dual-arm adapter 只保持 nominal q。完整 admittance、IK/Jacobian execution、配力、接觸控制、限幅平滑與 assistance tracking 仍待實作。Scalar AssistTorqueReference 僅保留明確資料轉換，舊 step(reference, inp) 執行分支已依單一版本要求移除。

換控制器後須驗證：支撐與 assistance 的責任、requested → allocated → commands → realized、接觸／碰撞／限制／飽和及 tracking error。`assist_reference_executed=True` 只是使用請求的旗標；不是 tracking 或 Expert quality 通過。

[硬體](CURRENT_HARDWARE_HANDOFF.md) · [資料格式與時間事件](DATA_SCHEMA_V2.md) · [測試結果](../results/phase1b/current/verification.json)
