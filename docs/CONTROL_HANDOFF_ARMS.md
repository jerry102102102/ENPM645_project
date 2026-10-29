# Track B — Dual Panda controller handoff

本文件適用 **Scenario 3.0.0 / Hardware 2.0.0**。舊 free-root 夾爪介面仍見 [CONTROL_HANDOFF.md](CONTROL_HANDOFF.md)，不可直接將其 `executed_virtual_drive_wrenches` 套用到本版。

## 目前可用範圍

雙臂七軸運動鏈、固定基座、custom tactile grippers、真實接觸支撐、q/dq/actuator torque logging 與可重現 A/B/C 已接通。完整 compiled hardware 與固定 named joint order 在 `phase1b/config/`，來源模型在 `phase1b/vendor/`。

低層採用 upstream **position servos**，controller 輸出 arm position targets（rad）及 pad forces（N）。這提供最小的持板與資料整合基準，沒有實作 grasp-matrix allocation、impedance/admittance、assistance tracking 或 Expert。若需 torque control，必須另建明確的 torque-actuator 模型版本與命令契約；目前沒有 `tau_command` adapter，不應繞過 pipeline 對 root 或 arm 加 virtual wrench。

## 使用者保留的高層介面

直接沿用 `phase1a.interfaces.controller_contract.AssistTorqueReference`：scalar `torque_nm` 是額外協助力矩，**World +Y、instantaneous Panel COM、Nm**。不是總支撐 wrench，也不是任何單一關節 torque。reference 更新 100 Hz，controller 500 Hz；五個子步持有同一 reference。

`PositionHoldBaseline` 忽略 reference，只回傳起始 IK 的 q targets 與名義四個 pad forces。±0.02 Nm reference 與零 reference 的完整 sensing/state trajectories 一致，證明隔離；沒有追蹤成功主張。

## 接入可執行的 position adapter

```python
from phase1b.interfaces import ArmCommand

class MyPositionAdapter:
    def step(self, reference, inp):
        # inp.observation includes current q/dq, tactile, contact F/T and object state.
        # Build your bounded rad targets and pad forces here.
        # This example only reproduces hold; it does not execute assistance.
        return ArmCommand(
            arm_position_target=inp.nominal_arm_position.copy(),
            pad_force_command=inp.nominal_pad_force.copy(),
            controller_mode="position_hold_baseline",
            assist_reference_executed=False,
        )
```

```bash
uv run --locked python -m phase1b.scripts.collect \
  --scenario B --reference-test \
  --controller my_control.adapter:MyPositionAdapter \
  --output results/phase1b/dev-controller
```

Import/shape/finite/limit errors 直接中止，不退回 baseline。controller 不收到 live simulator handle、human target、external-force label、future state 或 Expert。公開 MJCF 和 hardware contract 可用來建立獨立的 kinematic/dynamic model；不得藉此讀取 episode 的 privileged source。

| Input | 定義 |
|---|---|
| `timestamp / control_dt` | episode simulated s / 0.002 s |
| `observation` | 當下公開 sensing/state，arrays 為唯讀副本；新欄位見 DATA_SCHEMA_V2 |
| `nominal_arm_position` | (2,7) rad，left/right × joint1…7 |
| `nominal_pad_force` | (4,) N，[-40,+40,-40,+40] |
| `arm_position_limits` | (2,7,2) rad，actuator target bounds |
| `pad_force_limits` | (4,2) N |

| Output | 定義 |
|---|---|
| `arm_position_target` | (2,7) finite ndarray，rad，必須在 bounds 內 |
| `pad_force_command` | (4,) finite ndarray，N，必須在 bounds 內 |
| `controller_mode` | 非空字串 |
| `assist_reference_executed` | bool；是否使用 reference，不代表追蹤成功 |

實際 motor torque 由 MuJoCo servo 和傳動計算，logger 在 actuation 與 step 後分開取樣。機器人不能靠 arm `qfrc_applied` 或 body `xfrc_applied` 輸入支撐力；記錄驗證會拒絕這種情況。外部人手／擾動只作用於 panel free DOFs。

## 座標與感測

- `end_effector_pose` 是 between-jaw custom gripper frame，不是 link7 或原廠 Panda hand frame。
- 所有 wrench 都是 force xyz → torque xyz；pose quaternion 為 wxyz。
- EE/object twist 是 angular xyz → linear xyz，world frame。與 wrench 的排列不同。
- `wrist_ft_*` 沿用 contact reconstruction；`wrist_sensor_*` 是新 wrist transmitted wrench，含下游重量／動力學，不直接等於 panel 接觸力。詳細正負號／參考點見 [DATA_SCHEMA_V2.md](DATA_SCHEMA_V2.md)。
- Nominal grasp：panel-local L `[-.2,+.1,0]`、R `[+.2,+.1,0]`；human `[+.27,-.1,0]`。控制中使用實際姿態搬移 wrench，不把 nominal points 當量測。

## 組員下一步的驗收

先在明確選定的 position 或新 torque adapter 上建立零 assistance 的穩定支撐，再測小幅正負 desired torque。必須分開記錄 requested total、allocated contacts、actual commands、realized contact wrench，並處理 baseline support，不拿 total Ty 冒充額外 assistance。這是下一階段工作，本次沒有自行實作。

每個新控制器須重新檢查 joint/actuator limits、飽和、接觸保持、碰撞、numerical stability、力平衡與追蹤誤差。現有 `validate_episode` 的 physical sampling checks 不是完整 controller certification。通過前 expert labels 仍保持 unavailable，不訓練 AI。
