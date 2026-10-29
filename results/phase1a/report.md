# Phase 1A — Frozen Scenario and Data Pipeline

**Data Pipeline GO · Controller Integration Ready.** Desired Torque Tracking、Expert Label 與 Offline Learning 尚未通過；本階段不要求這三項。

本報告由 `python -m phase1a.scripts.run_all` 依實際驗證產生。這是 CPU MuJoCo 模擬結果，沒有新增力分配器或 expert。

## 執行結果

| Episode | Transitions | Physics steps | Expert valid | Assist executed |
|---|---:|---:|---:|---:|
| A | 400 | 2000 | 0 | 0 |
| B | 400 | 2000 | 0 | 0 |
| C | 400 | 2000 | 0 | 0 |
| reference_test | 500 | 2500 | 0 | 0 |

- 10 項 contract / causal isolation / corruption rejection tests 通過（詳見 `unit-tests.txt`）。
- 四個 episodes 重跑，共 **372 個 NPZ arrays 完全一致**，包含 NaN missing-value masks；只承諾本次 runtime 的確定性。
- 5 秒正負 reference 與零 reference 的 sensor trajectories、實際命令及接觸 wrench 完全一致，證實 support-only 沒有執行新指令。
- 原 v2 A/B/C 重新執行：位置、姿態、速度、root commands、命令施加後接觸 wrench 的最大絕對差 **5.12e-13**（各欄位單位詳見 validation.json；閾值 1e-9）。
- 原 v2 接觸移除、關閉 drives、等合力及 timestep-halving controls 再次通過。
- 最大線性力平衡殘差 **6.7e-13 N**；最大接觸 penetration **0.000297 m**；無 MuJoCo warnings。

## Q1–Q9

**Q1 場景凍結：已實作、已驗證。** 從已驗證 v2 block 0 的實際 compiled model 擷取，固定 geometry、接觸參數、初始姿態、human 和 disturbance。registry 同時驗證 config、MJCF、依賴原始碼及來源 metadata；變更需新版本。

**Q2 三個握持點：已驗證。** Panel-local left = [-0.2, 0, 0] m；right = [0.2, 0, 0] m；human = [0.27, -0.08, 0] m。左右相對 nominal quaternion 都是 [1, 0, 0, 0]。這些是名義幾何；gripper roots 仍自由運動，contact points 由物理解算。

**Q3 力矩語意：已實作、已驗證。** 額外 assistance，world +Y、瞬時 panel COM、Nm、右手定則。既有 `rotate_y` 與此一致：+X 側在正 pitch 時朝 -Z 移動；+X 點向 -Z 施力產生正 Y torque。測試幅度 ±0.02 Nm 僅驗證資料傳遞，未證明可追蹤。

**Q4 分離：已實作、已驗證。** observations / next_observations / supervision / controller / privileged 分檔。policy builder 只允許十項 sensor features，拒絕未來時間與額外欄位；毒化未來、控制與 privileged arrays 不影響當下 history。Object pose 暫用 MuJoCo state，屬明確允許的輸入；scenario/human goal 不允許。

**Q5 時序與讀寫：已實作、已驗證。** 100 Hz reference/logging、500 Hz controller/physics；每筆 transition 保存五個命令、physics ticks 與前後 timestamps，驗證 observation ≤ decision < next observation，並驗證相鄰 transition 連續。o_t 是舊命令持續作用時的量測；新命令施加後的同時間 force response 僅存 controller diagnostics。

**Q6 Reference 接入：已實作、已驗證。** 0、正、0、負、0 的測試訊號經 typed interface 傳入 adapter。source=scripted_test；assist_reference_executed=false；tracking error=NaN 且 mask=false；expert label=NaN 且 invalid。

**Q7 Baseline：已驗證。** 重用原 v2 MJCF、human/disturbance、support_commands 與 tactile。回歸量化結果見上方及 validation.json；未修改舊版原始碼、報告與資料。

**Q8 組員接入：介面已實作、接入機制已驗證。** 見 repository 的 `docs/CONTROL_HANDOFF.md`。使用 `--controller module:Class` 替換；測試已用獨立 delegating class 完成短 episode、儲存和驗證，無須修改核心 pipeline。此測試只證明接入，並未驗證另一個力分配控制器。

**Q9 正式 demonstrations 還缺什麼：尚未實作。** Track B 需完成額外 torque + baseline 的合成、左右接觸力分配、內力/飽和管理及 requested→allocated→executed→realized 的追蹤驗證；需定義 baseline 與 assistance 的可驗證分離方式。之後 Track A 才建立 model-based expert、label validity gates、示範品質篩選與專用 collection mode。現有 `LearnedForcePolicy` 僅是介面，尚無模型、訓練或有效 teacher。

## Evidence and provenance

- Scenario: `enpm645-dual-gripper@1.0.0`
- model_hash: `ee400eee401d0748a6fc1556aac84168fbe22ba4fffbbe0792ac3f94ee9a93b9`
- config_hash: `c657d4e6ad9ebb46e5b6e35001fb5dacc4983ce545c444acfc24238cd46abd6f`
- 完整驗證：`validation.json`；各 episode：`smoke/{A,B,C,reference_test}/`；欄位字典：`smoke/schema.json`。
- 圖表：`figures/reference_and_total_contact.png`、`figures/transition_timing.png`、`figures/sensor_smoke.png`。
- 重現：在 repository root 執行 `.venv/bin/python -m phase1a.scripts.run_all --output results/phase1a/reproduction`，選未存在的輸出位置。

這是 UMI-style 的 observation/action/transition 分離設計，並非 UMI 原始格式。現有 root-drive commands 是 MuJoCo-specific actuator adapter；policy features / desired object torque 不依賴特定機械臂關節配置。
