# Phase 1A — Frozen scenario、torque interface、data pipeline

**Data Pipeline GO · Controller Integration Ready。** 這次完成 Track A。support-only baseline 沒有執行額外 torque，沒有有效 expert labels，也沒有訓練模型。新結果放在 `results/phase1a/`；Phase 0 v1/v2 原始碼與結果保持不變。

先看 [結果報告](../results/phase1a/report.md)、[Controller handoff](../docs/CONTROL_HANDOFF.md)、[Frozen scenario](../docs/SCENARIO_V1.md) 和 [Data schema](../docs/DATA_SCHEMA.md)。完整使用者規格保存在 [specification.md](specification.md)。

## 執行

從 repository root，使用既有本機 `.venv`，無須 Docker。已驗證 Python 3.12.11 / MuJoCo 3.3.5 / NumPy 2.2.6；繪圖使用 matplotlib 3.10.6，沿用 v2 環境。若要在另一台機器建立環境，依 [v2 README](../phase0_v2/README.md) 的安裝方式與 pinned requirements。

```bash
# 一鍵：A/B/C + reference test → unit tests → read-back → replay → v2 regression → figures/report
.venv/bin/python -m phase1a.scripts.run_all --output results/phase1a/reproduction

# 只收集四個 smoke episodes，選新的輸出位置
.venv/bin/python -m phase1a.scripts.run_pipeline_smoke --output results/phase1a/dev-smoke

# 已存資料讀回、含完整回歸；不更改原資料
.venv/bin/python -m phase1a.scripts.validate_dataset --input results/phase1a/smoke --regression

# 只驗證邊界、因果隔離、錯誤拒絕與 controller injection
.venv/bin/python -m unittest phase1a.tests.test_contracts -v
```

輸出不能覆寫；重跑請選新目錄。`dev*`、`reproduction*` 忽略 Git；正式 `smoke/`、validation、圖表和 report 版本控管。

## 模組與沿用範圍

| 模組 | 用途 |
|---|---|
| `config/scenario_v1.json` + registry | 從 v2 有效模型擷取的唯一 frozen scenario |
| `simulation/scenario_loader.py` | 驗證 config/model/source hashes 與實際 geometry |
| `simulation/transition_world.py` | 包裝原 DualPilot；區分 observe、apply、physics step；不改模型 |
| `interfaces/controller_contract.py` | typed reference/input/output、bounds validation、support-only adapter |
| `data/observation_builder.py` | 十項 sensor allowlist 與因果 history |
| `data/episode_io.py` | 分檔 NPZ logger、reader、dataset validation |
| `scripts/run_pipeline_smoke.py` | scripted reference 與可替換 controller factory |
| `scripts/validate_dataset.py` | 四集 exact replay、零 reference 對照、原 v2 A/B/C 比較及物理 controls |
| `scripts/run_all.py` | 一鍵驗證、圖表與 Q1–Q9 報告 |
| `tests/test_contracts.py` | 10 項 contract / leakage / corruption / integration tests |

直接重用 `phase0_v2` 的 MJCF、support_commands、human/environment force、contact sensing 與 tactile；不是另一個平行 simulator。Reference/control 契約在本版本明確化，舊 v2 API 保留供舊實驗重現。

## 交接邊界

100 Hz 的高層輸出是 world +Y / panel COM / Nm 的 **額外協助力矩**。500 Hz controller 收到 typed input，回傳實際 bounded pad/root commands。每筆 100 Hz transition 保存全部五個命令與前後觀測。新 controller 以 `--controller module:Class` 接入，不修改 logger 核心。

Track B 尚需實作 force allocation、baseline/assistance 合成與 tracking；Track A 後續才建立 expert、validity gates、正式 demonstrations 與 learning。`LearnedForcePolicy.predict` 目前只有 Protocol，並沒有假造 policy 或 expert。
