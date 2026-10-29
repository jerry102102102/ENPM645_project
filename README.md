# ENPM 645 — Collaborative Object Reorientation

兩支 Franka Panda 與單一虛擬人手共同操作板材；使用本機 MuJoCo 3.3.5、Python 3.12.11、uv，無 Docker。

**目前已完成：** 雙臂接觸支撐、Joint State Logging、旋轉／抬升／固定外力展示。動作由預設軌跡驅動；尚無意圖辨識、assistance tracking、Expert 或模型訓練。

## 必要文件

| 要做什麼 | 看這裡 |
|---|---|
| 了解硬體、給 ChatGPT 討論背景 | [目前硬體](docs/CURRENT_HARDWARE_HANDOFF.md) |
| 看 A/B/C 動作與實測結果 | [動作展示](results/phase1b/motion_examples_v1/README.md) |
| 寫 Controller | [控制介面](docs/CONTROL_HANDOFF_ARMS.md) |
| 使用感測與關節資料 | [資料格式](docs/DATA_SCHEMA_V2.md) |
| 跑驗證、開 MuJoCo viewer | [執行方式](phase1b/README.md) |

## 執行動作展示

```bash
uv sync --locked
uv run --locked python -m phase1b.demos.motion_examples --output results/phase1b/reproduction-motion
uv run --locked python -m phase1b.demos.render_examples --input results/phase1b/reproduction-motion --output results/phase1b/reproduction-motion/visuals
```

渲染需本機 `ffmpeg`；輸出路徑須尚未存在。程式在 `phase1b/`，模型與固定設定在其 `models/`、`config/`，驗證資料在 `results/phase1b/`。

<details>
<summary>文獻與歷史版本</summary>

[文獻與閱讀筆記](01_literature/README.md) · [團隊協作](CONTRIBUTING.md) · [初版概念](docs/project-concept.md)

[Phase 0 v1](phase0_tactile/README.md) · [Phase 0 v2](phase0_v2/README.md) · [Phase 1A](phase1a/README.md) · [雙 Panda 整合驗收](docs/DUAL_PANDA_INTEGRATION_REPORT.md)

歷史程式、規格與結果保留；Phase 1B 仍共用其中的 sensing、geometry 與資料契約，不能整批移除。
</details>
