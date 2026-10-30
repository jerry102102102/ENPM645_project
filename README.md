# ENPM 645 — Human-Robot Collaboration

兩支 Franka Panda 與單一虛擬人手共同操作板材。只維護一套 MuJoCo 場景與資料管線：

**Policy → Object Controller → Dual-Arm Controller → MuJoCo**

已完成：三層介面、因果資料隔離、雙臂接觸支撐、Joint State Logging，以及 scripted 旋轉／抬升／固定外力展示。完整控制演算法、assistance tracking、Expert 與學習模型尚未實作。

| 用途 | 文件 |
|---|---|
| 硬體與場景 | [目前硬體](docs/CURRENT_HARDWARE_HANDOFF.md) |
| 三層型別、模式與時鐘 | [Interface Contract](docs/THREE_LAYER_INTERFACE.md) |
| 組員接入控制器 | [Controller Handoff](docs/CONTROL_HANDOFF_ARMS.md) |
| 感測、命令與標籤記錄 | [Data Schema](docs/DATA_SCHEMA_V2.md) |
| 測試證據與影片 | [目前結果](results/phase1b/current/README.md) |

## 執行

本機 Python 3.12.11、MuJoCo 3.3.5、uv；無 Docker。輸出使用新目錄，渲染需本機 ffmpeg。

```bash
uv sync --locked
uv run --locked python -m phase1b.scripts.run_suite --output results/phase1b/reproduction --render
```

`phase1b/` 是唯一程式實作，`results/phase1b/current/` 是目前驗證結果；單段收集與 viewer 見 [執行入口](phase1b/README.md)。

[文獻筆記](01_literature/README.md) · [團隊協作](CONTRIBUTING.md)。歷史實驗已移出目前工作樹，只留在 Git；清理前版本為 `996a524`。
