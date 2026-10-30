# Dual Panda — 執行入口

單一 Scenario 3.0.0 / Hardware 2.0.0。[硬體](../docs/CURRENT_HARDWARE_HANDOFF.md) · [三層介面](../docs/THREE_LAYER_INTERFACE.md) · [資料格式](../docs/DATA_SCHEMA_V2.md)。

```bash
uv sync --locked
# 完整測試、接觸支撐、三層記錄、重播及 A/B/C 動作
uv run --locked python -m phase1b.scripts.run_suite --output results/phase1b/reproduction --render
# 短三層 contract smoke；預設是測試 policy + reference-only + position hold
uv run --locked python -m phase1b.scripts.collect --duration .12 --output results/phase1b/dev-contract
# 原生 viewer；目前是 position hold
uv run --locked python -m phase1b.scripts.launch_viewer --scenario B
```

輸出路徑不得已存在；`--render` 需 ffmpeg，可省略。Viewer 不存資料，關閉視窗即可停止，啟動器已處理 macOS uv libpython 路徑。

程式分工：`interfaces.py` 型別／驗證、`control.py` 三層串接、`pipeline.py` 唯一正式資料管線、`simulation.py` 物理、`sensing.py` 感測、`geometry.py` 共用幾何與 demo IK。`models/`、`config/`、`vendor/` 是固定模型與來源。

`demos/` 是同一硬體上的 scripted motion 展示與 renderer；其診斷檔不是 Expert dataset，也不表示三層 adapter 已能執行 assistance。[目前影片與結果](../results/phase1b/current/README.md)。
