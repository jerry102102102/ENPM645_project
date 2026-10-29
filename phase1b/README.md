# Phase 1B — 執行與驗證

目前使用雙 Panda + 自訂 tactile 夾爪，Scenario 3.0.0 / Hardware 2.0.0。[硬體設定](../docs/CURRENT_HARDWARE_HANDOFF.md) · [資料格式](../docs/DATA_SCHEMA_V2.md) · [Controller 介面](../docs/CONTROL_HANDOFF_ARMS.md)。

## 動作展示

[A/B/C 影片與結果](../results/phase1b/motion_examples_v1/README.md)：旋轉、抬升後停留、承受固定 20 N 外力。使用預設 joint trajectories，尚無意圖辨識或 assistance tracking。

```bash
uv sync --locked
uv run --locked python -m phase1b.demos.motion_examples --output results/phase1b/reproduction-motion
uv run --locked python -m phase1b.demos.render_examples --input results/phase1b/reproduction-motion --output results/phase1b/reproduction-motion/visuals
```

## 整合驗證

```bash
uv run --locked python -m phase1b.scripts.run_suite --output results/phase1b/reproduction --render
```

包含 position-hold episodes、物理正反檢查、新舊單元測試與 exact replay；[既有驗收結果](../results/phase1b/dual_panda_v1/report.md)。渲染需本機 `ffmpeg`，可省略 `--render`。所有輸出須使用新路徑；`dev*` / `reproduction*` 不加入 Git。

## 原生 viewer／單段記錄

```bash
uv run --locked python -m phase1b.scripts.launch_viewer --scenario B
uv run --locked python -m phase1b.scripts.collect --scenario A --output results/phase1b/dev-A
```

這兩個入口跑 **position-hold baseline**，不是上方動作展示。Viewer 不存資料，關閉視窗即停止；啟動器已處理 macOS uv 的 libpython 路徑。

模型與初始設定在 `models/`、`config/`。`vendor/` 保留 Menagerie revision、SHA256 與 license，平常不需重新下載。使用 Python 入口才能套用完整 runtime 設定。
