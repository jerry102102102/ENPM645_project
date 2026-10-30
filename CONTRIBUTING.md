# 團隊協作

- 只維護 `phase1b/` 這套雙 Panda 實作；型別與執行順序以 [三層介面](docs/THREE_LAYER_INTERFACE.md) 為準。
- `uv sync --locked` 後執行；依賴變更提交 `pyproject.toml` 與 `uv.lock`。
- 變更前核對 [硬體](docs/CURRENT_HARDWARE_HANDOFF.md)、[控制器介面](docs/CONTROL_HANDOFF_ARMS.md)、[資料格式](docs/DATA_SCHEMA_V2.md)。硬體／握持幾何變更須新版本與驗證。
- 實驗用新輸出路徑；目前正式證據在 `results/phase1b/current/`，過時版本由 Git 保存，不另維護平行實作。
- 提交前跑相關測試；不得把 scripted decision、measured contact wrench 或 motion demo 當成有效 Expert Labels。
- 各自開分支、透過 PR 互看。文獻在 [閱讀入口](01_literature/README.md) 更新；只有實際閱讀者填 reader／read scope／日期，AI 摘要不代表已讀。
