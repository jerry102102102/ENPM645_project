# 團隊協作

## 程式與實驗

- 現行版本：Scenario 3.0.0 / Hardware 2.0.0。以 [目前硬體](docs/CURRENT_HARDWARE_HANDOFF.md)、[Controller 介面](docs/CONTROL_HANDOFF_ARMS.md)、[資料格式](docs/DATA_SCHEMA_V2.md) 為準。
- 使用 `uv sync --locked` 與 `uv run --locked python ...`；依賴變更提交 `pyproject.toml` 與 `uv.lock`。
- 新實驗用新目錄，保留舊資料與報告。硬體／握持幾何改動須建立新版本並重新驗證。
- 提交前依 [執行方式](phase1b/README.md) 跑相關驗證；不得把 scripted reference 或 baseline measured wrench 當成 Expert Labels。
- 各自開分支，透過 PR 互看；專題範圍變更更新 root README，歷史概念文件保持原版本語意。

## 文獻

在 [文獻入口](01_literature/README.md) 認領與更新 `queued → reading → discussed`。讀原文後，在筆記填入 reader、日期、read scope 與「核心概念 → 我們怎麼用 → 小實驗 → 限制」；AI 摘要不代表組員已讀。

新論文使用 [模板](01_literature/templates/paper-note.md)，確認原始題名、作者、年份、DOI／arXiv 版本及結果定位；更新文獻 README 與 references.bib，保留來源、更正紀錄與預印本／正式版差異。
