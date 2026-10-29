# ENPM 645 Final Project — Collaborative Object Reorientation

**兩個機器人握持端與一個人手共同操作板材；未來從物理互動歷史學習物體層級的協助力矩，由 Model-Based Controller 負責力分配與執行。**

目前題名：**Intent-Aware Physical Human-Robot Collaboration Using Force and Tactile Sensing**

UMD ENPM 645 的兩人專題。應用動機是焊接、噴漆或表面處理前的工件重新定向；目前已完成 CPU MuJoCo 的 Phase 0 v1/v2 pilot，以及 Phase 1A 固定場景與資料管線；尚未訓練模型。

## 從這裡開始

0. 先看 [Phase 1A 使用方式](phase1a/README.md)、[結果報告](results/phase1a/report.md) 與 [組員 Controller Handoff](docs/CONTROL_HANDOFF.md)：**Data Pipeline GO；Controller Integration Ready**。資料管線可獨立使用 support-only baseline；下一步由 Track B 開發力分配與追蹤，再建立 expert labels。
   [原 v2 架構](docs/system-architecture-v2.md)、[v2 報告](phase0_v2/report.md) 與 [v1 報告](phase0_tactile/report.md) 完整保留。先前 Learning-Data NO-GO 指教師標籤尚未驗證，不妨礙目前 Track A 資料管線與 Track B 控制器分工。
1. 看 [初版專題概念](docs/project-concept.md)：初期三模態探索；目前分工與範圍以 Phase 1A 規格為準。
2. 依 [優先閱讀清單](01_literature/README.md) 先讀 P01 → P02 → P03。
3. 看 [跨論文快速結論](01_literature/synthesis.md)：哪些概念能直接進入我們的設計。
4. 使用 [閱讀筆記模板](01_literature/templates/paper-note.md) 補上自己的結論，再依 [協作方式](CONTRIBUTING.md) 更新 repo。

## 目前決策

| 項目 | 狀態 |
| --- | --- |
| 第一階段 | **模擬環境**（使用者已確認） |
| 感知來源 | 左右獨立 F/T、tactile、object state；S1/S2/S3 分開評估，VLM 不在目前 MVP |
| 本次最小任務 | 單一虛擬人手 + 兩個相隔 40 cm、各自雙 pad 的夾爪 + 剛性板材 |
| 未來學習目標 | 物體 COM 上 world +y 軸額外協助力矩（Nm），不是關節動作或 A/B/C 分類 |
| 本次角度範圍 | 小角度主動旋轉、保持、保持＋外部擾動；不宣稱已完成 90° 翻轉 |
| 模擬器 | 本機 Python + MuJoCo 3.3.5，CPU、headless、無 Docker |
| 已完成 | 文獻卡、v1/v2 pilot；Phase 1A frozen scenario、typed controller interface、4 個 smoke episodes / 1,700 transitions、因果隔離與回歸驗證 |
| 下一步 | Track B 實作 object-wrench allocation / tracking；Track A 後續建立驗證過的 expert 與正式示範；目前不訓練或做 CG-DAgger |

## Repo 結構

```text
phase1a/               # 目前：固定 v2 場景、desired torque interface、因果 transition pipeline
results/phase1a/       # 4 個 smoke episodes、validation、圖表、Q1–Q9 report
phase0_v2/              # 保留：雙握持端、分層力矩學習架構與可重現驗證
  README.md / report.md
  simulation/ / sensing/ / control/ / data/ / scripts/
  results/pilot/        # v2 原始資料、分析與驗證
phase0_tactile/
  README.md / report.md  # 實作、結果、NO-GO 原因
  models/ / scripts/    # MuJoCo 場景與重現流程
  results/pilot/        # 完整 pilot 資料、CSV、圖表與物理檢查
01_literature/
  README.md              # 優先順序、來源連結、閱讀追蹤
  papers/                # 每篇論文的核心概念與初步筆記
  synthesis.md           # 跨論文設計啟示與尚待驗證的假設
  search-log.md          # 檢索／核對方法與更正紀錄
  references.bib         # 9 篇引用；區分正式出版與公開版本
  templates/paper-note.md
docs/
  CONTROL_HANDOFF.md / SCENARIO_V1.md / DATA_SCHEMA.md
  system-architecture-v2.md
  project-concept.md
CONTRIBUTING.md
```

筆記以繁體中文為主，保留英文術語與原始題名。來源查核日：**2026-10-10**。
初步筆記由 Codex 輔助整理；查核範圍逐篇標示，**不代表任何組員已讀完論文**。實驗結果另見 v1/v2 報告，與文獻閱讀狀態分開紀錄。
