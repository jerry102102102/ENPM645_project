# ENPM 645 Final Project — Collaborative Object Reorientation

**讓人與機械手臂共同握住工件，在沒有語音指令時辨識人的翻轉意圖，並協助將工件轉到另一面。**

目前題名：**Intent-Aware Physical Human-Robot Collaboration Using Force and Tactile Sensing**

UMD ENPM 645 的兩人專題。應用動機是焊接、噴漆或表面處理前的工件重新定向；目前已完成 CPU MuJoCo 的 Phase 0 感測可觀測性 pilot。

## 從這裡開始

0. 先看 [Phase 0 實驗報告](phase0_tactile/report.md) 與 [執行說明](phase0_tactile/README.md)：30 episodes 已執行；**NO-GO，尚未證明 tactile 的額外意圖辨識資訊**。
1. 看 [初版專題概念](docs/project-concept.md)：初期三模態探索；目前範圍以 Phase 0 規格為準。
2. 依 [優先閱讀清單](01_literature/README.md) 先讀 P01 → P02 → P03。
3. 看 [跨論文快速結論](01_literature/synthesis.md)：哪些概念能直接進入我們的設計。
4. 使用 [閱讀筆記模板](01_literature/templates/paper-note.md) 補上自己的結論，再依 [協作方式](CONTRIBUTING.md) 更新 repo。

## 目前決策

| 項目 | 狀態 |
| --- | --- |
| 第一階段 | **模擬環境**（使用者已確認） |
| 感知來源 | Phase 0：F/T 重建、接觸力分布、object state ground truth；VLM 不在目前 MVP |
| 本次最小任務 | 虛擬人手 impedance + 雙側摩擦夾爪 + 單軸 wrist fixture + 剛性板材 |
| 本次角度範圍 | 小角度主動旋轉、保持、保持＋外部擾動；不宣稱已完成 90° 翻轉 |
| 模擬器 | 本機 Python + MuJoCo 3.3.5，CPU、headless、無 Docker |
| 已完成 | 9 篇來源核對、閱讀卡；30 episodes、物理對照、6 張分析圖與報告 |
| 尚未完成 | tactile 額外資訊的有效 A/C 匹配驗證、意圖辨識、完整協作控制器、實體驗證 |

## Repo 結構

```text
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
  project-concept.md
CONTRIBUTING.md
```

筆記以繁體中文為主，保留英文術語與原始題名。來源查核日：**2026-10-10**。
初步筆記由 Codex 輔助整理；查核範圍逐篇標示，**不代表任何組員已讀完論文**。實驗結果另見 Phase 0，與文獻閱讀狀態分開紀錄。
