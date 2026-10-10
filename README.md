# ENPM 645 Final Project — Collaborative Object Reorientation

**讓人與機械手臂共同握住工件，在沒有語音指令時辨識人的翻轉意圖，並協助將工件轉到另一面。**

暫定英文題名：**Implicit Human Intent Recognition for Collaborative Object Reorientation Using Force/Torque, Tactile, and VLM Context**

UMD ENPM 645 的兩人專題。應用動機是焊接、噴漆或表面處理前的工件重新定向；目前階段為文獻整理與概念建立。

## 從這裡開始

1. 看 [專題概念](docs/project-concept.md)：場景、模擬範圍、三模態角色與可驗證問題。
2. 依 [優先閱讀清單](01_literature/README.md) 先讀 P01 → P02 → P03。
3. 看 [跨論文快速結論](01_literature/synthesis.md)：哪些概念能直接進入我們的設計。
4. 使用 [閱讀筆記模板](01_literature/templates/paper-note.md) 補上自己的結論，再依 [協作方式](CONTRIBUTING.md) 更新 repo。

## 目前決策

| 項目 | 狀態 |
| --- | --- |
| 第一階段 | **模擬環境**（使用者已確認） |
| 感知來源 | 假設 F/T、tactile、影像皆可用；視覺上下文由 **VLM** 提供 |
| 建議最小任務 | 一個人 + 單臂 + 剛性板材；固定握持、單一旋轉軸 |
| 建議初始目標 | 先驗證 ±90°，再依工作空間與握持限制考慮 180° 翻面 |
| 模擬器／robot model／VLM | 待團隊選定 |
| 已完成 | 9 篇來源核對、初步閱讀卡、概念與協作流程 |
| 尚未完成 | 團隊全文閱讀、程式實作、資料收集、任何成果驗證 |

## Repo 結構

```text
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
初步筆記由 Codex 輔助整理；查核範圍逐篇標示，**不代表任何組員已讀完論文**。目前沒有實驗結果。
