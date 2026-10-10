# P01 — Human-Robot Cooperative Object Manipulation with Contact Changes

> **最值得帶回專題的概念：把翻轉拆成「辨識方向、確認人已準備好、協作執行、交還主導權」，用力訊號同步各階段。**

## 來源與狀態

- 作者：Michael Gienger; Dirk Ruiken; Tamas Bates; Mohamed Regaieg; Michael Meißner; Jens Kober; Philipp Seiwald; Arne-Christoph Hildebrandt
- 年份／venue：2018 · IROS 2018, pp. 1354–1360
- DOI：[10.1109/IROS.2018.8594140](https://doi.org/10.1109/IROS.2018.8594140)
- 原文入口：[論文／摘要](https://www.honda-ri.de/pubs/pdf/3729.pdf) · [出版紀錄](https://research.tudelft.nl/en/publications/human-robot-cooperative-object-manipulation-with-contact-changes/) · [官方 Demo](https://www.honda-ri.de/human-robot-cooperative-object-manipulation-with-contact-changes/)
- 閱讀優先：必讀 1
- 核對日期：2026-10-10
- AI 查閱範圍：全文重點核對：III Interaction Layer、VI Robot Setup、VII Evaluation
- Human reading status：**queued**
- 團隊 reader：待認領
- Citation key：`gienger_2018_contact_changes`

## 論文結果（依上述查阅範圍）

雙臂移動機器人與人共同將箱子轉動 90°；該實驗以關節力矩估計手端力，人工調整門檻。整體硬體也裝有六軸 F/T 與 tactile pads，不能把實驗所用訊號等同於全部硬體能力。 來源：[原文](https://www.honda-ri.de/pubs/pdf/3729.pdf)。

## 我們可以怎麼用（設計推論）

先做不用換手的單軸狀態機，保留 readiness 與 UNKNOWN；把高階意圖估計、運動規劃、順應控制分開。

**第一個要驗證的問題：** 單臂固定握點能完成多少角度？人換握時，支撐與暫停條件如何定義？

## 不能直接推論的部分

雙臂與接觸變更的系統；並非單臂、任意形狀或三模態模型的直接驗證。初始提示動作仍需向使用者說明。

## 建議閱讀焦點

用原文確認感測器裝在哪裡、資料如何處理、意圖如何定義、控制介面是什麼，以及實驗是否真的與我們的單臂模擬翻轉相同。重要結果補上 section／figure／table locator；不要沿用其他任務的成功率作為我們的成果。

## 團隊閱讀結論（由實際閱讀者填）

- Reader／日期：
- Read scope（全文或哪些章節）：
- 用自己的話重寫核心概念：
- 支持段落／Figure／Table：
- 我們決定採用／暫緩的部分與理由：
- 最小驗證實驗：
- 另一位組員的問題：

---
初步筆記由 Codex 輔助整理；來源存在與局部內容核對不等於團隊已完成全文 review。
