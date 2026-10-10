# P08 — Multistep Intent Estimation Guided Adaptive Passive Control for Safety-Aware Physical Human-Robot Collaboration

> **最值得帶回專題的概念：未來意圖預測與協助強度可以結合，但控制穩定性需另外處理。**

## 來源與狀態

- 作者：Haotian Liu; Zhengtao Zhang; Yuchuang Tong; Zhaojie Ju
- 年份／venue：2026 · IEEE Transactions on Cybernetics, 56(1), 579–591
- DOI：[10.1109/TCYB.2025.3616392](https://doi.org/10.1109/TCYB.2025.3616392)
- 原文入口：[論文／摘要](https://pubmed.ncbi.nlm.nih.gov/41071693/) · [DOI 出版入口](https://doi.org/10.1109/TCYB.2025.3616392)
- 閱讀優先：進階選讀
- 核對日期：2026-10-10
- AI 查閱範圍：PubMed 收錄的論文摘要與 metadata；IEEE 全文未取得
- Human reading status：**queued**
- 團隊 reader：待認領
- Citation key：`liu_2026_multistep_passive_control`

## 論文結果（依上述查阅範圍）

摘要描述 Transformer + CVAE 預測 motion／force、GoRL 調節 impedance，以及 energy tank-based passive MPC。這是一個整合預測與控制的進階架構。 來源：[原文](https://pubmed.ncbi.nlm.nih.gov/41071693/)。

## 我們可以怎麼用（設計推論）

借用高階預測與低階控制分層；第一版用簡單可校準控制器，等 baseline 成立再考慮多步預測。

**第一個要驗證的問題：** 本專題需要提前多少時間預測？複雜控制器是否比簡單順應控制有可量測收益？

## 不能直接推論的部分

目前僅查閱摘要，未核實完整实验設定／基準；無法據此宣稱本專題具備 passivity 或安全保證。

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
