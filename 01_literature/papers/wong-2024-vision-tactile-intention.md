# P04 — Vision- and Tactile-Based Continuous Multimodal Intention and Attention Recognition for Safer Physical Human–Robot Interaction

> **最值得帶回專題的概念：接觸訊號需要視覺上下文，才可能判斷互動是否有意。**

## 來源與狀態

- 作者：Christopher Yee Wong; Lucas Vergez; Wael Suleiman
- 年份／venue：2024 · IEEE Transactions on Automation Science and Engineering, 21(3), 3205–3215; online 2023
- DOI：[10.1109/TASE.2023.3276856](https://doi.org/10.1109/TASE.2023.3276856)
- 原文入口：[論文／摘要](https://arxiv.org/abs/2206.11350) · [IEEE 出版紀錄](https://ieeexplore.ieee.org/document/10144527/)
- 閱讀優先：方法參考
- 核對日期：2026-10-10
- AI 查閱範圍：arXiv 摘要與 IEEE 索引摘要／出版 metadata；未逐節審讀全文
- Human reading status：**queued**
- 團隊 reader：待認領
- Citation key：`wong_2024_vision_tactile_intention`

## 論文結果（依上述查阅範圍）

以觸碰位置、人體姿態與注視等特徵分類 intentional／unintentional touch，摘要報告 F1 86%。使用的是機器人皮膚上的接觸資訊。 來源：[原文](https://arxiv.org/abs/2206.11350)。

## 我們可以怎麼用（設計推論）

借用多模態互補與低維特徵的思路；將 VLM 的上下文輸出與 F/T 意圖線索分別消融比較。

**第一個要驗證的問題：** 哪些視覺線索可由渲染影像可靠取得？VLM 在遮擋時如何表達不確定？

## 不能直接推論的部分

類別不是翻轉／平移；86% 不能當成本專題預期分數。人直接觸碰機器人的配置，也不同於共同握持物體。

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
