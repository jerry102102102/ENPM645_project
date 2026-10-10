# P03 — Understanding human co-manipulation via motion and haptic information to enable future physical human-robotic collaborations

> **最值得帶回專題的概念：已在運動時，速度很有資訊；起動、停止與低速定位應另外評估。**

## 來源與狀態

- 作者：Kody Shaw; John L. Salmon; Marc D. Killpack
- 年份／venue：2025 · Frontiers in Neurorobotics, 19, 1480399
- DOI：[10.3389/fnbot.2025.1480399](https://doi.org/10.3389/fnbot.2025.1480399)
- 原文入口：[論文／摘要](https://www.frontiersin.org/journals/neurorobotics/articles/10.3389/fnbot.2025.1480399/full) · [PMC 全文](https://pmc.ncbi.nlm.nih.gov/articles/PMC12222233/)
- 閱讀優先：必讀 3
- 核對日期：2026-10-10
- AI 查閱範圍：全文重點核對：4 Results and analysis、5 Conclusion
- Human reading status：**queued**
- 團隊 reader：待認領
- Citation key：`shaw_2025_motion_haptics`

## 論文結果（依上述查阅範圍）

分析兩人／三人共同操作的運動與 haptic 訊號。結果支持速度作為未來運動的主要線索，也指出低速階段的限制；靜止到活動的轉換另以噪聲統計分析。 來源：[原文](https://www.frontiersin.org/journals/neurorobotics/articles/10.3389/fnbot.2025.1480399/full)。

## 我們可以怎麼用（設計推論）

將模型評估切成 onset、持續旋轉、停止三段；對靜止資料做校準，而非直接抄用其他硬體的固定門檻。

**第一個要驗證的問題：** 新增 F/T 或 tactile 後，改善是否集中在起始／停止，而非已明顯轉動的時段？

## 不能直接推論的部分

人與人的資料研究，不是已完成的機器人控制器，也未證明 VLM 或 tactile fusion 的收益。

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
