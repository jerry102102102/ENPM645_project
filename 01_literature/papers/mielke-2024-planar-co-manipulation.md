# P02 — Human-robot planar co-manipulation of extended objects: data-driven models and control from human-human dyads

> **最值得帶回專題的概念：同一個端點受力可能代表平移或旋轉；意圖辨識要考慮時序與物體運動。**

## 來源與狀態

- 作者：Erich Mielke; Eric Townsend; David Wingate; John L. Salmon; Marc D. Killpack
- 年份／venue：2024 · Frontiers in Neurorobotics, 18, 1291694
- DOI：[10.3389/fnbot.2024.1291694](https://doi.org/10.3389/fnbot.2024.1291694)
- 原文入口：[論文／摘要](https://www.frontiersin.org/journals/neurorobotics/articles/10.3389/fnbot.2024.1291694/full) · [PMC 全文](https://pmc.ncbi.nlm.nih.gov/articles/PMC10894988/)
- 閱讀優先：必讀 2
- 核對日期：2026-10-10
- AI 查閱範圍：全文重點核對：4.2–4.4、5.3–5.4、6 Conclusion
- Human reading status：**queued**
- 團隊 reader：待認領
- Citation key：`mielke_2024_planar_co_manipulation`

## 論文結果（依上述查阅範圍）

比較力矩觸發的 EVIC 與運動預測的 NNPC，實際人機控制驗證限於平面 3-DoF。NNPC 的客觀表現較佳，但安全感、可預測性等偏好趨勢未達統計顯著；Good Force Amount 問項有顯著差異。 來源：[原文](https://www.frontiersin.org/journals/neurorobotics/articles/10.3389/fnbot.2024.1291694/full)。

## 我們可以怎麼用（設計推論）

把平移與調整握持設為翻轉辨識的負例；比較固定門檻與短時序模型。之後除了成功率，也評估協作是否容易理解。

**第一個要驗證的問題：** 在單軸翻轉中，哪些擾動最容易造成錯誤啟動？如何定義旋轉起始時間？

## 不能直接推論的部分

不是翻面旋轉的直接證據；系統使用雙臂／移動平台與 motion capture。不能把低延遲相機估計當成同等品質。

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
