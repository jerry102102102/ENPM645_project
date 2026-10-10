# P06 — Sensorless Human–Robot Interaction: Real-Time Estimation of Co-Grasped Object Mass and Human Wrench for Compliant Interaction

> **最值得帶回專題的概念：人類意圖力必須與物體重量及動態效應區分。**

## 來源與狀態

- 作者：Minghao Bai; Xizhe Zang; Lei Wu; Changle Li; Yanhe Zhu; Jie Zhao
- 年份／venue：2025 · Advanced Intelligent Systems, 7(6), 2400616; online 2024-09-30
- DOI：[10.1002/aisy.202400616](https://doi.org/10.1002/aisy.202400616)
- 原文入口：[論文／摘要](https://advanced.onlinelibrary.wiley.com/doi/10.1002/aisy.202400616)
- 閱讀優先：方法參考
- 核對日期：2026-10-10
- AI 查閱範圍：全文重點核對：Abstract、2 Experimental Section、2.1–2.4
- Human reading status：**queued**
- 團隊 reader：待認領
- Citation key：`bai_2025_object_mass_human_wrench`

## 論文結果（依上述查阅範圍）

以 EKF 估計物體參數，搭配 disturbance observer 估計人類 wrench。重要更正：不需要人側力感測器，但 Experimental Section 明寫機器人末端裝有 6D F/T sensor。 來源：[原文](https://advanced.onlinelibrary.wiley.com/doi/10.1002/aisy.202400616)。

## 我們可以怎麼用（設計推論）

模擬可用真值評估 wrench 估計误差；實際辨識輸入只用感測量／估計量。先校準載荷，再測換重量時的誤判。

**第一個要驗證的問題：** 簡單重力補償是否已足夠？何種負載／速度範圍才需要完整 observer？

## 不能直接推論的部分

sensorless 不代表整個系統沒有 F/T；需要模型與握持幾何假設，不能直接以末端讀值等同人類施力。

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
