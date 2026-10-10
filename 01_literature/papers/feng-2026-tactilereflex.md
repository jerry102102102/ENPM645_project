# P07 — TactileReflex: Noise-Statistics-Driven Vision-Tactile Reflex Control for Force-Sensitive Manipulation

> **最值得帶回專題的概念：把握持保護做成獨立觸覺反射層，與高階意圖模型分工。**

## 來源與狀態

- 作者：Ziyan Feng; Yulong Fu; Zheng Li; Yuxin He; Jieji Ren; Yudong Zhong; Lujia Wang; Jinni Zhou; Qiang Nie
- 年份／venue：2026 · IROS 2026 accepted（arXiv v4 與作者頁確認）；目前引用公開 arXiv 版本
- DOI：[10.48550/arXiv.2605.23568](https://doi.org/10.48550/arXiv.2605.23568)
- 原文入口：[論文／摘要](https://arxiv.org/abs/2605.23568v4) · [作者專案／Demo](https://shayfeng.github.io/TactileReflex/)
- 閱讀優先：觸覺延伸
- 核對日期：2026-10-10
- AI 查閱範圍：arXiv v4 摘要／metadata + 作者專案頁；未逐節審讀全文
- Human reading status：**queued**
- 團隊 reader：待認領
- Citation key：`feng_2026_tactilereflex`

## 論文結果（依上述查阅範圍）

從視覺式觸覺影像提取 shear、contact intensity 與 center of pressure proxies，利用噪聲統計設定防滑、釋力與力保護控制。作者頁與 arXiv v4 標記 IROS 2026 accepted。 來源：[原文](https://arxiv.org/abs/2605.23568v4)。

## 我們可以怎麼用（設計推論）

在模拟中先用接觸／滑移特徵建立握持監控，與意圖辨識分開報告成效。

**第一個要驗證的問題：** 增加 tactile 的主要收益是更準確的意圖分類，還是更少的滑移／失敗？

## 不能直接推論的部分

脆弱容器操作，不是人類翻轉意圖辨識；影像 proxy 不是已校準的牛頓力。模擬 contact force 也不是同一種 tactile image。

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
