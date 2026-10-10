# P05 — Robot to Human Object Handover Using Vision and Joint Torque Sensor Modalities

> **最值得帶回專題的概念：各模態回答不同問題，再融合成可解釋的動作決策。**

## 來源與狀態

- 作者：Mohammadhadi Mohandes; Behnam Moradi; Kamal Gupta; Mehran Mehrandezh
- 年份／venue：2023 · Robot Intelligence Technology and Applications 7 (RiTA 2022), LNNS 642, pp. 109–124; preprint 2022
- DOI：[10.1007/978-3-031-26889-2_11](https://doi.org/10.1007/978-3-031-26889-2_11)
- 原文入口：[論文／摘要](https://link.springer.com/chapter/10.1007/978-3-031-26889-2_11) · [開放 preprint](https://arxiv.org/abs/2210.15085)
- 閱讀優先：方法參考
- 核對日期：2026-10-10
- AI 查閱範圍：Springer metadata／摘要 + arXiv 摘要；未逐節審讀全文
- Human reading status：**queued**
- 團隊 reader：待認領
- Citation key：`mohandes_2023_handover_fusion`

## 論文結果（依上述查阅範圍）

關節力矩網路判斷 pull／hold／bump，RGB-D 網路辨識人手是否包覆物體，最後決定是否釋放。正式章節於 2023 出版，不能只標成 2022 未出版稿。 來源：[原文](https://link.springer.com/chapter/10.1007/978-3-031-26889-2_11)。

## 我們可以怎麼用（設計推論）

讓 F/T 預測旋轉方向、觸覺評估握持、VLM 描述可見情境，再用明確規則融合。

**第一個要驗證的問題：** 兩個模態不同意時，怎麼維持支撐並等待更可靠的資訊？

## 不能直接推論的部分

交接任務，不是共同旋轉；其準確率不代表翻轉控制表現。也不是 VLM 方法。

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
