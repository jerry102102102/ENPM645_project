# 01 — 應讀論文與快速結論

這個資料夾回答：**這篇論文最核心的概念是什麼？能如何幫助我們做無語音協作翻轉？**

先讀 P01、P02、P03，再依實作瓶頸選其他篇。每篇已有短版閱讀卡與原文入口；團隊尚未回報已讀，所有人類閱讀狀態先設為 queued。AI 核對範圍與團隊讀過的範圍分開記錄。

## 優先閱讀

| 順序／ID | 論文 | 最值得帶回專題的概念 | 原文 | 閱讀卡 |
| --- | --- | --- | --- | --- |
| 必讀 1 / P01 | Human-Robot Cooperative Object Manipulation with Contact Changes (2018) | 把翻轉拆成「辨識方向、確認人已準備好、協作執行、交還主導權」，用力訊號同步各階段。 | [開啟](https://www.honda-ri.de/pubs/pdf/3729.pdf) | [筆記](papers/gienger-2018-contact-changes.md) |
| 必讀 2 / P02 | Human-robot planar co-manipulation of extended objects: data-driven models and control from human-human dyads (2024) | 同一個端點受力可能代表平移或旋轉；意圖辨識要考慮時序與物體運動。 | [開啟](https://www.frontiersin.org/journals/neurorobotics/articles/10.3389/fnbot.2024.1291694/full) | [筆記](papers/mielke-2024-planar-co-manipulation.md) |
| 必讀 3 / P03 | Understanding human co-manipulation via motion and haptic information to enable future physical human-robotic collaborations (2025) | 已在運動時，速度很有資訊；起動、停止與低速定位應另外評估。 | [開啟](https://www.frontiersin.org/journals/neurorobotics/articles/10.3389/fnbot.2025.1480399/full) | [筆記](papers/shaw-2025-motion-haptics.md) |
| 方法參考 / P04 | Vision- and Tactile-Based Continuous Multimodal Intention and Attention Recognition for Safer Physical Human–Robot Interaction (2024) | 接觸訊號需要視覺上下文，才可能判斷互動是否有意。 | [開啟](https://arxiv.org/abs/2206.11350) | [筆記](papers/wong-2024-vision-tactile-intention.md) |
| 方法參考 / P05 | Robot to Human Object Handover Using Vision and Joint Torque Sensor Modalities (2023) | 各模態回答不同問題，再融合成可解釋的動作決策。 | [開啟](https://link.springer.com/chapter/10.1007/978-3-031-26889-2_11) | [筆記](papers/mohandes-2023-handover-fusion.md) |
| 方法參考 / P06 | Sensorless Human–Robot Interaction: Real-Time Estimation of Co-Grasped Object Mass and Human Wrench for Compliant Interaction (2025) | 人類意圖力必須與物體重量及動態效應區分。 | [開啟](https://advanced.onlinelibrary.wiley.com/doi/10.1002/aisy.202400616) | [筆記](papers/bai-2025-object-mass-human-wrench.md) |
| 觸覺延伸 / P07 | TactileReflex: Noise-Statistics-Driven Vision-Tactile Reflex Control for Force-Sensitive Manipulation (2026) | 把握持保護做成獨立觸覺反射層，與高階意圖模型分工。 | [開啟](https://arxiv.org/abs/2605.23568v4) | [筆記](papers/feng-2026-tactilereflex.md) |
| 進階選讀 / P08 | Multistep Intent Estimation Guided Adaptive Passive Control for Safety-Aware Physical Human-Robot Collaboration (2026) | 未來意圖預測與協助強度可以結合，但控制穩定性需另外處理。 | [開啟](https://pubmed.ncbi.nlm.nih.gov/41071693/) | [筆記](papers/liu-2026-multistep-passive-control.md) |
| 進階選讀 / P09 | Embodied sensorimotor integration for whole-arm tactile sensing and adaptive robotic manipulation (2026) | 分布式觸覺與 proprioception 結合，可將接觸資訊連到動作調整。 | [開啟](https://www.nature.com/articles/s44460-026-00097-1) | [筆記](papers/tang-2026-emarm.md) |

## 團隊閱讀追蹤

| ID | Reader | Human reading status | Team read scope／日期 |
| --- | --- | --- | --- |
| P01 | 待認領 | queued | — |
| P02 | 待認領 | queued | — |
| P03 | 待認領 | queued | — |
| P04 | 待認領 | queued | — |
| P05 | 待認領 | queued | — |
| P06 | 待認領 | queued | — |
| P07 | 待認領 | queued | — |
| P08 | 待認領 | queued | — |
| P09 | 待認領 | queued | — |

建議第一輪一人負責 P01、一人負責 P02，再共同討論 P03。分工是建議，認領後再填入姓名。

## 讀完要留下什麼

每篇最後填寫一個能帶回專題的判斷：

> 「這篇讓我們知道 ___。因此我們會用 ___。先做 ___ 的小實驗；它不能證明 ___。」

不要只重述 abstract。參考 [模板](templates/paper-note.md) 與 [協作方式](../CONTRIBUTING.md)。

## 合併後的設計啟示

[跨論文快速結論](synthesis.md) 將各篇映射到互動狀態機、意圖歧義、載荷補償、融合與觸覺保護層。VLM 是本專題想驗證的延伸，這 9 篇不能直接證明其效益。

## 引用與查核

- [references.bib](references.bib)：正式出版年份／公開版本分開處理。
- [search-log.md](search-log.md)：9 篇查核範圍、原始貼文更正與检索限制。
- 以合法原文連結提供存取；此 repo 不保存論文 PDF 的副本。
- 筆記是 AI 輔助的起始材料，部分只核對摘要；逐篇列出範圍，待組員補上全文閱讀與結論。

更新日：2026-10-10。
