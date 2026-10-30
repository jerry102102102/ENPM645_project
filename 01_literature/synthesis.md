# 快速結論：這些論文怎麼變成我們的設計

**第一版先完成可解釋的協作翻轉迴路，再量測 tactile 與 VLM 的額外價值。** 這是從目前文獻作出的設計建議，並非已有三模態實驗成果。

| 我們的設計問題 | 文獻線索 | 可帶回的決定（團隊待討論） |
| --- | --- | --- |
| 何時跟隨、何時主動協助？ | [P01](papers/gienger-2018-contact-changes.md)：mixed initiative 與 readiness | 意圖不明時維持支撐，確認方向與接觸後才開始旋轉 |
| 推一下真的代表翻轉嗎？ | [P02](papers/mielke-2024-planar-co-manipulation.md)：translation／rotation ambiguity | 保留平移、握持調整與碰撞負例，別只訓練乾淨的旋轉 |
| 起動與停止怎麼評估？ | [P03](papers/shaw-2025-motion-haptics.md)：速度訊號的低速限制 | 分段評估 onset、持續旋轉、停止 |
| 多模態怎麼融合？ | [P04](papers/wong-2024-vision-tactile-intention.md)、[P05](papers/mohandes-2023-handover-fusion.md)：互補感知 | 各來源先輸出有限、可檢查的特徵／決策，再融合 |
| F/T 讀值能直接當人類意圖力嗎？ | [P06](papers/bai-2025-object-mass-human-wrench.md)：object dynamics 與 human wrench estimation | 先做載荷補償；評估重量改變造成的誤判 |
| Tactile 最先能幫什麼？ | [P07](papers/feng-2026-tactilereflex.md)：local grasp reflex | 先做握持監控，分開測分類改善與滑移減少 |
| 如何逐步走向主動預測？ | [P08](papers/liu-2026-multistep-passive-control.md)：prediction + adaptive control | baseline 成立後再考慮多步預測，避免第一版重現整套架構 |
| Tactile 感測位置有何差別？ | [P09](papers/tang-2026-emarm.md)：whole-arm tactile + proprioception | 明確區分全臂接觸、夾爪接觸与 wrist F/T |

## 不要預先假設融合一定更好

- P01 已用力線索完成特定箱子轉動。因此 **F/T baseline 必須認真做**，否則不能判斷融合有沒有價值。[P01](papers/gienger-2018-contact-changes.md)
- P03 支持運動訊號的價值。VLM 可能只重述物體已在轉動，額外推論延遲卻錯過起始階段；這是我們需要實驗檢查的替代解釋。[P03](papers/shaw-2025-motion-haptics.md)
- P07 的觸覺成果屬於局部操作保護。若 tactile 只減少滑移、沒有提升 intention F1，仍可報告為接觸穩定性的收益。[P07](papers/feng-2026-tactilereflex.md)
- P02 提醒我們：客觀效率與人的感受可能不一致。其安全／可預測性偏好趨勢未達顯著，不能寫成「已證明人更喜歡較簡單的控制器」。[P02](papers/mielke-2024-planar-co-manipulation.md)

## 對 VLM 的工作假設

目前選定清單沒有直接驗證 **VLM + tactile + F/T → 無語音單臂翻转**。我們建議先讓 VLM 提供可見的手／物體情境與不確定性，再與快速力學訊號融合。它是否改善遮擋、接觸準備或歧義判斷，都屬於待測假設。

建議比較只用 pose／幾何特徵的視覺版本，並測量 VLM 的延遲；這能分辨語意模型的價值與一般視覺資訊的價值。後續檢索應優先補上 VLM 在 physical co-manipulation／接觸情境推理的原始研究。

## 我們的最小驗證路徑

1. 在固定握點與單軸場景中建立 B0，確認感測方向、載荷補償與旋轉幾何。
2. 加入時序辨識與非旋轉負例，評估錯誤啟動。
3. 分別加入 tactile／VLM，再測全部融合；保持低階控制器一致。
4. 測 noise、latency、遮擋、質量變化與缺失模態。
5. 根據收益決定下一步：180°、換握、真實人類互動或更複雜預測。

具體架構與指標見 [目前介面](../docs/THREE_LAYER_INTERFACE.md)。模擬成績只支持該場景與人類代理；使用者體驗需要後續真實人的證據。

## 查閱限制與 AI 揭露

這是從 9 篇定向搜尋材料建立的初步 synthesis；部分僅查阅摘要，閱讀範圍逐篇公開。不是 PRISMA review、不是完整可重現性審查，也未確認研究新穎性。由 Codex 輔助整理，採用決策由團隊討論。
