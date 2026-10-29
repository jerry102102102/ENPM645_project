# 目前專案架構：學習協助力矩，控制器負責實現

以 [Phase 0 v2 規格](../phase0_v2/specification.md) 為準，這份文件取代初期「三模態意圖分類＋直接翻轉 demo」作為目前開發方向；[初版概念](project-concept.md) 保留作背景。

## 研究問題與責任分工

兩個機器人握持端支撐剛性板材，一個人手引導，另一隻手預留加工。未來模型讀取 observation history，輸出主要旋轉軸的**額外協助力矩**。先用 scalar Nm，不輸出關節角、joint torque 或左右手獨立 trajectories。

| 層級 | 責任 | 當前狀態 |
| --- | --- | --- |
| 感測與狀態 | 左右 F/T、tactile、object pose/velocity；保留同步與座標 | v2 已驗證 |
| Model-Based Expert | 可使用 privileged goal/外力、模型與限制，產生可實現的 assistance reference | 介面完成；尚無可信 labels |
| Learned Force Policy | 僅使用部署可得的 observation history，預測額外 torque | 尚未訓練 |
| Object-Wrench Controller | 重力／baseline + assistance、左右力分配、回饋追蹤與限制 | 介面完成；目前僅獨立 baseline support |
| 完整機械臂執行 | IK、關節層與工作空間 | 本階段不做 |

`τ_assist*` 定義為 world +y、關於當下 object COM、Nm、基本承重與穩定控制之外的協作力矩。總 reference、allocation、executed drive 與 realized contact wrench 必須分開。

若 `W_i` 是 panel-on-gripper、以 root local frame 表示的 F/T，則 object wrench 為 `Σ G_i(-W_i)`，其中 `G_i` 包含旋轉及從 root 到 COM 的 moment arm。不能直接相加不同 local frames 的六個數字。

## 階段與通過條件

1. **Physical pilot：已完成。** v1 保留，v2 雙握持端、接觸／力平衡、同步與回歸通過。自然匹配仍不足，未證明 tactile 的條件式增益。
2. **小型 Expert Controller 驗證：建議下一步。** 先驗證給定正負 object torque 是否可達，明確測量 tracking error、飽和、接觸保持及三種 HRI 情境的人手負擔。此步尚未實作。
3. **少量 demonstration feasibility：待教師通過。** 記錄 `(observation_history, expert_assist_torque_ref)`，檢查近似觀測與教師 action 是否衝突。通過後才正式收集。
4. **Offline force learning：待資料通過。** 小型 MLP/GRU，按整個 episode／nuisance block 分割；比較 S1（合 wrench）、S2（左右獨立 F/T）、S3（再加 tactile）。可以先做 S1/S2，不需等 tactile 的正面結論。
5. **簡化 CG-DAgger：待 Offline BC 通過。** 學生 rollout；程式偵測錯向、追蹤失敗、抵抗不足、過大力或接觸失效；由 scripted expert 介入，保存介入原因與修正 reference，聚合資料後離線重訓。不能用來解決感測本身不可觀測的教師目標。

A/B/C 仍用於生成與評估，B/C 的 human intent 都是 HOLD；模型的主要輸出已改為力矩，分類成功不再是目標本身。

**目前建議繼續小型 expert/controller 驗證；現有量測或 baseline drive 不能直接當成教師標籤。**詳細結果見 [v2 報告](../phase0_v2/report.md)。
