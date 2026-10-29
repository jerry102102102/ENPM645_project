# 使用者指定範圍 — Dual Franka Panda Integration

以現有 Phase 1A / Scenario v2 / Hardware v1 為基礎，加入兩支 MuJoCo Menagerie Franka Emika Panda，各 7 個 arm joints，固定世界基座。主要目標是雙臂物理整合與資料記錄，不自行開發完整 Model-Based Controller 或 Expert。

1. 依既有 panel grasp locations 求合理初始 joints，檢查可達性、limits、碰撞。
2. 優先保留自訂夾爪與 tactile，接 Panda flange；明確記錄必要修改。
3. 正式物理承重由關節與接觸完成，不使用獨立 freejoint virtual wrench drive。
4. 保留單一 Virtual Human Hand、名義握持點與 A/B/C 定義。
5. 新增 q、dq、actuator torques、joint commands、EE poses、gripper states，正確同步。
6. 保留 object-level desired-assist torque 的 World +Y、Panel COM、Nm 定義。
7. 更新 whitelist，但 Joint States 預設只記錄，是否加入 learner 留給後續消融。
8. 新 Hardware / Scenario version；原版本與 Phase 1A 驗證結果保留。
9. 驗證基本支撐、接觸保持、logging、無不合理自碰撞／穿模／數值爆炸。
10. 分別報告 Arm Model Integration、Physical Coupling、Data Pipeline、Control Readiness。

不可將外觀顯示當成物理耦合，不可將 support-only 假稱完整雙臂控制。如物理耦合未通過，保留模型與 logger，誠實標示。禁止 AI 訓練、正式 Expert Labels、CG-DAgger、ViT、RL。交付可重現場景、報告、資料契約、測試與組員接入文件。
