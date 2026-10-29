# Frozen Scenario v1

> 歷史配置，保留供舊結果重現。新版「機器人同側、人手對側的邊緣握持」見 [SCENARIO_V2.md](SCENARIO_V2.md)。本檔的 robot grasp point 指 between-jaw frame origin，並非實體 pad 接觸點；舊 pads 在板材上下表面較內側，舊 human site 是內部虛擬施力點。

唯一設定：[scenario_v1.json](../phase1a/config/scenario_v1.json)；鎖定索引：[scenario_registry.json](../phase1a/config/scenario_registry.json)。設定由 `DualPilot(parameters(0,64600))` 的 **有效 compiled model / initialized state** 擷取，並核對既有 v2 A_000 metadata，不猜測座標。

`scenario_id=enpm645-dual-gripper`，`scenario_version=1.0.0`。model_hash 是原 v2 MJCF 的 SHA-256；config_hash 是 sorted-key、compact JSON 的 SHA-256。每個 episode 都保存四者。載入時檢查 registry、MJCF、依賴實作、來源 metadata 及重新建立的 scene snapshot。修改 geometry、friction 或名義參數時應建立新版本與新 registry entry；不要覆寫 v1。snapshot 顯示值四捨五入至 12 位，實際 parameters 與 dependency hashes 保存完整可重現設定。

## 幾何、接觸與支撐

| 項目 | 從 v2 擷取的值 |
|---|---|
| Panel | box，0.6 × 0.2 × 0.02 m，1.0 kg |
| Principal inertia | [0.003366666667, 0.030033333333, 0.033333333333] kg·m²；inertial quaternion [1,0,0,0] |
| 初始 world COM | [0,0,0.8] m，pitch 0.005870742649786773 rad |
| 初始 quaternion | [0.999995691801,0,0.002935367110,0] wxyz |
| Left nominal grasp | **[-0.2,0,0] m，panel-local**；relative quaternion [1,0,0,0] |
| Right nominal grasp | **[0.2,0,0] m，panel-local**；relative quaternion [1,0,0,0] |
| 唯一 human grasp | **[0.27,-0.08,0] m，panel-local** |
| Pads | 每個 gripper upper/lower 各一個 box，0.09 × 0.14 × 0.02 m；root-local body position [0,0,±0.02] m |
| Pad preload | four motor commands [-40,+40,-40,+40] N，這是 actuator 設定，不是假定量測接觸力 |
| 有效 geom friction | [1.0482960672343025,0.005,0.0001]，condim=3 |
| Contact compliance | solref=[0.01,1]；solimp=[0.95,0.99,0.001,0.5,2] |
| Root drive limits | 每端 force norm 35 N、torque norm 3 Nm |
| Pad command limits | upper [-80,0] N；lower [0,80] N |
| Gravity / timestep | [0,0,-9.81] m/s² / 0.002 s |

Roots 與 panel 都是 free joints，沒有 equality constraints，也沒有 world-fixed fixture。名義 grasp 固定於 panel-local 定義，**實際 root pose/contact points 仍由動力學決定**。

每端 upper touch sensor 在 pad-local [0,0,+0.08] m、quaternion [1,0,0,0]；lower 在 [0,0,-0.08] m、quaternion [0,1,0,0]。保留原 v2 angular-bin tactile，不重新設計 planar array。完整 collision masks、slide ranges、sensor names、baseline gains 與初始 gripper world poses 均在 JSON。

## 一個人手、獨立環境擾動與 A/B/C

Human 是單一 panel-attached point spring-damper：`F = Kh*(target-point) + Dh*(target_velocity-point_velocity)`，Kh=98.21089225920257 N/m、Dh=7.3012286492091985 N·s/m、force norm 上限 50 N；target speed 上限 0.35 m/s。人手改變目標位置，panel-local attachment 不變。

| Scenario | Human goal | 額外環境擾動 |
|---|---|---|
| A — ROTATE | 從初始 pitch 向正 Y 增加 0.36680278277879985 rad；0.7 s 開始、0.8 s smooth ramp | 無 |
| B — HOLD | 保持初始目標角度 | 無 |
| C — RESIST | **HOLD**，與 B 相同 human goal | 有 |

固定 C 擾動作用點 panel-local [0.06369127617716655,-0.015330968965703077,0] m，方向 world [0,0,-1]，peak 18.83430489381801 N。0.7 s 開始 smooth ramp-in、2.7 s 開始 ramp-out，各 0.15 s；因此是有限持續擾動，不是無限期常力。施力點隨板材運動，方向仍是 world vertical。

固定 seed=64600、block=0；先 settle 2 s（episode clock 尚未開始），A/B/C 各記錄 4 s；reference test 用 B 記錄 5 s。沒有額外 domain randomization。未來若改擾動幅度/時序，需明列版本或 episode parameters，不得靜默修改。

## Reference 與資料節奏

Desired assistance 是 world +Y、瞬時 panel COM、Nm，正負依右手定則。Reference test 使用 `0 → +0.02 → 0 → -0.02 → 0`，各 1 s；support_only 完全忽略它。幅度僅用於資料流驗證，沒有安全/可達或 tracking 宣稱。

Physics/control=500 Hz，reference/logging=100 Hz；每筆 transition 包含五個實際 control steps。更多 frame/time 定義見 [DATA_SCHEMA.md](DATA_SCHEMA.md)，接入方式見 [CONTROL_HANDOFF.md](CONTROL_HANDOFF.md)。
