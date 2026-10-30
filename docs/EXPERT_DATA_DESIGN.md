# Expert & Data Design v1.0

本版完成 **E0/E1/E2 數學 reference generators、離線 shadow annotation、感測歷史與標籤一致性檢查**。沿用唯一 `phase1b` 實作、Scenario 3.0.0 / Hardware 2.0.0。未執行物理模擬、未收集 demonstrations、未訓練模型。

| 狀態 | 結論 |
|---|---|
| Reference / Interface Ready | 是；純 Python/NumPy，可產生 `AssistanceDecision` |
| Controller Implemented | 仍只有 reference-only / position-hold adapters |
| Assistance Tracking Validated | 否；公式輸出不代表雙臂可實現 |
| Expert Labels Validated | 否；execution_valid、supervision_eligible 皆 false |
| Learnability Validated | 否；只有診斷工具，尚未評估實驗資料 |

## 物理與單位

所有力矩為 **World +Y、instantaneous Panel COM、Nm**。角度 rad、角速度 rad/s、角加速度 rad/s²、時間為 episode seconds。

完整剛體關係為 `I_W α + ω × (I_W ω) = τ_h + τ_d + τ_r`，其中 `I_W = R I_body Rᵀ`。`required_assistance_y()` 保留陀螺耦合項，扣除人、擾動與基本控制力矩，求出額外需求。這是逆動力學需求，沒有證明接觸、摩擦、關節或 position servo 能達成。均勻重力對 COM 的直接力矩為零；邊緣承重接觸的力矩仍需計入。

`panel_inertia()` 使用長方體慣量。1 kg、0.6×0.2×0.02 m 的 `J_y = 0.03003333 kg·m²`。單軸近似為 `J_y θ̈ = τ_h + τ_d + τ_b + τ_a`。理想精確逆動力學搭配正 PD 增益可得到 `ë + k_d ė + k_p e = 0`；這不是目前雙 Panda 的穩定性保證。

`human_pitch_impedance(k_h,d_h,r_world)` 使用 `J_h = e_y × r_world`，得到 `K_θ = k_h ‖J_h‖²`、`D_θ = d_h ‖J_h‖²`。人手的隱藏目標透過阻抗影響力與運動，但外部擾動也會造成相同合力；不能保證唯一反推目標。

Reference generators 採單軸近似：pitch=`atan2(-R20,R00)`、ω=`ω_world,y`。任意 3D 運動下兩者不是精確微分關係；`off_axis_omega_norm` 供診斷，不自動認證近似成立。此版使用小角度、近 Y 軸且不跨 ±π 的操作範圍；完整 3D 與接觸驗證留待閉迴路測試。

## 三個 reference generators

程式：[`phase1b/expert/reference.py`](../phase1b/expert/reference.py)。每個 episode、每種 expert 各建一個 `ReferenceGenerator`；100 Hz、時間連續，缺步／重複／future teacher input 都拒絕。`confidence=None`。

| 策略 | 可見資訊 | 未限幅輸出 |
|---|---|---|
| E0：反應式基準 | 公開 F/T、EE pose、object pose/ω；拒絕 goal/disturbance truth | `k_f LPF(J_y α_hat − τ_L − τ_R)` |
| E1：主要候選教師 | 同一公開狀態 + 當下 teacher mode/goal；拒絕 disturbance truth | ROTATE：`J_y α_ref + Kp_R(θ_ref−θ) + Kd_R(ω_ref−ω)`；HOLD：`Kp_H(θ_hold−θ)−Kd_H ω` |
| E2：oracle 參考 | E1 資訊 + 當下擾動真值 | `raw_E1 − λ_d τ_d`，`0≤λ_d≤1`；UNCERTAIN 優先歸零 |

E0 的 α_hat 使用後向差分；LPF 係數 `1−exp(−dt/T)`，第一個有效估計初始化濾波器。首筆沒有過去速度，輸出 invalid UNCERTAIN/0，估計欄位 NaN + false mask。E0 的 mode 只是反應方向分類，不是真實意圖辨識。

公開接觸重建 F/T 是 **panel-on-gripper、gripper-local、gripper origin**。先反號旋轉成 robot-on-panel，再加 `(p_gripper−p_COM)×F`；左右求和。不可把 raw wrist transmitted wrench 直接套入此式，它另含下游重量／動力學。E0 需要公開 EE pose 做座標轉換；這是額外的機器人運動學資訊，與只給 local F/T + object 的 student profile 比較時應明示。

E1 的 ROTATE 只有在公開估計力矩達 on threshold、角速度方向一致、持續 confirm_seconds，且方向符合 teacher mode 時開啟。啟動後從當下量測角度建立內部 reference，以 `reference_rate_max*dt` 逐步接近當下 goal。原方向力矩或角速度高於 off threshold 可維持；兩者都不足並持續 release_seconds 便解除。反向力矩達 on threshold 立即取消原方向，反向啟動需重新確認。這只是工程閘門，仍可能被外力觸發，不能聲稱已分離人與擾動。

HOLD 入口鎖定量測角度，之後保留；UNCERTAIN 不推進 reference、協助力矩立即為零。E1/E2 內部 reference 不交給 Object Controller。E1 沒有全額取消人手和擾動；E2 的 λ_d 也必須與底層協調，避免重複補償。簡化靜態模型下，偏差為 `(1−λ_d)τ_d/K_sum`，只有在正總剛性、無飽和且達到平衡等假設下成立。

### 限幅與模式

`ExpertConfig` 的增益、閘門、torque_max、torque_rate_max、reference_rate_max 與 `limits_provenance` 都要明確提供；不供應假稱已可達的預設數字。Kp 為 Nm/rad，Kd 為 Nm·s/rad；k_f、λ_d 無單位；LPF/confirm/release 為 s；閘門力矩 Nm、角速度 rad/s。

順序：raw → amplitude clip → slew clip。初始前次力矩為零。**UNCERTAIN/閘門取消優先立即歸零**，必要時突破 slew bound，記錄 `rate_limit_overridden=true`；下層仍須處理物理平滑／接觸安全。這個旗標不能被解讀成 slew 通過。

**Interface v1.1 語意修正：ROTATE_POS 可輸出負力矩煞車，ROTATE_NEG 也可輸出正力矩。** mode 是意圖，torque sign 是施力方向；HOLD 可非零，UNCERTAIN 必須零。更新同一份型別，不另留舊分支。

## API 與下游責任

```python
from phase1b.expert.reference import ExpertConfig, PublicSample, TeacherGoal, ReferenceGenerator

config = ExpertConfig(**explicit_configuration)  # 完整欄位見 dataclass；含限制來源
expert = ReferenceGenerator("E1", config)
public = PublicSample.from_observation(observation_t)
candidate = expert.step(public, goal=TeacherGoal(public.timestamp, mode_t, goal_angle_t))
# 待控制器接入時，跨越下游邊界的只有：
decision = candidate.decision
```

TeacherGoal / DisturbanceTruth 是獨立 privileged stream，必須與當下 observation 同時間；不是 PolicyHistory 欄位。E0 不接受它們；E1 不接受擾動真值。現行 ControlChain 的 student policy 仍只收到白名單，沒有為 oracle 放寬權限。此版尚未將 teacher callback 接到物理 runner；下一階段須在上層提供明確的 teacher-only 呼叫點，Object/Arm Controller 照常只取得公開輸入與 decision。

E0/E1/E2 與未來 student 必須使用相同的 Object Controller、Dual-Arm Controller、limits 與公開 observation；不得將 `reference_angle`、goal、scenario、future commands 偷傳下游。候選力矩超過現行 ControlChain 的 .02 Nm **測試限額**仍會拒絕，不能以改大常數當成 tracking 驗證。Position servo 接受 rad，不能直接接 Nm。

## Data contract：原始 episode + shadow sidecar

唯一物理 writer 仍是 `phase1b.pipeline`；本版 `phase1b.expert.data` 只讀原始檔，另寫新目錄，已有輸出就拒絕覆寫。不重寫既有 observations、commands、privileged 或 supervision。

| 檔案 | 欄位／用途 |
|---|---|
| metadata.json | episode_id、dataset version、seed、annotation commit/dirty flag、source commit（未知則 null）、schema/MuJoCo/scenario/hardware versions、model/config/hardware/source/code/output hashes |
| metadata.json：物理 | panel mass/principal inertia/dimensions/friction；三個 panel-local grasps；human stiffness/damping/force/speed limits；controller components、position-servo type；dt/duration/settle；frames |
| privileged_metadata.json | teacher stream path/hash；scenario；精確 human target/ramp 與 disturbance point/magnitude/start/end；不進 student |
| derived_features.npz | timestamp、theta_y、omega_y、delta_theta_y、acceleration_y、interaction_torque_estimate、tau_left_y、tau_right_y、off_axis_omega_norm；不可用估計各有 mask |
| expert_internal/E0.npz、E1.npz、E2.npz | candidate torque/mode、expert_id/source、gate/direction、reference angle/velocity/acceleration、raw/amplitude/output torque、saturation/slew/override、label status/reason、三層 validity |
| supervision/E*.npz | 正式 expert_assist_torque_ref **全 NaN**；expert_mode 保留供分析但 mode_valid/torque_valid/label_valid **false**；candidate_valid 獨立；execution_valid/supervision_eligible **false** |

`candidate_valid` 只表示數值計算可用，並不保證可觀測、可執行、穩定或可學。內部 trace 的 `expert_mode_valid` 也只屬於 candidate，不能拿它取代 supervision mask。整個 expert_internal 目錄視為教師內部資料；診斷工具只抽取待分析 target，不把其他欄位當特徵。

Metadata 從匹配 hash 的固定模型/config/hardware 讀取。未知 controller version、source commit、disturbance magnitude_range 保留 null；episode_valid=null、failure_reason 指明未重新驗證。固定 config 的 protocol 是來源描述，當次精確標註以明確提供的 teacher stream 為準。可隨機化條件的 collector 完成後，必須記錄實際每段參數，不能拿固定 config 代填。

所有 observations、arm commands、post-step feedback 仍由原 episode 的 timestamps 關聯，原始檔 hash 被 sidecar 綁定。requested assistance 在 object_commands；requested total wrench / allocation / assistance tracking 目前 unavailable。實際總 contact wrench 不等於額外 assistance。

### 離線使用（本次未對實驗資料執行）

必須另有 teacher-input NPZ，包含 timestamp `(N,)`、ground_truth_intent `(N,)`（四種 mode 字串）、human_goal_angle `(N,)` rad、external_disturbance_wrench `(N,6)` world force→torque、about instantaneous panel COM。與 observations 精確對齊；禁止以 A/B/C 自動推定 intent。現有舊記錄缺少明確 ground_truth_intent，因此工具會拒絕直接當成完整教師輸入。

```bash
uv run --locked python -m phase1b.expert.data \
  --input PATH_TO_EPISODE --teacher-inputs PATH_TO_TEACHER_NPZ \
  --config PATH_TO_EXPLICIT_EXPERT_CONFIG_JSON \
  --episode-id UNIQUE_EPISODE_ID --output NEW_SHADOW_DIRECTORY
```

Config JSON 的必填欄位完整定義於 `ExpertConfig`，測試檔有**僅供算術測試、未驗證硬體可達性**的具名 fixture。每次 annotation 都保存完整 config 及 limits_provenance；目前不提供可冒充實驗設定的預設配置。

`validate_sidecar()` 檢查來源／教師／程式／輸出雜湊、時鐘、finite/bounds、UNCERTAIN、slew exception 與 supervision 隔離。這是資料完整性檢查，不是物理品質驗證；搬移資料需要保留／重新解析記錄的絕對來源路徑。

## Label learnability：只做診斷

`phase1b.expert.learnability` 建立 `[t−H+1,t]` 的因果感測窗口，抽取當下 candidate target。三種 profile：

1. `ft_object`：左右 F/T + object position/quaternion/linear/angular velocity。
2. `ft_object_joints`：再加 q/dq。
3. `ft_object_joints_tactile`：再加四 pad tactile。

F/T 來源必須一致，預設 contact_reconstructed，可明選 wrist_transmitted。Metadata、timestamps、IDs、goal、truth forces、expert reference、未來 observation/commands 都不是特徵。Quaternion 做符號正規化；其歐氏距離仍是簡化近似，非 SO(3) geodesic。

用 reference split 的逐維標準差縮放；常數維度採一個原生單位。以標準化 RMS distance <= radius 查鄰居，排除同 episode；sidecar CLI 用原始 observation hash 當 group，改名也不能變成獨立樣本。實驗者仍應按 seed／人手條件／擾動 protocol 分組切分，避免近重複 runs 外洩。

報告 coverage、局部 label std/range、相對鄰居均值誤差、超過 torque tolerance 的不一致／反號比例。最少鄰居是窗口數，**不等於獨立 episode 數**；先增加獨立 episodes，再比較 history/radius/profile，並一起報告 coverage。不同維度的同一 radius 不保證可比，不能只比較 std 就宣稱 tactile 有益。

無足夠匹配回傳 `insufficient_matched_evidence`、metrics=null，不填 0。有限近鄰變異是診斷代理，**不是**真實 `Var(Y|O)`、Bayes irreducible error 或可學性的證明。E1/E2 分開分析；即使 E2 的 oracle 反應正確，也可能在 student 尚無訊號時提前補償。

```bash
uv run --locked python -m phase1b.expert.learnability \
  --reference REF_SIDECAR_1 REF_SIDECAR_2 --query HELD_OUT_SIDECAR \
  --expert E1 --profile ft_object --history 20 --stride 10 \
  --radius 0.1 --torque-tolerance-nm 0.05 --min-neighbors 3 \
  --output NEW_REPORT_JSON
```

上述窗口／radius／tolerance 只示範 CLI，不是經資料驗證的設定。每組最多 500 窗口避免精確近鄰掃描無界增長；先調 stride，之後資料規模需要時再替換索引。此工具沒有 fit/predict 學習模型。

## 下一階段與本次驗證

先選有來源的 gains/limits，再由組員完成 requested → allocated → executed → realized 的 assistance tracking、pitch 責任分配與接觸驗證。之後在相同 human goal/impedance/disturbance 下閉迴路比較 E0/E1/E2，讓人手力隨實際板材狀態重新計算，不重播固定 human force。

正式品質門檻包含錯誤啟動、停止漂移、姿態偏差、接觸／joint limits、飽和、追蹤誤差，以及 `∫‖F_h‖²dt` 人類施力負擔代理（不是能量消耗）。先做約 10–20 個有效 episodes 的 pilot；可控、可辨識、可學分別檢驗，不能互相替代。

```bash
# 本次只執行這個純算術／資料測試；不載入 MuJoCo。
uv run --locked python -m unittest phase1b.tests.test_expert -v
```

9 項測試通過，覆蓋剛體/人手公式、F/T frame、因果差分/LPF、閘門/暫停/反向、煞車、HOLD latch、E2 λ、限幅/安全取消、shadow metadata/validity/hash/拒絕覆寫、三種感測窗口與跨 episode 診斷。測試只用手算可核對的靜態輸入，沒有時間積分模擬；[執行證據](../results/phase1b/expert_math/verification.json)。

模型、hardware/config 與原實驗 arrays 未改。既有物理結果屬於 `37b8c24`；本次改了介面源碼，因此舊 episode 的嚴格 source-hash validator 應在該 revision 使用，不能偽造更新 hashes。物理回歸留待下一次允許模擬時執行。
