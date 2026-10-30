# Physical verification at 37b8c24 — single dual-Panda implementation

以下為 `37b8c24` 留存的物理驗證，不是 Expert 更新後重跑的結果。Expert v1.0 只執行[純數學／資料測試](../expert_math/verification.json)；既有 arrays/hashes 保留。嚴格 source-hash 驗證這些 episodes 須使用 `37b8c24`，或日後在新目錄重跑目前程式，不能直接改舊 provenance。

- **Interface Ready：是。** 15 項 contract／physics tests 通過。
- **Controller Implemented：只有 reference-only object adapter 與 position-hold dual-arm adapter。**
- **Assistance Tracking Validated：否。** 配力、總需求與 tracking error 保持 unavailable。
- **Expert Labels Validated：否。** Candidate decisions 與正式 supervision 分開，labels 全 invalid。

[完整結果](verification.json) · [測試輸出](tests.txt) · [清理前後回歸](migration_regression.json)

## 物理與資料

三層 A/B/C smoke 各 4 秒：共 1,200 transitions、6,000 physics steps。每個 100 Hz decision/object reference 對應五個 500 Hz arm commands；目前 adapter 不執行 assistance。

四片 pads 保持接觸，無非預期穿透超標或 MuJoCo warnings，無 arm actuator saturation。移除 pad 接觸／停用 arm actuators 的負對照都失去支撐；移除 human force 後機器人接觸力約 9.81 N，可基本支撐 1 kg 板材。各 episode 原始資料在 `episodes/A`、`B`、`C`。

相較清理前 `996a524`，A/B/C 各 103 個既有 arrays 完全一致，共 309 個；差異限於新的決策／命令欄位與測試 reference metadata。模型 XML 與 compiled hardware contract 完全未改。

## 動作展示

[同步 MP4](motion/visuals/ABC_comparison.mp4) · [A 旋轉](motion/visuals/A.mp4) · [B 抬升](motion/visuals/B.mp4) · [C 固定外力](motion/visuals/C.mp4)

![A/B/C](motion/visuals/ABC_comparison.gif)

| 範例 | 實際運動 | 人手 peak force，world XYZ | 外部擾動 |
|---|---|---|---|
| A | +20.00°，抬升 3.05 cm 後停留 | 2.05 N，`[-.84,-.04,-1.87]` N | 無 |
| B | 抬升 7.98 cm 後停留 | 5.41 N，`[+.02,-.02,+5.41]` N | 無 |
| C | −11.94° 後承受固定外力 | 4.20 N，`[-.29,-.03,+4.19]` N | 4.3–8 秒 world −Z 20 N |

每段 8 秒，human target 0.6 s 開始，預設 arm trajectory 1.4–3.8 s；C 外力 4.0–4.3 s 漸增。紅箭頭是人手、橘箭頭是外力，固定 0.12 m 表示方向，大小看 N 數字；tactile 共用 0–40 N/bin 色階。

三段共 12,000 physics steps，保持接觸。相較原展示，168 arrays 全部精確一致；5 秒畫面亦完全相同。A 最小 joint margin 約 0.04 rad，結果不代表任意旋轉或 90° 翻轉可達。

這是 scripted trajectory 物理展示，不是三層控制已能理解意圖。`motion/*/samples.npz` 混有 truth／render state，不能當成 policy input 或 Expert supervision。

## 重現

```bash
uv sync --locked
uv run --locked python -m phase1b.scripts.run_suite --output results/phase1b/reproduction --render
```

輸出需新路徑，渲染需 ffmpeg。歷史程式與資料只存於 Git，現行工作樹維護本套版本。
