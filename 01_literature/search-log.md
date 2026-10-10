# 檢索與來源核對紀錄

查核日：**2026-10-10（America/New_York）**

## 目的與範圍

起點是使用者提供的 ChatGPT 文字，其中列出 9 篇論文。這次逐篇尋找原文／出版 metadata，建立適合兩人專題的初步閱讀入口。

纳入條件：原始來源可確認存在，且與共同物體操作、意圖辨識、感測融合、載荷估計或觸覺控制相關。直接對應翻轉者優先；相關任務只標為方法參考。英文、2018–2026 的這 9 個種子題名為本次範圍，不代表完整年代檢索。

本次 9 篇都找到來源；按題名／DOI 對齊版本，未發現需去重的獨立種子。Workshop 不是論文，不列進閱讀清單。沒有保存完整搜尋結果數，因此不報告 PRISMA flow 或完整召回率。

## 檢索方式

以完整題名搜尋並搭配第一作者／年份。主要來源為作者頁、Honda Research Institute、TU Delft repository、Frontiers、IEEE 索引、Springer、Wiley、Nature、arXiv；P08 使用 PubMed 收錄的原論文摘要。

實際查詢家族：

- `"Human-Robot Cooperative Object Manipulation with Contact Changes" Gienger`
- `"Human-Robot Planar Co-Manipulation of Extended Objects" Mielke`
- `"Understanding Human Co-Manipulation via Motion and Haptic Information"`
- `"Vision- and tactile-based continuous multimodal" "10.1109"`
- `"Robot to Human Object Handover Using Vision" site:link.springer.com`
- `"Sensorless Human–Robot Interaction" "Mass" "Wrench" Bai`
- `"TactileReflex" IROS 2026 project`
- `"Multistep Intent Estimation Guided Adaptive Passive Control"`
- `"Embodied Sensorimotor Integration for Whole-Arm Tactile Sensing"`

## 核對範圍

| ID | Metadata／存在性依據 | 查閱內容 | 尚未完成 |
| --- | --- | --- | --- |
| P01 | TU Delft metadata + Honda accepted manuscript | III、VI、VII 等關鍵段落 | 團隊全文閱讀／程式與控制重現 |
| P02 | Frontiers／PMC | 方法、實驗、5.3–5.4、結論 | 團隊全文閱讀／原始資料重分析 |
| P03 | Frontiers／PMC | Results、Conclusion | 團隊全文閱讀／原始資料重分析 |
| P04 | IEEE metadata + arXiv | 摘要與版本核對 | 全文方法／實驗查核 |
| P05 | Springer chapter metadata + arXiv | 摘要與版本核對 | 全文方法／實驗查核 |
| P06 | Wiley | Abstract、2 Experimental Section、2.1–2.4 | 全文假設／控制穩定性審查 |
| P07 | arXiv v4 + 作者專案 | 摘要／版本／專案方法說明 | 全文／正式 proceedings DOI |
| P08 | PubMed PMID 41071693 | 原論文摘要與 metadata | IEEE 全文；DOI 入口在本次 web tool 無法開啟 |
| P09 | Nature 搜尋索引 + 作者實驗室頁 | 摘要與 metadata | Nature 直接 open 出錯；全文／supplement 未查阅 |

各篇 [閱讀卡](README.md) 提供原文來源。這裡的「存在性確認」不表示所有 DOI 已成功解析、所有結果已重現或每篇全文都已閱讀。未執行 Semantic Scholar／OpenAlex programmatic verification，也未進行全面撤稿、COI、venue indexing 審查。

## 對原始貼文的更正與保留

1. **P01：**90° 箱子轉動存在於 VII Evaluation；是雙臂系統。該實驗的互動判斷使用由關節力矩估出的力，硬體另有 F/T 與 tactile。[Honda manuscript](https://www.honda-ri.de/pubs/pdf/3729.pdf)
2. **P02：**安全／困惑／可預測性偏好不能宣稱統計顯著；論文另報 Good Force Amount 差異。[5.3–5.4](https://www.frontiersin.org/journals/neurorobotics/articles/10.3389/fnbot.2024.1291694/full)
3. **P04：**preprint 是 2022，online publication 是 2023，正式 issue 是 2024；引用以 2024 issue 為主。[IEEE](https://ieeexplore.ieee.org/document/10144527/)
4. **P05：**除 2022 preprint 外，已有 2023 Springer 正式章節。[Springer](https://link.springer.com/chapter/10.1007/978-3-031-26889-2_11)
5. **P06：**online 2024／issue 2025；sensorless 指人側不需要感測器，機器人末端仍使用 6D F/T。[Wiley §2](https://advanced.onlinelibrary.wiley.com/doi/10.1002/aisy.202400616)
6. **P07：**arXiv v4 與作者頁標明 IROS 2026 accepted；BibTeX 先引用實際查閱的 arXiv v4，不猜 proceedings DOI／頁碼。v4 作者順序與早期版本不同。[arXiv v4](https://arxiv.org/abs/2605.23568v4)、[作者頁](https://shayfeng.github.io/TactileReflex/)
7. **P08：**2026 issue 與 DOI 中的 2025 並不矛盾；目前僅摘要依據。[PubMed](https://pubmed.ncbi.nlm.nih.gov/41071693/)
8. **P09：**2026 Nature Sensors 出版可由 Nature 索引與實驗室出版頁對齊；不將硬體展示當成單臂共握翻轉成果。[Nature](https://www.nature.com/articles/s44460-026-00097-1)、[實驗室](https://mirs.hkust.edu.hk/publications/)
9. 原貼文中的 IROS workshop 日期、UMD 課程綱要並非本次論文核對的必要內容，因此未作為支持研究結論的證據。

## 後續補文獻的方向

以下是未執行的下一輪搜尋建議：

- `("co-manipulation" OR "physical human-robot collaboration") AND ("vision-language model" OR VLM) AND intent`
- `"collaborative object reorientation" single arm admittance`
- `"vision tactile force" intention ablation human robot`
- `"co-manipulation" ("negative results" OR latency OR ambiguity)`

目前清單依種子材料選取，不能由「找不到直接三模態例子」推論新穎性已成立。VLM、單臂翻面幾何、真實人類泛化與開放程式的可重現性仍是證據缺口。這是定向探索式整理，不是 systematic review。

## AI 使用說明

Codex 協助檢索、對齊 metadata、整理短摘要與設計推論；沒有代替團隊宣告任何論文已讀、沒有執行實驗，也沒有聲稱獨立跨模型審查。正式報告引用前，由團隊閱讀相關原文並確認 locator。
