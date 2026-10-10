# 團隊協作

## 每讀一篇論文

1. 在 [文獻入口](01_literature/README.md) 認領一篇，填入 reader。
2. 開原文，查看筆記列出的「核對範圍」，不要把初步摘要當作全文已讀。
3. 在原筆記下方填寫「團隊閱讀結論」，保留原有來源與更正紀錄。
4. 寫清楚：**一個核心概念 → 我們怎麼用 → 一個小實驗 → 適用限制**。
5. 更新 human reading status，另一位組員確認重要結論與引用段落。

閱讀狀態：`queued` → `reading` → `discussed`。reader、read scope、日期由實際閱讀者填寫；AI 不代填已讀狀態。

## 增加新論文

複製 [模板](01_literature/templates/paper-note.md)，命名為 `firstauthor-year-topic.md`。
確認原始題名、作者、年份、DOI／arXiv ID 與出版版本；在 README 加一列，在 references.bib 加對應 citation key。關鍵結果要有 section／figure／table locator。

repo 收錄閱讀入口與我們的筆記；論文全文以作者／出版社／學校連結開啟。保留預印本與正式版的差異，不用二手摘要取代原文。

## Git 更新

兩人可用各自的分支提交更新，再透過 PR 互看。commit message 寫明內容，例如 `docs: add P01 team reading conclusions`。更改專題範圍時，同步更新 root README 與 docs/project-concept.md。

目前只建立必要文件；等模擬器與實作方式確定後再建立程式、資料與結果目錄。
