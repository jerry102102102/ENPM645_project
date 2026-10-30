# P08 — Multistep Intent Estimation Guided Adaptive Passive Control for Safety-Aware Physical Human-Robot Collaboration

> **Core concept for our project: Future-intent prediction can be combined with assistance strength, but controller stability requires separate treatment.**

## Sources and status

- Authors: Haotian Liu; Zhengtao Zhang; Yuchuang Tong; Zhaojie Ju
- Year / venue: 2026 · IEEE Transactions on Cybernetics, 56(1), 579–591
- DOI: [10.1109/TCYB.2025.3616392](https://doi.org/10.1109/TCYB.2025.3616392)
- Source links: [Paper / abstract](https://pubmed.ncbi.nlm.nih.gov/41071693/) · [Publisher DOI link](https://doi.org/10.1109/TCYB.2025.3616392)
- Reading priority: Advanced optional reading
- Verification date: 2026-10-10
- AI inspection scope: Paper abstract and metadata indexed by PubMed; IEEE full text was not obtained
- Human reading status: **queued**
- Team reader: Unassigned
- Citation key: `liu_2026_multistep_passive_control`

## Paper findings (within the scope above)

The abstract describes Transformer + CVAE motion/force prediction, GoRL impedance adjustment, and energy-tank-based passive MPC. This is an advanced architecture integrating prediction and control. Source: [Paper](https://pubmed.ncbi.nlm.nih.gov/41071693/)

## Potential use in our project (design inference)

Borrow the separation of high-level prediction from low-level control. Use a simple, calibratable controller first and consider multistep prediction only after establishing the baseline.

**First question to test:** How far ahead must this project predict? Does a complex controller provide measurable benefits over simple compliant control?

## What this does not establish

Only the abstract has been inspected; the full experimental setup and baselines have not been verified. This is not evidence that our project provides passivity or safety guarantees.

## Suggested reading focus

Check the original paper for sensor placement, data processing, the definition of intent, control interfaces, and whether the experiment actually matches our original single-arm simulated-flipping concept. Add section/figure/table locators for important findings. Do not present another task's success rate as our own result.

## Team conclusions (complete after actually reading)

- Reader / date:
- Read scope (full text or specific sections):
- Core concept in your own words:
- Supporting section / figure / table:
- What we will adopt / defer, and why:
- Smallest validation experiment:
- Questions from the other teammate:

---
These preliminary notes were prepared with Codex assistance. Verifying a source and selected content does not mean the team has completed a full-paper review.
