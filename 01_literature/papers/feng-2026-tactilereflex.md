# P07 — TactileReflex: Noise-Statistics-Driven Vision-Tactile Reflex Control for Force-Sensitive Manipulation

> **Core concept for our project: Implement grasp protection as an independent tactile reflex layer, separate from high-level intent modeling.**

## Sources and status

- Authors: Ziyan Feng; Yulong Fu; Zheng Li; Yuxin He; Jieji Ren; Yudong Zhong; Lujia Wang; Jinni Zhou; Qiang Nie
- Year / venue: 2026 · IROS 2026 accepted (confirmed by arXiv v4 and the author page); currently citing the public arXiv version
- DOI: [10.48550/arXiv.2605.23568](https://doi.org/10.48550/arXiv.2605.23568)
- Source links: [Paper / abstract](https://arxiv.org/abs/2605.23568v4) · [Author project / demo](https://shayfeng.github.io/TactileReflex/)
- Reading priority: Tactile extension
- Verification date: 2026-10-10
- AI inspection scope: arXiv v4 abstract/metadata and author project page; no section-by-section full-text review
- Human reading status: **queued**
- Team reader: Unassigned
- Citation key: `feng_2026_tactilereflex`

## Paper findings (within the scope above)

The method extracts shear, contact-intensity, and center-of-pressure proxies from optical tactile images. Noise statistics configure anti-slip, force-release, and force-protection control. The author page and arXiv v4 identify the work as accepted to IROS 2026. Source: [Paper](https://arxiv.org/abs/2605.23568v4)

## Potential use in our project (design inference)

First build grasp monitoring from simulated contact/slip features and report its benefits separately from intent recognition.

**First question to test:** Is the main benefit of tactile more accurate intent classification, or fewer slips/failures?

## What this does not establish

The task is fragile-container manipulation, not recognition of human flipping intent. Image proxies are not calibrated forces in newtons. Simulated contact force is also not the same kind of tactile image.

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
