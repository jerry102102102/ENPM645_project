# P04 — Vision- and Tactile-Based Continuous Multimodal Intention and Attention Recognition for Safer Physical Human–Robot Interaction

> **Core concept for our project: Contact signals need visual context to help determine whether an interaction is intentional.**

## Sources and status

- Authors: Christopher Yee Wong; Lucas Vergez; Wael Suleiman
- Year / venue: 2024 · IEEE Transactions on Automation Science and Engineering, 21(3), 3205–3215; online 2023
- DOI: [10.1109/TASE.2023.3276856](https://doi.org/10.1109/TASE.2023.3276856)
- Source links: [Paper / abstract](https://arxiv.org/abs/2206.11350) · [IEEE Publication record](https://ieeexplore.ieee.org/document/10144527/)
- Reading priority: Method reference
- Verification date: 2026-10-10
- AI inspection scope: arXiv abstract and IEEE indexed abstract/publication metadata; no section-by-section full-text review
- Human reading status: **queued**
- Team reader: Unassigned
- Citation key: `wong_2024_vision_tactile_intention`

## Paper findings (within the scope above)

Touch location, human pose, gaze, and related features classify intentional versus unintentional touch. The abstract reports an F1 score of 86%. Contact information comes from the robot's skin. Source: [Paper](https://arxiv.org/abs/2206.11350)

## Potential use in our project (design inference)

Borrow the ideas of complementary modalities and low-dimensional features. Ablate VLM context outputs and F/T intent cues separately.

**First question to test:** Which visual cues can be extracted reliably from rendered images? How should a VLM express uncertainty under occlusion?

## What this does not establish

The classes are not flipping versus translation, and 86% is not an expected score for our project. Direct human contact with a robot also differs from jointly grasping an object.

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
