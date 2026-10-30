# P05 — Robot to Human Object Handover Using Vision and Joint Torque Sensor Modalities

> **Core concept for our project: Let each modality answer a different question, then combine them into an interpretable action decision.**

## Sources and status

- Authors: Mohammadhadi Mohandes; Behnam Moradi; Kamal Gupta; Mehran Mehrandezh
- Year / venue: 2023 · Robot Intelligence Technology and Applications 7 (RiTA 2022), LNNS 642, pp. 109–124; preprint 2022
- DOI: [10.1007/978-3-031-26889-2_11](https://doi.org/10.1007/978-3-031-26889-2_11)
- Source links: [Paper / abstract](https://link.springer.com/chapter/10.1007/978-3-031-26889-2_11) · [Open preprint](https://arxiv.org/abs/2210.15085)
- Reading priority: Method reference
- Verification date: 2026-10-10
- AI inspection scope: Springer metadata/abstract and arXiv abstract; no section-by-section full-text review
- Human reading status: **queued**
- Team reader: Unassigned
- Citation key: `mohandes_2023_handover_fusion`

## Paper findings (within the scope above)

A joint-torque network classifies pull/hold/bump, while an RGB-D network determines whether a human hand encloses the object; the combined decision controls release. The formal chapter was published in 2023 and should not be labeled only as a 2022 unpublished manuscript. Source: [Paper](https://link.springer.com/chapter/10.1007/978-3-031-26889-2_11)

## Potential use in our project (design inference)

Use F/T to predict rotation direction, tactile to assess grasping, and a VLM to describe visible context; combine them with explicit rules.

**First question to test:** When two modalities disagree, how can the system maintain support while waiting for more reliable information?

## What this does not establish

This is a handover task, not cooperative rotation. Its accuracy does not establish flipping-control performance. It is also not a VLM method.

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
