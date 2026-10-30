# P02 — Human-robot planar co-manipulation of extended objects: data-driven models and control from human-human dyads

> **Core concept for our project: The same endpoint force can imply translation or rotation; intent recognition must consider temporal context and object motion.**

## Sources and status

- Authors: Erich Mielke; Eric Townsend; David Wingate; John L. Salmon; Marc D. Killpack
- Year / venue: 2024 · Frontiers in Neurorobotics, 18, 1291694
- DOI: [10.3389/fnbot.2024.1291694](https://doi.org/10.3389/fnbot.2024.1291694)
- Source links: [Paper / abstract](https://www.frontiersin.org/journals/neurorobotics/articles/10.3389/fnbot.2024.1291694/full) · [PMC full text](https://pmc.ncbi.nlm.nih.gov/articles/PMC10894988/)
- Reading priority: Essential 2
- Verification date: 2026-10-10
- AI inspection scope: Selected full-text sections checked: 4.2–4.4、5.3–5.4、6 Conclusion
- Human reading status: **queued**
- Team reader: Unassigned
- Citation key: `mielke_2024_planar_co_manipulation`

## Paper findings (within the scope above)

The paper compares torque-triggered EVIC with motion-predictive NNPC; human-robot control validation is limited to planar 3-DoF motion. NNPC performs better on objective measures, but preference trends for perceived safety and predictability are not statistically significant. The Good Force Amount item shows a significant difference. Source: [Paper](https://www.frontiersin.org/journals/neurorobotics/articles/10.3389/fnbot.2024.1291694/full)

## Potential use in our project (design inference)

Use translation and grasp adjustment as negative examples for flipping recognition. Compare fixed thresholds with short-history models. Later, evaluate how understandable the collaboration is, in addition to success rate.

**First question to test:** Which disturbances are most likely to cause false starts during single-axis reorientation? How should rotation onset be defined?

## What this does not establish

This is not direct evidence for flipping an object to expose another face. The system uses dual arms, a mobile platform, and motion capture. A low-latency camera estimate cannot be assumed to have equivalent quality.

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
