# P06 — Sensorless Human–Robot Interaction: Real-Time Estimation of Co-Grasped Object Mass and Human Wrench for Compliant Interaction

> **Core concept for our project: Forces related to human intent must be distinguished from object weight and dynamic effects.**

## Sources and status

- Authors: Minghao Bai; Xizhe Zang; Lei Wu; Changle Li; Yanhe Zhu; Jie Zhao
- Year / venue: 2025 · Advanced Intelligent Systems, 7(6), 2400616; online 2024-09-30
- DOI: [10.1002/aisy.202400616](https://doi.org/10.1002/aisy.202400616)
- Source links: [Paper / abstract](https://advanced.onlinelibrary.wiley.com/doi/10.1002/aisy.202400616)
- Reading priority: Method reference
- Verification date: 2026-10-10
- AI inspection scope: Selected full-text sections checked: Abstract、2 Experimental Section、2.1–2.4
- Human reading status: **queued**
- Team reader: Unassigned
- Citation key: `bai_2025_object_mass_human_wrench`

## Paper findings (within the scope above)

An EKF estimates object parameters, and a disturbance observer estimates the human wrench. Important correction: no human-side force sensor is required, but the Experimental Section explicitly describes a 6D F/T sensor at the robot end effector. Source: [Paper](https://advanced.onlinelibrary.wiley.com/doi/10.1002/aisy.202400616)

## Potential use in our project (design inference)

Use simulation ground truth to evaluate wrench-estimation error, while recognition inputs use only sensing/estimates. Calibrate the payload first, then test misclassification when mass changes.

**First question to test:** Is simple gravity compensation sufficient? Which payload/speed ranges require a full observer?

## What this does not establish

Sensorless does not mean that the entire system lacks F/T sensing. The method requires model and grasp-geometry assumptions; end-effector readings cannot simply be equated with human force.

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
