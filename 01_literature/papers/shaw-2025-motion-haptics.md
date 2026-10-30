# P03 — Understanding human co-manipulation via motion and haptic information to enable future physical human-robotic collaborations

> **Core concept for our project: Velocity is informative once motion is underway; evaluate onset, stopping, and low-speed positioning separately.**

## Sources and status

- Authors: Kody Shaw; John L. Salmon; Marc D. Killpack
- Year / venue: 2025 · Frontiers in Neurorobotics, 19, 1480399
- DOI: [10.3389/fnbot.2025.1480399](https://doi.org/10.3389/fnbot.2025.1480399)
- Source links: [Paper / abstract](https://www.frontiersin.org/journals/neurorobotics/articles/10.3389/fnbot.2025.1480399/full) · [PMC full text](https://pmc.ncbi.nlm.nih.gov/articles/PMC12222233/)
- Reading priority: Essential 3
- Verification date: 2026-10-10
- AI inspection scope: Selected full-text sections checked: 4 Results and analysis、5 Conclusion
- Human reading status: **queued**
- Team reader: Unassigned
- Citation key: `shaw_2025_motion_haptics`

## Paper findings (within the scope above)

The study analyzes motion and haptic signals during two-person and three-person co-manipulation. Results support velocity as a major cue for future motion and identify limitations at low speeds. Transitions from rest to activity are analyzed separately using noise statistics. Source: [Paper](https://www.frontiersin.org/journals/neurorobotics/articles/10.3389/fnbot.2025.1480399/full)

## Potential use in our project (design inference)

Evaluate onset, sustained rotation, and stopping separately. Calibrate on stationary data instead of copying fixed thresholds from other hardware.

**First question to test:** Does adding F/T or tactile mainly improve onset/stopping, rather than intervals when rotation is already obvious?

## What this does not establish

This is a human-human data study, not a completed robot controller. It does not establish benefits from VLM or tactile fusion.

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
