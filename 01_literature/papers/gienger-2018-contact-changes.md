# P01 — Human-Robot Cooperative Object Manipulation with Contact Changes

> **Core concept for our project: Split reorientation into direction recognition, human-readiness confirmation, cooperative execution, and returning initiative; synchronize stages using force signals.**

## Sources and status

- Authors: Michael Gienger; Dirk Ruiken; Tamas Bates; Mohamed Regaieg; Michael Meißner; Jens Kober; Philipp Seiwald; Arne-Christoph Hildebrandt
- Year / venue: 2018 · IROS 2018, pp. 1354–1360
- DOI: [10.1109/IROS.2018.8594140](https://doi.org/10.1109/IROS.2018.8594140)
- Source links: [Paper / abstract](https://www.honda-ri.de/pubs/pdf/3729.pdf) · [Publication record](https://research.tudelft.nl/en/publications/human-robot-cooperative-object-manipulation-with-contact-changes/) · [Official demo](https://www.honda-ri.de/human-robot-cooperative-object-manipulation-with-contact-changes/)
- Reading priority: Essential 1
- Verification date: 2026-10-10
- AI inspection scope: Selected full-text sections checked: III Interaction Layer、VI Robot Setup、VII Evaluation
- Human reading status: **queued**
- Team reader: Unassigned
- Citation key: `gienger_2018_contact_changes`

## Paper findings (within the scope above)

A dual-arm mobile robot and a human jointly rotate a box by 90°. The experiment estimates hand forces from joint torques and uses manually tuned thresholds. The hardware also includes six-axis F/T sensors and tactile pads; the signals used in this experiment are not the same as the system's full sensing capabilities. Source: [Paper](https://www.honda-ri.de/pubs/pdf/3729.pdf)

## Potential use in our project (design inference)

Start with a single-axis state machine without regrasping, retaining readiness and UNKNOWN. Separate high-level intent estimation, motion planning, and compliant control.

**First question to test:** What rotation range can a single arm achieve with a fixed grasp? How should support and pause conditions be defined during human regrasping?

## What this does not establish

This is a dual-arm system with contact changes, not direct validation of single-arm manipulation, arbitrary shapes, or a three-modal model. Users still need instructions on the initial cueing action.

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
