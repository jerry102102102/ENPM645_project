# 01 — Reading List and Quick Takeaways

This folder answers: **What is the paper's core concept, and how can it help us build nonverbal cooperative reorientation?**

Read P01, P02, and P03 first, then choose other papers according to implementation bottlenecks. Each has a short reading card and original-source links. The team has not reported completed readings, so all human reading statuses remain queued. AI inspection scope is recorded separately from what team members have read.

## Reading priorities

| Priority / ID | Paper | Most useful concept for the project | Source | Reading card |
| --- | --- | --- | --- | --- |
| Essential 1 / P01 | Human-Robot Cooperative Object Manipulation with Contact Changes (2018) | Split reorientation into direction recognition, readiness confirmation, cooperative execution, and returning initiative; synchronize stages with force signals. | [Open](https://www.honda-ri.de/pubs/pdf/3729.pdf) | [Notes](papers/gienger-2018-contact-changes.md) |
| Essential 2 / P02 | Human-robot planar co-manipulation of extended objects: data-driven models and control from human-human dyads (2024) | The same endpoint force can imply translation or rotation; consider temporal context and object motion. | [Open](https://www.frontiersin.org/journals/neurorobotics/articles/10.3389/fnbot.2024.1291694/full) | [Notes](papers/mielke-2024-planar-co-manipulation.md) |
| Essential 3 / P03 | Understanding human co-manipulation via motion and haptic information to enable future physical human-robotic collaborations (2025) | Velocity is informative during motion; evaluate onset, stopping, and low-speed positioning separately. | [Open](https://www.frontiersin.org/journals/neurorobotics/articles/10.3389/fnbot.2025.1480399/full) | [Notes](papers/shaw-2025-motion-haptics.md) |
| Method reference / P04 | Vision- and Tactile-Based Continuous Multimodal Intention and Attention Recognition for Safer Physical Human–Robot Interaction (2024) | Contact signals need visual context to help determine whether an interaction is intentional. | [Open](https://arxiv.org/abs/2206.11350) | [Notes](papers/wong-2024-vision-tactile-intention.md) |
| Method reference / P05 | Robot to Human Object Handover Using Vision and Joint Torque Sensor Modalities (2023) | Let modalities answer different questions, then combine them into interpretable action decisions. | [Open](https://link.springer.com/chapter/10.1007/978-3-031-26889-2_11) | [Notes](papers/mohandes-2023-handover-fusion.md) |
| Method reference / P06 | Sensorless Human–Robot Interaction: Real-Time Estimation of Co-Grasped Object Mass and Human Wrench for Compliant Interaction (2025) | Distinguish forces related to human intent from object weight and dynamic effects. | [Open](https://advanced.onlinelibrary.wiley.com/doi/10.1002/aisy.202400616) | [Notes](papers/bai-2025-object-mass-human-wrench.md) |
| Tactile extension / P07 | TactileReflex: Noise-Statistics-Driven Vision-Tactile Reflex Control for Force-Sensitive Manipulation (2026) | Implement grasp protection as a separate tactile reflex layer alongside high-level intent modeling. | [Open](https://arxiv.org/abs/2605.23568v4) | [Notes](papers/feng-2026-tactilereflex.md) |
| Advanced optional / P08 | Multistep Intent Estimation Guided Adaptive Passive Control for Safety-Aware Physical Human-Robot Collaboration (2026) | Combine future-intent prediction with assistance strength, while addressing control stability separately. | [Open](https://pubmed.ncbi.nlm.nih.gov/41071693/) | [Notes](papers/liu-2026-multistep-passive-control.md) |
| Advanced optional / P09 | Embodied sensorimotor integration for whole-arm tactile sensing and adaptive robotic manipulation (2026) | Combine distributed tactile sensing with proprioception to connect contact information to motion adjustment. | [Open](https://www.nature.com/articles/s44460-026-00097-1) | [Notes](papers/tang-2026-emarm.md) |

## Team reading tracker

| ID | Reader | Human reading status | Team read scope / date |
| --- | --- | --- | --- |
| P01 | Unassigned | queued | — |
| P02 | Unassigned | queued | — |
| P03 | Unassigned | queued | — |
| P04 | Unassigned | queued | — |
| P05 | Unassigned | queued | — |
| P06 | Unassigned | queued | — |
| P07 | Unassigned | queued | — |
| P08 | Unassigned | queued | — |
| P09 | Unassigned | queued | — |

For the first round, one teammate could take P01 and the other P02, then discuss P03 together. This is a suggestion; add names only after someone claims a paper.

## What to record after reading

End each paper note with a decision relevant to the project:

> “This paper tells us ___. Therefore, we will use ___. First, we will test ___ in a small experiment; it does not establish ___.”

Do not merely repeat the abstract. See the [Template](templates/paper-note.md) and [Contributing Guide](../CONTRIBUTING.md).

## Combined design implications

The [Cross-Paper Takeaways](synthesis.md) map the papers to interaction state machines, intent ambiguity, payload compensation, fusion, and tactile protection. VLM use is an extension we want to evaluate; these nine papers do not directly establish its benefit.

## Citations and verification

- [references.bib](references.bib): distinguishes formal publication years from public manuscript versions.
- [search-log.md](search-log.md): inspection scope for all nine papers, corrections to the initial pasted material, and search limitations.
- Access is provided through legitimate original-source links; this repository does not store copies of paper PDFs.
- Notes are AI-assisted starting material. Some were checked only at abstract level; each card states its scope. Teammates should add full-text readings and conclusions.

Last updated: 2026-10-10.
