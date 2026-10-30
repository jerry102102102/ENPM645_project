# Literature — Optional Background

Start with P01–P03. This is supporting material, not a prerequisite for implementing the [shared controller](CONTROLLER.md).

All nine papers remain **queued / reader unassigned**; no teammate has reported completing a reading. Only an actual reader should add their name/date, sections read, takeaway, supporting locator, adoption decision, and next experiment. AI-assisted notes are not completed team readings.

**Evidence scope:** retained from the 2026-10-10 targeted check of nine user-supplied seed titles (English, 2018–2026); no new source verification was performed during this consolidation. Searches used full titles plus author/year across publisher/author sources. This is not a systematic review or novelty claim. Some checks cover only abstracts; no complete search counts, programmatic Semantic Scholar/OpenAlex check, comprehensive retraction/COI review, or reproduction was performed. Workshops/course schedules were not research evidence. Full authors and publication metadata remain in [references.bib](references.bib).

**Design takeaways:** separate intent from control; establish a serious force/motion baseline; evaluate false starts and stopping separately; compensate payload effects; test tactile benefits rather than assume them. The list does not establish a VLM + tactile + F/T advantage. VLM latency, occlusion, mass changes, modality ablations and real-human generalization are later questions, not the current controller assignment.

## P01 — Human-Robot Cooperative Object Manipulation with Contact Changes

**Takeaway:** Split reorientation into direction recognition, human-readiness confirmation, cooperative execution, and returning initiative; synchronize stages using force signals.

- Publication: 2018 · IROS 2018, pp. 1354–1360
- DOI: [10.1109/IROS.2018.8594140](https://doi.org/10.1109/IROS.2018.8594140) · Citation key: `gienger_2018_contact_changes`
- Sources: [Paper / abstract](https://www.honda-ri.de/pubs/pdf/3729.pdf) · [Publication record](https://research.tudelft.nl/en/publications/human-robot-cooperative-object-manipulation-with-contact-changes/) · [Official demo](https://www.honda-ri.de/human-robot-cooperative-object-manipulation-with-contact-changes/)
- Inspection scope: Selected full-text sections checked: III Interaction Layer, VI Robot Setup, VII Evaluation

A dual-arm mobile robot and a human jointly rotate a box by 90°. The experiment estimates hand forces from joint torques and uses manually tuned thresholds. The hardware also includes six-axis F/T sensors and tactile pads; the signals used in this experiment are not the same as the system's full sensing capabilities. Source: [Paper](https://www.honda-ri.de/pubs/pdf/3729.pdf)

**Potential use (our inference):** Start with a single-axis state machine without regrasping, retaining readiness and UNKNOWN. Separate high-level intent estimation, motion planning, and compliant control.

**Limit:** This is a dual-arm system with contact changes, not direct validation of single-arm manipulation, arbitrary shapes, or a three-modal model. Users still need instructions on the initial cueing action.

## P02 — Human-robot planar co-manipulation of extended objects: data-driven models and control from human-human dyads

**Takeaway:** The same endpoint force can imply translation or rotation; intent recognition must consider temporal context and object motion.

- Publication: 2024 · Frontiers in Neurorobotics, 18, 1291694
- DOI: [10.3389/fnbot.2024.1291694](https://doi.org/10.3389/fnbot.2024.1291694) · Citation key: `mielke_2024_planar_co_manipulation`
- Sources: [Paper / abstract](https://www.frontiersin.org/journals/neurorobotics/articles/10.3389/fnbot.2024.1291694/full) · [PMC full text](https://pmc.ncbi.nlm.nih.gov/articles/PMC10894988/)
- Inspection scope: Selected full-text sections checked: 4.2–4.4, 5.3–5.4, 6 Conclusion

The paper compares torque-triggered EVIC with motion-predictive NNPC; human-robot control validation is limited to planar 3-DoF motion. NNPC performs better on objective measures, but preference trends for perceived safety and predictability are not statistically significant. The Good Force Amount item shows a significant difference. Source: [Paper](https://www.frontiersin.org/journals/neurorobotics/articles/10.3389/fnbot.2024.1291694/full)

**Potential use (our inference):** Use translation and grasp adjustment as negative examples for flipping recognition. Compare fixed thresholds with short-history models. Later, evaluate how understandable the collaboration is, in addition to success rate.

**Limit:** This is not direct evidence for flipping an object to expose another face. The system uses dual arms, a mobile platform, and motion capture. A low-latency camera estimate cannot be assumed to have equivalent quality.

## P03 — Understanding human co-manipulation via motion and haptic information to enable future physical human-robotic collaborations

**Takeaway:** Velocity is informative once motion is underway; evaluate onset, stopping, and low-speed positioning separately.

- Publication: 2025 · Frontiers in Neurorobotics, 19, 1480399
- DOI: [10.3389/fnbot.2025.1480399](https://doi.org/10.3389/fnbot.2025.1480399) · Citation key: `shaw_2025_motion_haptics`
- Sources: [Paper / abstract](https://www.frontiersin.org/journals/neurorobotics/articles/10.3389/fnbot.2025.1480399/full) · [PMC full text](https://pmc.ncbi.nlm.nih.gov/articles/PMC12222233/)
- Inspection scope: Selected full-text sections checked: 4 Results and analysis, 5 Conclusion

The study analyzes motion and haptic signals during two-person and three-person co-manipulation. Results support velocity as a major cue for future motion and identify limitations at low speeds. Transitions from rest to activity are analyzed separately using noise statistics. Source: [Paper](https://www.frontiersin.org/journals/neurorobotics/articles/10.3389/fnbot.2025.1480399/full)

**Potential use (our inference):** Evaluate onset, sustained rotation, and stopping separately. Calibrate on stationary data instead of copying fixed thresholds from other hardware.

**Limit:** This is a human-human data study, not a completed robot controller. It does not establish benefits from VLM or tactile fusion.

## P04 — Vision- and Tactile-Based Continuous Multimodal Intention and Attention Recognition for Safer Physical Human–Robot Interaction

**Takeaway:** Contact signals need visual context to help determine whether an interaction is intentional.

- Publication: 2024 · IEEE Transactions on Automation Science and Engineering, 21(3), 3205–3215; online 2023
- DOI: [10.1109/TASE.2023.3276856](https://doi.org/10.1109/TASE.2023.3276856) · Citation key: `wong_2024_vision_tactile_intention`
- Sources: [Paper / abstract](https://arxiv.org/abs/2206.11350) · [IEEE Publication record](https://ieeexplore.ieee.org/document/10144527/)
- Inspection scope: arXiv abstract and IEEE indexed abstract/publication metadata; no section-by-section full-text review

Touch location, human pose, gaze, and related features classify intentional versus unintentional touch. The abstract reports an F1 score of 86%. Contact information comes from the robot's skin. Source: [Paper](https://arxiv.org/abs/2206.11350)

**Potential use (our inference):** Borrow the ideas of complementary modalities and low-dimensional features. Ablate VLM context outputs and F/T intent cues separately.

**Limit:** The classes are not flipping versus translation, and 86% is not an expected score for our project. Direct human contact with a robot also differs from jointly grasping an object.

## P05 — Robot to Human Object Handover Using Vision and Joint Torque Sensor Modalities

**Takeaway:** Let each modality answer a different question, then combine them into an interpretable action decision.

- Publication: 2023 · Robot Intelligence Technology and Applications 7 (RiTA 2022), LNNS 642, pp. 109–124; preprint 2022
- DOI: [10.1007/978-3-031-26889-2_11](https://doi.org/10.1007/978-3-031-26889-2_11) · Citation key: `mohandes_2023_handover_fusion`
- Sources: [Paper / abstract](https://link.springer.com/chapter/10.1007/978-3-031-26889-2_11) · [Open preprint](https://arxiv.org/abs/2210.15085)
- Inspection scope: Springer metadata/abstract and arXiv abstract; no section-by-section full-text review

A joint-torque network classifies pull/hold/bump, while an RGB-D network determines whether a human hand encloses the object; the combined decision controls release. The formal chapter was published in 2023 and should not be labeled only as a 2022 unpublished manuscript. Source: [Paper](https://link.springer.com/chapter/10.1007/978-3-031-26889-2_11)

**Potential use (our inference):** Use F/T to predict rotation direction, tactile to assess grasping, and a VLM to describe visible context; combine them with explicit rules.

**Limit:** This is a handover task, not cooperative rotation. Its accuracy does not establish flipping-control performance. It is also not a VLM method.

## P06 — Sensorless Human–Robot Interaction: Real-Time Estimation of Co-Grasped Object Mass and Human Wrench for Compliant Interaction

**Takeaway:** Forces related to human intent must be distinguished from object weight and dynamic effects.

- Publication: 2025 · Advanced Intelligent Systems, 7(6), 2400616; online 2024-09-30
- DOI: [10.1002/aisy.202400616](https://doi.org/10.1002/aisy.202400616) · Citation key: `bai_2025_object_mass_human_wrench`
- Sources: [Paper / abstract](https://advanced.onlinelibrary.wiley.com/doi/10.1002/aisy.202400616)
- Inspection scope: Selected full-text sections checked: Abstract, 2 Experimental Section, 2.1–2.4

An EKF estimates object parameters, and a disturbance observer estimates the human wrench. Important correction: no human-side force sensor is required, but the Experimental Section explicitly describes a 6D F/T sensor at the robot end effector. Source: [Paper](https://advanced.onlinelibrary.wiley.com/doi/10.1002/aisy.202400616)

**Potential use (our inference):** Use simulation ground truth to evaluate wrench-estimation error, while recognition inputs use only sensing/estimates. Calibrate the payload first, then test misclassification when mass changes.

**Limit:** Sensorless does not mean that the entire system lacks F/T sensing. The method requires model and grasp-geometry assumptions; end-effector readings cannot simply be equated with human force.

## P07 — TactileReflex: Noise-Statistics-Driven Vision-Tactile Reflex Control for Force-Sensitive Manipulation

**Takeaway:** Implement grasp protection as an independent tactile reflex layer, separate from high-level intent modeling.

- Publication: 2026 · IROS 2026 accepted (confirmed by arXiv v4 and the author page); currently citing the public arXiv version
- DOI: [10.48550/arXiv.2605.23568](https://doi.org/10.48550/arXiv.2605.23568) · Citation key: `feng_2026_tactilereflex`
- Sources: [Paper / abstract](https://arxiv.org/abs/2605.23568v4) · [Author project / demo](https://shayfeng.github.io/TactileReflex/)
- Inspection scope: arXiv v4 abstract/metadata and author project page; no section-by-section full-text review

The method extracts shear, contact-intensity, and center-of-pressure proxies from optical tactile images. Noise statistics configure anti-slip, force-release, and force-protection control. The author page and arXiv v4 identify the work as accepted to IROS 2026. Source: [Paper](https://arxiv.org/abs/2605.23568v4)

**Potential use (our inference):** First build grasp monitoring from simulated contact/slip features and report its benefits separately from intent recognition.

**Limit:** The task is fragile-container manipulation, not recognition of human flipping intent. Image proxies are not calibrated forces in newtons. Simulated contact force is also not the same kind of tactile image.

## P08 — Multistep Intent Estimation Guided Adaptive Passive Control for Safety-Aware Physical Human-Robot Collaboration

**Takeaway:** Future-intent prediction can be combined with assistance strength, but controller stability requires separate treatment.

- Publication: 2026 · IEEE Transactions on Cybernetics, 56(1), 579–591
- DOI: [10.1109/TCYB.2025.3616392](https://doi.org/10.1109/TCYB.2025.3616392) · Citation key: `liu_2026_multistep_passive_control`
- Sources: [Paper / abstract](https://pubmed.ncbi.nlm.nih.gov/41071693/) · [Publisher DOI link](https://doi.org/10.1109/TCYB.2025.3616392)
- Inspection scope: Paper abstract and metadata indexed by PubMed; IEEE full text was not obtained

The abstract describes Transformer + CVAE motion/force prediction, GoRL impedance adjustment, and energy-tank-based passive MPC. This is an advanced architecture integrating prediction and control. Source: [Paper](https://pubmed.ncbi.nlm.nih.gov/41071693/)

**Potential use (our inference):** Borrow the separation of high-level prediction from low-level control. Use a simple, calibratable controller first and consider multistep prediction only after establishing the baseline.

**Limit:** Only the abstract has been inspected; the full experimental setup and baselines have not been verified. This is not evidence that our project provides passivity or safety guarantees.

## P09 — Embodied sensorimotor integration for whole-arm tactile sensing and adaptive robotic manipulation

**Takeaway:** Combining distributed tactile sensing with proprioception can connect contact information to motion adjustment.

- Publication: 2026 · Nature Sensors, 1, 681–690; published 2026-07-20
- DOI: [10.1038/s44460-026-00097-1](https://doi.org/10.1038/s44460-026-00097-1) · Citation key: `tang_2026_emarm`
- Sources: [Paper / abstract](https://www.nature.com/articles/s44460-026-00097-1) · [Author laboratory publications](https://mirs.hkust.edu.hk/publications/)
- Inspection scope: Nature publication abstract/metadata and author laboratory publication list; no section-by-section full-text review

EmArm integrates large-area flexible tactile skin, proprioception, and closed-loop perception/action, demonstrating touch-intent recognition and tactile-driven replanning. Source: [Paper](https://www.nature.com/articles/s44460-026-00097-1)

**Potential use (our inference):** Compare the information available from whole-arm tactile sensing, gripper tactile sensing, and wrist F/T rather than treating them as interchangeable.

**Limit:** The system uses specialized whole-arm skin hardware. It is not direct validation of a single tactile gripper or cooperative workpiece flipping.

## Source qualifications

P04 is a 2022 preprint / 2023 online / 2024 issue publication; cite the issue. P05 has a formal 2023 chapter beyond its 2022 preprint. P06 appeared online in 2024 and in a 2025 issue. P07 cites inspected arXiv v4 (with its author order); proceedings DOI/pages were not verified. P08's 2026 issue and 2025 DOI are not contradictory; IEEE full text was not obtained. P09 was aligned through the Nature index and laboratory page because direct Nature access failed; full text/supplement were not inspected.

Existence and selected-content checks do not mean every DOI resolved, every paper was fully read, or results reproduced. Confirm source locators before the final report. Earlier search queries, templates and detailed reading cards remain in Git at `039e835`; maintain this single reference going forward.
