# Quick Takeaways: Turning the Papers into Our Design

**First build an interpretable cooperative-reorientation loop, then measure the additional value of tactile sensing and a VLM.** This is a design recommendation based on the literature reviewed so far, not an existing three-modal experimental result.

| Design question | Literature cue | Possible decision (for team discussion) |
| --- | --- | --- |
| When should the robot follow or actively assist? | [P01](papers/gienger-2018-contact-changes.md): mixed initiative and readiness | Maintain support when intent is unclear; rotate only after direction and contact are confirmed |
| Does a push really mean “flip”? | [P02](papers/mielke-2024-planar-co-manipulation.md): translation/rotation ambiguity | Include translation, grasp adjustment, and collision negatives, rather than training only on clean rotations |
| How should onset and stopping be evaluated? | [P03](papers/shaw-2025-motion-haptics.md): low-speed limitations of velocity signals | Evaluate onset, sustained rotation, and stopping separately |
| How should modalities be fused? | [P04](papers/wong-2024-vision-tactile-intention.md), [P05](papers/mohandes-2023-handover-fusion.md): complementary sensing | Have each source produce limited, inspectable features/decisions before fusion |
| Can F/T readings directly represent human-intent forces? | [P06](papers/bai-2025-object-mass-human-wrench.md): object dynamics and human-wrench estimation | Compensate for payload first; evaluate errors caused by mass changes |
| Where can tactile help first? | [P07](papers/feng-2026-tactilereflex.md): local grasp reflex | Start with grasp monitoring; measure classification improvement and slip reduction separately |
| How can we progress toward active prediction? | [P08](papers/liu-2026-multistep-passive-control.md): prediction + adaptive control | Consider multistep prediction after establishing a baseline; avoid reproducing the full architecture in the first version |
| How does tactile sensor placement matter? | [P09](papers/tang-2026-emarm.md): whole-arm tactile + proprioception | Distinguish whole-arm contact, gripper contact, and wrist F/T |

## Do not assume fusion is better

- P01 uses force cues to achieve a specific box rotation. **Build a serious F/T baseline** before judging the value of fusion. [P01](papers/gienger-2018-contact-changes.md)
- P03 supports the value of motion signals. A VLM might merely describe rotation already in progress, while inference latency misses onset. This alternative explanation needs experimental testing. [P03](papers/shaw-2025-motion-haptics.md)
- P07's tactile results concern local manipulation protection. If tactile reduces slips without improving intention F1, report that as a contact-stability benefit. [P07](papers/feng-2026-tactilereflex.md)
- P02 reminds us that objective efficiency and human perception may differ. Its safety/predictability preference trends were not significant; do not claim it “proved people prefer the simpler controller.” [P02](papers/mielke-2024-planar-co-manipulation.md)

## Working hypothesis for a VLM

The selected list does not directly validate **VLM + tactile + F/T → nonverbal single-arm flipping**, the original project concept. Initially, a VLM could provide visible hand/object context and uncertainty, fused with fast mechanical signals. Benefits for occlusion, contact readiness, or ambiguity remain hypotheses to test.

Compare against vision using only pose/geometric features and measure VLM latency. This separates the value of a semantic model from that of visual information generally. Future searches should prioritize primary research on VLMs in physical co-manipulation and reasoning about contact situations.

## Minimal validation path

1. Establish B0 in a fixed-grasp, single-axis setting; verify sensing directions, payload compensation, and rotation geometry.
2. Add temporal recognition and non-rotation negatives; evaluate false starts.
3. Add tactile and VLM separately, then test full fusion while holding the low-level controller constant.
4. Test noise, latency, occlusion, mass changes, and missing modalities.
5. Use measured benefits to choose the next step: 180° reorientation, regrasping, real human interaction, or more complex prediction.

See the [Current Interface](../docs/REFERENCE.md#interface-contract) for the concrete architecture and metrics. Simulation results support only the tested scenario and human proxy; user experience requires later evidence from real people.

## Review limitations and AI disclosure

This preliminary synthesis comes from a targeted set of nine papers, some inspected only at abstract level. Each paper discloses its reading scope. This is not a PRISMA review, a complete reproducibility audit, or confirmation of novelty. Codex assisted with organization; the team makes adoption decisions.
