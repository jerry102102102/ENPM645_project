# Physical verification at 37b8c24 — single dual-Panda implementation

The following physical validation is retained from `37b8c24`; it was not rerun after the Expert update. Expert v1.0 ran only [Mathematical / Data Tests](../expert_math/verification.json). Existing arrays/hashes are preserved. Strict source-hash validation of these episodes requires `37b8c24`, or a future rerun of the current code into a new directory; do not modify old provenance.

- **Interface Ready: yes.** All 15 contract/physics tests passed.
- **Controller Implemented: only the reference-only object adapter and position-hold dual-arm adapter.**
- **Assistance Tracking Validated: no.** Allocation, total demand, and tracking error remain unavailable.
- **Expert Labels Validated: no.** Candidate decisions are separate from formal supervision; all labels are invalid.

[Full Results](verification.json) · [Test Output](tests.txt) · [Cleanup Regression](migration_regression.json)

## Physics and data

Three-layer A/B/C smoke runs lasted 4 seconds each: 1,200 transitions and 6,000 physics steps in total. Each 100 Hz decision/object reference corresponds to five 500 Hz arm commands. The current adapters do not execute assistance.

All four pads maintained contact, without unexpected penetration above the threshold, MuJoCo warnings, or arm actuator saturation. Negative controls removing pad contact or disabling arm actuators both lost support. After removing human force, robot contact force was approximately 9.81 N, providing basic support for the 1 kg panel. Raw episodes are in `episodes/A`, `B`, and `C`.

Compared with the pre-cleanup revision `996a524`, all 103 existing arrays per A/B/C episode were exactly identical, 309 in total. Differences were limited to new decision/command fields and test-reference metadata. Model XML and the compiled hardware contract were unchanged.

## Motion demonstrations

[Synchronized MP4](motion/visuals/ABC_comparison.mp4) · [A: Rotation](motion/visuals/A.mp4) · [B: Lifting](motion/visuals/B.mp4) · [C: Sustained External Force](motion/visuals/C.mp4)

![A/B/C](motion/visuals/ABC_comparison.gif)

| Example | Actual motion | Human peak force, world XYZ | External disturbance |
|---|---|---|---|
| A | +20.00°, lift 3.05 cm, then hold | 2.05 N, `[-.84,-.04,-1.87]` N | None |
| B | Lift 7.98 cm, then hold | 5.41 N, `[+.02,-.02,+5.41]` N | None |
| C | −11.94°, then withstand sustained external force | 4.20 N, `[-.29,-.03,+4.19]` N | World −Z 20 N during 4.3–8 seconds |

Each demonstration lasts 8 seconds. The human target begins at 0.6 s; the predefined arm trajectory runs during 1.4–3.8 s. In C, external force ramps up during 4.0–4.3 s. Red arrows show human force and orange arrows show external force. Arrows have a fixed 0.12 m length to indicate direction; read the N values for magnitude. Tactile maps share a 0–40 N/bin scale.

The three demonstrations total 12,000 physics steps and maintain contact. All 168 arrays are exactly identical to the original demonstrations; the rendered frames at 5 seconds are also identical. A's minimum joint margin is approximately 0.04 rad. These results do not establish reachability for arbitrary rotations or 90° flips.

These are physical demonstrations of scripted trajectories, not evidence that the three-layer controller understands intent. `motion/*/samples.npz` mixes truth and render state and must not be used as policy input or Expert supervision.

## Reproduction

```bash
uv sync --locked
uv run --locked python -m phase1b.scripts.run_suite --output results/phase1b/reproduction --render
```

Use a new output path. Rendering requires ffmpeg. Historical code and data remain in Git; the working tree maintains only the current implementation.
