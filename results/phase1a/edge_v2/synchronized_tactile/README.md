# Synchronized MuJoCo scene and tactile

`scene_tactile_sync.mp4` (1280 x 900) and `scene_tactile_sync.gif` (960 x 675) show the same four seconds at real-time speed, 20 fps. A is human-target rotation; C is an external push with the human target held. Events start at 0.70 s. Each scene has L-upper, L-lower, R-upper, R-lower raw sensor-z maps directly below it.

- Red: actual human application point and applied human force arrow.
- Cyan: virtual human target and reference path. This is a target, not actual panel motion or an articulated hand.
- Orange: external application point, with an incoming arrow along the world-frame force. Zero force has a marker but no arrow.
- Both force-arrow lengths use 0.006 m/N. Motion is not magnified. Target-angle change and actual absolute panel pitch are separately displayed.
- Maps use a common 0-25 N/bin range in each original pad sensor frame; y bins increase upward. No temporal averaging or spatial interpolation. This is contact force, not pressure in Pa.

The renderer replays all recorded 500 Hz commands in the frozen scenario/hardware and compares all 400 pre-decision observations per episode exactly, including panel/gripper poses and tactile. Rendered frames select every fifth 100 Hz observation, using that same sample's annotations and tactile. Rendering is checked not to modify qpos, qvel or sensordata. `verification.json` binds timestamps, hashes and the exact-match checks. Original recordings and frozen model files are unchanged.

This support-only baseline is not a completed cooperative flip. Target motion and actual panel rotation differ; both angles are displayed. The external push can also produce substantial panel rotation. Analysis annotations contain privileged information, never policy inputs. Sparse contact bins may flicker even in HOLD.

Reproduce with uv and an installed ffmpeg, choosing a fresh directory:

```bash
uv run --locked python -m phase1a.scripts.render_tactile_sync --output results/phase1a/dev-sync
```
