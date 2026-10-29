# Recorded tactile distributions

These plots use the frozen scenario v2 / hardware v1 recordings, without rerunning or changing physics.

- `normal_distribution.png`: A rotation and C disturbance; all four pads, three 0.20 s windows; one shared 0–25 N/bin scale.
- `distribution_changes.png`: later minus before, and later minus same-time HOLD B; one shared signed scale.
- `three_force_channels.png`: sensor-z/x/y channels at 1.40–1.60 s; shared normal and tangential scales.
- `raw_tactile_evolution.gif`: raw samples every 0.10 s, played at 10 fps, without averaging or interpolation. Human target and external force are privileged annotations for analysis only.
- `window_means.npz`, `summary.json`: exact plotted values, row/frame conventions, load sums and source hashes.

The static maps average 20 samples; they can contain fractional occupancy across bins visited at different instants. The raw animation retains the actual flicker. Each pad uses its own sensor frame, including the rotated lower-pad frame. The bins are angular regions, not equal-area physical pressure taxels. Zero means no force reported in that bin, not a failed sensor. No conversion to pressure (Pa) is performed.

The HOLD comparison helps expose spontaneous simulator/contact-grid variation. It does not establish tactile information beyond wrist F/T. Normal-load change and spatial redistribution are different quantities; a pad can redistribute force while keeping nearly the same sum.

Source semantics: https://github.com/google-deepmind/mujoco/blob/3.3.5/plugin/sensor/README.md

Reproduce from the repository root, using a fresh output folder:

```bash
uv run --locked python -m phase1a.scripts.plot_tactile_distribution --output results/phase1a/dev-tactile-distribution
```
