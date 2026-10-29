"""Visualize recorded tactile arrays; no physics, hardware, or sensor edits."""

import argparse
import hashlib
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter

from phase1a.data.episode_io import DatasetReader, validate_dataset
from phase1a.simulation.scenario_loader import REPO

ROWS = [
    ("left", 0, "Robot L / upper"),
    ("left", 1, "Robot L / lower"),
    ("right", 0, "Robot R / upper"),
    ("right", 1, "Robot R / lower"),
]
WINDOWS = [
    (0.4, 0.6, "Before: 0.40-0.60 s"),
    (0.7, 0.9, "Onset: 0.70-0.90 s"),
    (1.4, 1.6, "Later: 1.40-1.60 s"),
]


def heatmap(ax, values, limit, *, signed=False):
    im = ax.imshow(
        values,
        origin="lower",
        interpolation="nearest",
        cmap="RdBu_r" if signed else "magma",
        vmin=-limit if signed else 0,
        vmax=limit,
    )
    ax.set_xticks(range(4))
    ax.set_yticks(range(4))
    ax.tick_params(labelsize=8, length=0)
    ax.set_xticks(np.arange(-0.5, 4, 1), minor=True)
    ax.set_yticks(np.arange(-0.5, 4, 1), minor=True)
    ax.grid(which="minor", color="#999999", lw=0.5, alpha=0.4)
    ax.tick_params(which="minor", length=0)
    for y in range(4):
        for x in range(4):
            value = values[y, x]
            text = (
                "·"
                if abs(value) < 0.005
                else f"{value:+.1f}"
                if signed
                else f"{value:.1f}"
            )
            white = abs(value) > 0.6 * limit if signed else value < 0.55 * limit
            ax.text(
                x,
                y,
                text,
                ha="center",
                va="center",
                fontsize=9,
                color="white" if white else "#161616",
            )
    return im


def absolute_maps(means, limit, out):
    fig = plt.figure(figsize=(13.5, 9))
    gs = fig.add_gridspec(
        4,
        7,
        width_ratios=[1, 1, 1, 0.22, 1, 1, 1],
        left=0.10,
        right=0.91,
        bottom=0.09,
        top=0.86,
        wspace=0.25,
        hspace=0.45,
    )
    for r, (_, _, label) in enumerate(ROWS):
        for scenario, cols in [("A", (0, 1, 2)), ("C", (4, 5, 6))]:
            for w, col in enumerate(cols):
                ax = fig.add_subplot(gs[r, col])
                values = means[scenario][w, r, 0]
                im = heatmap(ax, values, limit)
                ax.set_title(
                    (WINDOWS[w][2] + "\n" if r == 0 else "")
                    + f"Sum = {values.sum():.2f} N",
                    fontsize=9,
                    pad=4,
                )
                if col == 0:
                    ax.set_ylabel(
                        label,
                        fontsize=11,
                        labelpad=12,
                        color="#2376bb" if r < 2 else "#8b4caf",
                    )
    fig.text(
        0.30,
        0.93,
        "A: human target starts rotating at 0.70 s",
        ha="center",
        fontsize=12,
        weight="bold",
    )
    fig.text(
        0.72,
        0.93,
        "C: disturbance starts at 0.70 s; human holds",
        ha="center",
        fontsize=12,
        weight="bold",
    )
    cb = fig.colorbar(im, cax=fig.add_axes([0.935, 0.21, 0.014, 0.53]))
    cb.set_label("Sensor-z contact force (N / angular bin)")
    fig.text(
        0.5,
        0.035,
        "Each map averages 20 samples (0.20 s); no spatial interpolation. Bin indices 0-3 are in EACH sensor frame, not panel coordinates.",
        ha="center",
        fontsize=9,
    )
    fig.savefig(out / "normal_distribution.png", dpi=170)
    plt.close(fig)


def difference_maps(means, out):
    maps = [
        means["A"][2, :, 0] - means["A"][0, :, 0],
        means["A"][2, :, 0] - means["B"][2, :, 0],
        means["C"][2, :, 0] - means["C"][0, :, 0],
        means["C"][2, :, 0] - means["B"][2, :, 0],
    ]
    limit = max(1.0, float(np.ceil(np.max(np.abs(maps)))))
    titles = [
        "A: later minus BEFORE",
        "A: later minus same-time HOLD",
        "C: later minus BEFORE",
        "C: later minus same-time HOLD",
    ]
    fig, axes = plt.subplots(4, 4, figsize=(11, 9))
    for row, (_, _, label) in enumerate(ROWS):
        for col in range(4):
            values = maps[col][row]
            im = heatmap(axes[row, col], values, limit, signed=True)
            axes[row, col].set_title(
                (titles[col] + "\n" if row == 0 else "")
                + f"Change in sum = {values.sum():+.2f} N",
                fontsize=8.5,
            )
            if col == 0:
                axes[row, col].set_ylabel(label, fontsize=10, labelpad=10)
    fig.subplots_adjust(
        left=0.10, right=0.90, top=0.88, bottom=0.09, wspace=0.28, hspace=0.5
    )
    fig.suptitle(
        "Force redistribution at 1.40-1.60 s: red = increase, blue = decrease",
        fontsize=12,
    )
    fig.colorbar(
        im,
        cax=fig.add_axes([0.93, 0.22, 0.015, 0.52]),
        label="Change in sensor-z force (N / bin)",
    )
    fig.text(
        0.5,
        0.035,
        "BEFORE = 0.40-0.60 s. HOLD = B at 1.40-1.60 s. Differences do not establish information beyond wrist F/T.",
        ha="center",
        fontsize=9,
    )
    fig.savefig(out / "distribution_changes.png", dpi=170)
    plt.close(fig)


def xyz_maps(means, normal_limit, out):
    shear = max(
        1.0, float(np.ceil(max(np.max(np.abs(means[s][2, :, 1:])) for s in ("A", "C"))))
    )
    fig, axes = plt.subplots(4, 6, figsize=(13, 9))
    for row, (_, _, label) in enumerate(ROWS):
        for col in range(6):
            scenario = ("A", "C")[col // 3]
            channel = col % 3
            values = means[scenario][2, row, channel]
            im = heatmap(
                axes[row, col],
                values,
                normal_limit if channel == 0 else shear,
                signed=channel != 0,
            )
            if channel == 0:
                normal_im = im
            else:
                shear_im = im
            axes[row, col].set_title(
                (
                    scenario + " / " + ("Fz", "Fx", "Fy")[channel] + "\n"
                    if row == 0
                    else ""
                )
                + f"Sum={values.sum():+.2f} N",
                fontsize=9,
            )
            if col == 0:
                axes[row, col].set_ylabel(label, fontsize=10, labelpad=10)
    fig.subplots_adjust(
        left=0.09, right=0.89, top=0.89, bottom=0.08, wspace=0.3, hspace=0.45
    )
    fig.suptitle(
        "Three recorded force channels at 1.40-1.60 s (20-sample mean)", fontsize=13
    )
    fig.colorbar(
        normal_im, cax=fig.add_axes([0.92, 0.56, 0.014, 0.27]), label="Fz (N/bin)"
    )
    fig.colorbar(
        shear_im, cax=fig.add_axes([0.92, 0.17, 0.014, 0.27]), label="Fx / Fy (N/bin)"
    )
    fig.text(
        0.5,
        0.025,
        "Each map uses its own pad SENSOR frame. Opposing upper/lower frames are not reprojected into a common panel frame.",
        ha="center",
        fontsize=9,
    )
    fig.savefig(out / "three_force_channels.png", dpi=170)
    plt.close(fig)


def animation(readers, limit, out):
    fig, axes = plt.subplots(2, 4, figsize=(9, 5.7))
    fig.subplots_adjust(
        left=0.06, right=0.90, bottom=0.1, top=0.80, wspace=0.25, hspace=0.32
    )
    images = []
    sums = []
    for pad in range(2):
        for col, (scenario, side) in enumerate(
            [("A", "left"), ("A", "right"), ("C", "left"), ("C", "right")]
        ):
            ax = axes[pad, col]
            im = ax.imshow(
                np.zeros((4, 4)),
                origin="lower",
                interpolation="nearest",
                cmap="magma",
                vmin=0,
                vmax=limit,
            )
            ax.set_xticks(range(4))
            ax.set_yticks(range(4))
            ax.tick_params(labelsize=7)
            if pad == 0:
                ax.set_title(f"{scenario}: Robot {side[0].upper()}", fontsize=11)
            if col == 0:
                ax.set_ylabel(("upper", "lower")[pad])
            total = ax.text(
                0.5, -0.18, "", transform=ax.transAxes, ha="center", fontsize=9
            )
            images.append((im, scenario, side, pad))
            sums.append(total)
    title = fig.suptitle("", fontsize=11)
    fig.colorbar(
        im, cax=fig.add_axes([0.93, 0.22, 0.015, 0.48]), label="Sensor-z force (N/bin)"
    )
    fig.text(
        0.5,
        0.018,
        "Raw individual samples every 0.10 s; no temporal average or spatial smoothing. Grid flicker is retained.",
        ha="center",
        fontsize=8,
    )
    time = readers["A"].sections["observations"]["timestamp"]
    indices = list(range(0, len(time), 10))

    def update(index):
        for (im, scenario, side, pad), total in zip(images, sums):
            values = readers[scenario].sections["observations"]["tactile_" + side][
                index, pad, 0
            ]
            im.set_data(values)
            total.set_text(f"Sum={values.sum():.2f} N")
        human = readers["A"].sections["privileged"]["human_goal"]
        angle = np.rad2deg(human[index] - human[0])
        force = readers["C"].sections["privileged"]["external_disturbance_force"][
            index, 2
        ]
        title.set_text(
            f"t = {time[index]:.2f} s   |   A human target change: {angle:.1f} deg   |   C external Fz: {force:.1f} N"
        )

    movie = FuncAnimation(fig, update, frames=indices, interval=100, repeat=True)
    movie.save(out / "raw_tactile_evolution.gif", writer=PillowWriter(fps=10), dpi=100)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--input", type=Path, default=REPO / "results/phase1a/edge_v2/smoke"
    )
    ap.add_argument(
        "--output",
        type=Path,
        default=REPO / "results/phase1a/edge_v2/tactile_distribution",
    )
    a = ap.parse_args()
    if a.output.exists():
        ap.error("choose a new output directory")
    readers = {s: DatasetReader(a.input / s) for s in "ABC"}
    for s in "ABC":
        validate_dataset(a.input / s)
    time = readers["A"].sections["observations"]["timestamp"]
    for s in "BC":
        np.testing.assert_array_equal(
            time, readers[s].sections["observations"]["timestamp"]
        )
    means = {}
    summary = {}
    for scenario, reader in readers.items():
        o = reader.sections["observations"]
        windows = []
        for lo, hi, _ in WINDOWS:
            mask = (time >= lo) & (time < hi)
            assert mask.sum() == 20
            windows.append(
                np.stack(
                    [
                        o["tactile_" + side][mask, pad].mean(axis=0)
                        for side, pad, _ in ROWS
                    ]
                )
            )
        means[scenario] = np.stack(windows)
        summary[scenario] = {
            label: {
                "sensor_z_sum_N": means[scenario][:, r, 0].sum(axis=(1, 2)).tolist(),
                "instantaneous_occupied_z_bins_min_max": [
                    int(
                        np.count_nonzero(
                            o["tactile_" + side][:, pad, 0] > 1e-8, axis=(1, 2)
                        ).min()
                    ),
                    int(
                        np.count_nonzero(
                            o["tactile_" + side][:, pad, 0] > 1e-8, axis=(1, 2)
                        ).max()
                    ),
                ],
            }
            for r, (side, pad, label) in enumerate(ROWS)
        }
    for side in ("left", "right"):
        for scenario in "AC":
            np.testing.assert_array_equal(
                readers[scenario].sections["observations"]["tactile_" + side][
                    time < 0.7
                ],
                readers["B"].sections["observations"]["tactile_" + side][time < 0.7],
            )
    limit = float(
        np.ceil(
            max(
                np.max(readers[s].sections["observations"]["tactile_" + side][:, :, 0])
                for s in "ABC"
                for side in ("left", "right")
            )
            / 5
        )
        * 5
    )
    a.output.mkdir(parents=True)
    absolute_maps(means, limit, a.output)
    difference_maps(means, a.output)
    xyz_maps(means, limit, a.output)
    animation(readers, limit, a.output)
    np.savez_compressed(
        a.output / "window_means.npz",
        **means,
        windows_seconds=np.array([[lo, hi] for lo, hi, _ in WINDOWS]),
    )
    metadata = {
        "windows": WINDOWS,
        "array_order": "scenario -> window, pad [L upper,L lower,R upper,R lower], channel [z,x,y], y_bin,x_bin",
        "unit": "N/bin; force, not pressure in Pa",
        "map_axes": "each original sensor angular-bin frame; lower pads are not mirrored to panel coordinates",
        "temporal_average": "20 original 100 Hz samples per static window; no spatial interpolation",
        "absolute_z_color_range_N_per_bin": [0, limit],
        "reference_metadata": {s: readers[s].metadata for s in "ABC"},
        "source_sha256": {
            s: hashlib.sha256(
                (a.input / s / "observations.npz").read_bytes()
            ).hexdigest()
            for s in "ABC"
        },
        "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "summary": summary,
        "limitations": [
            "sparse contact solver manifold, not 16 independent pressure sensors",
            "bin-boundary/contact-manifold changes can cause abrupt redistribution even in HOLD",
            "no claim of incremental information over F/T",
        ],
    }
    (a.output / "summary.json").write_text(json.dumps(metadata, indent=2) + "\n")
    (a.output / "README.md").write_text("""# Recorded tactile distributions

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
""")
    print(a.output, flush=True)


if __name__ == "__main__":
    main()
