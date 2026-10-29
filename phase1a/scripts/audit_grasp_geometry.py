"""Native MuJoCo transforms/contact manifold -> auditable geometry and figures."""

import argparse
import itertools
import json
from pathlib import Path

import matplotlib
import mujoco
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

from phase1a.simulation.scenario_loader import (
    CONFIG,
    EDGE_CONFIG,
    REPO,
    config_hash,
    load_scenario,
    make_sim,
)

COLORS = {"left": "#2376bb", "right": "#8b4caf", "human": "#d8423a"}


def snapshot(sim):
    m, d = sim.m, sim.d
    origin = d.xpos[sim.panel]
    R = d.xmat[sim.panel].reshape(3, 3)

    def local(p):
        return R.T @ (p - origin)

    pads = {}
    for side, ids in zip(("left", "right"), sim.pad_g):
        for gid in ids:
            center = d.geom_xpos[gid]
            rot = d.geom_xmat[gid].reshape(3, 3)
            size = m.geom_size[gid]
            corners = np.array(
                [
                    local(center + rot @ (np.array(sign) * size))
                    for sign in itertools.product((-1, 1), repeat=3)
                ]
            )
            pads[m.geom(gid).name] = {
                "center_panel_local_m": local(center).tolist(),
                "bounds_panel_local_m": [
                    corners.min(axis=0).tolist(),
                    corners.max(axis=0).tolist(),
                ],
            }
    contacts = []
    force = np.zeros(6)
    for i, c in enumerate(d.contact):
        if sim.pg not in (c.geom1, c.geom2):
            continue
        other = c.geom2 if c.geom1 == sim.pg else c.geom1
        if other not in [gid for ids in sim.pad_g for gid in ids]:
            continue
        mujoco.mj_contactForce(m, d, i, force)
        if force[0] <= 1e-8:
            continue
        contacts.append(
            {
                "pad": m.geom(other).name,
                "position_panel_local_m": local(c.pos).tolist(),
                "normal_force_N": float(force[0]),
                "signed_distance_m": float(c.dist),
            }
        )
    return {
        "engine_time_s": float(d.time),
        "panel_half_extents_m": m.geom_size[sim.pg].tolist(),
        "grasp_centers_panel_local_m": {
            side: local(d.xpos[root]).tolist()
            for side, root in zip(("left", "right"), sim.roots)
        },
        "human_point_panel_local_m": local(d.site_xpos[sim.hsite]).tolist(),
        "pads": pads,
        "active_contacts": contacts,
    }


def top_view(ax, nominal, settled, title):
    half = np.array(nominal["panel_half_extents_m"])
    ax.add_patch(
        Rectangle(
            -half[:2],
            *(2 * half[:2]),
            facecolor="#efcc7f",
            edgecolor="#6e541f",
            alpha=0.55,
            lw=2,
        )
    )
    for side in ("left", "right"):
        color = COLORS[side]
        pad = nominal["pads"][side + "_upper_pad"]
        lo, hi = np.array(pad["bounds_panel_local_m"])
        ax.add_patch(
            Rectangle(
                lo[:2],
                *(hi - lo)[:2],
                facecolor=color,
                edgecolor=color,
                alpha=0.24,
                lw=2,
            )
        )
        p = np.array(nominal["grasp_centers_panel_local_m"][side])
        ax.scatter(*p[:2], s=65, color=color, zorder=5)
        ax.annotate(
            f"Robot {side[0].upper()} ({p[0]:+.2f}, {p[1]:+.2f}, 0)",
            xy=p[:2],
            xytext=(p[0], 0.205),
            ha="center",
            fontsize=9,
            color=color,
            arrowprops={"arrowstyle": "-", "color": color},
        )
        pts = np.array(
            [
                c["position_panel_local_m"]
                for c in settled["active_contacts"]
                if c["pad"].startswith(side)
            ]
        )
        if len(pts):
            ax.scatter(pts[:, 0], pts[:, 1], s=18, color=color, marker="x", zorder=6)
    h = np.array(nominal["human_point_panel_local_m"])
    ax.scatter(*h[:2], s=90, color=COLORS["human"], marker="*", zorder=7)
    ax.annotate(
        f"Human ({h[0]:+.2f}, {h[1]:+.2f}, 0)",
        xy=h[:2],
        xytext=(0.1, -0.18),
        ha="center",
        fontsize=9,
        color=COLORS["human"],
        arrowprops={"arrowstyle": "-", "color": COLORS["human"]},
    )
    ax.text(-0.3, -0.14, "Panel: 0.60 x 0.20 m", fontsize=9, color="#6e541f")
    ax.scatter(0, 0, marker="+", color="black", s=45)
    ax.text(0.01, 0.008, "COM", fontsize=8)
    ax.set(
        xlim=(-0.36, 0.36),
        ylim=(-0.23, 0.25),
        aspect="equal",
        xlabel="Panel X (m)",
        ylabel="Panel Y (m)",
        title=title,
    )
    ax.grid(alpha=0.2)


def render(sim, path):
    # Rendering changes no model, contact, or command values.
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [0, 0, 0.8]
    camera.distance = 1.05
    camera.azimuth = 90
    camera.elevation = -30
    with mujoco.Renderer(sim.m, height=480, width=640) as renderer:
        renderer.update_scene(sim.d, camera=camera)
        # Hide only plugin debug overlays, not authored geometry or contact data.
        for geom in renderer.scene.geoms[: renderer.scene.ngeom]:
            if geom.objtype == mujoco.mjtObj.mjOBJ_UNKNOWN:
                geom.rgba[3] = 0.0
        plt.imsave(path, renderer.render())


def audit(output):
    if output.exists():
        raise FileExistsError(output)
    output.mkdir(parents=True)
    records = {}
    for name, path in [("v1", CONFIG), ("v2", EDGE_CONFIG)]:
        cfg = load_scenario(path)
        sim = make_sim(cfg)
        nominal = snapshot(sim)
        render(sim, output / (name + "_mujoco.png"))
        sim.settle()
        sim.forces(0.0, "B")
        settled = snapshot(sim)
        records[name] = {
            "scenario_id": cfg["scenario_id"],
            "scenario_version": cfg["scenario_version"],
            "model_hash": cfg["model_hash"],
            "config_hash": config_hash(cfg),
            "nominal": nominal,
            "settled": settled,
        }
    (output / "geometry_audit.json").write_text(json.dumps(records, indent=2) + "\n")
    fig, axes = plt.subplots(1, 2, figsize=(13, 5.5))
    for ax, key, title in zip(
        axes,
        ("v1", "v2"),
        ("Historical v1: pads over interior faces", "Edge v2: robots +Y, human -Y"),
    ):
        top_view(ax, records[key]["nominal"], records[key]["settled"], title)
    fig.text(
        0.5,
        0.035,
        "Filled rectangles: nominal pad projection (upper/lower overlap). Dots: between-jaw grasp centers. x: measured contacts after settling.",
        ha="center",
        fontsize=9,
    )
    fig.tight_layout(rect=(0, 0.07, 1, 1))
    fig.savefig(output / "before_after_top.png", dpi=180)
    plt.close(fig)
    fig, axes = plt.subplots(
        1, 2, figsize=(13, 5.5), gridspec_kw={"width_ratios": [1.4, 1]}
    )
    top_view(
        axes[0],
        records["v2"]["nominal"],
        records["v2"]["settled"],
        "Fixed nominal positions: panel-local frame",
    )
    ax = axes[1]
    nom = records["v2"]["nominal"]
    half = np.array(nom["panel_half_extents_m"])
    ax.add_patch(
        Rectangle(
            (-half[1], -half[2]),
            2 * half[1],
            2 * half[2],
            facecolor="#efcc7f",
            edgecolor="#6e541f",
            lw=2,
        )
    )
    for name, pad in nom["pads"].items():
        if not name.startswith("left"):
            continue
        lo, hi = np.array(pad["bounds_panel_local_m"])
        ax.add_patch(
            Rectangle(
                lo[1:],
                *(hi - lo)[1:],
                facecolor=COLORS["left"],
                edgecolor=COLORS["left"],
                alpha=0.4,
                lw=2,
            )
        )
    ax.plot([0.03, 0.10], [0.01, 0.01], lw=4, color="#d17900")
    ax.plot([0.03, 0.10], [-0.01, -0.01], lw=4, color="#d17900")
    ax.scatter(0.1, 0, color=COLORS["left"], s=65, zorder=4)
    ax.annotate(
        "Between-jaw anchor\nY = +0.10, Z = 0",
        xy=(0.1, 0),
        xytext=(0.15, -0.095),
        ha="center",
        fontsize=9,
        arrowprops={"arrowstyle": "-"},
    )
    ax.annotate(
        "Actual jaws contact TOP / BOTTOM\nNominal overlap: Y = 0.03 to 0.10 m",
        xy=(0.055, 0.01),
        xytext=(0.015, 0.10),
        ha="center",
        fontsize=9,
        arrowprops={"arrowstyle": "-"},
    )
    ax.text(-0.10, -0.055, "Panel thickness\n0.02 m", fontsize=9, ha="center")
    ax.set(
        xlim=(-0.15, 0.25),
        ylim=(-0.135, 0.145),
        aspect="equal",
        xlabel="Panel Y (m)",
        ylabel="Panel Z (m)",
        title="Section through Robot L (X = -0.20 m)",
    )
    ax.grid(alpha=0.2)
    fig.tight_layout()
    fig.savefig(output / "fixed_edge_layout.png", dpi=180)
    plt.close(fig)
    return records


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--output", type=Path, default=REPO / "results/phase1a/edge_v2/geometry"
    )
    a = ap.parse_args()
    audit(a.output)
    print(a.output)


if __name__ == "__main__":
    main()
