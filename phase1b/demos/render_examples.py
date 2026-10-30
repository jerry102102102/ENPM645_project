"""Three synchronized, visibly moving physical demonstrations with force vectors."""

import argparse
import hashlib
import json
import subprocess
from pathlib import Path

import matplotlib
import mujoco
import numpy as np
from PIL import Image, ImageDraw

from phase1b.demos.render_helpers import arrow, font, marker
from phase1b.simulation import ROOT, PandaWorld

COLORS = {"human": (255, 90, 90), "target": (49, 224, 205), "external": (255, 180, 45)}


def phase(t, case):
    if t < 0.6:
        return "READY"
    if t < 1.4:
        return "HUMAN STARTS"
    if t < 3.8:
        return "ARMS MOVING"
    if case == "C" and t >= 4.3:
        return "HOLD + CONSTANT 20 N"
    if case == "C" and t >= 4:
        return "LOAD RAMP"
    return "HOLD POSITION"


def scene_image(renderer, w, data, i, camera):
    w.d.qpos[:] = data["qpos"][i]
    w.d.qvel[:] = data["qvel"][i]
    mujoco.mj_fwdPosition(w.m, w.d)
    np.testing.assert_allclose(
        w.d.site_xpos[w.hsite], data["human_application_point"][i], atol=1e-12
    )
    for arm, b in enumerate(w.roots):
        np.testing.assert_allclose(
            w.d.xpos[b], data["end_effector_pose"][i, arm, :3], atol=1e-12
        )
    renderer.update_scene(w.d, camera=camera)
    scene = renderer.scene
    for geom in scene.geoms[: scene.ngeom]:
        if geom.objtype == mujoco.mjtObj.mjOBJ_UNKNOWN:
            geom.rgba[3] = 0
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, data["object_orientation"][0])
    R = R.reshape(3, 3)
    corners = [
        data["object_position"][0] + R @ np.array([x, y, 0.011])
        for x, y in [(-0.3, -0.1), (0.3, -0.1), (0.3, 0.1), (-0.3, 0.1)]
    ]
    for a, b in zip(corners, corners[1:] + corners[:1]):
        geom = marker(scene, a, (150, 170, 190), radius=0.002)
        mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_LINE, 2.0, a, b)
    marker(scene, data["human_target_trajectory"][i], COLORS["target"], radius=0.008)
    point = data["human_application_point"][i]
    f = data["human_applied_force"][i]
    marker(scene, point, COLORS["human"], radius=0.010)
    if np.linalg.norm(f) > 0.05:
        arrow(scene, point, point + 0.12 * f / np.linalg.norm(f), COLORS["human"])
    force = data["external_disturbance_force"][i]
    point = data["external_application_point"][i]
    if np.linalg.norm(force) > 0.05:
        marker(scene, point, COLORS["external"], radius=0.006)
        arrow(
            scene,
            point - 0.12 * force / np.linalg.norm(force),
            point,
            COLORS["external"],
        )
    for label, b, color in zip(("L", "R"), w.roots, ((60, 150, 245), (190, 100, 230))):
        marker(scene, w.d.xpos[b] + [0, 0, 0.065], color, label, radius=0.003)
    return Image.fromarray(renderer.render().copy())


def vector_text(force):
    return "[" + ", ".join(f"{v:+.2f}" for v in force) + "]"


def panel(case, data, i, image, limit):
    frame = Image.new("RGB", (640, 1000), "#111923")
    d = ImageDraw.Draw(frame)
    t = data["timestamp"][i]
    titles = {
        "A": "A | ROTATE 20 DEG, THEN HOLD",
        "B": "B | LIFT 8 CM, THEN HOLD",
        "C": "C | TILT -12 DEG, THEN HOLD 20 N",
    }
    d.text((14, 12), titles[case], font=font(23), fill="white")
    d.text(
        (14, 46), f"t = {t:4.2f} s    {phase(t, case)}", font=font(20), fill="#63ded0"
    )
    frame.paste(image, (0, 82))
    delta_angle = data["panel_pitch_deg"][i] - data["panel_pitch_deg"][0]
    delta_z = (data["object_position"][i, 2] - data["object_position"][0, 2]) * 100
    d.text(
        (14, 570),
        f"Actual panel: angle {delta_angle:+.2f} deg | height {delta_z:+.2f} cm",
        font=font(20),
        fill="white",
    )
    f = data["human_applied_force"][i]
    e = data["external_disturbance_force"][i]
    d.text(
        (14, 604),
        f"Human Fxyz = {vector_text(f)} N",
        font=font(18),
        fill=COLORS["human"],
    )
    d.text(
        (14, 630),
        f"|Human force| = {np.linalg.norm(f):.2f} N",
        font=font(18),
        fill=COLORS["human"],
    )
    d.text(
        (14, 659),
        f"External Fxyz = {vector_text(e)} N",
        font=font(18),
        fill=COLORS["external"],
    )
    max_dq = np.max(np.abs(data["arm_joint_velocity"][i]))
    d.text(
        (14, 687),
        f"Max joint speed = {max_dq:.3f} rad/s",
        font=font(17),
        fill="#d5dfeb",
    )
    for num, (side, padidx) in enumerate(
        (("left", 0), ("left", 1), ("right", 0), ("right", 1))
    ):
        x = 16 + num * 160
        values = data["tactile_" + side][i, padidx, 0]
        d.text(
            (x, 720),
            f"{side[0].upper()} {('upper', 'lower')[padidx]}",
            font=font(16),
            fill="#7fbafa" if side == "left" else "#cf97ef",
        )
        for y in range(4):
            for bx in range(4):
                v = values[y, bx]
                rgb = tuple(
                    int(c * 255) for c in matplotlib.colormaps["magma"](v / limit)[:3]
                )
                px = x + bx * 28
                py = 744 + (3 - y) * 28
                d.rectangle((px, py, px + 27, py + 27), fill=rgb, outline="#50505b")
                if v > 0.05:
                    d.text(
                        (px + 14, py + 14),
                        f"{v:.1f}",
                        anchor="mm",
                        font=font(10),
                        fill="white" if v < limit * 0.55 else "black",
                    )
        d.text((x, 862), f"Sum {values.sum():.1f} N", font=font(14), fill="white")
    # Both traces use the same force-magnitude scale; orange plateaus at 20 N in C.
    left, top, width, height = 40, 902, 570, 55
    d.line((left, top, left, top + height, left + width, top + height), fill="#758398")
    d.text((4, top - 5), "22", font=font(12), fill="#b8c3d3")
    d.text((10, top + height - 10), "0", font=font(12), fill="#b8c3d3")
    for values, color in [
        (np.linalg.norm(data["human_applied_force"], axis=1), COLORS["human"]),
        (
            np.linalg.norm(data["external_disturbance_force"], axis=1),
            COLORS["external"],
        ),
    ]:
        points = [
            (
                left + data["timestamp"][j] / 8 * width,
                top + height - values[j] / 22 * height,
            )
            for j in range(i + 1)
        ]
        if len(points) > 1:
            d.line(points, fill=color, width=2)
    d.text(
        (40, 965),
        "Force magnitudes (N), 0-8 s: human / external",
        font=font(14),
        fill="#b8c3d3",
    )
    return frame


def encoder(path, size):
    return subprocess.Popen(
        [
            "ffmpeg",
            "-v",
            "error",
            "-f",
            "rawvideo",
            "-pix_fmt",
            "rgb24",
            "-s",
            size,
            "-r",
            "20",
            "-i",
            "-",
            "-an",
            "-c:v",
            "libx264",
            "-crf",
            "18",
            "-pix_fmt",
            "yuv420p",
            "-movflags",
            "+faststart",
            str(path),
        ],
        stdin=subprocess.PIPE,
    )


def render(source, output):
    if output.exists():
        raise FileExistsError(output)
    data = {}
    reports = {}
    for case in "ABC":
        with np.load(source / case / "samples.npz", allow_pickle=False) as f:
            data[case] = {k: f[k] for k in f.files}
        reports[case] = json.loads((source / case / "report.json").read_text())
        assert reports[case]["metrics"]["passed"]
        for path, digest in reports[case]["hashes"].items():
            assert hashlib.sha256((ROOT / path).read_bytes()).hexdigest() == digest
    for case in "BC":
        np.testing.assert_array_equal(data["A"]["timestamp"], data[case]["timestamp"])
    limit = float(
        5
        * np.ceil(
            max(
                data[c]["tactile_" + s][:, :, 0].max()
                for c in "ABC"
                for s in ("left", "right")
            )
            / 5
        )
    )
    output.mkdir(parents=True)
    worlds = {c: PandaWorld() for c in "ABC"}
    renderers = {c: mujoco.Renderer(w.m, 480, 640) for c, w in worlds.items()}
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [0, 0.3, 0.69]
    camera.distance = 1.70
    camera.azimuth = 90
    camera.elevation = -20
    encoders = {c: encoder(output / f"{c}.mp4", "640x1000") for c in "ABC"}
    overview = encoder(output / "ABC_comparison.mp4", "1920x1060")
    gifs = []
    frames = []
    try:
        for i in range(0, len(data["A"]["timestamp"]), 5):
            combo = Image.new("RGB", (1920, 1060), "#111923")
            for col, c in enumerate("ABC"):
                image = scene_image(renderers[c], worlds[c], data[c], i, camera)
                result = panel(c, data[c], i, image, limit)
                encoders[c].stdin.write(result.tobytes())
                combo.paste(result, (col * 640, 0))
                if i in (0, 260, 500):
                    result.save(output / f"{c}_{i:03d}.png")
            draw = ImageDraw.Draw(combo)
            draw.text(
                (16, 1007),
                f"World XYZ forces. Direction arrows: fixed 0.12 m, magnitude shown in N. Tactile: common 0-{limit:.0f} N/bin. Grey outline: starting panel pose.",
                font=font(21),
                fill="white",
            )
            draw.text(
                (16, 1034),
                "SCRIPTED JOINT MOTION + PHYSICS CONTACT. No inferred human intent, learned policy, expert labels, or force-tracking claim. Playback: 1x.",
                font=font(21),
                fill="#b8c3d3",
            )
            overview.stdin.write(combo.tobytes())
            if i % 10 == 0:
                gifs.append(combo.resize((1440, 795), Image.Resampling.LANCZOS))
            frames.append(
                {
                    "frame": len(frames),
                    "sample_index": i,
                    "timestamp_s": float(data["A"]["timestamp"][i]),
                }
            )
            if i in (0, 260, 500):
                combo.save(output / f"ABC_{i:03d}.png")
    finally:
        for p in [*encoders.values(), overview]:
            p.stdin.close()
        statuses = [p.wait() for p in [*encoders.values(), overview]]
        for r in renderers.values():
            r.close()
    assert statuses == [0] * 4, statuses
    gifs[0].save(
        output / "ABC_comparison.gif",
        save_all=True,
        append_images=gifs[1:],
        duration=100,
        loop=0,
    )
    info = dict(
        frames=frames,
        fps=20,
        gif_fps=10,
        duration_s=8.0,
        tactile_range_N_per_bin=[0, limit],
        arrow_mode="fixed 0.12 m length, direction only",
        source_hashes={
            c: hashlib.sha256((source / c / "samples.npz").read_bytes()).hexdigest()
            for c in "ABC"
        },
        script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        scope="Native MuJoCo FK of recorded physics states; raw tactile and applied force vectors from same samples",
    )
    (output / "frame_capture.json").write_text(json.dumps(info, indent=2) + "\n")
    print("Rendered A/B/C moving examples:", output, flush=True)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--input", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    render(a.input, a.output)


if __name__ == "__main__":
    main()
