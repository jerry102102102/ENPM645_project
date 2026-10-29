"""Frame capture from recorded articulated poses, synchronized with joint/tactile data."""

import argparse
import json
import shutil
import subprocess
from pathlib import Path
from types import SimpleNamespace

import matplotlib
import mujoco
import numpy as np
from PIL import Image, ImageDraw

from phase1a.scripts.render_tactile_sync import compose, font, render_scene
from phase1b.pipeline import read_episode, sha, validate_episode
from phase1b.simulation import ROOT, PandaWorld


def render(source, output):
    source, output = Path(source), Path(output)
    if output.exists():
        raise FileExistsError(output)
    if not shutil.which("ffmpeg"):
        raise RuntimeError("ffmpeg is required for video encoding")
    records = {}
    readers = {}
    worlds = {}
    for s in "AC":
        validate_episode(source / s)
        records[s], _ = read_episode(source / s)
        readers[s] = SimpleNamespace(sections=records[s])
        worlds[s] = PandaWorld(s)
    np.testing.assert_array_equal(
        records["A"]["observations"]["timestamp"],
        records["C"]["observations"]["timestamp"],
    )
    normal_limit = float(
        5
        * np.ceil(
            max(
                np.max(records[s]["observations"]["tactile_" + side][:, :, 0])
                for s in "AC"
                for side in ("left", "right")
            )
            / 5
        )
    )
    output.mkdir(parents=True)
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [0, 0.3, 0.65]
    camera.distance = 1.8
    camera.azimuth = 90
    camera.elevation = -25
    renderers = {s: mujoco.Renderer(worlds[s].m, 480, 640) for s in "AC"}
    proc = subprocess.Popen(
        [
            "ffmpeg",
            "-v",
            "error",
            "-f",
            "rawvideo",
            "-pix_fmt",
            "rgb24",
            "-s",
            "1280x1080",
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
            str(output / "dual_panda_sync.mp4"),
        ],
        stdin=subprocess.PIPE,
    )
    frames = []
    timeline = []
    try:
        for i in range(0, len(records["A"]["observations"]["timestamp"]), 5):
            images = {}
            for s, w in worlds.items():
                o = records[s]["observations"]
                state = records[s]["render_states"]
                w.d.qpos[:] = state["qpos"][i]
                w.d.qvel[:] = state["qvel"][i]
                w.d.time = o["engine_timestamp"][i]
                mujoco.mj_forward(w.m, w.d)
                np.testing.assert_allclose(
                    w.d.site_xpos[w.hsite],
                    records[s]["privileged"]["human_application_point"][i],
                    atol=1e-12,
                )
                for g, b in enumerate(w.roots):
                    np.testing.assert_allclose(
                        w.d.xpos[b], o["end_effector_pose"][i, g, :3], atol=1e-12
                    )
                images[s] = render_scene(
                    renderers[s], SimpleNamespace(sim=w), readers[s], i, camera
                )
            base = compose(images, readers, i)
            frame = Image.new("RGB", (1280, 1080), "#111923")
            frame.paste(base, (0, 0))
            draw = ImageDraw.Draw(frame)
            # The articulated runs can exceed the older plot's fixed 25 N/bin range.
            # Redraw only the heatmaps with a common, unclipped range for this dataset.
            for col, s in enumerate("AC"):
                for padnum, (side, pad) in enumerate(
                    (("left", 0), ("left", 1), ("right", 0), ("right", 1))
                ):
                    values = records[s]["observations"]["tactile_" + side][i, pad, 0]
                    for y in range(4):
                        for x in range(4):
                            value = values[y, x]
                            rgb = tuple(
                                int(v * 255)
                                for v in matplotlib.colormaps["magma"](
                                    value / normal_limit
                                )[:3]
                            )
                            px, py = (
                                col * 640 + padnum * 160 + 16 + x * 32,
                                648 + (3 - y) * 32,
                            )
                            draw.rectangle(
                                (px, py, px + 31, py + 31), fill=rgb, outline="#50505b"
                            )
                            if value > 0.05:
                                draw.text(
                                    (px + 16, py + 16),
                                    f"{value:.1f}",
                                    anchor="mm",
                                    font=font(11),
                                    fill="white"
                                    if value < normal_limit * 0.55
                                    else "black",
                                )
            draw.rectangle((0, 840, 1279, 866), fill="#111923")
            draw.text(
                (16, 844),
                f"Raw sensor-z maps: 0-{normal_limit:.0f} N/bin, each sensor frame (bin y upward). No averaging; no motion magnification.",
                font=font(16),
                fill="#c6d0dd",
            )
            draw.rectangle((0, 0, 1279, 45), fill="#111923")
            t = float(records["A"]["observations"]["timestamp"][i])
            draw.text(
                (20, 12),
                f"Dual Panda | t = {t:.2f} s | 20 fps / 1x | joint-position hold baseline",
                font=font(23),
                fill="white",
            )
            for col, s in enumerate("AC"):
                o = records[s]["observations"]
                x = col * 640 + 16
                draw.text(
                    (x, 907),
                    "Joints 1 ... 7 (rad), measured during physical simulation",
                    font=font(16),
                    fill="white",
                )
                for g, side in enumerate(("L", "R")):
                    q = " ".join(f"{v:+.3f}" for v in o["arm_joint_position"][i, g])
                    draw.text(
                        (x, 936 + g * 27),
                        f"{side} q:  {q}",
                        font=font(17),
                        fill="#7fbafa" if g == 0 else "#cf97ef",
                    )
                speed = np.max(np.abs(o["arm_joint_velocity"][i]))
                torque = np.max(np.abs(o["arm_actuator_torque"][i]))
                draw.text(
                    (x, 997),
                    f"max |dq| {speed:.4f} rad/s   max |actuator torque| {torque:.2f} Nm",
                    font=font(16),
                    fill="#d8e1ed",
                )
            draw.text(
                (16, 1040),
                "All 14 q/dq and actuator torques logged. Joint states excluded from default policy input. No assistance tracking.",
                font=font(17),
                fill="white",
            )
            proc.stdin.write(frame.tobytes())
            frames.append(frame.resize((960, 810), Image.Resampling.LANCZOS))
            timeline.append(
                {
                    "frame": len(timeline),
                    "observation_index": i,
                    "timestamp_s": t,
                    "physics_tick": int(
                        records["A"]["observations"]["physics_tick"][i]
                    ),
                }
            )
            if i in (0, 90, 150, 280):
                frame.save(output / f"frame_{i:03d}.png")
    finally:
        proc.stdin.close()
        status = proc.wait()
        for r in renderers.values():
            r.close()
    if status:
        raise RuntimeError("ffmpeg failed")
    frames[0].save(
        output / "dual_panda_sync.gif",
        save_all=True,
        append_images=frames[1:],
        duration=50,
        loop=0,
    )
    report = {
        "tactile_color_range_N_per_bin": [0, normal_limit],
        "frames": timeline,
        "fps": 20,
        "render_method": "offline native MuJoCo FK from recorded qpos/qvel; no dynamics rerun; heatmaps use same-sample raw recorded tactile",
        "hand_and_ee_fk_verified": True,
        "source_hashes": {
            s: {
                name: sha(source / s / name)
                for name in (
                    "observations.npz",
                    "privileged.npz",
                    "render_states.npz",
                    "metadata.json",
                )
            }
            for s in "AC"
        },
        "script_hash": sha(__file__),
        "shared_render_helpers_hash": sha(
            ROOT / "phase1a/scripts/render_tactile_sync.py"
        ),
    }
    (output / "frame_capture.json").write_text(json.dumps(report, indent=2) + "\n")
    # Native scene alone, to inspect the full mechanical assembly clearly.
    w = PandaWorld()
    w.settle()
    with mujoco.Renderer(w.m, 960, 1280) as r:
        r.update_scene(w.d, camera=camera)
        for geom in r.scene.geoms[: r.scene.ngeom]:
            if geom.objtype == mujoco.mjtObj.mjOBJ_UNKNOWN:
                geom.rgba[3] = 0
        Image.fromarray(r.render()).save(output / "dual_panda_scene.png")
    print("Saved synchronized dual-Panda frame capture:", output, flush=True)


def main():
    p = argparse.ArgumentParser()
    p.add_argument(
        "--input", type=Path, default=ROOT / "results/phase1b/dual_panda_v1/smoke"
    )
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    render(a.input, a.output)


if __name__ == "__main__":
    main()
