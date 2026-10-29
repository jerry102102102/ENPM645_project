"""Replay recorded commands, verify synchronized state, and render scene + tactile."""

import argparse
import hashlib
import json
import shutil
import subprocess
from pathlib import Path
from types import SimpleNamespace

import matplotlib
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from phase1a.data.episode_io import DatasetReader, validate_dataset
from phase1a.simulation.scenario_loader import REPO, load_episode_scenario
from phase1a.simulation.transition_world import TransitionWorld

FPS = 20
SIZE = (1280, 900)
FORCE_SCALE = 0.006  # Render-only arrow length in m/N, same for both forces.
COLORS = {"human": (239, 91, 91), "target": (49, 224, 205), "external": (255, 180, 45)}
PAD_ORDER = [("left", 0), ("left", 1), ("right", 0), ("right", 1)]


def font(size):
    path = Path(matplotlib.get_data_path()) / "fonts/ttf/DejaVuSans.ttf"
    return ImageFont.truetype(str(path), size)


def marker(scene, position, color, label="", radius=0.009):
    geom = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(
        geom,
        mujoco.mjtGeom.mjGEOM_SPHERE,
        np.full(3, radius),
        np.asarray(position),
        np.eye(3).ravel(),
        np.array([*color, 255]) / 255,
    )
    geom.label = label
    scene.ngeom += 1
    return geom


def arrow(scene, start, end, color):
    if np.linalg.norm(end - start) < 1e-6:
        return
    geom = marker(scene, start, color)
    mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_ARROW, 0.007, start, end)


def render_scene(renderer, world, reader, index, camera):
    s = world.sim
    state = [v.copy() for v in (s.d.qpos, s.d.qvel, s.d.sensordata)]
    renderer.update_scene(s.d, camera=camera)
    scene = renderer.scene
    for geom in scene.geoms[: scene.ngeom]:
        if geom.objtype == mujoco.mjtObj.mjOBJ_UNKNOWN:
            geom.rgba[3] = 0
    p = reader.sections["privileged"]
    human = p["human_application_point"][index]
    target = p["human_target_trajectory"][index]
    external = p["external_application_point"][index]
    fh = p["human_applied_force"][index]
    fe = p["external_disturbance_force"][index]
    # Offline path annotation, not collision geometry or an input to a policy.
    for point in p["human_target_trajectory"][70:151:5]:
        marker(scene, point, COLORS["target"], radius=0.002)
    marker(scene, target, COLORS["target"], radius=0.009)
    marker(scene, human, COLORS["human"], radius=0.010)
    arrow(scene, human, human + FORCE_SCALE * fh, COLORS["human"])
    marker(scene, external, COLORS["external"], radius=0.005)
    # An incoming arrow terminates at the actual world-space application point.
    arrow(scene, external - FORCE_SCALE * fe, external, COLORS["external"])
    for name, b, color in zip(("L", "R"), s.roots, ((60, 150, 245), (190, 100, 230))):
        marker(scene, s.d.xpos[b] + [0, 0, 0.065], color, name, radius=0.004)
    result = renderer.render().copy()
    for before, after in zip(state, (s.d.qpos, s.d.qvel, s.d.sensordata)):
        np.testing.assert_array_equal(before, after)
    return result


def compose(images, readers, index):
    image = Image.new("RGB", SIZE, "#111923")
    draw = ImageDraw.Draw(image)
    t = float(readers["A"].sections["observations"]["timestamp"][index])
    draw.text(
        (20, 12),
        f"MuJoCo + tactile | t = {t:.2f} s | 1x speed | onset 0.70 s",
        font=font(25),
        fill="white",
    )
    cmap = matplotlib.colormaps["magma"]
    for col, scenario in enumerate("AC"):
        x0 = col * 640
        reader = readers[scenario]
        p, o = reader.sections["privileged"], reader.sections["observations"]
        name = (
            "A: human rotation target"
            if scenario == "A"
            else "C: external push; human target holds"
        )
        draw.text((x0 + 16, 52), name, font=font(20), fill="white")
        image.paste(Image.fromarray(images[scenario]), (x0, 82))
        q = o["object_orientation"][index]
        pitch = np.rad2deg(np.arcsin(np.clip(2 * (q[0] * q[2] - q[3] * q[1]), -1, 1)))
        goal = np.rad2deg(p["human_goal"][index] - p["human_goal"][0])
        fz = p["external_disturbance_force"][index, 2]
        draw.text(
            (x0 + 12, 567),
            f"Target change {goal:+.1f} deg | panel pitch {pitch:+.2f} deg",
            font=font(17),
            fill="#d8e1ed",
        )
        draw.text(
            (x0 + 12, 590),
            f"Human force {np.linalg.norm(p['human_applied_force'][index]):.2f} N | external Fz {fz:+.2f} N",
            font=font(17),
            fill="#d8e1ed",
        )
        for padnum, (side, pad) in enumerate(PAD_ORDER):
            x = x0 + padnum * 160 + 16
            label = f"{side[0].upper()} {'upper' if pad == 0 else 'lower'}"
            draw.text(
                (x, 622),
                label,
                font=font(17),
                fill="#7fbafa" if side == "left" else "#cf97ef",
            )
            values = o["tactile_" + side][index, pad, 0]
            for y in range(4):
                for bx in range(4):
                    value = values[y, bx]
                    rgb = tuple(int(v * 255) for v in cmap(value / 25)[:3])
                    px, py = x + bx * 32, 648 + (3 - y) * 32
                    draw.rectangle(
                        (px, py, px + 31, py + 31), fill=rgb, outline="#50505b"
                    )
                    if value > 0.05:
                        draw.text(
                            (px + 16, py + 16),
                            f"{value:.1f}",
                            anchor="mm",
                            font=font(11),
                            fill="white" if value < 14 else "black",
                        )
            draw.text(
                (x, 783), f"Sum {values.sum():.2f} N", font=font(15), fill="white"
            )
    draw.line((640, 46, 640, 810), fill="#687584", width=1)
    draw.text(
        (16, 819),
        "Red: human point / force    Cyan: virtual hand target / path    Orange: incoming external force",
        font=font(17),
        fill="white",
    )
    draw.text(
        (16, 844),
        "Raw sensor-z maps: 0-25 N/bin, each sensor frame (bin y upward). No averaging; no motion magnification.",
        font=font(16),
        fill="#c6d0dd",
    )
    draw.text(
        (16, 868),
        "Virtual hand only; no articulated hand. Force arrows: 0.006 m/N. Annotations are analysis-only.",
        font=font(16),
        fill="#c6d0dd",
    )
    return image


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--input", type=Path, default=REPO / "results/phase1a/edge_v2/smoke"
    )
    ap.add_argument(
        "--output",
        type=Path,
        default=REPO / "results/phase1a/edge_v2/synchronized_tactile",
    )
    a = ap.parse_args()
    if a.output.exists():
        ap.error("choose a new output folder")
    if not shutil.which("ffmpeg"):
        ap.error("ffmpeg is required to encode the video")
    readers = {s: DatasetReader(a.input / s) for s in "AC"}
    worlds = {}
    for s in "AC":
        validate_dataset(a.input / s)
        worlds[s] = TransitionWorld(load_episode_scenario(readers[s].metadata), s)
    time = readers["A"].sections["observations"]["timestamp"]
    np.testing.assert_array_equal(
        time, readers["C"].sections["observations"]["timestamp"]
    )
    np.testing.assert_allclose(np.diff(time), 0.01, atol=1e-12)
    if len(time) != 400:
        raise ValueError(
            "this comparison expects the frozen four-second smoke episodes"
        )
    a.output.mkdir(parents=True)
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [0, 0.015, 0.74]
    camera.distance = 0.90
    camera.azimuth = 90
    camera.elevation = -25
    renderers = {
        s: mujoco.Renderer(worlds[s].sim.m, height=480, width=640) for s in "AC"
    }
    ffmpeg = subprocess.Popen(
        [
            "ffmpeg",
            "-v",
            "error",
            "-f",
            "rawvideo",
            "-pix_fmt",
            "rgb24",
            "-s",
            "1280x900",
            "-r",
            str(FPS),
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
            str(a.output / "scene_tactile_sync.mp4"),
        ],
        stdin=subprocess.PIPE,
    )
    gif_frames, frame_records = [], []
    checks = 0
    try:
        for i in range(len(time)):
            for s, world in worlds.items():
                o = readers[s].sections["observations"]
                assert world.current.timestamp == o["timestamp"][i]
                assert world.current.physics_tick == o["physics_tick"][i]
                for key, value in world.current.fields.items():
                    np.testing.assert_array_equal(
                        value, o[key][i], err_msg=f"{s}:{i}:{key}"
                    )
                    checks += 1
                p = readers[s].sections["privileged"]
                for key in (
                    "human_goal",
                    "human_target_trajectory",
                    "human_application_point",
                    "external_application_point",
                ):
                    np.testing.assert_array_equal(world.truth[key], p[key][i])
                for key, truthkey in (
                    ("human_applied_force", "human_applied_wrench"),
                    ("external_disturbance_force", "external_disturbance_wrench"),
                ):
                    np.testing.assert_array_equal(world.truth[truthkey][:3], p[key][i])
            if i % 5 == 0:
                images = {
                    s: render_scene(renderers[s], worlds[s], readers[s], i, camera)
                    for s in "AC"
                }
                frame = compose(images, readers, i)
                ffmpeg.stdin.write(frame.tobytes())
                gif_frames.append(frame.resize((960, 675), Image.Resampling.LANCZOS))
                frame_records.append(
                    {
                        "frame": len(frame_records),
                        "observation_index": i,
                        "timestamp_s": float(time[i]),
                        "physics_tick": int(worlds["A"].current.physics_tick),
                    }
                )
                if i in (40, 90, 150, 280):
                    frame.save(a.output / f"frame_{i:03d}.png")
                if i % 100 == 0:
                    print(
                        f"Rendered {time[i]:.2f}s; state and tactile exactly match recordings",
                        flush=True,
                    )
            for s, world in worlds.items():
                commands = readers[s].sections["controller"]
                for step in range(5):
                    world.advance(
                        SimpleNamespace(
                            executed_virtual_drive_wrenches=commands[
                                "executed_virtual_drive_wrenches"
                            ][i, step],
                            executed_actuator_commands=commands[
                                "executed_actuator_commands"
                            ][i, step],
                        )
                    )
    finally:
        ffmpeg.stdin.close()
        status = ffmpeg.wait()
        for renderer in renderers.values():
            renderer.close()
    if status:
        raise RuntimeError(f"ffmpeg failed: {status}")
    gif_frames[0].save(
        a.output / "scene_tactile_sync.gif",
        save_all=True,
        append_images=gif_frames[1:],
        duration=50,
        loop=0,
    )
    report = {
        "passed": True,
        "fps": FPS,
        "duration_s": 4,
        "frames": frame_records,
        "exact_observation_field_comparisons": checks,
        "replay": "500 Hz recorded commands; all 400 pre-decision observations per episode verified exactly; rendering at 20 Hz",
        "force_and_target_annotations": "verified against same-timestamp privileged records; not policy inputs",
        "renderer_state_unchanged": True,
        "force_arrow_scale_m_per_N": FORCE_SCALE,
        "external_arrow": "tail = application_point - scale * force; tip = application_point; world-frame force",
        "human_arrow": "tail = human_application_point; tip = point + scale * applied_force",
        "tactile": "raw sensor-z angular bins; original per-pad frames; 0..25 N/bin; no interpolation or averaging",
        "camera": {
            "lookat": camera.lookat.tolist(),
            "azimuth": camera.azimuth,
            "elevation": camera.elevation,
            "distance": camera.distance,
        },
        "sources": {
            s: {
                name: hashlib.sha256((a.input / s / name).read_bytes()).hexdigest()
                for name in (
                    "metadata.json",
                    "observations.npz",
                    "controller.npz",
                    "privileged.npz",
                )
            }
            for s in "AC"
        },
        "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "limitations": [
            "Virtual spring-damper human point, no articulated hand model",
            "Support-only controller, not a completed cooperative flip",
            "Contact-bin flicker also occurs in HOLD; no intention-classification claim",
        ],
    }
    (a.output / "verification.json").write_text(json.dumps(report, indent=2) + "\n")
    (a.output / "README.md").write_text("""# Synchronized MuJoCo scene and tactile

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
""")
    print(
        f"Verified {checks} field comparisons; saved {len(frame_records)} synchronized frames to {a.output}",
        flush=True,
    )


if __name__ == "__main__":
    main()
