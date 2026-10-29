"""Native viewer; macOS: uv run --locked python -m phase1b.scripts.launch_viewer."""

import argparse
import time

import mujoco.viewer

from phase1b.simulation import PandaWorld


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--scenario", choices=list("ABC"), default="B")
    p.add_argument(
        "--duration",
        type=float,
        default=0,
        help="simulated seconds before closing; 0 keeps the viewer open",
    )
    a = p.parse_args()
    world = PandaWorld(a.scenario)
    world.settle()
    with mujoco.viewer.launch_passive(world.m, world.d) as viewer:
        viewer.cam.lookat[:] = [0, 0.3, 0.65]
        viewer.cam.distance = 1.8
        viewer.cam.azimuth = 90
        viewer.cam.elevation = -25
        while viewer.is_running() and (
            a.duration <= 0 or world.tick * world.m.opt.timestep < a.duration
        ):
            start = time.monotonic()
            world.step()
            viewer.sync()
            time.sleep(max(0, world.m.opt.timestep - (time.monotonic() - start)))

    print(
        f"Native viewer completed {world.tick} physics steps; warnings={world.d.warning.number.tolist()}",
        flush=True,
    )


if __name__ == "__main__":
    main()
