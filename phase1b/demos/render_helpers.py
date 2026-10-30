"""Shared native rendering annotations."""

from pathlib import Path

import matplotlib
import mujoco
import numpy as np
from PIL import ImageFont


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
