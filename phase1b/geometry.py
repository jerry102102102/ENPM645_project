"""Shared panel geometry and demo IK; no runtime pose overrides."""

import mujoco
import numpy as np


def rotate_y(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def clip_norm(v, limit):
    return v * min(1.0, limit / max(np.linalg.norm(v), 1e-12))


def smooth(t, start, duration):
    u = np.clip((t - start) / duration, 0.0, 1.0)
    return u * u * (3 - 2 * u), (6 * u * (1 - u) / duration if 0 < u < 1 else 0.0)


def point_wrench(force, point, origin):
    return np.r_[force, np.cross(point - origin, force)]


def rotation_error(target, current):
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, (target @ current.T).ravel())
    if q[0] < 0:
        q = -q
    norm = np.linalg.norm(q[1:])
    return 2 * q[1:] if norm < 1e-10 else q[1:] / norm * (2 * np.arctan2(norm, q[0]))


def solve(m, d, side, target_pos, target_rot, seed):
    joints = np.array([m.joint(f"{side}_joint{i}").id for i in range(1, 8)])
    qa = m.jnt_qposadr[joints]
    va = m.jnt_dofadr[joints]
    limits = m.jnt_range[joints]
    site = m.site(side + "_ee").id
    d.qpos[qa] = seed
    for iteration in range(600):
        mujoco.mj_forward(m, d)
        err = np.r_[
            target_pos - d.site_xpos[site],
            rotation_error(target_rot, d.site_xmat[site].reshape(3, 3)),
        ]
        if np.linalg.norm(err[:3]) < 1e-6 and np.linalg.norm(err[3:]) < 1e-5:
            return {
                "position_error_m": float(np.linalg.norm(err[:3])),
                "orientation_error_rad": float(np.linalg.norm(err[3:])),
                "iterations": iteration,
            }
        jp = np.zeros((3, m.nv))
        jr = jp.copy()
        mujoco.mj_jacSite(m, d, jp, jr, site)
        j = np.vstack((jp[:, va], jr[:, va]))
        dq = j.T @ np.linalg.solve(j @ j.T + 1e-5 * np.eye(6), err)
        dq *= min(1, 0.15 / max(np.linalg.norm(dq), 1e-12))
        d.qpos[qa] = np.clip(d.qpos[qa] + dq, limits[:, 0] + 0.01, limits[:, 1] - 0.01)
    return None
