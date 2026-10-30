"""Contact-derived F/T, touch grids and physical diagnostics."""

import mujoco
import numpy as np

SIDES = ("left", "right")


def observe(sim):
    m, d = sim.m, sim.d
    wlocal = np.zeros((2, 6))
    wobject = np.zeros((2, 6))
    qcontact = np.zeros(m.nv)
    grid = np.zeros((2, 2, 3, 4, 4))
    count = np.zeros((2, 2), int)
    normal = np.zeros((2, 2))
    copnum = np.zeros((2, 2, 2))
    slip = np.zeros(2)
    raw = []
    depth = 0.0
    force = np.zeros(6)
    com = d.xipos[sim.panel]
    for ci in range(d.ncon):
        c = d.contact[ci]
        if sim.pg not in (c.geom1, c.geom2):
            continue
        other = c.geom2 if c.geom1 == sim.pg else c.geom1
        found = [(g, k) for g in range(2) for k in range(2) if sim.pad_g[g][k] == other]
        if not found:
            continue
        g, k = found[0]
        mujoco.mj_contactForce(m, d, ci, force)
        if force[0] <= 1e-8:
            continue
        f = c.frame.reshape(3, 3).T @ force[:3] * (1 if c.geom2 == other else -1)
        torque = c.frame.reshape(3, 3).T @ force[3:] * (1 if c.geom2 == other else -1)
        root = sim.roots[g]
        rr = d.xmat[root].reshape(3, 3)
        pad = sim.pad_b[g][k]
        pr = d.xmat[pad].reshape(3, 3)
        pos = pr.T @ (c.pos - d.xpos[pad])
        pf = pr.T @ f
        ix, iy = np.clip(np.floor((pos[:2] / [0.09, 0.14] + 0.5) * 4).astype(int), 0, 3)
        grid[g, k, :, iy, ix] += [force[0], pf[0], pf[1]]
        count[g, k] += 1
        normal[g, k] += force[0]
        copnum[g, k] += force[0] * pos[:2]
        wlocal[g, :3] += rr.T @ f
        wlocal[g, 3:] += rr.T @ (np.cross(c.pos - d.xpos[root], f) + torque)
        wobject[g, :3] -= f
        wobject[g, 3:] -= np.cross(c.pos - com, f) + torque
        mujoco.mj_applyFT(m, d, -f, -torque, c.pos, sim.panel, qcontact)
        j1 = np.zeros((3, m.nv))
        j2 = np.zeros_like(j1)
        mujoco.mj_jac(m, d, j1, None, c.pos, sim.panel)
        mujoco.mj_jac(m, d, j2, None, c.pos, pad)
        rel = (j1 - j2) @ d.qvel
        n = c.frame.reshape(3, 3)[0]
        v = float(np.linalg.norm(rel - n * (rel @ n)))
        slip[g] = max(slip[g], v)
        depth = max(depth, -float(c.dist))
        raw.append([g, k, *pos, *c.pos, *pf, force[0], v, c.dist])
    cop = np.divide(
        copnum,
        normal[:, :, None],
        out=np.full((2, 2, 2), np.nan),
        where=normal[:, :, None] > 0,
    )
    vel = np.zeros(6)
    mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, sim.panel, vel, 0)
    R = d.xmat[sim.panel].reshape(3, 3)
    o = dict(
        object_pose=np.r_[d.xpos[sim.panel], d.xquat[sim.panel]],
        object_velocity=vel,
        object_rotation_matrix=R.copy(),
        pitch=np.arctan2(-R[2, 0], R[0, 0]),
        aggregated_object_wrench=wobject.sum(axis=0),
        independent_object_wrenches=wobject,
        gripper_pose=np.array([np.r_[d.xpos[b], d.xquat[b]] for b in sim.roots]),
    )
    for g, s in enumerate(SIDES):
        o["wrist_ft_" + s] = wlocal[g]
        o["tactile_" + s] = np.array(
            [
                d.sensor(n + "_grid").data.reshape(3, 4, 4).copy()
                for n in sim.pad_names[g]
            ]
        )
        o["planar_tactile_" + s] = grid[g]
        o["contact_state_" + s] = count[g]
        o["normal_load_" + s] = normal[g]
        o["contact_center_" + s] = cop[g]
        o["slip_speed_" + s] = slip[g]
    # Check frame and moment-arm transforms independently of direct contact summation.
    transform = []
    for g, body in enumerate(sim.roots):
        Rg = d.xmat[body].reshape(3, 3)
        f = -Rg @ wlocal[g, :3]
        transform.append(
            np.r_[f, -Rg @ wlocal[g, 3:] + np.cross(d.xpos[body] - com, f)]
        )
    diag = dict(
        penetration=depth,
        panel_acceleration=d.qacc[sim.vadr : sim.vadr + 3].copy(),
        contact_generalized_error=np.max(
            np.abs(
                qcontact[sim.vadr : sim.vadr + 6]
                - d.qfrc_constraint[sim.vadr : sim.vadr + 6]
            )
        ),
        wrench_transform_error=np.max(np.abs(np.array(transform) - wobject)),
        raw_contacts=raw,
    )
    return o, diag
