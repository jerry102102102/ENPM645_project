"""Independent bounded virtual wrench drives, not a desired-object-wrench allocator."""
import mujoco
import numpy as np
from phase0_tactile.scripts.run_experiment import clip_norm

MODE = 'independent_bounded_support_only'
KPOS = np.array([100., 100., 35.])
DPOS = np.array([8., 8., 5.])
KROT = np.array([6., .08, 6.])
DROT = np.array([.5, .08, .5])
FORCE_LIMIT = 35.
TORQUE_LIMIT = 3.


def support_commands(sim, disabled=False):
    """World-frame force/torque applied at each root origin, saved separately from contacts."""
    commands, feedforward, feedback = [], [], []
    for i, body in enumerate(sim.roots):
        velocity = np.zeros(6)
        mujoco.mj_objectVelocity(sim.m, sim.d, mujoco.mjtObj.mjOBJ_BODY, body, velocity, 0)
        rot = sim.d.xmat[body].reshape(3, 3)
        error = .5*sum((np.cross(rot[:,j], sim.reference_rot[:,j]) for j in range(3)), np.zeros(3))
        gravity_torque = sum((np.cross(sim.d.xipos[k]-sim.d.xpos[body], -sim.m.body_mass[k]*sim.m.opt.gravity) for k in [body, *sim.pad_b[i]]), np.zeros(3))
        ff = np.r_[-(sim.gripper_masses[i]+.5*sim.m.body_mass[sim.panel])*sim.m.opt.gravity, gravity_torque]
        fb = np.r_[KPOS*(sim.anchors[i]-sim.d.xpos[body])-DPOS*velocity[3:], KROT*error-DROT*velocity[:3]]
        cmd = np.r_[clip_norm((ff+fb)[:3], FORCE_LIMIT), clip_norm((ff+fb)[3:], TORQUE_LIMIT)]
        if disabled: cmd[:] = 0
        commands.append(cmd);feedforward.append(ff);feedback.append(fb)
    return np.array(commands), np.array(feedforward), np.array(feedback)
