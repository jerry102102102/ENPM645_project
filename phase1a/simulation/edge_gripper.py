"""Edge-grasp variant: declarative placements, existing v2 contacts and support.

Only initialization and the human target geometry change. The inherited v2 force
routine supplies baseline/environment forces; the human term is recomputed from
the actual model site instead of the historical module-level HAND constant.
"""

from pathlib import Path

import mujoco
import numpy as np

from phase0_tactile.scripts.run_experiment import clip_norm, rotate_y, smooth
from phase0_v2.simulation.dual_gripper import SIDES, DualPilot, point_wrench


class EdgeGraspPilot(DualPilot):
    def __init__(self, parameters, model_path):
        self.p = dict(parameters)
        self.m = mujoco.MjModel.from_xml_path(str(Path(model_path)))
        self.d = mujoco.MjData(self.m)
        m, d, p = self.m, self.d, self.p
        m.opt.timestep = p["timestep"]
        m.geom_friction[:, 0] = p["friction"]
        self.panel = m.body("panel").id
        self.pg = m.geom("panel_geom").id
        self.roots = [m.body(side).id for side in SIDES]
        self.pad_names = [
            [side + "_" + pad for pad in ("upper", "lower")] for side in SIDES
        ]
        self.pad_g = [
            [m.geom(name + "_pad").id for name in names] for names in self.pad_names
        ]
        self.pad_b = [[m.body(name).id for name in names] for names in self.pad_names]
        self.gripper_masses = [
            float(m.body_mass[root] + sum(m.body_mass[k] for k in pads))
            for root, pads in zip(self.roots, self.pad_b)
        ]
        self.center = m.body_pos[self.panel].copy()
        self.reference_rot = rotate_y(p["initial_angle"])
        self.anchors = [
            self.center + self.reference_rot @ (m.body_pos[root] - self.center)
            for root in self.roots
        ]
        for name, pos in [
            ("left", self.anchors[0]),
            ("right", self.anchors[1]),
            ("panel", self.center),
        ]:
            q = m.jnt_qposadr[m.joint(name + "_free").id]
            angle = p["initial_angle"]
            d.qpos[q : q + 3] = pos
            d.qpos[q + 3 : q + 7] = [np.cos(angle / 2), 0, np.sin(angle / 2), 0]
        self.vadr = m.jnt_dofadr[m.joint("panel_free").id]
        self.hsite = m.site("human_grasp").id
        self.human_local = m.site_pos[self.hsite].copy()
        d.ctrl[:] = [-p["preload"], p["preload"]] * 2
        mujoco.mj_forward(m, d)

    def forces(
        self,
        t,
        scenario,
        *,
        disable_human=False,
        disable_robot=False,
        wrench_override=None,
    ):
        truth, control = super().forces(
            t,
            scenario,
            disable_human=True,
            disable_robot=disable_robot,
            wrench_override=wrench_override,
        )
        m, d, p = self.m, self.d, self.p
        u, du = smooth(t, 0.7, p["ramp_duration"])
        delta = p["direction"] * p["target_angle"] if scenario == "A" else 0.0
        goal = p["initial_angle"] + delta * u
        radius = rotate_y(goal) @ self.human_local
        target = self.center + radius
        target_velocity = clip_norm(
            np.cross([0.0, delta * du, 0.0], radius), p["human_target_speed_limit"]
        )
        point = d.site_xpos[self.hsite].copy()
        jac = np.zeros((3, m.nv))
        mujoco.mj_jacSite(m, d, jac, None, self.hsite)
        force = clip_norm(
            p["kh"] * (target - point) + p["dh"] * (target_velocity - jac @ d.qvel),
            p["human_force_limit"],
        )
        if disable_human:
            force[:] = 0.0
        if wrench_override is None:
            mujoco.mj_applyFT(
                m, d, force, np.zeros(3), point, self.panel, d.qfrc_applied
            )
        mujoco.mj_forward(m, d)
        truth.update(
            human_goal=goal,
            human_target_trajectory=target,
            human_target_velocity=target_velocity,
            human_applied_wrench=point_wrench(force, point, d.xipos[self.panel]),
            human_application_point=point,
        )
        return truth, control
