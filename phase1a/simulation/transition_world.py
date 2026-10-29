"""Adapter over unchanged v2 physics; separate observe, command application and step."""

import mujoco
import numpy as np

from phase0_v2.control.baseline import support_commands
from phase0_v2.sensing.contacts import observe
from phase0_v2.simulation.dual_gripper import DualPilot
from phase1a.data.observation_builder import POLICY_SHAPES, SensorObservation
from phase1a.interfaces.controller_contract import ControllerInput, immutable


class TransitionWorld:
    def __init__(self, config, scenario):
        self.config = config
        self.scenario = scenario
        self.sim = DualPilot(dict(config["parameters"]))
        self.dt = self.sim.m.opt.timestep
        self.tick = 0
        for _ in range(round(config["parameters"]["settle_seconds"] / self.dt)):
            _, last = self.sim.forces(-1.0, "B")
            mujoco.mj_step(self.sim.m, self.sim.d)
        self.origin = float(self.sim.d.time)
        self.last_drive = last["executed_virtual_drive_wrenches"].copy()
        self.last_pad = last["executed_actuator_commands"].copy()
        scene = config["scene"]
        self.calibration = {
            k: scene[k]
            for k in (
                "panel",
                "grippers",
                "pad_actuator_limits_N",
                "root_drive_force_limit_N",
                "root_drive_torque_limit_Nm",
                "gravity_world_m_s2",
            )
        }
        self.current, self.truth = self._refresh(self.last_drive, self.last_pad)

    def _set_forces(self, drive, pad):
        s = self.sim
        s.d.ctrl[:] = pad
        truth, _ = s.forces(self.tick * self.dt, self.scenario, disable_robot=True)
        for b, w in zip(s.roots, drive):
            mujoco.mj_applyFT(s.m, s.d, w[:3], w[3:], s.d.xpos[b], b, s.d.qfrc_applied)
        mujoco.mj_forward(s.m, s.d)
        return truth

    def _refresh(self, drive, pad):
        truth = self._set_forces(drive, pad)
        s = self.sim
        o, _ = observe(s)
        velocity = []
        for b in s.roots:
            v = np.zeros(6)
            mujoco.mj_objectVelocity(s.m, s.d, mujoco.mjtObj.mjOBJ_BODY, b, v, 0)
            velocity.append(v)
        fields = {
            k: o[k]
            for k in (
                "wrist_ft_left",
                "wrist_ft_right",
                "tactile_left",
                "tactile_right",
                "contact_state_left",
                "contact_state_right",
            )
        }
        fields.update(
            object_position=o["object_pose"][:3],
            object_orientation=o["object_pose"][3:],
            object_angular_velocity=o["object_velocity"][:3],
            object_linear_velocity=o["object_velocity"][3:],
            wrist_ft_left_world_at_panel_com=o["independent_object_wrenches"][0],
            wrist_ft_right_world_at_panel_com=o["independent_object_wrenches"][1],
            object_contact_wrench=o["aggregated_object_wrench"],
            gripper_pose=o["gripper_pose"],
            gripper_velocity=np.array(velocity),
        )
        if abs(s.d.time - (self.origin + self.tick * self.dt)) > 1e-8:
            raise RuntimeError("engine clock drift/reset")
        return SensorObservation(
            self.tick * self.dt, float(s.d.time), self.tick, fields
        ), truth

    def controller_input(self):
        drive, ff, fb = support_commands(self.sim)
        o = self.current
        return ControllerInput(
            o.timestamp,
            self.dt,
            {k: immutable(o.fields[k]) for k in POLICY_SHAPES},
            immutable(o.fields["gripper_pose"]),
            immutable(o.fields["gripper_velocity"]),
            immutable(drive),
            immutable(self.config["scene"]["pad_preload_commands_N"]),
            immutable(ff),
            immutable(fb),
            self.calibration,
        )

    def advance(self, commands):
        # Exactly one physics step. The caller already chose commands using o_t.
        truth = self._set_forces(
            commands.executed_virtual_drive_wrenches,
            commands.executed_actuator_commands,
        )
        o, diag = observe(self.sim)
        applied = dict(
            contact_wrench_at_actuation=o["aggregated_object_wrench"].copy(),
            actual_pad_actuator_force=self.sim.d.actuator_force.copy(),
            panel_acceleration=diag["panel_acceleration"].copy(),
        )
        total = (
            o["aggregated_object_wrench"]
            + truth["human_applied_wrench"]
            + truth["external_disturbance_wrench"]
        )
        applied["linear_balance_residual_N"] = np.linalg.norm(
            self.sim.m.body_mass[self.sim.panel]
            * (diag["panel_acceleration"] - self.sim.m.opt.gravity)
            - total[:3]
        )
        applied["contact_generalized_error"] = diag["contact_generalized_error"]
        applied["penetration_m"] = diag["penetration"]
        mujoco.mj_step(self.sim.m, self.sim.d)
        self.tick += 1
        self.last_drive = commands.executed_virtual_drive_wrenches.copy()
        self.last_pad = commands.executed_actuator_commands.copy()
        self.current, self.truth = self._refresh(self.last_drive, self.last_pad)
        return truth, applied, self.current
