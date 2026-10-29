"""Compiled hardware snapshot, separate from controller gains in runtime code."""

import hashlib
import json
from pathlib import Path

import mujoco


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def snapshot(world):
    m = world.m
    return {
        "hardware_version": "2.0.0",
        "scenario_version": "3.0.0",
        "description": "Two fixed-base Menagerie Panda arms, upstream position servos, custom two-pad tactile grippers and 0.08 kg adapters",
        "mujoco_version": mujoco.__version__,
        "joint_names": [m.joint(i).name for i in range(m.njnt)],
        "actuator_names": [m.actuator(i).name for i in range(m.nu)],
        "body_names": [m.body(i).name for i in range(m.nbody)],
        "arrays": {
            name: getattr(m, name).tolist()
            for name in (
                "body_parentid",
                "body_pos",
                "body_quat",
                "body_mass",
                "body_inertia",
                "body_ipos",
                "body_iquat",
                "jnt_type",
                "jnt_bodyid",
                "jnt_pos",
                "jnt_axis",
                "jnt_range",
                "jnt_limited",
                "jnt_qposadr",
                "jnt_dofadr",
                "dof_armature",
                "dof_damping",
                "dof_frictionloss",
                "geom_bodyid",
                "geom_type",
                "geom_pos",
                "geom_quat",
                "geom_size",
                "geom_contype",
                "geom_conaffinity",
                "geom_friction",
                "geom_condim",
                "geom_solref",
                "geom_solimp",
                "actuator_trnid",
                "actuator_trntype",
                "actuator_gear",
                "actuator_dyntype",
                "actuator_gaintype",
                "actuator_biastype",
                "actuator_gainprm",
                "actuator_biasprm",
                "actuator_ctrlrange",
                "actuator_forcerange",
                "actuator_ctrllimited",
                "actuator_forcelimited",
                "site_bodyid",
                "site_pos",
                "site_quat",
                "sensor_type",
                "sensor_dim",
                "sensor_objid",
                "sensor_objtype",
            )
        },
        "equality_constraints": m.neq,
        "physics": {
            "timestep": m.opt.timestep,
            "gravity": m.opt.gravity.tolist(),
            "integrator": int(m.opt.integrator),
            "iterations": m.opt.iterations,
            "tolerance": m.opt.tolerance,
        },
        "human": world.p,
        "frames": {
            "grasp": "custom gripper origin between pads",
            "joint_order": world.config["arm_joint_names"],
            "wrist_ft": "reconstructed panel-on-gripper contact wrench, gripper frame about gripper origin",
            "wrist_sensor": "MuJoCo child-parent transmitted wrench at site y=+0.12 in gripper frame; includes downstream dynamics, distinct from panel contact wrench",
            "tactile": "sensor-local z/x/y angular bins, N/bin",
        },
    }


def verify_vendor(root):
    manifest = json.loads((root / "provenance.json").read_text())
    for name, digest in manifest["sha256"].items():
        if sha(root / name) != digest:
            raise ValueError("upstream asset hash mismatch: " + name)
