"""Compiled simulation hardware specification, separate from controller logic."""

import hashlib
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np


def number(value):
    return np.asarray(value).round(12).tolist()


def hardware_snapshot(sim, config, model_path):
    m = sim.m
    grippers = {}
    for index, side in enumerate(("left", "right")):
        root = sim.roots[index]
        body_ids = [root, *sim.pad_b[index]]
        bodies = {}
        for body in body_ids:
            name = m.body(body).name
            joint = int(m.body_jntadr[body])
            dof = int(m.jnt_dofadr[joint])
            ndof = 6 if m.jnt_type[joint] == 0 else 1
            geometries = {}
            for gid in np.flatnonzero(m.geom_bodyid == body):
                geometries[m.geom(gid).name] = {
                    "type": int(m.geom_type[gid]),
                    "half_size_m": number(m.geom_size[gid]),
                    "body_local_position_m": number(m.geom_pos[gid]),
                    "body_local_quaternion_wxyz": number(m.geom_quat[gid]),
                    "friction": number(m.geom_friction[gid]),
                    "condim": int(m.geom_condim[gid]),
                    "contype": int(m.geom_contype[gid]),
                    "conaffinity": int(m.geom_conaffinity[gid]),
                    "solref": number(m.geom_solref[gid]),
                    "solimp": number(m.geom_solimp[gid]),
                }
            bodies[name] = {
                "mass_kg": float(m.body_mass[body]),
                "principal_inertia_kg_m2": number(m.body_inertia[body]),
                "inertial_position_body_local_m": number(m.body_ipos[body]),
                "inertial_quaternion_wxyz": number(m.body_iquat[body]),
                "joint": {
                    "name": m.joint(joint).name,
                    "type": int(m.jnt_type[joint]),
                    "axis": number(m.jnt_axis[joint]),
                    "limited": bool(m.jnt_limited[joint]),
                    "range_m": number(m.jnt_range[joint]),
                    "damping": number(m.dof_damping[dof : dof + ndof]),
                    "armature": number(m.dof_armature[dof : dof + ndof]),
                    "frictionloss": number(m.dof_frictionloss[dof : dof + ndof]),
                    "stiffness": float(m.jnt_stiffness[joint]),
                },
                "geometries": geometries,
            }
        grippers[side] = {
            "kind": "simplified free-root two-pad gripper; not a complete arm",
            "total_mass_kg": float(sum(m.body_mass[b] for b in body_ids)),
            "nominal_frame_panel_local_m": config["scene"]["grippers"][side][
                "nominal_grasp_position_panel_local"
            ],
            "nominal_frame_quaternion_panel_local_wxyz": config["scene"]["grippers"][
                side
            ]["nominal_grasp_quaternion_panel_local_wxyz"],
            "bodies": bodies,
        }
    motors = {}
    for aid in range(m.nu):
        motors[m.actuator(aid).name] = {
            "joint": m.joint(int(m.actuator_trnid[aid, 0])).name,
            "transmission_type": int(m.actuator_trntype[aid]),
            "gear": number(m.actuator_gear[aid]),
            "ctrl_limited": bool(m.actuator_ctrllimited[aid]),
            "ctrl_range_N": number(m.actuator_ctrlrange[aid]),
            "force_limited": bool(m.actuator_forcelimited[aid]),
            "force_range_if_limited": number(m.actuator_forcerange[aid]),
            "gain_parameters": number(m.actuator_gainprm[aid]),
            "bias_parameters": number(m.actuator_biasprm[aid]),
        }
    source = ET.parse(model_path).getroot()
    tactile = {}
    for sensor in source.findall("./sensor/plugin"):
        name = sensor.get("name")
        sid = m.sensor(name).id
        site = int(m.sensor_objid[sid])
        tactile[name] = {
            "implementation": sensor.get("plugin"),
            "source_plugin_config": {
                c.get("key"): c.get("value") for c in sensor.findall("config")
            },
            "compiled_dimension": int(m.sensor_dim[sid]),
            "site_name": m.site(site).name,
            "site_position_pad_local_m": number(m.site_pos[site]),
            "site_quaternion_pad_local_wxyz": number(m.site_quat[site]),
        }
    return {
        "robot_grippers": grippers,
        "pad_motors": motors,
        "nominal_preload_commands_N": config["scene"]["pad_preload_commands_N"],
        "root_drive_limits": {
            "force_norm_N": config["scene"]["root_drive_force_limit_N"],
            "torque_norm_Nm": config["scene"]["root_drive_torque_limit_Nm"],
            "kind": "simulation virtual wrench drive, not a commercial motor rating",
        },
        "tactile": tactile,
        "wrist_ft": {
            "kind": "independent simulated contact-wrench reconstruction per gripper",
            "order": "force xyz then torque xyz",
            "frame": "root-local, about root; panel-on-gripper",
        },
        "human": {
            "kind": "single virtual spring-damper interaction point; no hand collision geometry",
            "count": 1,
            "attachment_body": "panel",
            "attachment_panel_local_m": number(m.site_pos[sim.hsite]),
            "marker_radius_m": float(m.site_size[sim.hsite, 0]),
            "marker_is_collision_geometry": False,
            "rigid_hand_mass_kg": None,
            "stiffness_N_m": sim.p["kh"],
            "damping_N_s_m": sim.p["dh"],
            "force_norm_limit_N": sim.p["human_force_limit"],
            "target_speed_limit_m_s": sim.p["human_target_speed_limit"],
        },
        "fixed_geometry_semantics": config["grasp_semantics"],
        "equality_constraints": int(m.neq),
    }


def validate_hardware(sim, config, model_path, profile_path, expected_hash):
    path = Path(profile_path)
    content = path.read_bytes()
    if hashlib.sha256(content).hexdigest() != expected_hash:
        raise ValueError(
            "Frozen hardware file hash mismatch; create a new hardware version"
        )
    profile = json.loads(content)
    from phase1a.simulation.scenario_loader import config_hash

    if profile["config_hash"] != config_hash(config):
        raise ValueError("Hardware/config binding mismatch")
    if profile["model_hash"] != config["model_hash"]:
        raise ValueError("Hardware/model binding mismatch")
    if hardware_snapshot(sim, config, model_path) != profile["hardware"]:
        raise ValueError("Compiled hardware differs from frozen specification")
    return {
        "hardware_id": profile["hardware_id"],
        "hardware_version": profile["hardware_version"],
        "hardware_hash": expected_hash,
    }
