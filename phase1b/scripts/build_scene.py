"""Build articulated Panda arms with the existing custom tactile end effectors."""

import copy
import hashlib
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
HERE = ROOT / "phase1b"


def build():
    source = ET.parse(HERE / "vendor/franka_emika_panda/panda.xml").getroot()
    scene = ET.parse(ROOT / "phase1a/models/edge_grasp_v2.xml").getroot()
    scene.set("model", "dual_panda_custom_tactile_v1")
    scene.find("compiler").set("meshdir", "../vendor/franka_emika_panda/assets")
    scene.find("compiler").set("autolimits", "true")
    ET.SubElement(scene, "visual")
    ET.SubElement(scene.find("visual"), "global", offwidth="1280", offheight="960")
    ET.SubElement(
        scene.find("visual"), "headlight", ambient=".35 .35 .35", diffuse=".7 .7 .7"
    )
    olddefault = scene.find("default")
    olddefault.append(copy.deepcopy(source.find("./default/default")))
    # Retain upstream collision geometry and enable arm/environment/panel collisions.
    collision = olddefault.find(".//default[@class='collision']/geom")
    collision.set("contype", "16")
    collision.set("conaffinity", "31")
    scene.append(copy.deepcopy(source.find("asset")))
    wb = scene.find("worldbody")
    floor = wb.find("geom[@name='floor']")
    floor.set("contype", "4")
    floor.set("conaffinity", "31")
    panel = wb.find("body[@name='panel']/geom")
    panel.set("conaffinity", "29")
    for side, x in [("left", -0.45), ("right", 0.45)]:
        original = wb.find(f"body[@name='{side}']")
        wb.remove(original)
        original.remove(original.find("freejoint"))
        original.set("childclass", "")
        del original.attrib["childclass"]
        original.set("pos", "0 0 0.12")
        original.set("quat", "0.7071067811865476 -0.7071067811865476 0 0")
        # Reset inherited Panda defaults for the custom hand's pad slide joints.
        for joint in original.iter("joint"):
            joint.set("class", "custom_pad")
        flange = original.find("geom")
        flange.set("contype", "16")
        flange.set("conaffinity", "31")
        ET.SubElement(
            original,
            "geom",
            name=side + "_mount_adapter",
            type="box",
            pos="0 .122 0",
            size=".025 .012 .025",
            mass=".08",
            rgba=".25 .3 .35 1",
            contype="16",
            conaffinity="31",
        )
        ET.SubElement(
            original,
            "site",
            name=side + "_ee",
            pos="0 0 0",
            size=".004",
            rgba="0 0 0 0",
        )
        # Physical wrist sensor includes the downstream hand's inertia and loads.
        ET.SubElement(
            original,
            "site",
            name=side + "_wrist_sensor",
            pos="0 .12 0",
            size=".004",
            rgba="0 0 0 0",
        )
        arm = copy.deepcopy(source.find("./worldbody/body"))
        for node in arm.iter():
            if "name" in node.attrib:
                node.set("name", side + "_" + node.get("name"))
        arm.set("pos", f"{x} .65 .25")
        arm.set("quat", "0.7071067811865476 0 0 -0.7071067811865476")
        link7 = arm.find(f".//body[@name='{side}_link7']")
        link7.remove(link7.find("body"))
        flange_body = ET.SubElement(
            link7, "body", name=side + "_tool_flange", pos="0 0 .107"
        )
        flange_body.append(original)
        wb.append(arm)
        ET.SubElement(
            wb,
            "geom",
            name=side + "_pedestal",
            type="cylinder",
            pos=f"{x} .65 .125",
            size=".11 .125",
            rgba=".22 .25 .29 1",
            contype="4",
            conaffinity="31",
        )
        for a in list(source.find("actuator"))[:7]:
            node = copy.deepcopy(a)
            node.set("name", side + "_" + node.get("name"))
            node.set("joint", side + "_" + node.get("joint"))
            scene.find("actuator").append(node)
        sensors = scene.find("sensor")
        ET.SubElement(
            sensors, "force", name=side + "_wrist_force", site=side + "_wrist_sensor"
        )
        ET.SubElement(
            sensors, "torque", name=side + "_wrist_torque", site=side + "_wrist_sensor"
        )
    # Custom pads must keep the previous armature/friction/joint semantics.
    custom = ET.SubElement(olddefault, "default", {"class": "custom_pad"})
    ET.SubElement(custom, "joint", armature="0", frictionloss="0", stiffness="0")
    contacts = ET.SubElement(scene, "contact")
    for side in ("left", "right"):
        ET.SubElement(contacts, "exclude", body1=side + "_link0", body2=side + "_link1")
    ET.indent(scene, space="  ")
    dest = HERE / "models/dual_panda_v1.xml"
    ET.ElementTree(scene).write(dest, encoding="unicode")
    return dest


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


def main():
    model_path = build()
    m = mujoco.MjModel.from_xml_path(str(model_path))
    d = mujoco.MjData(m)
    old = json.loads((ROOT / "phase1a/config/scenario_v2.json").read_text())
    angle = old["parameters"]["initial_angle"]
    c, s = np.cos(angle), np.sin(angle)
    R = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    pa = m.jnt_qposadr[m.joint("panel_free").id]
    d.qpos[pa : pa + 7] = [0, 0, 0.8, np.cos(angle / 2), 0, np.sin(angle / 2), 0]
    records = {}
    rng = np.random.default_rng(645)
    for side, x in [("left", -0.2), ("right", 0.2)]:
        goal = np.array([0, 0, 0.8]) + R @ np.array([x, 0.1, 0])
        for attempt in range(40):
            seed = (
                np.array([0, -0.5, 0, -2, 0, 1.5, 0.785])
                if attempt == 0
                else np.array(
                    [
                        rng.uniform(-1.5, 1.5),
                        rng.uniform(-1, 1),
                        rng.uniform(-1.5, 1.5),
                        rng.uniform(-2.8, -0.5),
                        rng.uniform(-1.5, 1.5),
                        rng.uniform(0.3, 3),
                        rng.uniform(-2, 2),
                    ]
                )
            )
            result = solve(m, d, side, goal, R, seed)
            if (
                result
                and min(
                    min(
                        d.qpos[m.joint(f"{side}_joint{i}").qposadr[0]]
                        - m.joint(f"{side}_joint{i}").range[0],
                        m.joint(f"{side}_joint{i}").range[1]
                        - d.qpos[m.joint(f"{side}_joint{i}").qposadr[0]],
                    )
                    for i in range(1, 8)
                )
                > 0.12
            ):
                records[side] = result
                break
        else:
            raise RuntimeError("IK failed " + side)
    mujoco.mj_forward(m, d)
    collisions = [
        (
            m.geom(c.geom1).name or str(c.geom1),
            m.geom(c.geom2).name or str(c.geom2),
            float(c.dist),
        )
        for c in d.contact
        if c.dist < -1e-5
    ]
    if collisions:
        raise RuntimeError(f"Initial collision check failed: {collisions}")
    # A named reset pose also makes the authored XML usable in native viewers.
    tree = ET.parse(model_path)
    ctrl = np.zeros(m.nu)
    for side in ("left", "right"):
        for i in range(1, 8):
            ctrl[m.actuator(f"{side}_actuator{i}").id] = d.qpos[
                m.joint(f"{side}_joint{i}").qposadr[0]
            ]
        ctrl[m.actuator(side + "_upper_preload").id] = -40
        ctrl[m.actuator(side + "_lower_preload").id] = 40
    ET.SubElement(
        ET.SubElement(tree.getroot(), "keyframe"),
        "key",
        name="nominal_grasp",
        qpos=" ".join(map(str, d.qpos)),
        ctrl=" ".join(map(str, ctrl)),
    )
    ET.indent(tree, space="  ")
    tree.write(model_path, encoding="unicode")
    result = {
        "scenario_version": "3.0.0",
        "hardware_version": "2.0.0",
        "model": str(model_path.relative_to(ROOT)),
        "model_sha256": hashlib.sha256(model_path.read_bytes()).hexdigest(),
        "qpos": d.qpos.tolist(),
        "ik": records,
        "initial_penetrations": collisions,
        "arm_joint_names": {
            s: [f"{s}_joint{i}" for i in range(1, 8)] for s in ("left", "right")
        },
        "parameters": old["parameters"],
        "source_scenario": "phase1a/config/scenario_v2.json",
        "source_scenario_hash": hashlib.sha256(
            (ROOT / "phase1a/config/scenario_v2.json").read_bytes()
        ).hexdigest(),
        "vendor_manifest_hash": hashlib.sha256(
            (HERE / "vendor/franka_emika_panda/provenance.json").read_bytes()
        ).hexdigest(),
    }
    (HERE / "config/dual_panda_v1.json").write_text(json.dumps(result, indent=2) + "\n")
    print(
        json.dumps(
            {
                "nq": m.nq,
                "nv": m.nv,
                "nu": m.nu,
                "ik": records,
                "penetrations": collisions,
                "qpos": d.qpos.tolist(),
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
