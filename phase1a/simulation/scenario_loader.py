"""A locked, extracted snapshot of the existing v2 model and validated block 0."""

import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np

from phase0_v2.control import baseline
from phase0_v2.simulation.dual_gripper import CENTER, HAND, MODEL, DualPilot

REPO = Path(__file__).resolve().parents[2]
CONFIG = REPO / "phase1a/config/scenario_v1.json"
EDGE_CONFIG = REPO / "phase1a/config/scenario_v2.json"
REGISTRY = CONFIG.parent / "scenario_registry.json"
DEPENDENCIES = [
    "phase0_v2/simulation/dual_gripper.py",
    "phase0_v2/control/baseline.py",
    "phase0_v2/sensing/contacts.py",
    "phase0_tactile/scripts/run_experiment.py",
]


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def config_hash(config):
    return hashlib.sha256(
        json.dumps(
            config, sort_keys=True, separators=(",", ":"), allow_nan=False
        ).encode()
    ).hexdigest()


def values(a):
    return np.asarray(a).round(12).tolist()


def extract_scene(sim):
    m, d = sim.m, sim.d
    panel = sim.panel
    R = d.xmat[panel].reshape(3, 3)

    def geom(name):
        g = m.geom(name).id
        return dict(
            type=int(m.geom_type[g]),
            half_size_m=values(m.geom_size[g]),
            local_position=values(m.geom_pos[g]),
            local_quaternion_wxyz=values(m.geom_quat[g]),
            friction=values(m.geom_friction[g]),
            condim=int(m.geom_condim[g]),
            contype=int(m.geom_contype[g]),
            conaffinity=int(m.geom_conaffinity[g]),
            solref=values(m.geom_solref[g]),
            solimp=values(m.geom_solimp[g]),
        )

    grippers = {}
    for i, side in enumerate(("left", "right")):
        b = sim.roots[i]
        rb = R.T @ d.xmat[b].reshape(3, 3)
        q = np.zeros(4)
        mujoco.mju_mat2Quat(q, rb.ravel())
        pads = {}
        for name in sim.pad_names[i]:
            body = m.body(name).id
            site = m.site(name + "_touch").id
            j = m.joint(name + "_slide").id
            pads[name] = dict(
                geometry=geom(name + "_pad"),
                body_position_in_root=values(m.body_pos[body]),
                sensor_frame=dict(
                    name=name + "_touch",
                    parent=name,
                    position_in_pad=values(m.site_pos[site]),
                    quaternion_wxyz_in_pad=values(m.site_quat[site]),
                ),
                slide_range_m=values(m.jnt_range[j]),
            )
        grippers[side] = dict(
            nominal_grasp_position_panel_local=values(
                R.T @ (d.xpos[b] - d.xpos[panel])
            ),
            nominal_grasp_quaternion_panel_local_wxyz=values(q),
            root_joint_type=int(m.jnt_type[m.body_jntadr[b]]),
            initial_world_pose=values(np.r_[d.xpos[b], d.xquat[b]]),
            pads=pads,
        )
    return dict(
        panel=dict(
            dimensions_m=values(2 * m.geom_size[sim.pg]),
            mass_kg=float(m.body_mass[panel]),
            principal_inertia_kg_m2=values(m.body_inertia[panel]),
            inertial_quaternion_wxyz=values(m.body_iquat[panel]),
            initial_world_pose=values(np.r_[d.xpos[panel], d.xquat[panel]]),
            collision=geom("panel_geom"),
        ),
        grippers=grippers,
        human_grasp_point_panel_local=values(m.site_pos[sim.hsite]),
        pad_preload_commands_N=values(d.ctrl),
        pad_actuator_limits_N=values(m.actuator_ctrlrange),
        root_drive_force_limit_N=baseline.FORCE_LIMIT,
        root_drive_torque_limit_Nm=baseline.TORQUE_LIMIT,
        baseline_gains=dict(
            kpos=values(baseline.KPOS),
            dpos=values(baseline.DPOS),
            krot=values(baseline.KROT),
            drot=values(baseline.DROT),
        ),
        gravity_world_m_s2=values(m.opt.gravity),
        physics_dt=float(m.opt.timestep),
        equality_constraints=int(m.neq),
    )


def freeze():
    if CONFIG.exists() or REGISTRY.exists():
        raise FileExistsError("Frozen v1 exists; create a new scenario version instead")
    from phase0_v2.simulation.dual_gripper import parameters

    p = parameters(0, 64600)
    source = REPO / "phase0_v2/results/pilot/data/A_000/metadata.json"
    record = json.loads(source.read_text())
    if any(record[k] != v for k, v in p.items()):
        raise ValueError("v2 parameter generator changed from validated block 0")
    sim = DualPilot(p)
    config = dict(
        scenario_id="enpm645-dual-gripper",
        scenario_version="1.0.0",
        model_path=str(MODEL.relative_to(REPO)),
        model_hash=digest(MODEL),
        nominal_parameter_source=str(source.relative_to(REPO)),
        nominal_parameter_source_hash=digest(source),
        parameters=p,
        scene=extract_scene(sim),
        implementation_hashes={p: digest(REPO / p) for p in DEPENDENCIES},
        human=dict(
            model="single point spring-damper; F=Kh*(target-point)+Dh*(target_velocity-point_velocity)",
            stiffness_N_m=p["kh"],
            damping_N_s_m=p["dh"],
            force_norm_limit_N=p["human_force_limit"],
            target_speed_limit_m_s=p["human_target_speed_limit"],
        ),
        disturbance=dict(
            application_point_panel_local=p["disturbance_point"],
            world_direction=[0, 0, -p["direction"]],
            magnitude_N=p["disturbance_magnitude"],
            start_s=p["disturbance_start"],
            end_s=p["disturbance_end"],
            ramp_s=0.15,
            scope="fixed v2 block-0 vertical force; future magnitude/time changes must be explicit episode parameters",
        ),
        tasks={
            "A": {"name": "ROTATE", "human_goal": "ROTATE", "disturbance": False},
            "B": {"name": "HOLD", "human_goal": "HOLD", "disturbance": False},
            "C": {"name": "RESIST", "human_goal": "HOLD", "disturbance": True},
        },
        timing=dict(
            reference_hz=100,
            logging_hz=100,
            controller_hz=500,
            physics_steps_per_transition=5,
        ),
        scripted_reference=dict(
            amplitude_Nm=0.02,
            source="scripted_test",
            schedule_seconds=[[0, 1, 0], [1, 2, 1], [2, 3, 0], [3, 4, -1], [4, 5, 0]],
            interpretation="data-flow test only, not executed by support_only; attainability not verified",
        ),
    )
    CONFIG.write_text(json.dumps(config, indent=2) + "\n")
    key = config["scenario_id"] + "@" + config["scenario_version"]
    REGISTRY.write_text(
        json.dumps(
            {key: {"config_hash": config_hash(config), "model_hash": digest(MODEL)}},
            indent=2,
        )
        + "\n"
    )


def make_sim(config):
    variant = config.get("simulation_variant", "legacy_v2")
    if variant == "legacy_v2":
        return DualPilot(dict(config["parameters"]))
    if variant == "edge_grasp_v2":
        from phase1a.simulation.edge_gripper import EdgeGraspPilot

        return EdgeGraspPilot(config["parameters"], REPO / config["model_path"])
    raise ValueError("Unknown simulation variant: " + variant)


def load_episode_scenario(metadata):
    key = metadata["scenario_id"] + "@" + metadata["scenario_version"]
    registry = json.loads(REGISTRY.read_text())
    if key not in registry:
        raise ValueError("Unknown episode scenario: " + key)
    path = registry[key].get("config_path", str(CONFIG.relative_to(REPO)))
    config = load_scenario(REPO / path)
    if config_hash(config) != metadata["config_hash"]:
        raise ValueError("Episode config hash mismatch")
    return config


def hardware_binding(config, sim=None):
    key = config["scenario_id"] + "@" + config["scenario_version"]
    lock = json.loads(REGISTRY.read_text())[key]
    if "hardware_path" not in lock:
        return {}
    if config_hash(config) != lock["config_hash"]:
        raise ValueError("Hardware scenario binding mismatch")
    from phase1a.simulation.hardware_contract import validate_hardware

    return validate_hardware(
        make_sim(config) if sim is None else sim,
        config,
        REPO / config["model_path"],
        REPO / lock["hardware_path"],
        lock["hardware_hash"],
    )


def load_scenario(path=CONFIG):
    config = json.loads(Path(path).read_text())
    key = config["scenario_id"] + "@" + config["scenario_version"]
    lock = json.loads(REGISTRY.read_text()).get(key)
    if lock is None or lock["config_hash"] != config_hash(config):
        raise ValueError("Scenario registry/config mismatch; register a new version")
    if (
        config["model_hash"] != digest(REPO / config["model_path"])
        or config["model_hash"] != lock["model_hash"]
    ):
        raise ValueError("MJCF hash mismatch")
    for p, h in config["implementation_hashes"].items():
        if digest(REPO / p) != h:
            raise ValueError("Frozen implementation changed: " + p)
    if (
        digest(REPO / config["nominal_parameter_source"])
        != config["nominal_parameter_source_hash"]
    ):
        raise ValueError("Nominal v2 evidence changed")
    sim = make_sim(config)
    if extract_scene(sim) != config["scene"]:
        raise ValueError("Effective simulation differs from frozen scene")
    human_local = getattr(sim, "human_local", HAND)
    if not np.array_equal(human_local, sim.m.site_pos[sim.hsite]) or not np.array_equal(
        CENTER, sim.m.body_pos[sim.panel]
    ):
        raise ValueError("v2 geometry constants disagree with MJCF")
    hardware_binding(config, sim)
    return config


if __name__ == "__main__":
    freeze()
    print(CONFIG)
