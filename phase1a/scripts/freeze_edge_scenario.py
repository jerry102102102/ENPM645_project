"""Create a new frozen version; never rewrite historical scenario_v1."""

import json

import numpy as np

from phase1a.simulation.scenario_loader import (
    CONFIG,
    EDGE_CONFIG,
    REGISTRY,
    REPO,
    config_hash,
    digest,
    extract_scene,
    load_scenario,
    make_sim,
)


def freeze_edge():
    if EDGE_CONFIG.exists():
        raise FileExistsError(
            "Edge version exists; create a new version for further edits"
        )
    config = load_scenario(CONFIG)
    config.update(
        scenario_version="2.0.0",
        simulation_variant="edge_grasp_v2",
        model_path="phase1a/models/edge_grasp_v2.xml",
    )
    config["model_hash"] = digest(REPO / config["model_path"])
    config["implementation_hashes"]["phase1a/simulation/edge_gripper.py"] = digest(
        REPO / "phase1a/simulation/edge_gripper.py"
    )
    sim = make_sim(config)
    scene = extract_scene(sim)
    config["scene"] = scene
    half = np.array(scene["panel"]["dimensions_m"]) / 2
    semantics = {
        "frame": "panel_local",
        "units": "m",
        "fixed_meaning": "nominal material-relative anchors; no weld, qpos overwrite during stepping, or fixed world fixture",
        "panel_bounds_min_m": (-half).tolist(),
        "panel_bounds_max_m": half.tolist(),
        "robot": {},
    }
    for side, g in scene["grippers"].items():
        anchor = np.array(g["nominal_grasp_position_panel_local"])
        pads = {}
        for name, p in g["pads"].items():
            center = (
                anchor
                + np.array(p["body_position_in_root"])
                + np.array(p["geometry"]["local_position"])
            )
            size = np.array(p["geometry"]["half_size_m"])
            is_upper = name.endswith("upper")
            z = half[2] if is_upper else -half[2]
            footprint_min = center - size
            footprint_max = center + size
            pads[name] = {
                "nominal_geom_center_panel_local_m": center.tolist(),
                "nominal_pad_bounds_panel_local_m": [
                    footprint_min.tolist(),
                    footprint_max.tolist(),
                ],
                "nominal_panel_contact_patch_bounds_m": [
                    [
                        float(max(footprint_min[0], -half[0])),
                        float(max(footprint_min[1], -half[1])),
                        float(z),
                    ],
                    [
                        float(min(footprint_max[0], half[0])),
                        float(min(footprint_max[1], half[1])),
                        float(z),
                    ],
                ],
                "meaning": "nominal geometric overlap with top/bottom face; actual contact manifold is solved by MuJoCo",
            }
        semantics["robot"][side] = {
            "edge": "+Y long edge",
            "edge_grasp_center_panel_local_m": anchor.tolist(),
            "anchor_meaning": "between-jaw grasp frame at board edge; not a material contact point",
            "jaw_axis_panel_local": [0, 0, 1],
            "pads": pads,
        }
    semantics["human"] = {
        "edge": "-Y long edge",
        "interaction_point_panel_local_m": scene["human_grasp_point_panel_local"],
        "model": "single virtual point spring-damper on panel side surface; no human finger collision geometry",
        "why_off_center_x": "retains nonzero moment arm for world +Y rotation",
    }
    config["grasp_semantics"] = semantics
    key = config["scenario_id"] + "@" + config["scenario_version"]
    registry = json.loads(REGISTRY.read_text())
    if key in registry:
        raise ValueError("Version already registered")
    EDGE_CONFIG.write_text(json.dumps(config, indent=2) + "\n")
    registry[key] = {
        "config_path": str(EDGE_CONFIG.relative_to(REPO)),
        "config_hash": config_hash(config),
        "model_hash": config["model_hash"],
    }
    REGISTRY.write_text(json.dumps(registry, indent=2) + "\n")
    load_scenario(EDGE_CONFIG)
    print(EDGE_CONFIG)


if __name__ == "__main__":
    freeze_edge()
