"""Snapshot current edge-scene hardware once and bind it in the scenario registry."""

import json

from phase1a.simulation.hardware_contract import hardware_snapshot
from phase1a.simulation.scenario_loader import (
    EDGE_CONFIG,
    REGISTRY,
    REPO,
    config_hash,
    digest,
    load_scenario,
    make_sim,
)


def main():
    destination = REPO / "phase1a/config/hardware_v1.json"
    if destination.exists():
        raise FileExistsError(
            "Hardware v1 exists; register a new version for hardware changes"
        )
    config = load_scenario(EDGE_CONFIG)
    sim = make_sim(config)
    profile = {
        "hardware_id": "enpm645-edge-dual-gripper-virtual-human",
        "hardware_version": "1.0.0",
        "scope": "simulation-only hardware contract; no commercial hardware selected",
        "scenario_id": config["scenario_id"],
        "scenario_version": config["scenario_version"],
        "config_hash": config_hash(config),
        "model_hash": config["model_hash"],
        "hardware": hardware_snapshot(sim, config, REPO / config["model_path"]),
    }
    destination.write_text(json.dumps(profile, indent=2) + "\n")
    registry = json.loads(REGISTRY.read_text())
    key = config["scenario_id"] + "@" + config["scenario_version"]
    registry[key].update(
        hardware_path=str(destination.relative_to(REPO)),
        hardware_hash=digest(destination),
    )
    REGISTRY.write_text(json.dumps(registry, indent=2) + "\n")
    print(destination)


if __name__ == "__main__":
    main()
