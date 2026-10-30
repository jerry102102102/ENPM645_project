"""Collect through the single Policy -> Object -> Arm -> MuJoCo pipeline."""

import argparse
import importlib
import json
from pathlib import Path

from phase1b.pipeline import run_episode


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--scenario", choices=list("ABC"), default="B")
    parser.add_argument("--duration", type=float, default=4.0)
    parser.add_argument(
        "--profile", choices=("baseline", "with_joint_states"), default="baseline"
    )
    for role in ("policy", "object-controller", "dual-arm-controller"):
        parser.add_argument("--" + role, help="module:Class; no-argument constructor")
    args = parser.parse_args()
    components = {}
    for role in ("policy", "object_controller", "dual_arm_controller"):
        value = getattr(args, role)
        if value:
            module, name = value.split(":")
            components[role] = getattr(importlib.import_module(module), name)()
    print(
        json.dumps(
            run_episode(
                args.output,
                args.scenario,
                args.duration,
                profile=args.profile,
                **components,
            ),
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
