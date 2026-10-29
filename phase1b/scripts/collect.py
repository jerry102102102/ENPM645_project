"""Collect a fresh episode using a baseline or a user-provided position adapter."""

import argparse
import importlib
import json
from pathlib import Path

from phase1b.interfaces import PositionHoldBaseline
from phase1b.pipeline import run_episode


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--scenario", choices=list("ABC"), default="B")
    p.add_argument("--duration", type=float, default=4.0)
    p.add_argument("--reference-test", action="store_true")
    p.add_argument(
        "--controller",
        help="module:Class; returns bounded joint-position and pad-force commands",
    )
    a = p.parse_args()
    factory = PositionHoldBaseline
    if a.controller:
        module, name = a.controller.split(":")
        factory = getattr(importlib.import_module(module), name)
    print(
        json.dumps(
            run_episode(a.output, a.scenario, a.duration, factory, a.reference_test),
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
