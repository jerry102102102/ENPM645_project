# Dual Panda — Run Entry Points

One Scenario 3.0.0 / Hardware 2.0.0. [Hardware](../docs/CURRENT_HARDWARE_HANDOFF.md) · [Three-Layer Interface](../docs/THREE_LAYER_INTERFACE.md) · [Data Schema](../docs/DATA_SCHEMA_V2.md).

```bash
uv sync --locked
# Full tests, contact support, three-layer logging, replay, and A/B/C motion
uv run --locked python -m phase1b.scripts.run_suite --output results/phase1b/reproduction --render
# Short three-layer contract smoke test: test policy + reference-only + position hold
uv run --locked python -m phase1b.scripts.collect --duration .12 --output results/phase1b/dev-contract
# Native viewer; currently uses position hold
uv run --locked python -m phase1b.scripts.launch_viewer --scenario B
```

The output path must not already exist. `--render` requires ffmpeg and can be omitted. The viewer does not save data; close its window to stop it. The launcher handles the macOS uv libpython path.

Module responsibilities: `interfaces.py` defines types and validation; `control.py` connects the three layers; `pipeline.py` is the only formal data pipeline; `simulation.py` handles physics; `sensing.py` handles sensors; `geometry.py` provides shared geometry and demo IK. `models/`, `config/`, and `vendor/` contain the fixed model, configuration, and sources.

`demos/` contains scripted motion demonstrations and rendering for the same hardware. Its diagnostic files are not an Expert dataset and do not establish that the three-layer adapters can execute assistance. [Current Videos and Results](../results/phase1b/current/README.md).

`expert/` contains pure mathematical references and read-only offline annotation/diagnostic tools for the same system, without another scene. See [Design and Commands](../docs/EXPERT_DATA_DESIGN.md). Validation without running physics: `uv run --locked python -m unittest phase1b.tests.test_expert -v`.
