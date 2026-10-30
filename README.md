# ENPM 645 — Human-Robot Collaboration

Two Franka Panda arms and one virtual human hand jointly manipulate a panel. The repository maintains one MuJoCo scene and one data pipeline:

**Policy → Object Controller → Dual-Arm Controller → MuJoCo**

Implemented: three-layer interfaces, causal data isolation, dual-arm contact support, joint-state logging, and scripted rotation, lifting, and sustained-force demonstrations. E0/E1/E2 candidate references and offline data diagnostics are also available. Full control algorithms, assistance tracking, validated Expert demonstrations, and learned models are not yet complete.

| Purpose | Document |
|---|---|
| Hardware and scenario | [Current Hardware](docs/CURRENT_HARDWARE_HANDOFF.md) |
| Three-layer types, modes, and clocks | [Interface Contract](docs/THREE_LAYER_INTERFACE.md) |
| Controller integration for teammates | [Controller Handoff](docs/CONTROL_HANDOFF_ARMS.md) |
| Expert mathematics, candidate labels, and learnability diagnostics | [Expert & Data Design](docs/EXPERT_DATA_DESIGN.md) |
| Sensor, command, and label logging | [Data Schema](docs/DATA_SCHEMA_V2.md) |
| Test evidence and videos | [Current Results](results/phase1b/current/README.md) |

## Running the project

Local Python 3.12.11, MuJoCo 3.3.5, and uv; no Docker. Use a new output directory. Rendering requires a local ffmpeg installation.

```bash
uv sync --locked
uv run --locked python -m phase1b.scripts.run_suite --output results/phase1b/reproduction --render
```

`phase1b/` is the only maintained implementation. `results/phase1b/current/` contains physical validation results from `37b8c24`; the Expert update did not rerun simulation. See [Run Entry Points](phase1b/README.md) for individual episodes and the viewer.

[Literature Notes](01_literature/README.md) · [Contributing](CONTRIBUTING.md). Historical experiments have been removed from the working tree and remain in Git; the revision before cleanup is `996a524`.
