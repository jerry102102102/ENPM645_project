# Team Collaboration

- Maintain only the dual-Panda implementation in `phase1b/`. Follow the [Three-Layer Interface](docs/THREE_LAYER_INTERFACE.md) for types and execution order.
- Run `uv sync --locked` before execution. Commit both `pyproject.toml` and `uv.lock` when dependencies change.
- Before making changes, check the [Hardware](docs/CURRENT_HARDWARE_HANDOFF.md), [Controller Interface](docs/CONTROL_HANDOFF_ARMS.md), and [Data Schema](docs/DATA_SCHEMA_V2.md). Hardware or grasp-geometry changes require a new version and validation.
- Use a new output path for each experiment. Retained physical evidence is in `results/phase1b/current/`; Git preserves superseded versions, without parallel maintained implementations.
- Run relevant tests before committing. Never treat scripted decisions, measured contact wrenches, or motion demonstrations as valid Expert labels.
- Work on separate branches and review each other's pull requests. Update literature through the [Reading Guide](01_literature/README.md). Only the actual reader should fill in the reader, reading scope, and date; an AI summary does not count as having read the paper.
