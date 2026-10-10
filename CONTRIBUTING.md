# Contributing

Thanks for helping. The highest-impact contributions are:

1. **New engines** (Isaac Lab/Sim, Genesis, Drake, Newton, Gazebo). See [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md#adding-a-backend). An engine is ready when `tests/fixtures/arm6.urdf` and `tests/fixtures/hand.urdf` pass every core check against Pinocchio.
2. **New divergences.** If you find a file that two engines read differently, add a minimal URDF under `tests/fixtures/` and a test in `tests/test_engines.py` that pins the behaviour, with a link to the upstream issue.
3. **Checks** from the [roadmap](ROADMAP.md).

## Set up

```bash
git clone https://github.com/isospin01/simcompat && cd simcompat
python -m venv .venv && source .venv/bin/activate   # Python 3.10-3.12 (PyBullet has no 3.13 wheels yet)
pip install -e ".[dev]"
pytest                    # about 100 tests, a few seconds
ruff check . && ruff format --check .
```

## Ground rules

- **Do not hide differences.** If an engine default must change for a fair comparison, record it in `LoadInfo.settings`, so that it appears in every report.
- **Keep tolerances strict where physics is deterministic.** Do not loosen one to make an engine pass; find the cause instead.
- **Give every finding an actionable message.** Name the joint or link, give the numbers, and include a `hint` that says how to fix it.
- **Back claims in docs with a source:** an upstream issue, the engine's documentation, or a fixture that reproduces the behaviour.
- **Re-run the validation** (`validation/README.md`) when you change a check's logic, and update `VALIDATION.md` if the numbers move.
