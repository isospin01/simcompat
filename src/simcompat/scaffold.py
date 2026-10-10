"""``simcompat init``: write a commented profile and a GitHub Actions workflow into a project."""

from __future__ import annotations

from pathlib import Path

REPOSITORY = "https://github.com/isospin01/simcompat"
DEFAULT_INSTALL = f"simcompat[all] @ git+{REPOSITORY}"  # not on PyPI yet

PROFILE = """\
# simcompat profile: `simcompat check {robot} --profile simcompat.toml`
# Every key is optional; the values below are the defaults.

[run]
samples = 6            # sampled configurations for kinematics/dynamics checks
rollout_samples = 2    # random initial states for rollouts
duration = 0.5         # seconds per rollout
dt = 0.001
seed = 0

[tolerances]
# Strict: contact-free rigid-body quantities from a correct import agree to ~1e-11 or better.
mass_rel = 1e-6
com_abs = 1e-6           # m
inertia_rel = 1e-5
position_abs = 1e-6      # m
angle_abs = 1e-6         # rad
mass_matrix_rel = 1e-6
gravity_rel = 1e-6
coriolis_rel = 1e-5
rigid_rollout_abs = 1e-5
# Looser: integrated behaviour depends on how each engine integrates damping.
passive_rollout_abs = 0.01
passive_rollout_rel = 0.02   # or 2% of how much damping/friction change the motion, if larger

[severity]
# Re-grade checks for this project (PASS, INFO, WARN or FAIL). For example, if your deployment
# code always maps joints by name, ordering differences are only informational:
# "import.joint_order" = "info"
"""

WORKFLOW = """\
name: simcompat
on: [push, pull_request]
jobs:
  simcompat:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-python@v5
        with:
          python-version: "3.11"
      - run: pip install "{install}"
      # The report is appended to the job summary; the job fails if an engine simulates
      # something different from the file (add --strict to fail on warnings too).
      - run: simcompat check {robot} --profile simcompat.toml -o simcompat-report.html
      - if: always()
        uses: actions/upload-artifact@v4
        with:
          name: simcompat-report
          path: simcompat-report.html
"""


def init_project(
    directory: str | Path, robot: str, force: bool = False, install: str = DEFAULT_INSTALL
) -> list[tuple[Path, str]]:
    """Create the files; returns ``(path, status)`` pairs where status is "created" or "kept"."""
    directory = Path(directory)
    files = {
        directory / "simcompat.toml": PROFILE.format(robot=robot),
        directory / ".github" / "workflows" / "simcompat.yml": WORKFLOW.format(robot=robot, install=install),
    }
    results = []
    for path, content in files.items():
        if path.exists() and not force:
            results.append((path, "kept (exists; use --force to overwrite)"))
            continue
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content)
        results.append((path, "created"))
    return results


def guess_robot(directory: str | Path) -> str | None:
    """The project's robot file (relative path) if there is exactly one URDF, or one xacro file."""
    from .inputs import find_robot_files

    directory = Path(directory)
    candidates = find_robot_files(directory)
    return str(candidates[0].relative_to(directory)) if len(candidates) == 1 else None
