"""Shared test helpers. Engine-dependent tests are skipped when the engine is not installed."""

import importlib.util
from pathlib import Path

import pytest

FIXTURES = Path(__file__).parent / "fixtures"


def fixture(name: str) -> str:
    return str(FIXTURES / name)


def installed(module: str) -> bool:
    return importlib.util.find_spec(module) is not None


needs_mujoco = pytest.mark.skipif(not installed("mujoco"), reason="mujoco not installed")
needs_pybullet = pytest.mark.skipif(not installed("pybullet"), reason="pybullet not installed")
needs_pinocchio = pytest.mark.skipif(not installed("pinocchio"), reason="pinocchio not installed")
needs_engines = pytest.mark.skipif(
    not all(map(installed, ("mujoco", "pybullet", "pinocchio"))), reason="needs mujoco, pybullet and pinocchio"
)
needs_xacrodoc = pytest.mark.skipif(not installed("xacrodoc"), reason="xacrodoc not installed")


def by(report, check, backend=None, subject=None):
    """Findings of ``report`` filtered by check id, backend label and subject."""
    return [
        f
        for f in report.findings
        if f.check == check and (backend is None or f.backend == backend) and (subject is None or f.subject == subject)
    ]


def worst(report, check, backend=None):
    findings = by(report, check, backend)
    assert findings, f"no findings for {check} [{backend}]"
    return max(f.severity for f in findings).name
