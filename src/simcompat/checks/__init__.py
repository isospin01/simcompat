"""Checks, one module per tier. Importing this package registers every built-in check."""

from . import (  # noqa: F401 - imported for registration
    dynamics_checks,
    file_checks,
    import_checks,
    inertia_checks,
    kinematics_checks,
    rollout_checks,
)
from .base import (
    REGISTRY,
    CheckSpec,
    Context,
    Finding,
    Severity,
    Tier,
    check,
    ordered_checks,
    problems_or_pass,
    validate_registry,
)

validate_registry()

__all__ = [
    "REGISTRY",
    "CheckSpec",
    "Context",
    "Finding",
    "Severity",
    "Tier",
    "check",
    "ordered_checks",
    "problems_or_pass",
    "validate_registry",
]
