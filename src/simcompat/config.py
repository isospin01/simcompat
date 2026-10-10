"""Run configuration and tolerance profiles.

Tolerances are strict for quantities every engine should reproduce exactly (contact-free
rigid-body dynamics from a correct import agree to ~1e-11 or better) and looser for behaviour that
legitimately depends on the integrator. Override them in a TOML profile (``simcompat init`` writes
a commented one)::

    [run]
    samples = 8

    [tolerances]
    passive_rollout_abs = 0.02

    [severity]                       # re-grade a check for your project
    "import.joint_order" = "info"
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, fields
from pathlib import Path

from .errors import SimCompatError


class ProfileError(SimCompatError):
    """A profile (simcompat.toml) cannot be used; the message names the file and the key."""


SEVERITY_OVERRIDES = ("PASS", "INFO", "WARN", "FAIL")
PROFILE_SECTIONS = ("run", "tolerances", "severity")
RUN_KEYS = {"samples": int, "rollout_samples": int, "seed": int, "dt": float, "duration": float}


@dataclass
class Tolerances:
    # import
    limit_abs: float = 1e-6  # rad or m
    param_rel: float = 1e-6  # damping, friction, effort, velocity
    # inertia (composite per moving body, world frame, q = 0)
    mass_rel: float = 1e-6
    com_abs: float = 1e-6  # m
    inertia_rel: float = 1e-5
    inertia_abs: float = 1e-9  # kg m^2 floor for near-zero tensors
    # kinematics
    position_abs: float = 1e-6  # m
    angle_abs: float = 1e-6  # rad
    # dynamics (vs. the reference backend)
    mass_matrix_rel: float = 1e-6
    gravity_rel: float = 1e-6
    coriolis_rel: float = 1e-5
    torque_abs: float = 1e-8  # N m floor for near-zero vectors
    # rollouts
    rigid_rollout_abs: float = 1e-5  # rad or m; damping, friction and contact off
    passive_rollout_abs: float = 1e-2  # rad or m; each engine's own damping and friction
    passive_rollout_rel: float = 2e-2  # ... or this fraction of how much damping/friction change the motion
    effect_abs: float = 1e-9  # below this, a declared parameter "has no effect"


@dataclass
class RunConfig:
    tolerances: Tolerances = field(default_factory=Tolerances)
    severity_overrides: dict[str, str] = field(default_factory=dict)  # check id -> PASS/INFO/WARN/FAIL
    samples: int = 6  # sampled configurations for kinematics/dynamics
    rollout_samples: int = 2  # random initial states for rollouts
    seed: int = 0
    dt: float = 1e-3  # s
    duration: float = 0.5  # s per rollout

    @classmethod
    def from_toml(cls, path: str | Path | None) -> RunConfig:
        """Load a profile. Unknown sections or keys and invalid values raise :class:`ProfileError`.

        Check ids under ``[severity]`` are validated by the runner, which knows the registered checks.
        """
        config = cls()
        if not path:
            return config
        try:
            import tomllib
        except ModuleNotFoundError:  # Python < 3.11
            import tomli as tomllib
        try:
            data = tomllib.loads(Path(path).read_text(encoding="utf-8"))
        except FileNotFoundError as e:
            raise ProfileError(f"profile not found: {path} (`simcompat init` writes one)") from e
        except tomllib.TOMLDecodeError as e:
            raise ProfileError(f"{path}: invalid TOML: {e}") from e

        def unknown(section: str, key: str, known) -> ProfileError:
            return ProfileError(f"{path}: unknown {section} {key!r} (known: {', '.join(sorted(known))})")

        for section in data:
            if section not in PROFILE_SECTIONS:
                raise unknown("section", section, PROFILE_SECTIONS)
        tolerances = {f.name for f in fields(Tolerances)}
        for key, value in data.get("tolerances", {}).items():
            if key not in tolerances:
                raise unknown("tolerance", key, tolerances)
            setattr(config.tolerances, key, _number(path, key, value, float))
        for check_id, severity in data.get("severity", {}).items():
            severity = str(severity).upper()
            if severity not in SEVERITY_OVERRIDES:
                allowed = ", ".join(SEVERITY_OVERRIDES)
                raise ProfileError(f"{path}: [severity] {check_id!r} = {severity!r} must be one of {allowed}")
            config.severity_overrides[str(check_id)] = severity
        for key, value in data.get("run", {}).items():
            if key not in RUN_KEYS:
                raise unknown("[run] key", key, RUN_KEYS)
            setattr(config, key, _number(path, key, value, RUN_KEYS[key]))
        config.validate(source=str(path))
        return config

    def validate(self, source: str = "options") -> None:
        """Raise :class:`ProfileError` for values no run can use (checked for profiles and command-line flags)."""
        problems = [
            f"{name} must be a number >= 0"
            for name, value in vars(self.tolerances).items()
            if not 0 <= value < math.inf
        ]
        if self.samples < 1:
            problems.append("samples must be >= 1")
        if self.rollout_samples < 1:
            problems.append("rollout_samples must be >= 1 (use --skip rollout to skip rollouts)")
        if not (0 < self.dt < math.inf and 0 < self.duration < math.inf):
            problems.append("dt and duration must be finite numbers > 0")
        if problems:
            raise ProfileError(f"{source}: " + "; ".join(problems))


def _number(path, key: str, value, cast):
    if isinstance(value, bool):
        raise ProfileError(f"{path}: {key} = {value!r} is not a number")
    try:
        return cast(value)
    except (TypeError, ValueError) as e:
        raise ProfileError(f"{path}: {key} = {value!r} is not a number") from e
