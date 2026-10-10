"""Backend registry.

A backend spec on the command line is ``name`` or ``name:key=value,key2=value``, for example
``pybullet:inertia_from_file=1``. The same engine may appear several times with different options.

Third-party packages can add engines without changing simcompat, through an entry point::

    [project.entry-points."simcompat.backends"]
    myengine = "my_package.backend:MyEngineBackend"
"""

from __future__ import annotations

import difflib
from importlib import metadata

from .base import (
    Backend,
    BackendLoadError,
    BackendSpecError,
    BackendUnavailable,
    BodyView,
    JointView,
    LoadInfo,
    LoadResult,
    Option,
    SimSettings,
    label_for,
)
from .mujoco_backend import MuJoCoBackend
from .pinocchio_backend import PinocchioBackend
from .pybullet_backend import PyBulletBackend

ENTRY_POINT_GROUP = "simcompat.backends"

REGISTRY: dict[str, type[Backend]] = {
    PinocchioBackend.name: PinocchioBackend,
    MuJoCoBackend.name: MuJoCoBackend,
    PyBulletBackend.name: PyBulletBackend,
}

#: preferred dynamics reference when installed
DEFAULT_REFERENCE = PinocchioBackend.name

_plugins_loaded = False


def register(cls: type[Backend]) -> type[Backend]:
    """Register a backend class (usable as a decorator)."""
    REGISTRY[cls.name] = cls
    return cls


def _load_plugins() -> None:
    """Register backends that other packages declare under the ``simcompat.backends`` entry point."""
    global _plugins_loaded
    if _plugins_loaded:
        return
    _plugins_loaded = True
    try:
        entry_points = metadata.entry_points(group=ENTRY_POINT_GROUP)
    except TypeError:  # Python 3.9
        entry_points = metadata.entry_points().get(ENTRY_POINT_GROUP, [])
    for entry_point in entry_points:
        try:
            register(entry_point.load())
        except Exception as e:  # a broken plugin must not break simcompat
            import warnings

            warnings.warn(f"simcompat: could not load backend plugin {entry_point.name!r}: {e}", stacklevel=2)


def parse_spec(spec: str) -> tuple[str, dict[str, str]]:
    """``"pybullet:inertia_from_file=1"`` -> ``("pybullet", {"inertia_from_file": "1"})``."""
    name, _, rest = spec.partition(":")
    options = {}
    for item in filter(None, (part.strip() for part in rest.split(","))):
        key, has_value, value = item.partition("=")
        options[key.strip()] = value.strip() if has_value else "1"
    return name.strip(), options


def _did_you_mean(word: str, choices) -> str:
    close = difflib.get_close_matches(word, list(choices), n=1, cutoff=0.6)
    return f" Did you mean {close[0]!r}?" if close else ""


def validate_spec(spec: str) -> tuple[str, dict[str, str]]:
    """``(name, normalized options)`` of a spec; raises :class:`BackendSpecError` with a suggestion."""
    _load_plugins()
    name, options = parse_spec(spec)
    if name not in REGISTRY:
        raise BackendSpecError(
            f"unknown backend {name!r}.{_did_you_mean(name, REGISTRY)} Available: {', '.join(sorted(REGISTRY))}"
        )
    known = REGISTRY[name].options_spec
    normalized = {}
    for key, value in options.items():
        if key not in known:
            listing = ", ".join(sorted(known)) or "none"
            raise BackendSpecError(
                f"backend {name!r} has no option {key!r}.{_did_you_mean(key, known)} Options: {listing} "
                "(see `simcompat backends`)"
            )
        normalized[key] = known[key].normalize(name, key, value)
    return name, normalized


def spec_label(spec: str) -> str:
    """The label a spec gets in reports: ``"pybullet:inertia_from_file=true"`` -> ``pybullet[inertia_from_file=1]``."""
    return label_for(*validate_spec(spec))


def create(spec: str) -> Backend:
    """Instantiate a backend from its spec.

    Raises :class:`BackendSpecError` for an invalid spec and :class:`BackendUnavailable` when the
    engine is not installed.
    """
    name, options = validate_spec(spec)
    cls = REGISTRY[name]
    installed, detail = cls.availability()
    if not installed:
        raise BackendUnavailable(f"backend {name!r} is not installed: {detail}")
    return cls(options)


def installed() -> list[tuple[str, bool, str]]:
    """``(name, installed, version-or-install-hint)`` for every registered backend."""
    _load_plugins()
    return [(name, *cls.availability()) for name, cls in REGISTRY.items()]


def default_specs() -> list[str]:
    """Every installed backend, with default options."""
    return [name for name, ok, _ in installed() if ok]


__all__ = [
    "DEFAULT_REFERENCE",
    "ENTRY_POINT_GROUP",
    "REGISTRY",
    "Backend",
    "BackendLoadError",
    "BackendSpecError",
    "BackendUnavailable",
    "BodyView",
    "JointView",
    "LoadInfo",
    "LoadResult",
    "Option",
    "SimSettings",
    "create",
    "default_specs",
    "installed",
    "parse_spec",
    "register",
    "spec_label",
    "validate_spec",
]
