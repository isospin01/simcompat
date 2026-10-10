"""The backend interface: one adapter per physics engine.

A backend answers a small set of engine-neutral questions about the model *as that engine
built it*. Everything is keyed by URDF joint name, so checks never depend on an engine's
internal ordering. To add an engine, subclass :class:`Backend`, implement what you can,
declare it in ``capabilities`` and register the class (see docs/ARCHITECTURE.md).
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Sequence
from dataclasses import dataclass, field
from typing import ClassVar

import numpy as np

from ..declared import DeclaredModel
from ..errors import SimCompatError
from ..inputs import PreparedURDF

# --- capabilities: which questions a backend can answer (checks skip what is missing) --------
CAP_IMPORT = "import"  # joint_order(), joint_views(), default_base()
CAP_INERTIA = "inertia"  # bodies()
CAP_KINEMATICS = "kinematics"  # joint_frames()
CAP_DYNAMICS = "dynamics"  # mass_matrix(), bias()
CAP_ROLLOUT = "rollout"  # rollout(..., damping=False, friction=False, mimic=False)
CAP_PASSIVE = "passive"  # rollout() honouring the engine's own damping/friction/mimic handling

# --- hint topics: backends put fix suggestions in ``self.hints[topic]`` ----------------------
HINT_INERTIA = "inertia"
HINT_DYNAMICS = "dynamics"
HINT_DAMPING = "damping"
HINT_FRICTION = "friction"
HINT_MIMIC = "mimic"

# --- shared vocabulary: how an engine represents things (JointView.mimic, multi_dof_joints()) -
MIMIC_ENFORCED = "equality constraint"
MIMIC_IGNORED = "independent joint"
BUILT_FREE = "free (6 DoF)"
BUILT_BALL = "ball (3 DoF)"
BUILT_PLANAR = "planar (3 DoF)"
BUILT_FIXED = "fixed"
NOT_BUILT = "fixed or merged"  # absent from the engine's joints: welded, fused away or dropped

TRUE_WORDS = ("1", "true", "yes", "on")
FALSE_WORDS = ("0", "false", "no", "off")

JointFrame = tuple[np.ndarray, np.ndarray, np.ndarray]  # (child frame position, rotation, axis), world


class BackendUnavailable(RuntimeError):
    """The engine is not installed (the message says how to install it)."""


class BackendSpecError(SimCompatError):
    """A backend spec names an unknown engine, option or option value (the message suggests a fix)."""


class EngineQueryError(RuntimeError):
    """The engine failed to answer a query for this model (an engine limitation, not a simcompat bug)."""


class BackendLoadError(RuntimeError):
    """The engine refused to load the model. ``hint`` explains why, when known."""

    def __init__(self, message: str, hint: str | None = None):
        super().__init__(message)
        self.hint = hint


@dataclass
class SimSettings:
    """Runtime settings simcompat harmonizes across engines (listed in every report)."""

    dt: float = 1e-3
    gravity: tuple[float, float, float] = (0.0, 0.0, -9.81)


@dataclass
class JointView:
    """A joint as the engine represents it. ``None`` means "not represented by this engine"."""

    name: str
    kind: str  # "revolute" | "prismatic"
    limited: bool | None = None
    lower: float | None = None
    upper: float | None = None
    effort: float | None = None
    velocity: float | None = None
    damping: float | None = None
    friction: float | None = None
    armature: float | None = None
    mimic: str | None = None  # MIMIC_ENFORCED or MIMIC_IGNORED


@dataclass
class BodyView:
    """One engine body at q = 0, in the world frame."""

    name: str
    carrier: str | None  # movable joint that moves this body (None = static)
    mass: float
    com: np.ndarray
    inertia: np.ndarray  # 3x3 about its own COM


@dataclass
class LoadInfo:
    """What a backend reports after loading."""

    version: str
    settings: list[str] = field(default_factory=list)  # what simcompat changed vs. engine defaults
    defaults: dict[str, str] = field(default_factory=dict)  # notable engine defaults (documentation)
    warnings: list[str] = field(default_factory=list)  # importer messages and fallbacks


@dataclass
class LoadResult:
    """Outcome of loading one backend; one per requested backend spec, kept in the report."""

    label: str
    name: str
    ok: bool
    version: str | None = None
    error: str | None = None
    hint: str | None = None
    info: LoadInfo | None = None
    is_simulator: bool = True
    is_reference: bool = False

    @property
    def title(self) -> str:
        text = f"{self.label} {self.version}" if self.version else self.label
        return text + (" (reference)" if self.is_reference else "")


@dataclass(frozen=True)
class Option:
    """A user option of a backend (``-b name:key=value``), validated before anything runs."""

    help: str
    kind: str = "bool"  # "bool" (1/0, true/false, yes/no, on/off), "number", "choice" or "text"
    choices: tuple[str, ...] = ()  # for "choice"

    @property
    def placeholder(self) -> str:
        """What ``simcompat backends`` shows as the value, e.g. ``1``, ``<number>``, ``true|false|auto``."""
        return {"bool": "1", "number": "<number>", "text": "<text>"}.get(self.kind) or "|".join(self.choices)

    def normalize(self, backend: str, key: str, value: str) -> str:
        """The canonical spelling of ``value`` ("1"/"0" for booleans); raises :class:`BackendSpecError`."""
        text = str(value).strip()
        lowered = text.lower()
        if self.kind == "bool" and lowered in TRUE_WORDS + FALSE_WORDS:
            return "1" if lowered in TRUE_WORDS else "0"
        if self.kind == "choice" and lowered in self.choices:
            return lowered
        if self.kind == "choice" and {"true", "false"} <= set(self.choices) and lowered in TRUE_WORDS + FALSE_WORDS:
            return "true" if lowered in TRUE_WORDS else "false"
        if self.kind == "number":
            try:
                float(text)
                return text
            except ValueError:
                pass
        if self.kind == "text" and text:
            return text
        expected = {"bool": "1/0, true/false, yes/no or on/off", "number": "a number", "text": "a value"}
        raise BackendSpecError(
            f"backend {backend!r}: {key}={value!r} is not valid; expected "
            f"{expected.get(self.kind) or ' or '.join(self.choices)} ({self.help})"
        )


def label_for(name: str, options: dict[str, str]) -> str:
    """Display name of a backend configuration, e.g. ``pybullet[inertia_from_file=1]``."""
    if not options:
        return name
    return f"{name}[{','.join(f'{k}={v}' for k, v in sorted(options.items()))}]"


class Backend(ABC):
    """Base class for engine adapters. Load with a FIXED base; key everything by joint name."""

    name: ClassVar[str] = "base"
    is_simulator: ClassVar[bool] = True  # False for analytical libraries (the reference)
    capabilities: ClassVar[frozenset[str]] = frozenset()
    options_spec: ClassVar[dict[str, Option]] = {}  # user options, validated by backends.validate_spec

    def __init__(self, options: dict[str, str] | None = None):
        """``options`` are already validated and normalized (see :func:`simcompat.backends.create`)."""
        self.options = dict(options or {})
        self.hints: dict[str, str] = {}  # HINT_* topic -> how to fix it in this engine

    @property
    def label(self) -> str:
        """Unique display name, e.g. ``pybullet[inertia_from_file=1]``."""
        return label_for(self.name, self.options)

    def option_enabled(self, key: str) -> bool:
        return self.options.get(key) == "1"

    @classmethod
    @abstractmethod
    def availability(cls) -> tuple[bool, str]:
        """``(installed, version-or-install-hint)``. Must never raise."""

    @abstractmethod
    def load(self, prepared: PreparedURDF, declared: DeclaredModel, settings: SimSettings) -> LoadInfo:
        """Load the prepared URDF with a fixed base. Raise :class:`BackendLoadError` on failure."""

    def close(self) -> None:  # noqa: B027 - optional hook
        """Release engine resources."""

    # CAP_IMPORT
    def joint_order(self) -> list[str]:
        """Movable joint names in the engine's native order."""
        raise NotImplementedError

    def joint_views(self) -> dict[str, JointView]:
        raise NotImplementedError

    def default_base(self) -> str:
        """Base mode the engine's standard loader gives with default arguments: "fixed" or "floating"."""
        return "unknown"

    def multi_dof_joints(self) -> dict[str, str]:
        """How the engine built the joints that are not 1-DoF, by URDF joint name.

        Values are BUILT_FREE, BUILT_BALL, BUILT_PLANAR or BUILT_FIXED; joints the engine merged away
        may be absent (reported as NOT_BUILT).
        """
        return {}

    # CAP_INERTIA
    def bodies(self) -> list[BodyView]:
        raise NotImplementedError

    # CAP_KINEMATICS
    def joint_frames(self, q: dict[str, float]) -> dict[str, JointFrame]:
        raise NotImplementedError

    # CAP_DYNAMICS
    def mass_matrix(self, q: dict[str, float]) -> np.ndarray:
        """Joint-space mass matrix, rows/columns in ``joint_order()``."""
        raise NotImplementedError

    def bias(self, q: dict[str, float], qd: dict[str, float]) -> np.ndarray:
        """Coriolis + centrifugal + gravity forces in ``joint_order()`` (no damping or friction)."""
        raise NotImplementedError

    # CAP_ROLLOUT / CAP_PASSIVE
    def rollout(
        self, q0: dict[str, float], qd0: dict[str, float], steps: int, *, damping: bool, friction: bool, mimic: bool
    ) -> np.ndarray:
        """Zero-torque, contact-free rollout; ``(steps, n)`` joint positions in ``joint_order()``.

        ``damping``/``friction``/``mimic`` switch the engine's *own* handling of those URDF elements.
        """
        raise NotImplementedError


# ----------------------------------------------------------------------- reordering helpers
def _index(names: Sequence[str], target: Sequence[str]) -> list[int]:
    names = list(names)
    return [names.index(n) for n in target]


def reorder_vector(values: np.ndarray, names: Sequence[str], target: Sequence[str]) -> np.ndarray:
    """``(n,)`` engine-ordered vector -> ``target`` order."""
    return np.asarray(values)[_index(names, target)]


def reorder_matrix(values: np.ndarray, names: Sequence[str], target: Sequence[str]) -> np.ndarray:
    """``(n, n)`` engine-ordered matrix -> ``target`` order."""
    idx = _index(names, target)
    return np.asarray(values)[np.ix_(idx, idx)]


def reorder_columns(values: np.ndarray, names: Sequence[str], target: Sequence[str]) -> np.ndarray:
    """``(T, n)`` engine-ordered trajectory -> ``target`` order."""
    return np.asarray(values)[:, _index(names, target)]
