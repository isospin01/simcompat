"""Check infrastructure: severities, findings, the run context and the check registry.

A check is a function ``fn(ctx) -> list[Finding]`` registered with :func:`check`. The runner fills
in each finding's check id, so checks never repeat their own id. The function's docstring is user
documentation, shown by ``simcompat explain`` and in HTML reports.
"""

from __future__ import annotations

import difflib
import enum
import fnmatch
import inspect
from collections.abc import Callable, Iterable
from dataclasses import dataclass, field
from typing import Any

from ..backends.base import CAP_DYNAMICS, Backend, LoadResult
from ..config import RunConfig, Tolerances
from ..declared import DeclaredModel
from ..inputs import PreparedURDF
from ..sampling import Samples
from .evaluate import Evaluator


class Severity(enum.IntEnum):
    PASS = 0
    SKIP = 1  # not applicable (no mimic joints, missing capability, ...)
    INFO = 2  # a documented convention worth knowing
    WARN = 3  # likely to bite: ordering, mimic, integrated behaviour
    FAIL = 4  # the engine simulates something different from the file where it must match
    ERROR = 5  # the engine refused the model or failed to answer, or a check crashed

    def __str__(self) -> str:
        return self.name

    @classmethod
    def parse(cls, text: str) -> Severity:
        return cls[text.strip().upper()]


class Tier(enum.IntEnum):
    FILE = 0
    IMPORT = 1
    INERTIA = 2
    KINEMATICS = 3
    DYNAMICS = 4
    ROLLOUT = 5

    @property
    def label(self) -> str:
        return self.name.capitalize()


@dataclass
class Finding:
    severity: Severity
    message: str
    backend: str | None = None  # backend label; None = about the file itself
    subject: str | None = None  # joint or link name
    data: dict[str, Any] = field(default_factory=dict)  # numbers behind the message (reports, JSON)
    hint: str | None = None  # how to fix it
    score: float = 0.0  # size of the difference, so the largest is shown first
    compared_with: str | None = None  # the engine this result was compared against (reference, baseline)
    engine_failure: bool = False  # the engine could not answer: an engine limitation, not a difference
    regraded_from: Severity | None = None  # the severity before a project profile re-graded it
    check: str = ""  # check id; filled in by the runner

    @property
    def accepted(self) -> bool:
        """A project profile lowered this finding from a problem (WARN or worse) to below WARN."""
        return self.regraded_from is not None and self.regraded_from >= Severity.WARN > self.severity


@dataclass
class CheckSpec:
    id: str
    tier: Tier
    title: str  # what is compared ("Mass, COM and inertia per moving body")
    problem: str  # plain-language headline when it fails
    requires: frozenset[str]  # backend capabilities; the check is skipped if no engine has them
    explained_by: tuple[str, ...]  # root-cause checks that explain this one, most specific first
    fn: Callable[[Context], list[Finding]]

    @property
    def doc(self) -> str:
        return inspect.cleandoc(self.fn.__doc__ or self.title)


REGISTRY: dict[str, CheckSpec] = {}


def check(
    check_id: str,
    tier: Tier,
    title: str,
    problem: str,
    requires: Iterable[str] = (),
    explained_by: Iterable[str] = (),
):
    """Register a check (see docs/ARCHITECTURE.md, "Adding a check").

    ``explained_by`` marks the check as a *symptom*: when one of those checks (a root cause) has a
    problem for the same engine, the report attaches this check's findings to that cause.
    """

    def register(fn: Callable[[Context], list[Finding]]):
        REGISTRY[check_id] = CheckSpec(
            check_id, Tier(tier), title, problem, frozenset(requires), tuple(explained_by), fn
        )
        return fn

    return register


def ordered_checks() -> list[CheckSpec]:
    return sorted(REGISTRY.values(), key=lambda spec: spec.tier)


def matching_checks(pattern: str) -> list[str]:
    """Check ids matching ``pattern``: an id, a glob ("rollout.*") or a tier prefix ("dynamics")."""
    if pattern in REGISTRY:
        return [pattern]
    if not any(char in pattern for char in "*?["):
        pattern += ".*"  # a bare prefix: "dynamics" means "dynamics.*"
    return [check_id for check_id in REGISTRY if fnmatch.fnmatch(check_id, pattern)]


def did_you_mean(pattern: str) -> str:
    """`` Did you mean 'dynamics.gravity'?`` for a pattern that matches no check, or ``""``."""
    candidates = [*REGISTRY, *sorted({check_id.split(".")[0] for check_id in REGISTRY})]
    close = difflib.get_close_matches(pattern, candidates, n=1, cutoff=0.6)
    return f" Did you mean {close[0]!r}?" if close else ""


def validate_registry() -> None:
    """Fail fast on a mistake in ``explained_by``: an unknown check, or a symptom named as a cause."""
    for spec in REGISTRY.values():
        unknown = [c for c in spec.explained_by if c not in REGISTRY]
        if unknown:
            raise ValueError(f"check {spec.id!r}: explained_by names unknown checks {unknown}")
        symptoms = [c for c in spec.explained_by if REGISTRY[c].explained_by]
        if symptoms:
            raise ValueError(f"check {spec.id!r}: explained_by names checks that are symptoms themselves {symptoms}")


@dataclass
class Context:
    """Everything a check may look at. Built once per run by the runner."""

    declared: DeclaredModel
    prepared: PreparedURDF
    backends: list[Backend]  # successfully loaded
    reference: Backend | None  # dynamics reference; None => compare with the first backend
    config: RunConfig
    samples: Samples
    load_results: list[LoadResult]
    ev: Evaluator = field(init=False)  # memoized engine queries in canonical joint order

    def __post_init__(self) -> None:
        self.ev = Evaluator(self.declared, self.samples, self.config)

    @property
    def tol(self) -> Tolerances:
        return self.config.tolerances

    @property
    def joints(self) -> list[str]:
        """Movable joints in canonical order."""
        return self.declared.movable_joints

    def with_cap(self, capability: str) -> list[Backend]:
        return [b for b in self.backends if capability in b.capabilities]

    def has_all_joints(self, backend: Backend) -> bool:
        return set(self.joints) <= set(backend.joint_order())

    def dynamics_reference(self) -> Backend | None:
        if self.reference is not None:
            return self.reference
        return next(iter(self.with_cap(CAP_DYNAMICS)), None)


def problems_or_pass(backend: Backend, problems: list[Finding], ok_message: str) -> list[Finding]:
    """The usual per-backend result: the problems found, or one PASS finding."""
    return problems or [Finding(Severity.PASS, ok_message, backend=backend.label)]
