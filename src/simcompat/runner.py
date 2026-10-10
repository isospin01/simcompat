"""Orchestration: resolve input -> prepare URDF -> load backends -> run checks -> Report."""

from __future__ import annotations

import contextlib
import dataclasses
import tempfile
import traceback
from collections.abc import Callable, Sequence
from pathlib import Path

from . import __version__
from . import backends as bk
from .backends.base import CAP_DYNAMICS, CAP_ROLLOUT, Backend, EngineQueryError, LoadResult, SimSettings
from .checks import REGISTRY, Context, Finding, Severity, ordered_checks
from .checks.base import CheckSpec, did_you_mean, matching_checks
from .config import RunConfig
from .declared import DeclaredModel, parse_urdf
from .errors import UsageError
from .inputs import PreparedURDF, RobotInput, prepare_urdf, resolve
from .report.model import Report
from .sampling import draw_samples

ALWAYS_RUN = "import.load"  # load failures are never hidden by --only/--skip
COMPARED_WITH_REFERENCE = frozenset({CAP_DYNAMICS, CAP_ROLLOUT})  # checks needing these compare with the reference


def run(
    robot: str | RobotInput,
    *,
    backends: str | Sequence[str] | None = None,
    reference: str = "auto",
    package_dirs: dict[str, str] | None = None,
    xacro_args: dict[str, str] | None = None,
    config: RunConfig | None = None,
    only: str | Sequence[str] | None = None,
    skip: str | Sequence[str] | None = None,
    log: Callable[[str], None] | None = None,
) -> Report:
    """Check one robot and return a :class:`Report`.

    ``robot`` is anything :func:`simcompat.inputs.resolve` accepts (a URDF, a .xacro, a directory,
    ``rd:<name>``). ``backends`` are specs such as ``"pybullet:inertia_from_file=1"`` (default: every
    installed engine). ``only``/``skip`` are glob patterns over check ids.
    """
    config = config or RunConfig()
    specs, checks = prepare_options(backends, reference, config, only, skip)
    robot_input = robot if isinstance(robot, RobotInput) else resolve(robot, package_dirs, xacro_args, log)
    declared = parse_urdf(robot_input.urdf)
    with tempfile.TemporaryDirectory(prefix="simcompat-") as workdir:
        prepared = prepare_urdf(robot_input.urdf, Path(workdir), robot_input.package_dirs)
        loaded, results = load_backends(specs, prepared, declared, SimSettings(dt=config.dt))
        try:
            ref = pick_reference(loaded, reference)
            for result in results:
                result.is_reference = ref is not None and result.label == ref.label
            ctx = Context(
                declared=declared,
                prepared=prepared,
                backends=loaded,
                reference=ref,
                config=config,
                samples=draw_samples(declared, config),
                load_results=results,
            )
            findings = [finding for spec in checks for finding in run_check(spec, ctx)]
        finally:
            for backend in loaded:
                backend.close()
    apply_severity_overrides(findings, config.severity_overrides)
    return Report(
        tool_version=__version__,
        model={
            "name": declared.name,
            "input": robot_input.spec,
            "label": robot_input.label,
            "path": str(robot_input.urdf),
            "root": declared.root,
            "links": len(declared.links),
            "joints": len(declared.joints),
            "movable_joints": declared.movable_joints,
            "notes": robot_input.notes,
        },
        backends=results,
        findings=findings,
        config=config,
        reference=ref.label if ref else None,
    )


def prepare_options(
    backends: str | Sequence[str] | None,
    reference: str,
    config: RunConfig,
    only: str | Sequence[str] | None,
    skip: str | Sequence[str] | None,
) -> tuple[list[str], list[CheckSpec]]:
    """The backend specs and checks to run. A mistake is a :class:`UsageError`, raised before any work."""
    specs = [backends] if isinstance(backends, str) else list(backends or bk.default_specs())
    validate_options(specs, reference, config)
    only = [only] if isinstance(only, str) else only
    skip = [skip] if isinstance(skip, str) else skip
    return specs, select_checks(only, skip)


def validate_options(specs: Sequence[str], reference: str, config: RunConfig) -> None:
    """Reject unknown backends, options, references, unusable settings and unknown checks in overrides."""
    labels = {}
    for spec in specs:
        label = bk.spec_label(spec)
        if label in labels:
            raise UsageError(f"backend {label} is requested twice; give each configuration once")
        labels[label] = bk.parse_spec(spec)[0]
    if reference not in ("auto", "none", "") and reference not in {*labels, *labels.values()}:
        raise UsageError(f"--reference {reference!r} is not one of the requested backends ({', '.join(labels)})")
    config.validate()
    for pattern in config.severity_overrides:
        _require_match("[severity]", pattern)


def _require_match(option: str, pattern: str) -> None:
    if not matching_checks(pattern):
        raise UsageError(
            f"{option} {pattern!r} matches no check.{did_you_mean(pattern)} Use a check id, a tier such as "
            "'dynamics', or a glob such as 'rollout.*'; `simcompat explain` lists every check."
        )


def select_checks(only: Sequence[str] | None, skip: Sequence[str] | None) -> list[CheckSpec]:
    """Checks matching ``only`` and not ``skip`` (see :func:`matching_checks`); ``import.load`` always runs."""
    for option, patterns in (("--only", only), ("--skip", skip)):
        for pattern in patterns or ():
            _require_match(option, pattern)
    wanted = {c for p in only for c in matching_checks(p)} if only else set(REGISTRY)
    unwanted = {c for p in skip or () for c in matching_checks(p)}
    selected = [spec for spec in ordered_checks() if spec.id in wanted - unwanted]
    if all(spec.id != ALWAYS_RUN for spec in selected):
        selected.insert(0, REGISTRY[ALWAYS_RUN])
    return selected


def load_backends(
    specs: Sequence[str], prepared: PreparedURDF, declared: DeclaredModel, settings: SimSettings
) -> tuple[list[Backend], list[LoadResult]]:
    """Create and load every backend; failures become ``LoadResult(ok=False)`` instead of exceptions."""
    loaded, results = [], []
    for spec in specs:
        try:
            backend = bk.create(spec)
        except bk.BackendUnavailable as e:
            results.append(LoadResult(label=bk.spec_label(spec), name=bk.parse_spec(spec)[0], ok=False, error=str(e)))
            continue
        try:
            info = backend.load(prepared, declared, settings)
        except Exception as e:  # an engine may fail in ways its adapter did not foresee
            hint = e.hint if isinstance(e, bk.BackendLoadError) else None
            error = str(e) if isinstance(e, bk.BackendLoadError) else f"{type(e).__name__}: {e}"
            results.append(
                LoadResult(
                    label=backend.label,
                    name=backend.name,
                    ok=False,
                    version=backend.availability()[1],
                    error=error,
                    hint=hint,
                    is_simulator=backend.is_simulator,
                )
            )
            with contextlib.suppress(Exception):  # cleaning up after a failed load must not fail the run
                backend.close()
            continue
        loaded.append(backend)
        results.append(
            LoadResult(
                label=backend.label,
                name=backend.name,
                ok=True,
                version=info.version,
                info=info,
                is_simulator=backend.is_simulator,
            )
        )
    return loaded, results


def pick_reference(loaded: list[Backend], reference: str) -> Backend | None:
    """``"auto"`` = Pinocchio if loaded; ``"none"`` = compare with the first backend; else a label or name."""
    if reference == "auto":
        return next((b for b in loaded if b.name == bk.DEFAULT_REFERENCE), None)
    if reference in ("none", ""):
        return None
    # None if the requested reference failed to load: import.load reports it, and engines are then
    # compared with the first one (validate_options already rejected unknown references)
    return next((b for b in loaded if reference in (b.label, b.name)), None)


def run_check(spec: CheckSpec, ctx: Context) -> list[Finding]:
    """Run one check and label its findings with the check id."""
    able = [b for b in ctx.backends if spec.requires <= b.capabilities]
    if spec.requires and not able:
        findings = [Finding(Severity.SKIP, "no loaded engine can answer this check")]
    else:
        try:
            findings = spec.fn(ctx)
        except Exception as error:
            findings = _isolate_failure(spec, ctx, error)
    for finding in findings:
        finding.check = finding.check or spec.id
    return findings


def _isolate_failure(spec: CheckSpec, ctx: Context, error: Exception) -> list[Finding]:
    """Re-run a failed check per engine, so one engine's failure is neither hidden nor blamed on others.

    The reference runs alone first. If it is the one that fails, engines that would be compared
    with it are reported as not compared (SKIP), and the others are checked on their own.
    """
    reference = ctx.reference
    findings: list[Finding] = []
    reference_failed = False
    if reference is not None:
        try:
            findings += _run_for(spec, ctx, reference, [reference], reference)
        except Exception as e:
            findings.append(_error_finding(e, reference.label))
            reference_failed = True
    for backend in ctx.backends:
        if backend is reference:
            continue
        if reference_failed and spec.requires & COMPARED_WITH_REFERENCE:
            message = f"not compared: the reference {reference.label} failed on this check"
            findings.append(Finding(Severity.SKIP, message, backend=backend.label))
            continue
        try:
            if reference is None or reference_failed:
                findings += _run_for(spec, ctx, backend, [backend], None)
            else:
                findings += _run_for(spec, ctx, backend, [backend, reference], reference)
        except Exception as e:
            findings.append(_error_finding(e, backend.label))
    if not any(f.severity == Severity.ERROR for f in findings):
        findings.append(_error_finding(error, None))  # failed only with all engines together
    return findings


def _run_for(
    spec: CheckSpec, ctx: Context, backend: Backend, backends: list[Backend], reference: Backend | None
) -> list[Finding]:
    """``spec`` with only ``backends`` loaded; keep the findings about ``backend``."""
    single = dataclasses.replace(ctx, backends=backends, reference=reference)
    single.ev = ctx.ev  # keep the memoized engine queries
    return [f for f in spec.fn(single) if f.backend == backend.label]


def _error_finding(error: Exception, backend: str | None) -> Finding:
    if isinstance(error, EngineQueryError):  # an engine limitation, reported as such
        return Finding(
            Severity.ERROR,
            f"the engine could not answer: {error}",
            backend=backend,
            hint="An engine limitation on this model, not a difference in the file's physics; "
            "the other checks still ran.",
            engine_failure=backend is not None,
        )
    return Finding(
        Severity.ERROR,
        f"check crashed: {type(error).__name__}: {error}",
        backend=backend,
        hint="This is a simcompat bug; please report it with the JSON report attached.",
        data={"traceback": "".join(traceback.format_exception(type(error), error, error.__traceback__, limit=4))},
    )


def apply_severity_overrides(findings: list[Finding], overrides: dict[str, str]) -> None:
    """Re-grade INFO/WARN/FAIL findings per a project's profile; PASS/SKIP stay, ERROR is never hidden.

    Override keys are check ids or patterns ("rollout.*", "dynamics"); an exact id wins over a
    pattern, and a longer pattern over a shorter one.
    """
    for finding in findings:
        matching = [key for key in overrides if finding.check in matching_checks(key)]
        if not matching or not Severity.INFO <= finding.severity <= Severity.FAIL:
            continue
        key = max(matching, key=lambda k: (k == finding.check, len(k)))
        finding.regraded_from = finding.severity
        finding.severity = Severity.parse(overrides[key])
