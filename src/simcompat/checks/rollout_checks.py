"""Tier 5 - zero-torque, contact-free rollouts."""

from __future__ import annotations

import numpy as np

from ..backends.base import (
    CAP_PASSIVE,
    CAP_ROLLOUT,
    HINT_DAMPING,
    HINT_DYNAMICS,
    HINT_FRICTION,
    Backend,
    EngineQueryError,
)
from .base import Context, Finding, Severity, Tier, check
from .dynamics_checks import MODEL_CAUSES
from .evaluate import MIN_ROLLOUT_STEPS

#: Causes of a rigid-body motion difference, most specific first: the model itself, plus limits.
RIGID_MOTION_CAUSES = (
    *MODEL_CAUSES,
    "import.limits",
    "file.limits",
    "file.inertial",
    "file.inertia_validity",
)
#: ... plus each engine's own handling of mimic couplings, damping and friction.
PASSIVE_MOTION_CAUSES = (
    *RIGID_MOTION_CAUSES,
    "import.mimic",
    "rollout.passive_effect",
    "import.damping_friction",
)


def _joint_list(joints: list[str], shown: int = 3) -> str:
    """``j1, j2, j3 and 4 more``."""
    more = f" and {len(joints) - shown} more" if len(joints) > shown else ""
    return ", ".join(joints[:shown]) + more


def _limit_cut(ctx: Context, trajectories: list[np.ndarray]) -> int:
    """First step at which any trajectory nears a declared position limit.

    Engines model limits differently (soft vs. hard constraints), so comparisons stop there.
    """
    cut = trajectories[0].shape[0]
    for k, name in enumerate(ctx.joints):
        joint = ctx.declared.joints[name]
        if not joint.limited or joint.upper <= joint.lower:
            continue
        margin = max(1e-3, 0.02 * (joint.upper - joint.lower))
        for trajectory in trajectories:
            near = np.nonzero((trajectory[:, k] < joint.lower + margin) | (trajectory[:, k] > joint.upper - margin))[0]
            if near.size:
                cut = min(cut, int(near[0]))
    return cut


def _max_deviation(ctx: Context, a: np.ndarray, b: np.ndarray) -> tuple[float, int, str]:
    """Largest joint-space deviation, with rotational joints compared modulo 2*pi."""
    diff = a - b
    for k, name in enumerate(ctx.joints):
        if ctx.declared.joints[name].is_rotational:
            diff[:, k] = (diff[:, k] + np.pi) % (2 * np.pi) - np.pi
    step, k = np.unravel_index(int(np.argmax(np.abs(diff))), diff.shape)
    return float(abs(diff[step, k])), int(step), ctx.joints[k]


def _usable(ctx: Context, backends: list[Backend], **switches) -> tuple[list[Backend], list[Finding]]:
    """Backends whose rollouts finished with finite values, plus findings for the others."""
    usable, findings = [], []
    for backend in backends:
        try:
            ok = all(
                np.all(np.isfinite(ctx.ev.rollout(backend, i, **switches)))
                for i in range(len(ctx.samples.rollout_starts))
            )
        except EngineQueryError as e:  # anything else is a simcompat bug, reported by the runner
            findings.append(
                Finding(
                    Severity.ERROR,
                    f"rollout failed: {e}",
                    backend=backend.label,
                    engine_failure=True,
                )
            )
            continue
        if ok:
            usable.append(backend)
        else:
            findings.append(
                Finding(
                    Severity.WARN,
                    "rollout produced NaN/inf (a moving body with zero mass?)",
                    backend=backend.label,
                    hint="Give every moving link a non-zero <inertial>.",
                )
            )
    return usable, findings


def _compare(
    ctx: Context,
    backend: Backend,
    baseline: Backend,
    tolerance: float,
    severity: Severity,
    hint: str | None,
    **switches,
) -> Finding:
    worst, where, compared = 0.0, None, 0
    for i in range(len(ctx.samples.rollout_starts)):
        a, b = ctx.ev.rollout(backend, i, **switches), ctx.ev.rollout(baseline, i, **switches)
        cut = _limit_cut(ctx, [a, b])
        if cut < MIN_ROLLOUT_STEPS:
            continue
        compared += 1
        deviation, step, joint = _max_deviation(ctx, a[:cut], b[:cut])
        if where is None or deviation > worst:
            worst, where = deviation, (step, joint)
    if compared == 0:
        return Finding(Severity.SKIP, "every rollout reached a joint limit almost immediately", backend=backend.label)
    t = (where[0] + 1) * ctx.config.dt
    if worst > tolerance:
        return Finding(
            severity,
            f"deviates from {baseline.label} by {worst:.3g} at t={t:.3f}s (tolerance {tolerance:.2g})",
            backend=backend.label,
            subject=where[1],
            hint=hint,
            data={"max_err": worst, "t": t, "joint": where[1]},
            compared_with=baseline.label,
            score=worst,
        )
    return Finding(
        Severity.PASS,
        f"matches {baseline.label} (max deviation {worst:.2g} over {compared} rollout(s))",
        backend=backend.label,
        data={"max_err": worst},
    )


@check(
    "rollout.rigid",
    Tier.ROLLOUT,
    "Rigid-body rollout",
    requires=[CAP_ROLLOUT],
    problem="Rigid-body motion differs between engines",
    explained_by=RIGID_MOTION_CAUSES,
)
def rigid(ctx: Context) -> list[Finding]:
    """Zero-torque rollouts with damping, friction, mimic couplings and contacts off match the reference.

    This integrates everything the static checks compare and catches integration-level
    surprises they miss. Comparisons stop when a joint nears a position limit, because engines
    model limits differently.
    """
    switches = {"damping": False, "friction": False, "mimic": False}
    if not ctx.joints:
        return [Finding(Severity.SKIP, "the model has no movable joints")]
    usable, findings = _usable(ctx, [b for b in ctx.with_cap(CAP_ROLLOUT) if ctx.has_all_joints(b)], **switches)
    if len(usable) < 2:
        return [*findings, Finding(Severity.SKIP, "needs two backends that can roll out")]
    reference = ctx.reference if ctx.reference in usable else usable[0]
    findings.append(Finding(Severity.PASS, "reference", backend=reference.label))
    findings += [
        _compare(ctx, b, reference, ctx.tol.rigid_rollout_abs, Severity.FAIL, b.hints.get(HINT_DYNAMICS), **switches)
        for b in usable
        if b is not reference
    ]
    return findings


@check(
    "rollout.passive_effect",
    Tier.ROLLOUT,
    "Declared damping/friction actually act",
    requires=[CAP_PASSIVE],
    problem="An engine reads joint damping or friction but does not simulate it",
)
def passive_effect(ctx: Context) -> list[Finding]:
    """Turning each engine's own damping (or friction) handling on changes the motion.

    This compares every simulator with itself, so it needs no cross-engine tolerance. A
    parameter that is stored but has no effect is silently ignored: the robot is less damped
    than its file says. (Observed: PyBullet stores <dynamics friction> but does not simulate it.)
    """
    damped = [n for n in ctx.joints if (ctx.declared.joints[n].damping or 0) > 0]
    frictional = [n for n in ctx.joints if (ctx.declared.joints[n].friction or 0) > 0]
    if not damped and not frictional:
        return [Finding(Severity.SKIP, "the file declares no joint damping or friction")]
    findings = []
    for backend in ctx.with_cap(CAP_PASSIVE):
        if not backend.is_simulator or not ctx.has_all_joints(backend):
            continue
        for attr, joints, hint in (("damping", damped, HINT_DAMPING), ("friction", frictional, HINT_FRICTION)):
            if not joints:
                continue
            changes = [
                ctx.ev.rollout(backend, i, damping=attr == "damping", friction=attr == "friction", mimic=True)
                - ctx.ev.rollout(backend, i, damping=False, friction=False, mimic=True)
                for i in range(len(ctx.samples.rollout_starts))
            ]
            effect = max(float(np.max(np.abs(c))) for c in changes)
            if not all(np.all(np.isfinite(c)) for c in changes):
                findings.append(
                    Finding(
                        Severity.WARN,
                        f"cannot assess {attr}: rollout produced NaN/inf",
                        backend=backend.label,
                    )
                )
            elif effect <= ctx.tol.effect_abs:
                findings.append(
                    Finding(
                        Severity.FAIL,
                        f"declared <dynamics {attr}> has NO effect on the motion (read but not simulated) "
                        f"on {_joint_list(joints)}",
                        backend=backend.label,
                        data={"joints": joints, "effect": effect},
                        hint=backend.hints.get(hint),
                    )
                )
            else:
                findings.append(
                    Finding(
                        Severity.PASS,
                        f"{attr} acts (changes the trajectory by up to {effect:.3g})",
                        backend=backend.label,
                        data={"joints": joints, "effect": effect},
                    )
                )
    return findings or [Finding(Severity.SKIP, "no simulator backend loaded")]


@check(
    "rollout.passive",
    Tier.ROLLOUT,
    "Passive rollout across simulators",
    requires=[CAP_PASSIVE],
    problem="Simulators move the robot differently with their own damping, friction and mimic handling",
    explained_by=PASSIVE_MOTION_CAUSES,
)
def passive(ctx: Context) -> list[Finding]:
    """Zero-torque, contact-free rollouts with each engine's own damping, friction and mimic handling agree.

    Compared with the first simulator. Engines integrate damping and friction differently, so the
    tolerance is looser: the larger of passive_rollout_abs and passive_rollout_rel times how much
    damping and friction change the motion in the first place. Differences here are symptoms;
    import and passive_effect findings name causes.
    """
    switches = {"damping": True, "friction": True, "mimic": True}
    if not ctx.joints:
        return [Finding(Severity.SKIP, "the model has no movable joints")]
    simulators = [b for b in ctx.with_cap(CAP_PASSIVE) if b.is_simulator and ctx.has_all_joints(b)]
    if len(simulators) < 2:
        return [Finding(Severity.SKIP, "needs at least two simulators")]
    usable, findings = _usable(ctx, simulators, **switches)
    if len(usable) < 2:
        return [*findings, Finding(Severity.SKIP, "fewer than two usable rollouts")]
    baseline = usable[0]
    findings.append(Finding(Severity.PASS, "baseline", backend=baseline.label))
    for backend in usable[1:]:
        effect = max(_passive_effect(ctx, backend), _passive_effect(ctx, baseline))
        tolerance = max(ctx.tol.passive_rollout_abs, ctx.tol.passive_rollout_rel * effect)
        finding = _compare(ctx, backend, baseline, tolerance, Severity.WARN, None, **switches)
        finding.data.update(tolerance=tolerance, passive_effect=effect)
        findings.append(finding)
    return findings


def _passive_effect(ctx: Context, backend: Backend) -> float:
    """How much the engine's own damping and friction change its motion (largest joint deviation)."""
    effect = 0.0
    for i in range(len(ctx.samples.rollout_starts)):
        with_passive = ctx.ev.rollout(backend, i, damping=True, friction=True, mimic=True)
        without = ctx.ev.rollout(backend, i, damping=False, friction=False, mimic=True)
        change = np.abs(with_passive - without)
        effect = max(effect, float(np.max(change)) if np.all(np.isfinite(change)) else 0.0)
    return effect
