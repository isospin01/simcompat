"""Tier 4 - contact-free rigid-body dynamics, compared with the reference backend."""

from __future__ import annotations

from collections.abc import Callable

import numpy as np

from ..backends.base import CAP_DYNAMICS, HINT_DYNAMICS, Backend
from .base import Context, Finding, Severity, Tier, check

#: Root causes that change the rigid-body model itself (most specific first). A dynamics difference
#: on an engine that has one of these problems is reported as a consequence of it.
MODEL_CAUSES = (
    "import.dofs",
    "import.multi_dof",
    "kinematics.frames",
    "inertia.clusters",
    "import.armature",
    "file.axis",
)


def compare_with_reference(
    ctx: Context, label: str, rel_tol: float, quantity: Callable[[Backend, int], np.ndarray]
) -> list[Finding]:
    """Compare ``quantity(backend, sample)`` of every dynamics backend with the reference's."""
    if not ctx.joints:
        return [Finding(Severity.SKIP, "the model has no movable joints")]
    reference = ctx.dynamics_reference()
    if reference is None or not ctx.has_all_joints(reference):
        return [Finding(Severity.SKIP, "no usable dynamics reference backend")]
    if not any(b is not reference for b in ctx.with_cap(CAP_DYNAMICS)):
        return [Finding(Severity.SKIP, "needs a second engine to compare with", backend=reference.label)]
    samples = range(ctx.samples.count)
    expected = [quantity(reference, i) for i in samples]
    if not all(np.all(np.isfinite(value)) for value in expected):
        return [Finding(Severity.ERROR, f"reference {label} is NaN/inf", backend=reference.label)]

    findings = [Finding(Severity.PASS, "reference", backend=reference.label)]
    for backend in ctx.with_cap(CAP_DYNAMICS):
        if backend is reference:
            continue
        if not ctx.has_all_joints(backend):
            findings.append(
                Finding(
                    Severity.SKIP,
                    "degrees of freedom differ from the file (see import.dofs)",
                    backend=backend.label,
                )
            )
            continue
        actual = [quantity(backend, i) for i in samples]
        if not all(np.all(np.isfinite(value)) for value in actual):
            findings.append(
                Finding(
                    Severity.FAIL,
                    f"{label} is NaN/inf",
                    backend=backend.label,
                    hint=backend.hints.get(HINT_DYNAMICS),
                    score=float("inf"),
                )
            )
            continue
        errors = [
            float(np.linalg.norm(a - e)) / max(float(np.linalg.norm(e)), ctx.tol.torque_abs)
            for a, e in zip(actual, expected, strict=True)
        ]
        worst = int(np.argmax(errors))
        diff = actual[worst] - expected[worst]
        if errors[worst] > rel_tol and float(np.linalg.norm(diff)) > ctx.tol.torque_abs:
            entry = np.unravel_index(int(np.argmax(np.abs(diff))), diff.shape)
            where = "/".join(ctx.joints[k] for k in entry)
            findings.append(
                Finding(
                    Severity.FAIL,
                    f"{label} differs from {reference.label}: rel. error {errors[worst]:.3g} "
                    f"(largest at {where}: {actual[worst][entry]:.6g} vs {expected[worst][entry]:.6g})",
                    backend=backend.label,
                    hint=backend.hints.get(HINT_DYNAMICS),
                    data={
                        "rel_err": errors[worst],
                        "sample": worst,
                        "entry": where,
                        "engine": float(actual[worst][entry]),
                        "reference": float(expected[worst][entry]),
                    },
                    score=errors[worst],
                    compared_with=reference.label,
                )
            )
        else:
            findings.append(
                Finding(
                    Severity.PASS,
                    f"matches {reference.label} (max rel. error {max(errors):.2g} over {ctx.samples.count} samples)",
                    backend=backend.label,
                    data={"rel_err": max(errors)},
                )
            )
    return findings


@check(
    "dynamics.mass_matrix",
    Tier.DYNAMICS,
    "Joint-space mass matrix M(q)",
    requires=[CAP_DYNAMICS],
    problem="The engines' mass matrices disagree",
    explained_by=MODEL_CAUSES,
)
def mass_matrix(ctx: Context) -> list[Finding]:
    """The fixed-base mass matrix M(q) matches the reference at sampled configurations.

    With a correct import, independent implementations agree to about 1e-12, so the tolerance is
    strict (relative 1e-6). A difference here is usually explained by an inertia, kinematics or
    armature finding; if none explains it, it is a genuine engine or importer discrepancy.
    """
    return compare_with_reference(ctx, "M(q)", ctx.tol.mass_matrix_rel, ctx.ev.mass_matrix)


@check(
    "dynamics.gravity",
    Tier.DYNAMICS,
    "Gravity torques g(q)",
    requires=[CAP_DYNAMICS],
    problem="The engines compute different gravity torques",
    explained_by=[cause for cause in MODEL_CAUSES if cause != "import.armature"],  # armature has no weight
)
def gravity(ctx: Context) -> list[Finding]:
    """Generalized gravity forces g(q) match the reference at sampled configurations.

    Gravity torques depend only on masses, centers of mass and kinematics, so they isolate those
    from inertia-tensor problems.
    """
    return compare_with_reference(ctx, "g(q)", ctx.tol.gravity_rel, ctx.ev.gravity)


@check(
    "dynamics.coriolis",
    Tier.DYNAMICS,
    "Coriolis/centrifugal forces C(q, qd) qd",
    requires=[CAP_DYNAMICS],
    problem="The engines compute different Coriolis/centrifugal forces",
    explained_by=MODEL_CAUSES,
)
def coriolis(ctx: Context) -> list[Finding]:
    """Velocity-dependent bias forces C(q, qd) qd match the reference at sampled states."""
    return compare_with_reference(ctx, "C(q,qd)qd", ctx.tol.coriolis_rel, ctx.ev.coriolis)
