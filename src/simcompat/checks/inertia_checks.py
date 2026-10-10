"""Tier 2 - mass properties each engine simulates, compared with the file."""

from __future__ import annotations

import numpy as np

from ..backends.base import CAP_INERTIA, HINT_INERTIA, Backend
from ..declared import DeclaredModel
from ..spatial import combine_inertias
from .base import Context, Finding, Severity, Tier, check, problems_or_pass

COM_SCALE = 0.01  # m; for ordering, a 1 cm COM offset weighs like a 100% mass or inertia error

MassProperties = tuple[float, np.ndarray, np.ndarray]  # mass, COM (world), inertia about the COM (world)


def engine_moving_bodies(backend: Backend) -> dict[str | None, MassProperties]:
    """The engine's bodies merged per carrying joint, mirroring ``DeclaredModel.moving_bodies``."""
    groups: dict[str | None, list] = {}
    for body in backend.bodies():
        groups.setdefault(body.carrier, []).append((body.mass, body.com, body.inertia))
    return {carrier: combine_inertias(parts) for carrier, parts in groups.items()}


@check(
    "inertia.clusters",
    Tier.INERTIA,
    "Mass, COM and inertia per moving body",
    requires=[CAP_INERTIA],
    problem="Engines simulate different mass, center of mass or inertia than the file declares",
)
def moving_bodies(ctx: Context) -> list[Finding]:
    """Each engine's mass properties match the file, for everything each joint moves.

    For every movable joint, simcompat compares the composite mass, center of mass and inertia
    tensor of all links that joint moves, in the world frame at q = 0. Fixed-joint children are
    merged first, so engines that fuse static bodies (MuJoCo) compare fairly with engines that
    keep them (PyBullet).

    Typical causes: links without <inertial> (each engine substitutes a different default);
    links with zero mass but non-zero inertia (MuJoCo drops that inertia); engines that recompute
    inertia from collision geometry (PyBullet without URDF_USE_INERTIA_FROM_FILE); a lost
    inertial-frame rotation.

    Fix: give every moving link an explicit, physical <inertial>, and use the engine's
    "inertia from file" option where it has one.
    """
    declared = ctx.declared.moving_bodies()
    findings = []
    for backend in ctx.with_cap(CAP_INERTIA):
        engine = engine_moving_bodies(backend)
        compared = [joint for joint in ctx.joints if joint in declared and joint in engine]  # else: import.dofs
        problems = []
        for joint in compared:
            finding = _compare(ctx, backend, joint, declared[joint], engine[joint])
            if finding is not None:
                problems.append(finding)
        message = f"{len(compared)}/{len(compared)} moving bodies match the declared mass, COM and inertia"
        findings += problems_or_pass(backend, problems, message)
    return findings


def _compare(
    ctx: Context, backend: Backend, joint: str, declared: MassProperties, engine: MassProperties
) -> Finding | None:
    """A FAIL finding if the engine's mass properties for ``joint`` differ from the file, else None."""
    (mass0, com0, inertia0), (mass1, com1, inertia1) = declared, engine
    if not (np.isfinite(mass1) and np.all(np.isfinite(com1)) and np.all(np.isfinite(inertia1))):
        message = "mass properties are NaN/inf"
        return Finding(Severity.FAIL, message, backend=backend.label, subject=joint, score=float("inf"))
    tol = ctx.tol
    com_err = float(np.linalg.norm(com1 - com0)) if (mass0 > 0 and mass1 > 0) else 0.0
    inertia_err = float(np.linalg.norm(inertia1 - inertia0))
    inertia_rel = inertia_err / max(float(np.linalg.norm(inertia0)), tol.inertia_abs)
    mass_rel = abs(mass1 - mass0) / max(abs(mass0), 1e-12)
    differences = []
    if abs(mass1 - mass0) > max(tol.mass_rel * abs(mass0), 1e-12):
        differences.append(f"mass {mass1:.6g} kg vs declared {mass0:.6g} kg")
    if com_err > tol.com_abs:
        differences.append(f"COM off by {com_err * 1000:.3g} mm")
    if inertia_err > max(tol.inertia_rel * float(np.linalg.norm(inertia0)), tol.inertia_abs):
        differences.append(f"inertia tensor rel. error {inertia_rel:.3g}")
    if not differences:
        return None
    suspects = _suspect_links(ctx.declared, joint)
    return Finding(
        Severity.FAIL,
        "; ".join(differences),
        backend=backend.label,
        subject=joint,
        hint=_hint(backend, suspects),
        data={
            "declared": {"mass": mass0, "com": com0, "inertia": inertia0},
            "engine": {"mass": mass1, "com": com1, "inertia": inertia1},
            "com_err_m": com_err,
            "inertia_rel_err": inertia_rel,
            **suspects,
        },
        score=max(mass_rel, inertia_rel, com_err / COM_SCALE),
    )


def _suspect_links(declared: DeclaredModel, joint: str) -> dict[str, list[str]]:
    """Links moved by ``joint`` that engines are known to read differently, by reason."""
    suspects: dict[str, list[str]] = {
        "links_without_inertial": [],  # each engine substitutes its own default mass
        "links_with_nonphysical_inertia": [],  # (or on the edge): MuJoCo refuses, PyBullet zeroes, Pinocchio keeps
        "links_with_zero_mass": [],  # zero mass but non-zero inertia: MuJoCo drops the inertia
    }
    for name in sorted(declared.links):
        if declared.carrier(name) != joint:
            continue
        inertial = declared.links[name].inertial
        if inertial is None:
            suspects["links_without_inertial"].append(name)
        elif inertial.defect() is not None:
            suspects["links_with_nonphysical_inertia"].append(name)
        elif inertial.mass <= 0 and inertial.inertia.any():
            suspects["links_with_zero_mass"].append(name)
    return suspects


HINTS = {  # the same text for every joint with the same cause, so a report lists each fix once
    "links_without_inertial": (
        "These bodies include links without <inertial>, for which each engine substitutes its own default mass "
        "(the links are listed under file.inertial). Add explicit <inertial> blocks."
    ),
    "links_with_nonphysical_inertia": (
        "These bodies include links whose inertia tensor is not physical, or on the edge of it (listed under "
        "file.inertia_validity); MuJoCo refuses such tensors, PyBullet sets them to zero, Pinocchio keeps them. "
        "Fix the tensors."
    ),
    "links_with_zero_mass": (
        "These bodies include links with zero mass but non-zero inertia (listed under file.inertia_validity); "
        "MuJoCo drops that inertia, other engines keep it. Give those links a mass, or remove their inertia."
    ),
}


def _hint(backend: Backend, suspects: dict[str, list[str]]) -> str | None:
    """The fix for the first suspect found in the file, else the engine's own advice."""
    reason = next((reason for reason, links in suspects.items() if links), None)
    return HINTS[reason] if reason else backend.hints.get(HINT_INERTIA)
