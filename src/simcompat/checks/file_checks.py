"""Tier 0 - properties of the file itself that predict cross-engine divergence."""

from __future__ import annotations

import numpy as np

from ..declared import INERTIA_EDGE, MULTI_DOF_TYPES, Inertial
from .base import Context, Finding, Severity, Tier, check


@check(
    "file.inertial",
    Tier.FILE,
    "Moving links without <inertial>",
    problem="Moving links have no <inertial>, so each engine invents its own mass for them",
)
def missing_inertial(ctx: Context) -> list[Finding]:
    """Every link that a joint moves declares an <inertial> block.

    The URDF specification says a link without <inertial> has zero mass, but engines disagree:
    MuJoCo derives mass from the collision geometry, PyBullet assigns 1 kg and Pinocchio uses 0.
    The robot therefore weighs something different in each engine.

    Fix: give every moving link an explicit <inertial> (even a tiny one for sensor frames).
    """
    declared = ctx.declared
    findings = [
        Finding(
            Severity.WARN,
            f"moved by '{declared.carrier(name)}' but has no <inertial>",
            subject=name,
            hint="Add an explicit <inertial> so every engine uses the same mass.",
        )
        for name in declared.links_without_inertial()
        if declared.carrier(name) is not None
    ]
    return findings or [Finding(Severity.PASS, "every moving link declares <inertial>")]


@check(
    "file.inertia_validity",
    Tier.FILE,
    "Physically valid inertia",
    problem="Some inertia tensors are impossible or degenerate, and engines refuse, zero or keep them",
)
def inertia_validity(ctx: Context) -> list[Finding]:
    """Inertia tensors are positive semi-definite and satisfy the triangle inequality.

    Engines disagree on what to do with an impossible tensor (not positive semi-definite, or
    principal moments that violate A + B >= C): MuJoCo refuses to load the model, even for a static
    link; PyBullet (with inertia from file) silently sets that link's inertia to zero; Pinocchio
    keeps the tensor. A tensor exactly on the edge (A + B = C, a flat or thin body) can tip either
    way after rounding. A link with zero mass but non-zero inertia is dropped by MuJoCo and kept by
    the others.

    Fix: correct the tensor (CAD exports often round it badly), or check it with
    `-b mujoco:balanceinertia=true`.
    """
    findings = []
    for name, link in ctx.declared.links.items():
        inertial = link.inertial
        if inertial is None or (inertial.mass == 0 and not inertial.inertia.any()):
            continue  # no data, or a pure frame link
        moving = ctx.declared.carrier(name) is not None
        problems, hint, score = _inertia_problems(inertial, moving)
        if problems:
            findings.append(
                Finding(
                    Severity.WARN,
                    "; ".join(problems),
                    subject=name,
                    data={"mass": inertial.mass, "principal_moments": np.linalg.eigvalsh(inertial.inertia)},
                    hint=hint,
                    score=score,
                )
            )
    return findings or [Finding(Severity.PASS, "all declared inertias are physical")]


INERTIA_HINTS = {
    "impossible": "MuJoCo refuses such tensors, PyBullet sets them to zero, Pinocchio keeps them. Fix the tensor.",
    INERTIA_EDGE: "After rounding, PyBullet may find A + B < C and set the inertia to zero. Make A + B a bit larger.",
    "massless": "Give the link a mass, or remove its inertia: MuJoCo drops the inertia of massless bodies.",
    "degenerate": "Some engines regularize degenerate inertia, others fail.",
}


def _inertia_problems(inertial: Inertial, moving: bool) -> tuple[list[str], str, float]:
    """What engines may read differently in one <inertial>: (problems, how to fix them, order key).

    The order key puts impossible tensors first, then tensors on the edge, then the rest.
    """
    problems, reason, score = [], "degenerate", 0.0
    defect = inertial.defect()
    if defect is not None:
        kind, text, size = defect
        problems.append(text)
        reason = INERTIA_EDGE if kind == INERTIA_EDGE else "impossible"
        score = 1.0 if kind == INERTIA_EDGE else 2.0 + size
        if reason == "impossible" and not moving:
            problems.append("static link, but MuJoCo validates every body")
    if moving and inertial.mass <= 0:
        if inertial.inertia.any():
            problems.append(f"mass {inertial.mass:g} but non-zero inertia on a moving link")
            reason = "massless" if defect is None else reason
        else:
            problems.append(f"mass {inertial.mass:g} on a moving link")
    if moving and defect is None and np.linalg.eigvalsh(inertial.inertia).min() <= 0:
        problems.append("singular inertia (a principal moment is zero)")
    return problems, INERTIA_HINTS[reason], score


@check(
    "file.limits",
    Tier.FILE,
    "Ambiguous joint ranges",
    problem="Some joint ranges are ambiguous, and engines read them differently (locked, limited or free)",
)
def ambiguous_ranges(ctx: Context) -> list[Finding]:
    """No joint range that engines interpret differently.

    Two cases are ambiguous. A revolute/prismatic joint with lower == upper (also what an omitted
    lower/upper defaults to): PyBullet locks the joint, MuJoCo and Pinocchio leave it free. A
    continuous joint with lower/upper written in its <limit>: the URDF specification says a
    continuous joint has no limits, Pinocchio ignores them, but MuJoCo and PyBullet enforce them.

    Fix: declare the intended range explicitly with a revolute joint, or drop lower/upper from the
    continuous joint.
    """
    findings = []
    for joint in (ctx.declared.joints[name] for name in ctx.joints):
        if joint.zero_width:
            findings.append(
                Finding(
                    Severity.WARN,
                    f"{joint.type} joint declares lower == upper == {joint.lower:g}",
                    subject=joint.name,
                    hint="Declare the intended range explicitly.",
                )
            )
        elif joint.ignored_range is not None:
            lower, upper = joint.ignored_range
            findings.append(
                Finding(
                    Severity.WARN,
                    f"continuous joint declares lower={lower:g}, upper={upper:g}, which the URDF spec ignores",
                    subject=joint.name,
                    hint="Use a revolute joint if the range is intended; otherwise remove lower/upper.",
                )
            )
    return findings or [Finding(Severity.PASS, "no ambiguous joint ranges")]


@check("file.axis", Tier.FILE, "Unit joint axes", problem="Some joint axes are not unit vectors")
def axis_norm(ctx: Context) -> list[Finding]:
    """Every <axis xyz> is a unit vector, as URDF expects.

    Engines differ in whether, and where, they normalize it. Observed: PyBullet uses the raw axis
    when stepping (and for prismatic offsets) but normalizes it for mass-matrix and inverse-dynamics
    queries, so the same file moves differently.

    Fix: normalize the axis in the file.
    """
    findings = [
        Finding(
            Severity.WARN,
            f"axis has norm {joint.axis_norm:.6g}, not 1",
            subject=joint.name,
            hint="Normalize the axis in the file.",
            score=abs(joint.axis_norm - 1.0),
        )
        for joint in (ctx.declared.joints[name] for name in ctx.joints)
        if abs(joint.axis_norm - 1.0) > 1e-6
    ]
    return findings or [Finding(Severity.PASS, "all joint axes are unit vectors")]


@check("file.meshes", Tier.FILE, "Collision mesh resolution", problem="Some collision meshes could not be found")
def meshes(ctx: Context) -> list[Finding]:
    """Every collision mesh URI resolves to a file.

    Unresolved meshes are removed before loading. They only matter for links without <inertial>
    (engines that derive mass from geometry) and for contact, which simcompat does not compare yet.

    Fix: pass --package-dir NAME=PATH for package:// URIs.
    """
    prepared = ctx.prepared
    findings = []
    unresolved = prepared.unresolved_meshes
    if unresolved:
        packages = sorted({uri.split("/")[2] for uri in unresolved if uri.startswith("package://")})
        hint = (
            "Point simcompat at the package(s): " + " ".join(f"--package-dir {p}=PATH" for p in packages)
            if packages
            else "Check the mesh paths; relative paths are resolved from the URDF's directory."
        )
        shown = ", ".join(unresolved[:3]) + (f" and {len(unresolved) - 3} more" if len(unresolved) > 3 else "")
        findings.append(
            Finding(
                Severity.WARN,
                f"{len(unresolved)} collision mesh(es) not found: {shown}",
                data={"unresolved": unresolved[:50], "packages": packages},
                hint=hint,
            )
        )
    formats = ", ".join(prepared.mesh_extensions) or "none"
    findings.append(
        Finding(
            Severity.PASS,
            f"{prepared.stripped_visuals} <visual> element(s) ignored; collision mesh formats: {formats}",
        )
    )
    return findings


@check("file.joint_types", Tier.FILE, "Joint types covered", problem="Some joint types are not compared yet")
def joint_types(ctx: Context) -> list[Finding]:
    """simcompat compares revolute, continuous, prismatic and fixed joints.

    Floating and planar joints are reported, and how each engine builds them is compared by
    import.multi_dof, but their motion is not compared yet.
    """
    other = [j for j in ctx.declared.joints.values() if j.type in MULTI_DOF_TYPES]
    if other:
        return [
            Finding(
                Severity.INFO,
                f"{joint.type} joint: only how engines build it is compared (see import.multi_dof)",
                subject=joint.name,
            )
            for joint in other
        ]
    return [Finding(Severity.PASS, f"{len(ctx.joints)} movable joint(s), all of supported types")]
