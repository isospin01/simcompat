"""Tier 3 - link frames and joint axes each engine simulates, compared with the file."""

from __future__ import annotations

import numpy as np

from ..backends.base import CAP_KINEMATICS
from ..spatial import rotation_angle, vector_angle
from .base import Context, Finding, Severity, Tier, check

REVERSED = np.pi - 1e-3  # an axis this far off points the other way
POSITION_SCALE = 0.01  # m; for ordering, a 1 cm offset weighs like a 1 rad rotation


def _worse(current: float, new: float) -> float:
    """Running maximum that keeps NaN once seen (NaN must never look like a small error)."""
    if not np.isfinite(current) or not np.isfinite(new):
        return float("nan")
    return max(current, new)


@check(
    "kinematics.frames",
    Tier.KINEMATICS,
    "Link frames and joint axes",
    requires=[CAP_KINEMATICS],
    problem="Engines place links or joint axes differently than the file declares",
)
def frames(ctx: Context) -> list[Finding]:
    """Child-link poses and joint axes match forward kinematics of the file.

    At every sampled configuration, simcompat compares the world pose of each movable joint's
    child link and the joint's axis direction. A reversed axis (positive q turns the other way)
    is called out explicitly.

    Typical causes: axis or origin conversion bugs, non-unit axes handled differently, rpy
    convention errors in converters.
    """
    tol = ctx.tol
    declared = [ctx.declared.joint_frames(q) for q in ctx.samples.configurations]
    findings = []
    for backend in ctx.with_cap(CAP_KINEMATICS):
        worst: dict[str, list[float]] = {}  # joint -> [position error, orientation error, axis error]
        for q, expected in zip(ctx.samples.configurations, declared, strict=True):
            actual = backend.joint_frames(q)
            for joint, (pos0, rot0, axis0) in expected.items():
                if joint not in actual:
                    continue
                pos1, rot1, axis1 = actual[joint]
                errors = (float(np.linalg.norm(pos1 - pos0)), rotation_angle(rot0, rot1), vector_angle(axis0, axis1))
                current = worst.setdefault(joint, [0.0, 0.0, 0.0])
                worst[joint] = [_worse(c, e) for c, e in zip(current, errors, strict=True)]
        problems = []
        for joint, (pos_err, rot_err, axis_err) in worst.items():
            if not all(np.isfinite([pos_err, rot_err, axis_err])):
                differences = ["frame or axis is NaN/inf (or the axis is zero)"]
            else:
                differences = []
                if axis_err > REVERSED:
                    differences.append("joint axis is REVERSED (positive q turns the other way)")
                elif axis_err > tol.angle_abs:
                    differences.append(f"joint axis off by {np.degrees(axis_err):.3g} deg")
                if pos_err > tol.position_abs:
                    differences.append(f"child frame off by {pos_err * 1000:.3g} mm")
                if rot_err > tol.angle_abs:
                    differences.append(f"child frame rotated by {np.degrees(rot_err):.3g} deg")
            if differences:
                score = max(pos_err / POSITION_SCALE, rot_err, axis_err)
                problems.append(
                    Finding(
                        Severity.FAIL,
                        "; ".join(differences),
                        backend=backend.label,
                        subject=joint,
                        data={"position_err_m": pos_err, "orientation_err_rad": rot_err, "axis_err_rad": axis_err},
                        score=score if np.isfinite(score) else float("inf"),
                    )
                )
        findings += problems or [
            Finding(
                Severity.PASS,
                f"{len(worst)} joint frames match at {ctx.samples.count} configurations",
                backend=backend.label,
                data={
                    "max_position_err_m": max((w[0] for w in worst.values()), default=0.0),
                    "max_angle_err_rad": max((max(w[1], w[2]) for w in worst.values()), default=0.0),
                },
            )
        ]
    return findings
