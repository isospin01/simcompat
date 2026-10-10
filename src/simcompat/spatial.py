"""Small rigid-body math helpers (numpy only, no engine dependencies).

Conventions: rotation matrices are 3x3 numpy arrays; inertia tensors are 3x3, about the body's
own center of mass, expressed in the frame named by the caller.
"""

from __future__ import annotations

from collections.abc import Iterable, Sequence

import numpy as np


def rpy_to_matrix(rpy: Sequence[float]) -> np.ndarray:
    """URDF roll-pitch-yaw (fixed axes X, then Y, then Z): ``R = Rz(yaw) @ Ry(pitch) @ Rx(roll)``."""
    roll, pitch, yaw = (float(v) for v in rpy)
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def axis_angle_to_matrix(axis: Sequence[float], angle: float) -> np.ndarray:
    """Rodrigues' formula. A zero axis gives the identity."""
    a = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(a)
    if norm == 0:
        return np.eye(3)
    a = a / norm
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(angle) * k + (1 - np.cos(angle)) * (k @ k)


def quat_xyzw_to_matrix(quat: Sequence[float]) -> np.ndarray:
    """Quaternion in (x, y, z, w) order, as used by PyBullet, to a rotation matrix."""
    x, y, z, w = (float(v) for v in quat)
    n = np.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def rotation_angle(rot_a: np.ndarray, rot_b: np.ndarray) -> float:
    """Geodesic angle (rad) between two rotation matrices."""
    cos = (np.trace(rot_a.T @ rot_b) - 1.0) / 2.0
    return float(np.arccos(np.clip(cos, -1.0, 1.0)))


def vector_angle(a: Sequence[float], b: Sequence[float]) -> float:
    """Angle (rad) between two vectors; NaN if either is zero."""
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    na, nb = np.linalg.norm(a), np.linalg.norm(b)
    if na == 0 or nb == 0:
        return float("nan")
    return float(np.arccos(np.clip(np.dot(a, b) / (na * nb), -1.0, 1.0)))


def inertia_matrix(ixx: float, ixy: float, ixz: float, iyy: float, iyz: float, izz: float) -> np.ndarray:
    """Symmetric 3x3 inertia tensor from the six URDF components."""
    return np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]], dtype=float)


def combine_inertias(parts: Iterable[tuple[float, np.ndarray, np.ndarray]]) -> tuple[float, np.ndarray, np.ndarray]:
    """Merge rigid parts into one body (parallel-axis theorem).

    ``parts`` are ``(mass, com, inertia_about_own_com)``, all in one common frame.
    Returns ``(mass, com, inertia_about_combined_com)`` in that frame.
    """
    parts = list(parts)
    total = float(sum(mass for mass, _, _ in parts))
    if total <= 0:
        return 0.0, np.zeros(3), np.zeros((3, 3))
    com = sum(mass * np.asarray(c, dtype=float) for mass, c, _ in parts) / total
    inertia = np.zeros((3, 3))
    for mass, c, own in parts:
        r = np.asarray(c, dtype=float) - com
        inertia += np.asarray(own, dtype=float) + mass * (np.dot(r, r) * np.eye(3) - np.outer(r, r))
    return total, com, inertia


class Transform:
    """Rigid transform ``x -> R @ x + p``."""

    __slots__ = ("R", "p")

    def __init__(self, R: np.ndarray | None = None, p: np.ndarray | None = None):
        self.R = np.eye(3) if R is None else np.asarray(R, dtype=float)
        self.p = np.zeros(3) if p is None else np.asarray(p, dtype=float)

    def __matmul__(self, other: Transform) -> Transform:
        return Transform(self.R @ other.R, self.R @ other.p + self.p)

    def apply(self, x: Sequence[float]) -> np.ndarray:
        return self.R @ np.asarray(x, dtype=float) + self.p

    @staticmethod
    def from_xyz_rpy(xyz: Sequence[float], rpy: Sequence[float]) -> Transform:
        return Transform(rpy_to_matrix(rpy), np.asarray(xyz, dtype=float))
