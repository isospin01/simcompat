"""Memoized engine queries, converted to the canonical (declared) joint order.

Several checks need the same expensive results (mass matrices, rollouts); computing each once per
backend keeps runs fast and checks short.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

from ..backends.base import reorder_columns, reorder_matrix, reorder_vector
from ..config import RunConfig
from ..declared import DeclaredModel
from ..sampling import Samples

if TYPE_CHECKING:
    from ..backends.base import Backend


MIN_ROLLOUT_STEPS = 10  # a rollout shorter than this says nothing (also the minimum compared length)


class Evaluator:
    def __init__(self, declared: DeclaredModel, samples: Samples, config: RunConfig):
        self.joints = declared.movable_joints
        self.samples = samples
        self.steps = max(MIN_ROLLOUT_STEPS, round(config.duration / config.dt))
        self._memo: dict[tuple, np.ndarray] = {}

    def _cached(self, key: tuple, compute) -> np.ndarray:
        if key not in self._memo:
            self._memo[key] = compute()
        return self._memo[key]

    def mass_matrix(self, backend: Backend, i: int) -> np.ndarray:
        q = self.samples.configurations[i]
        return self._cached(
            (backend.label, "M", i), lambda: reorder_matrix(backend.mass_matrix(q), backend.joint_order(), self.joints)
        )

    def gravity(self, backend: Backend, i: int) -> np.ndarray:
        q = self.samples.configurations[i]
        return self._cached(
            (backend.label, "g", i), lambda: reorder_vector(backend.bias(q, {}), backend.joint_order(), self.joints)
        )

    def coriolis(self, backend: Backend, i: int) -> np.ndarray:
        q, qd = self.samples.configurations[i], self.samples.velocities[i]
        return self._cached(
            (backend.label, "c", i),
            lambda: reorder_vector(backend.bias(q, qd), backend.joint_order(), self.joints) - self.gravity(backend, i),
        )

    def rollout(self, backend: Backend, i: int, *, damping: bool, friction: bool, mimic: bool) -> np.ndarray:
        q0, qd0 = self.samples.rollout_starts[i]
        return self._cached(
            (backend.label, "rollout", i, damping, friction, mimic),
            lambda: reorder_columns(
                backend.rollout(q0, qd0, self.steps, damping=damping, friction=friction, mimic=mimic),
                backend.joint_order(),
                self.joints,
            ),
        )
