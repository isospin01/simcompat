"""Where the model is evaluated: sampled configurations, velocities and rollout starts."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .config import RunConfig
from .declared import DeclaredModel

State = dict[str, float]  # joint name -> value


@dataclass
class Samples:
    configurations: list[State]  # q, within declared limits; the first one is q = 0
    velocities: list[State]  # qd paired with each configuration (for Coriolis checks)
    rollout_starts: list[tuple[State, State]]  # (q0, qd0), away from limits

    @property
    def count(self) -> int:
        return len(self.configurations)


def _draw(declared: DeclaredModel, rng: np.random.Generator, margin: float) -> State:
    """One configuration; ``margin`` is the fraction of each range kept away from the limits."""
    q = {}
    for name in declared.movable_joints:
        joint = declared.joints[name]
        if joint.limited and joint.upper > joint.lower:
            span = joint.upper - joint.lower
            q[name] = float(rng.uniform(joint.lower + margin * span, joint.upper - margin * span))
        elif joint.zero_width:
            q[name] = float(joint.lower)
        elif joint.type == "prismatic":
            q[name] = float(rng.uniform(-0.2, 0.2))
        else:  # continuous or unlimited revolute
            q[name] = float(rng.uniform(-np.pi, np.pi) if margin == 0 else rng.uniform(-1.0, 1.0))
    return q


def draw_samples(declared: DeclaredModel, config: RunConfig) -> Samples:
    """Deterministic (seeded) evaluation points for a run."""
    rng = np.random.default_rng(config.seed)
    joints = declared.movable_joints
    configurations = [dict.fromkeys(joints, 0.0)]
    configurations += [_draw(declared, rng, margin=0.0) for _ in range(max(0, config.samples - 1))]
    velocities = [{n: float(rng.normal(0.0, 1.0)) for n in joints} for _ in configurations]
    starts = [
        (_draw(declared, rng, margin=0.3), {n: float(rng.normal(0.0, 0.5)) for n in joints})
        for _ in range(config.rollout_samples)
    ]
    return Samples(configurations, velocities, starts)
