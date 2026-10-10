"""Pinocchio backend: an analytical rigid-body dynamics library, used as the *reference*.

Pinocchio is not a simulator. It reads the URDF faithfully and evaluates textbook algorithms
(CRBA, RNEA, ABA), so it answers "what dynamics does this file declare?". simcompat still checks
Pinocchio's import against the declared model, like any other backend.
"""

from __future__ import annotations

import numpy as np

from ..declared import DeclaredModel
from ..inputs import PreparedURDF
from .base import (
    BUILT_BALL,
    BUILT_FREE,
    BUILT_PLANAR,
    CAP_DYNAMICS,
    CAP_IMPORT,
    CAP_INERTIA,
    CAP_KINEMATICS,
    CAP_ROLLOUT,
    HINT_MIMIC,
    MIMIC_IGNORED,
    Backend,
    BackendLoadError,
    BodyView,
    JointFrame,
    JointView,
    LoadInfo,
    SimSettings,
)


class PinocchioBackend(Backend):
    name = "pinocchio"
    is_simulator = False
    capabilities = frozenset({CAP_IMPORT, CAP_INERTIA, CAP_KINEMATICS, CAP_DYNAMICS, CAP_ROLLOUT})

    @classmethod
    def availability(cls) -> tuple[bool, str]:
        try:
            import pinocchio

            return True, pinocchio.__version__
        except Exception as e:
            return False, f"pip install pin ({e.__class__.__name__})"

    def load(self, prepared: PreparedURDF, declared: DeclaredModel, settings: SimSettings) -> LoadInfo:
        import pinocchio

        self.pin = pinocchio
        try:
            self.model = pinocchio.buildModelFromUrdf(str(prepared.path))
        except Exception as e:
            raise BackendLoadError(f"buildModelFromUrdf failed: {e}") from e
        self.model.gravity.linear = np.array(settings.gravity, dtype=float)
        self.data = self.model.createData()
        self.dt = settings.dt
        self._joint_ids = [i for i in range(1, self.model.njoints) if self.model.joints[i].nv == 1]
        self._names = [self.model.names[i] for i in self._joint_ids]
        self._id = dict(zip(self._names, self._joint_ids, strict=True))
        self.hints[HINT_MIMIC] = (
            "pinocchio.buildModelFromUrdf ignores <mimic> unless built with mimic=True; as a "
            "reference for unconstrained dynamics this is expected."
        )
        return LoadInfo(
            version=pinocchio.__version__,
            settings=[
                "pinocchio.buildModelFromUrdf (fixed base)",
                "rigid rollouts: semi-implicit Euler on ABA with the shared timestep (no limits)",
            ],
            defaults={
                "base": "fixed unless a root joint (e.g. JointModelFreeFlyer) is passed",
                "missing <inertial>": "zero mass and inertia (the URDF specification)",
                "mimic": "independent joint unless built with mimic=True",
            },
        )

    # ------------------------------------------------------------------------------ helpers
    def _configuration(self, q: dict[str, float]) -> np.ndarray:
        x = self.pin.neutral(self.model)
        for name, jid in self._id.items():
            joint, value = self.model.joints[jid], float(q.get(name, 0.0))
            if joint.nq == 1:
                x[joint.idx_q] = value
            else:  # continuous joint: (cos, sin) parameterization
                x[joint.idx_q : joint.idx_q + 2] = (np.cos(value), np.sin(value))
        return x

    def _velocity(self, qd: dict[str, float] | None) -> np.ndarray:
        v = np.zeros(self.model.nv)
        for name, jid in self._id.items():
            v[self.model.joints[jid].idx_v] = float((qd or {}).get(name, 0.0))
        return v

    def _angles(self, x: np.ndarray) -> np.ndarray:
        values = []
        for jid in self._joint_ids:
            joint = self.model.joints[jid]
            values.append(x[joint.idx_q] if joint.nq == 1 else np.arctan2(x[joint.idx_q + 1], x[joint.idx_q]))
        return np.array(values)

    @property
    def _velocity_indices(self) -> list[int]:
        return [self.model.joints[j].idx_v for j in self._joint_ids]

    # ----------------------------------------------------------------------------- queries
    def joint_order(self) -> list[str]:
        return list(self._names)

    def default_base(self) -> str:
        return "fixed"

    def multi_dof_joints(self) -> dict[str, str]:
        kinds = {"FreeFlyer": BUILT_FREE, "Planar": BUILT_PLANAR, "Spherical": BUILT_BALL}
        built = {}
        for jid in range(1, self.model.njoints):
            if self.model.joints[jid].nv != 1:
                short = self.model.joints[jid].shortname()
                built[self.model.names[jid]] = next((v for k, v in kinds.items() if k in short), short)
        return built

    def joint_views(self) -> dict[str, JointView]:
        model = self.model
        views = {}
        for name, jid in self._id.items():
            joint = model.joints[jid]
            short = joint.shortname()
            iv = joint.idx_v
            lower = upper = None
            limited = False
            if joint.nq == 1:  # continuous joints (nq == 2) are unlimited
                lo, hi = float(model.lowerPositionLimit[joint.idx_q]), float(model.upperPositionLimit[joint.idx_q])
                limited = bool(np.isfinite(lo) and np.isfinite(hi) and lo < hi and abs(lo) < 1e300)
                lower, upper = (lo, hi) if limited else (None, None)
            views[name] = JointView(
                name=name,
                kind="prismatic" if ("Prismatic" in short or short.startswith("JointModelP")) else "revolute",
                limited=limited,
                lower=lower,
                upper=upper,
                effort=float(model.effortLimit[iv]),
                velocity=float(model.velocityLimit[iv]),
                damping=float(model.damping[iv]),
                friction=float(model.friction[iv]),
                armature=float(model.armature[iv]) if hasattr(model, "armature") else None,
                mimic=MIMIC_IGNORED,
            )
        return views

    def bodies(self) -> list[BodyView]:
        self.pin.forwardKinematics(self.model, self.data, self._configuration({}))
        out = []
        for jid in range(1, self.model.njoints):
            inertia, pose = self.model.inertias[jid], self.data.oMi[jid]
            carrier = self.model.names[jid] if jid in self._joint_ids else None
            out.append(
                BodyView(
                    name=self.model.names[jid],
                    carrier=carrier,
                    mass=float(inertia.mass),
                    com=pose.act(inertia.lever),
                    inertia=pose.rotation @ inertia.inertia @ pose.rotation.T,
                )
            )
        return out

    def joint_frames(self, q: dict[str, float]) -> dict[str, JointFrame]:
        pin, model, data = self.pin, self.model, self.data
        pin.computeJointJacobians(model, data, self._configuration(q))
        frames = {}
        for name, jid in self._id.items():
            pose = data.oMi[jid]
            jacobian = np.asarray(pin.getJointJacobian(model, data, jid, pin.ReferenceFrame.LOCAL)).reshape(6, model.nv)
            motion = jacobian[:, model.joints[jid].idx_v]  # (linear, angular) in the joint frame
            local_axis = motion[3:] if np.linalg.norm(motion[3:]) > 0.5 else motion[:3]
            frames[name] = (pose.translation.copy(), pose.rotation.copy(), pose.rotation @ local_axis)
        return frames

    def mass_matrix(self, q: dict[str, float]) -> np.ndarray:
        upper = self.pin.crba(self.model, self.data, self._configuration(q))
        full = np.triu(upper) + np.triu(upper, 1).T
        idx = self._velocity_indices
        return full[np.ix_(idx, idx)]

    def bias(self, q: dict[str, float], qd: dict[str, float]) -> np.ndarray:
        forces = self.pin.nonLinearEffects(self.model, self.data, self._configuration(q), self._velocity(qd))
        return np.asarray(forces)[self._velocity_indices]

    def rollout(
        self, q0: dict[str, float], qd0: dict[str, float], steps: int, *, damping: bool, friction: bool, mimic: bool
    ) -> np.ndarray:
        if damping or friction:
            raise NotImplementedError("pinocchio is the rigid-body reference; it does not simulate damping or friction")
        pin, model, data = self.pin, self.model, self.data
        x, v, tau = self._configuration(q0), self._velocity(qd0), np.zeros(model.nv)
        trajectory = np.zeros((steps, len(self._joint_ids)))
        for k in range(steps):  # semi-implicit Euler, like MuJoCo's and PyBullet's default integrators
            v = v + self.dt * pin.aba(model, data, x, v, tau)
            x = pin.integrate(model, x, v * self.dt)
            trajectory[k] = self._angles(x)
        return np.unwrap(trajectory, axis=0)
