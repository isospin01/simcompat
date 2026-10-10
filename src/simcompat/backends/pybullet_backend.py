"""PyBullet backend: the model as ``pybullet.loadURDF`` builds it (Featherstone multibody)."""

from __future__ import annotations

import contextlib
import re
import xml.etree.ElementTree as ET
from collections.abc import Callable
from typing import TypeVar

import numpy as np

from ..declared import DeclaredModel
from ..inputs import PreparedURDF
from ..spatial import quat_xyzw_to_matrix
from ._quiet import captured_native_output
from .base import (
    BUILT_BALL,
    BUILT_FIXED,
    BUILT_PLANAR,
    CAP_DYNAMICS,
    CAP_IMPORT,
    CAP_INERTIA,
    CAP_KINEMATICS,
    CAP_PASSIVE,
    CAP_ROLLOUT,
    HINT_DYNAMICS,
    HINT_FRICTION,
    HINT_INERTIA,
    HINT_MIMIC,
    MIMIC_IGNORED,
    Backend,
    BackendLoadError,
    BodyView,
    EngineQueryError,
    JointFrame,
    JointView,
    LoadInfo,
    Option,
    SimSettings,
)

T = TypeVar("T")
SPEED_CAP = 1e4  # rad/s or m/s; replaces PyBullet's default joint speed clamp of 100


class PyBulletBackend(Backend):
    name = "pybullet"
    is_simulator = True
    capabilities = frozenset({CAP_IMPORT, CAP_INERTIA, CAP_KINEMATICS, CAP_DYNAMICS, CAP_ROLLOUT, CAP_PASSIVE})
    options_spec = {
        "inertia_from_file": Option("pass URDF_USE_INERTIA_FROM_FILE (default off, as in PyBullet)"),
        "merge_fixed_links": Option("pass URDF_MERGE_FIXED_LINKS (default off)"),
    }
    client: int | None = None  # physics client of the model; None until load() succeeds
    _twin: tuple[int, int] | None = None  # (client, body) of the friction-free twin, if any

    @classmethod
    def availability(cls) -> tuple[bool, str]:
        try:
            with captured_native_output():  # pybullet prints a banner on import
                import pybullet  # noqa: F401
            from importlib.metadata import version

            return True, version("pybullet")
        except Exception as e:
            return False, f"pip install pybullet ({e.__class__.__name__})"

    # ------------------------------------------------------------------------------ loading
    def load(self, prepared: PreparedURDF, declared: DeclaredModel, settings: SimSettings) -> LoadInfo:
        with captured_native_output():
            import pybullet

        self.p = pybullet
        self._failure_reason: str | None = None
        flags = 0
        if self.option_enabled("inertia_from_file"):
            flags |= pybullet.URDF_USE_INERTIA_FROM_FILE
        if self.option_enabled("merge_fixed_links"):
            flags |= pybullet.URDF_MERGE_FIXED_LINKS
        inertia_from_file = bool(flags & pybullet.URDF_USE_INERTIA_FROM_FILE)

        self.client, self.body, importer_text = self._open(str(prepared.path), flags, settings)
        # A friction-free twin of the model makes rollout(friction=False) a real ablation, so a future
        # PyBullet that starts applying URDF joint friction would be detected rather than assumed.
        if any((declared.joints[n].friction or 0) > 0 for n in declared.movable_joints):
            self._twin = self._open(str(_without_friction(prepared)), flags, settings)[:2]

        joint_count = self.p.getNumJoints(self.body, physicsClientId=self.client)
        self._info = [self.p.getJointInfo(self.body, j, physicsClientId=self.client) for j in range(joint_count)]
        self._movable = self._movable_joints(self.client, self.body)
        self._names = [self._info[j][1].decode() for j in self._movable]
        self._index = dict(zip(self._names, self._movable, strict=True))
        self._damping = {j: float(self._info[j][6]) for j in self._movable}

        self._set_hints(inertia_from_file)
        return LoadInfo(
            version=self.availability()[1],
            settings=[
                f"loadURDF(useFixedBase=True, flags={flags})",
                "setGravity(0, 0, -9.81)  [PyBullet's default gravity is 0]",
                f"setTimeStep({settings.dt})",
                "default joint velocity motors disabled (force=0)  [PyBullet holds every joint with a motor]",
                "link linear/angular damping set to 0  [PyBullet's default is 0.04]",
                f"joint speed cap raised to {SPEED_CAP:g} rad/s  [PyBullet clamps joint speeds at 100 by default]",
            ],
            defaults={
                "base": "floating (loadURDF useFixedBase=False)",
                "gravity": "0 until setGravity is called",
                "joint motors": "a velocity motor on every joint, max force = URDF effort",
                "inertia": "from the URDF" if inertia_from_file else "recomputed from collision shapes",
                "missing <inertial>": "mass 1 kg",
                "mimic": "not supported by loadURDF (the joint stays independent)",
                "floating joint": "built as a fixed joint",
                "joint speed": "clamped at 100 rad/s",
            },
            warnings=_importer_messages(importer_text),
        )

    def _open(self, path: str, flags: int, settings: SimSettings) -> tuple[int, int, str]:
        """Load ``path`` into its own physics client with simcompat's runtime settings."""
        p = self.p
        client = p.connect(p.DIRECT)
        p.setGravity(*settings.gravity, physicsClientId=client)
        p.setTimeStep(settings.dt, physicsClientId=client)
        with captured_native_output() as printed:
            try:
                body = p.loadURDF(path, useFixedBase=True, flags=flags, physicsClientId=client)
            except Exception as e:
                p.disconnect(client)
                raise BackendLoadError(f"loadURDF failed: {e}", "Check mesh paths (--package-dir NAME=PATH).") from e
        joint_count = p.getNumJoints(body, physicsClientId=client)
        movable = self._movable_joints(client, body)
        if movable:  # PyBullet holds every joint with a velocity motor by default
            p.setJointMotorControlArray(
                body, movable, p.VELOCITY_CONTROL, forces=[0.0] * len(movable), physicsClientId=client
            )
        for link in range(-1, joint_count):  # and adds 0.04 linear/angular damping to every link
            p.changeDynamics(body, link, linearDamping=0.0, angularDamping=0.0, physicsClientId=client)
        for j in movable:  # and clamps joint speeds at 100 rad/s
            p.changeDynamics(body, j, maxJointVelocity=SPEED_CAP, physicsClientId=client)
        return client, body, printed["text"]

    def _query(self, fn: Callable[..., T], *args: object) -> T:
        """Call a pybullet function; turn its failures into EngineQueryError with PyBullet's own message."""
        error = None
        with captured_native_output() as printed:
            try:
                result = fn(*args, physicsClientId=self.client)
            except Exception as e:  # pybullet raises SystemError or pybullet.error
                error = e
        if error is None:
            return result
        lines = [line.strip() for line in printed["text"].splitlines() if line.strip()]
        reasons = [lines[i + 1] for i, line in enumerate(lines[:-1]) if line.startswith("b3Error")]  # message follows
        if reasons:  # PyBullet explains a failure only the first time; remember it for later calls
            self._failure_reason = reasons[0]
        raise EngineQueryError(f"pybullet.{fn.__name__} failed: {self._failure_reason or error}") from error

    def _set_hints(self, inertia_from_file: bool) -> None:
        if not inertia_from_file:
            self.hints[HINT_INERTIA] = self.hints[HINT_DYNAMICS] = (
                "PyBullet's loadURDF ignores URDF inertia tensors by default and recomputes them from collision "
                "shapes. Pass flags=URDF_USE_INERTIA_FROM_FILE (simcompat: -b pybullet:inertia_from_file=1)."
            )
        self.hints[HINT_FRICTION] = (
            "pybullet stores <dynamics friction> (getJointInfo) but loadURDF does not simulate it. Workaround: a "
            "VELOCITY_CONTROL motor with targetVelocity=0 and force=friction on each joint."
        )
        self.hints[HINT_MIMIC] = (
            "pybullet.loadURDF does not create <mimic> couplings; add them yourself, e.g. "
            "createConstraint(..., JOINT_GEAR, ...) with gearRatio = -multiplier."
        )

    def close(self) -> None:
        for client in (self.client, self._twin[0] if self._twin else None):
            if client is not None:
                with contextlib.suppress(Exception):
                    self.p.disconnect(client)

    def _movable_joints(self, client: int, body: int) -> list[int]:
        """Indices of the revolute and prismatic joints of ``body``."""
        movable = (self.p.JOINT_REVOLUTE, self.p.JOINT_PRISMATIC)
        count = self.p.getNumJoints(body, physicsClientId=client)
        return [j for j in range(count) if self.p.getJointInfo(body, j, physicsClientId=client)[2] in movable]

    # ------------------------------------------------------------------------------ helpers
    def _set_state(
        self,
        q: dict[str, float],
        qd: dict[str, float] | None = None,
        client: int | None = None,
        body: int | None = None,
    ) -> None:
        client = self.client if client is None else client
        body = self.body if body is None else body
        qd = qd or {}
        for name, j in self._index.items():
            self.p.resetJointState(body, j, float(q.get(name, 0.0)), float(qd.get(name, 0.0)), physicsClientId=client)

    def _vector(self, values: dict[str, float]) -> list[float]:
        return [float(values.get(n, 0.0)) for n in self._names]

    def _carrier(self, link: int) -> str | None:
        while link >= 0:
            if link in self._movable:
                return self._info[link][1].decode()
            link = int(self._info[link][16])  # parent index
        return None

    # ----------------------------------------------------------------------------- queries
    def joint_order(self) -> list[str]:
        return list(self._names)

    def default_base(self) -> str:
        return "floating"

    def multi_dof_joints(self) -> dict[str, str]:
        kinds = {
            self.p.JOINT_FIXED: BUILT_FIXED,
            self.p.JOINT_PLANAR: BUILT_PLANAR,
            self.p.JOINT_SPHERICAL: BUILT_BALL,
        }
        return {info[1].decode(): kinds[info[2]] for info in self._info if info[2] in kinds}

    def joint_views(self) -> dict[str, JointView]:
        views = {}
        for name, j in self._index.items():
            info = self._info[j]
            lower, upper = float(info[8]), float(info[9])
            limited = lower <= upper  # PyBullet's convention: lower > upper means unlimited
            views[name] = JointView(
                name=name,
                kind="revolute" if info[2] == self.p.JOINT_REVOLUTE else "prismatic",
                limited=limited,
                lower=lower if limited else None,
                upper=upper if limited else None,
                effort=float(info[10]),
                velocity=float(info[11]),
                damping=float(info[6]),
                friction=float(info[7]),
                armature=None,
                mimic=MIMIC_IGNORED,
            )
        return views

    def bodies(self) -> list[BodyView]:
        self._set_state({})
        out = []
        for link in range(len(self._info)):
            dynamics = self._query(self.p.getDynamicsInfo, self.body, link)
            state = self._query(self.p.getLinkState, self.body, link, 0, 1)  # computeForwardKinematics=1
            rot = quat_xyzw_to_matrix(state[1])  # COM (inertial) frame orientation
            out.append(
                BodyView(
                    name=self._info[link][12].decode(),
                    carrier=self._carrier(link),
                    mass=float(dynamics[0]),
                    com=np.array(state[0]),
                    inertia=rot @ np.diag(dynamics[2]) @ rot.T,
                )
            )
        return out

    def joint_frames(self, q: dict[str, float]) -> dict[str, JointFrame]:
        self._set_state(q)
        frames = {}
        for name, j in self._index.items():
            state = self._query(self.p.getLinkState, self.body, j, 0, 1)  # computeForwardKinematics=1
            link_rot = quat_xyzw_to_matrix(state[5])
            com_rot = quat_xyzw_to_matrix(state[1])  # pybullet expresses jointAxis in the child's inertial frame
            frames[name] = (np.array(state[4]), link_rot, com_rot @ np.array(self._info[j][13]))
        return frames

    def mass_matrix(self, q: dict[str, float]) -> np.ndarray:
        return np.array(self._query(self.p.calculateMassMatrix, self.body, self._vector(q)))

    def bias(self, q: dict[str, float], qd: dict[str, float]) -> np.ndarray:
        zeros = [0.0] * len(self._names)
        return np.array(
            self._query(self.p.calculateInverseDynamics, self.body, self._vector(q), self._vector(qd), zeros)
        )

    def rollout(
        self, q0: dict[str, float], qd0: dict[str, float], steps: int, *, damping: bool, friction: bool, mimic: bool
    ) -> np.ndarray:
        # friction=False runs the friction-free twin; mimic needs no switch (loadURDF creates no couplings)
        client, body = self._twin if (not friction and self._twin) else (self.client, self.body)
        trajectory = np.zeros((steps, len(self._names)))
        if not self._movable:
            return trajectory
        with captured_native_output():
            for j in self._movable:
                self.p.changeDynamics(
                    body, j, jointDamping=self._damping[j] if damping else 0.0, physicsClientId=client
                )
            self._set_state(q0, qd0, client, body)
            try:
                for k in range(steps):
                    self.p.stepSimulation(physicsClientId=client)
                    states = self.p.getJointStates(body, self._movable, physicsClientId=client)
                    trajectory[k] = [s[0] for s in states]
            except Exception as e:  # pybullet.error, e.g. on models with very many links
                raise EngineQueryError(f"pybullet rollout failed: {e}") from e
            finally:
                for j in self._movable:
                    self.p.changeDynamics(body, j, jointDamping=self._damping[j], physicsClientId=client)
        return trajectory


def _without_friction(prepared: PreparedURDF):
    """Copy of the prepared URDF with every <dynamics friction> set to 0."""
    tree = ET.parse(prepared.path)
    for dynamics in tree.getroot().iter("dynamics"):
        if dynamics.get("friction") is not None:
            dynamics.set("friction", "0")
    out = prepared.path.with_name(prepared.path.stem + ".nofriction.urdf")
    tree.write(out, encoding="unicode")
    return out


_MARKER = re.compile(r"b3(?:Warning|Error|Printf)\[[^\]]*\]:")
MAX_NAMES_PER_MESSAGE = 8


def _importer_messages(text: str) -> list[str]:
    """PyBullet's importer output as one readable line per message: ``message: link1, link2, ...``.

    PyBullet prints a message and the link it concerns as two separate warnings, without newlines.
    """
    parts = [line.strip() for chunk in _MARKER.split(text) for line in chunk.splitlines()]
    grouped: dict[str, list[str]] = {}
    last = None
    for part in parts:
        if not part or part.startswith("pybullet build time"):
            continue
        message, colon, name = part.rpartition(": ")
        if " " not in part and last is not None:  # a bare name belongs to the previous message
            grouped[last].append(part)
        elif colon and " " not in name:  # "message for link: name"
            last = message
            grouped.setdefault(message, []).append(name)
        else:
            last = part
            grouped.setdefault(part, [])
    lines = []
    for message, names in grouped.items():
        shown = ", ".join(dict.fromkeys(names[:MAX_NAMES_PER_MESSAGE]))
        more = f" and {len(names) - MAX_NAMES_PER_MESSAGE} more" if len(names) > MAX_NAMES_PER_MESSAGE else ""
        lines.append(f"importer: {message}" + (f": {shown}{more}" if names else ""))
    return lines[:20]
