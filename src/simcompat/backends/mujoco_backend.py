"""MuJoCo backend: the model as MuJoCo's built-in URDF importer builds it."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import numpy as np

from ..declared import DeclaredModel
from ..inputs import PreparedURDF
from ._quiet import captured_native_output
from .base import (
    BUILT_BALL,
    BUILT_FREE,
    CAP_DYNAMICS,
    CAP_IMPORT,
    CAP_INERTIA,
    CAP_KINEMATICS,
    CAP_PASSIVE,
    CAP_ROLLOUT,
    MIMIC_ENFORCED,
    MIMIC_IGNORED,
    Backend,
    BackendLoadError,
    BodyView,
    JointFrame,
    JointView,
    LoadInfo,
    Option,
    SimSettings,
)

SUPPORTED_MESHES = {".stl", ".obj", ".msh"}
#: User options: MuJoCo <compiler> attributes (the defaults are MuJoCo's URDF defaults).
COMPILER_OPTIONS = {
    "fusestatic": Option("merge bodies joined by fixed joints (MuJoCo's URDF default: on)"),
    "discardvisual": Option("discard visual geoms (MuJoCo's URDF default: on)"),
    "balanceinertia": Option("repair inertias that violate the triangle inequality (default: off)"),
    "inertiafromgeom": Option(
        "derive inertia from collision geometry (default: auto, only for bodies without <inertial>)",
        kind="choice",
        choices=("true", "false", "auto"),
    ),
    "boundmass": Option("lower bound on body mass in kg (default: 0)", kind="number"),
    "boundinertia": Option("lower bound on body inertia in kg m^2 (default: 0)", kind="number"),
}


def _compiler_value(key: str, value: str) -> str:
    """The XML spelling of a normalized option value (booleans are "true"/"false" in MuJoCo)."""
    if COMPILER_OPTIONS[key].kind == "bool":
        return "true" if value == "1" else "false"
    return value


class MuJoCoBackend(Backend):
    name = "mujoco"
    is_simulator = True
    capabilities = frozenset({CAP_IMPORT, CAP_INERTIA, CAP_KINEMATICS, CAP_DYNAMICS, CAP_ROLLOUT, CAP_PASSIVE})
    options_spec = COMPILER_OPTIONS

    @classmethod
    def availability(cls) -> tuple[bool, str]:
        try:
            import mujoco

            return True, mujoco.__version__
        except Exception as e:
            return False, f"pip install mujoco ({e.__class__.__name__})"

    # ------------------------------------------------------------------------------ loading
    def load(self, prepared: PreparedURDF, declared: DeclaredModel, settings: SimSettings) -> LoadInfo:
        import mujoco

        self._mj = mujoco
        info = LoadInfo(
            version=mujoco.__version__,
            settings=[
                "loaded with MuJoCo's URDF importer (MjModel.from_xml_path)",
                "<compiler strippath='false'> injected (mesh file resolution only)",
                f"timestep = {settings.dt}",
                "rollouts: contacts disabled; integrator left at the model default",
            ],
            defaults={
                "base": "fixed (the root link is welded to the world unless a 'floating' joint is declared)",
                "fusestatic": "true for URDF (bodies joined by fixed joints are merged)",
                "velocity limit": "not represented (MuJoCo has no joint velocity limit)",
                "effort limit": "mapped to the joint's actuatorfrcrange",
                "missing <inertial>": "mass and inertia inferred from collision geometry (inertiafromgeom=auto)",
            },
        )
        if self.options:
            info.settings.append("compiler options: " + ", ".join(f"{k}={v}" for k, v in sorted(self.options.items())))

        self.m = self._compile(prepared, info)
        self.m.opt.timestep = settings.dt
        self.m.opt.gravity[:] = settings.gravity
        self.d = mujoco.MjData(self.m)
        hinge, slide = int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE)
        self._joint_ids = [j for j in range(self.m.njnt) if int(self.m.jnt_type[j]) in (hinge, slide)]
        self._names = [self.m.joint(j).name for j in self._joint_ids]
        self._dofs = [int(self.m.jnt_dofadr[j]) for j in self._joint_ids]
        self._qpos_adr = {n: int(self.m.jnt_qposadr[j]) for n, j in zip(self._names, self._joint_ids, strict=True)}
        self._dof_adr = dict(zip(self._names, self._dofs, strict=True))
        return info

    def _compile(self, prepared: PreparedURDF, info: LoadInfo) -> Any:  # mujoco.MjModel
        """Compile the URDF; if MuJoCo cannot read a mesh format, retry without those collision meshes."""
        try:
            return self._from_xml(self._write_variant(prepared), info)
        except Exception as first:
            unreadable = {e for e in prepared.mesh_extensions if e not in SUPPORTED_MESHES}
            if not unreadable:
                raise BackendLoadError(str(first).strip(), load_hint(str(first))) from first
        try:
            model = self._from_xml(self._write_variant(prepared, drop=unreadable), info)
        except Exception as e:
            raise BackendLoadError(str(e).strip(), load_hint(str(e))) from e
        info.warnings.append(
            f"MuJoCo cannot read {sorted(unreadable)} meshes: those collision meshes were dropped "
            "(this only matters for links without <inertial>)"
        )
        return model

    def _from_xml(self, path: Path, info: LoadInfo) -> Any:
        """Compile, keeping MuJoCo's printed warnings for the report instead of the terminal."""
        with captured_native_output() as printed:
            model = self._mj.MjModel.from_xml_path(str(path))
        lines = [line.strip() for line in printed["text"].splitlines() if line.strip()]
        info.warnings += [f"importer: {line}" for line in dict.fromkeys(lines)][:20]
        return model

    def _write_variant(self, prepared: PreparedURDF, drop: set[str] | None = None) -> Path:
        tree = ET.parse(prepared.path)
        robot = tree.getroot()
        for link in robot.findall("link"):
            for collision in link.findall("collision"):
                mesh = collision.find("geometry/mesh")
                if drop and mesh is not None and Path(mesh.get("filename", "")).suffix.lower() in drop:
                    link.remove(collision)
        mujoco_el = robot.find("mujoco")
        if mujoco_el is None:
            mujoco_el = ET.SubElement(robot, "mujoco")
        compiler = mujoco_el.find("compiler")
        if compiler is None:
            compiler = ET.SubElement(mujoco_el, "compiler")
        compiler.set("strippath", "false")  # prepared URDFs use absolute mesh paths
        for key, value in self.options.items():
            compiler.set(key, _compiler_value(key, value))
        out = prepared.path.with_name(prepared.path.stem + ".mujoco.urdf")
        tree.write(out, encoding="unicode")
        return out

    # ------------------------------------------------------------------------------ helpers
    def _set_state(self, q: dict[str, float], qd: dict[str, float] | None = None) -> None:
        self._mj.mj_resetData(self.m, self.d)
        for name, value in q.items():
            if name in self._qpos_adr:
                self.d.qpos[self._qpos_adr[name]] = value
        for name, value in (qd or {}).items():
            if name in self._dof_adr:
                self.d.qvel[self._dof_adr[name]] = value
        self._mj.mj_forward(self.m, self.d)

    def _carrier(self, body: int) -> str | None:
        while body > 0:
            first, count = int(self.m.body_jntadr[body]), int(self.m.body_jntnum[body])
            for j in range(first, first + count):
                if j in self._joint_ids:
                    return self.m.joint(j).name
            body = int(self.m.body_parentid[body])
        return None

    # ----------------------------------------------------------------------------- queries
    def joint_order(self) -> list[str]:
        return list(self._names)

    def default_base(self) -> str:
        return "fixed"

    def multi_dof_joints(self) -> dict[str, str]:
        kinds = {int(self._mj.mjtJoint.mjJNT_FREE): BUILT_FREE, int(self._mj.mjtJoint.mjJNT_BALL): BUILT_BALL}
        return {
            self.m.joint(j).name: kinds[int(self.m.jnt_type[j])]
            for j in range(self.m.njnt)
            if int(self.m.jnt_type[j]) in kinds
        }

    def joint_views(self) -> dict[str, JointView]:
        m, mj = self.m, self._mj
        coupled = {int(m.eq_obj1id[e]) for e in range(m.neq) if int(m.eq_type[e]) == int(mj.mjtEq.mjEQ_JOINT)}
        views = {}
        for name, j in zip(self._names, self._joint_ids, strict=True):
            dof = int(m.jnt_dofadr[j])
            limited = bool(m.jnt_limited[j])
            views[name] = JointView(
                name=name,
                kind="revolute" if int(m.jnt_type[j]) == int(mj.mjtJoint.mjJNT_HINGE) else "prismatic",
                limited=limited,
                lower=float(m.jnt_range[j][0]) if limited else None,
                upper=float(m.jnt_range[j][1]) if limited else None,
                effort=float(m.jnt_actfrcrange[j][1]) if m.jnt_actfrclimited[j] else None,
                velocity=None,
                damping=float(m.dof_damping[dof]),
                friction=float(m.dof_frictionloss[dof]),
                armature=float(m.dof_armature[dof]),
                mimic=MIMIC_ENFORCED if j in coupled else MIMIC_IGNORED,
            )
        return views

    def bodies(self) -> list[BodyView]:
        self._set_state({})
        out = []
        for b in range(1, self.m.nbody):
            rot = self.d.ximat[b].reshape(3, 3)
            out.append(
                BodyView(
                    name=self.m.body(b).name,
                    carrier=self._carrier(b),
                    mass=float(self.m.body_mass[b]),
                    com=self.d.xipos[b].copy(),
                    inertia=rot @ np.diag(self.m.body_inertia[b]) @ rot.T,
                )
            )
        return out

    def joint_frames(self, q: dict[str, float]) -> dict[str, JointFrame]:
        self._set_state(q)
        frames = {}
        for name, j in zip(self._names, self._joint_ids, strict=True):
            body = int(self.m.jnt_bodyid[j])
            frames[name] = (self.d.xpos[body].copy(), self.d.xmat[body].reshape(3, 3).copy(), self.d.xaxis[j].copy())
        return frames

    def mass_matrix(self, q: dict[str, float]) -> np.ndarray:
        self._set_state(q)
        full = np.zeros((self.m.nv, self.m.nv))
        try:
            self._mj.mj_fullM(self.m, self.d, full)  # MuJoCo >= 3.3
        except TypeError:  # older signature
            self._mj.mj_fullM(self.m, full, self.d.qM)
        return full[np.ix_(self._dofs, self._dofs)]

    def bias(self, q: dict[str, float], qd: dict[str, float]) -> np.ndarray:
        self._set_state(q, qd)
        return self.d.qfrc_bias[self._dofs].copy()

    def rollout(
        self, q0: dict[str, float], qd0: dict[str, float], steps: int, *, damping: bool, friction: bool, mimic: bool
    ) -> np.ndarray:
        mj, m = self._mj, self.m
        saved_flags, saved_damping = m.opt.disableflags, m.dof_damping.copy()
        try:
            flags = saved_flags | mj.mjtDisableBit.mjDSBL_CONTACT
            if not friction:
                flags |= mj.mjtDisableBit.mjDSBL_FRICTIONLOSS
            if not mimic:
                flags |= mj.mjtDisableBit.mjDSBL_EQUALITY
            m.opt.disableflags = flags
            if not damping:
                m.dof_damping[:] = 0.0
            self._set_state(q0, qd0)
            qpos = [self._qpos_adr[n] for n in self._names]
            trajectory = np.zeros((steps, len(self._names)))
            for k in range(steps):
                mj.mj_step(m, self.d)
                trajectory[k] = self.d.qpos[qpos]
            return trajectory
        finally:
            m.opt.disableflags = saved_flags
            m.dof_damping[:] = saved_damping


def load_hint(message: str) -> str | None:
    """Plain-language explanation for common MuJoCo compile errors."""
    low = message.lower()
    if "inertia must satisfy" in low or "inertia must have positive eigenvalues" in low:
        return (
            "MuJoCo rejects physically impossible inertia tensors (A + B >= C, positive eigenvalues), even on "
            "static links, while other engines accept them silently. Fix the tensor, or try "
            "`-b mujoco:balanceinertia=true`."
        )
    if "size 0 must be positive" in low:
        return "A collision primitive has a zero size; MuJoCo rejects it, other engines accept it."
    if "mass and inertia of moving bodies" in low:
        return (
            "A moving link has zero mass and inertia (often: no <inertial> and no collision geometry). "
            "MuJoCo refuses it; other engines substitute defaults. Add an <inertial> block."
        )
    if "mesh" in low and ("volume" in low or "faces" in low):
        return "A collision mesh is degenerate for MuJoCo's mesh-inertia computation."
    if "resource not found" in low or "could not open" in low or "error opening file" in low:
        return "A mesh could not be found. Pass --package-dir NAME=PATH for package:// URIs."
    return None
