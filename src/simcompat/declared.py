"""The *declared* model: what the URDF file says, read by simcompat's own parser.

This is the ground truth for the import, inertia and kinematics checks. It deliberately uses
no engine library, so "what the file says" can never be contaminated by an importer's
interpretation of it.
"""

from __future__ import annotations

import re
import xml.etree.ElementTree as ET
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .errors import SimCompatError
from .spatial import Transform, axis_angle_to_matrix, combine_inertias, inertia_matrix, rpy_to_matrix

MOVABLE_TYPES = ("revolute", "continuous", "prismatic")
MULTI_DOF_TYPES = ("floating", "planar")  # reported (import.multi_dof) but not compared yet
JOINT_TYPES = (*MOVABLE_TYPES, "fixed", *MULTI_DOF_TYPES)
UNBOUNDED = 1e9  # rad or m; a range at least this wide is effectively unlimited
INERTIA_NOT_PSD, INERTIA_TRIANGLE, INERTIA_EDGE = "not_psd", "triangle", "edge"  # Inertial.defect() kinds

# Joint-order conventions engines follow when they traverse the kinematic tree
DFS_FILE_ORDER = "depth-first, file order"
DFS_SORTED = "depth-first, siblings sorted by name"
BFS_FILE_ORDER = "breadth-first, file order"
BFS_SORTED = "breadth-first, siblings sorted by name"
DECLARATION_ORDER = "joint declaration order"


class URDFError(SimCompatError):
    """The file is not a URDF simcompat can read (the message says what to do)."""


# ----------------------------------------------------------------------------------------- model
@dataclass
class Inertial:
    mass: float
    xyz: np.ndarray  # COM in the link frame
    rpy: np.ndarray  # orientation of the inertia frame in the link frame
    inertia: np.ndarray  # 3x3 about the COM, in the inertia frame

    def inertia_in_link_frame(self) -> np.ndarray:
        rot = rpy_to_matrix(self.rpy)
        return rot @ self.inertia @ rot.T

    def defect(self) -> tuple[str, str, float] | None:
        """``(kind, description, size)`` if engines may not accept the tensor as it is, else None.

        A physical tensor is positive semi-definite and its principal moments satisfy A + B >= C.
        Kinds: INERTIA_NOT_PSD and INERTIA_TRIANGLE (impossible for a rigid body), and INERTIA_EDGE
        (A + B = C up to rounding: a perfectly flat or thin body, which a strict check may reject).
        """
        moments = np.linalg.eigvalsh(self.inertia)
        a, b, c = sorted(moments)
        if a < -1e-12:
            moments_text = np.round(moments, 9).tolist()
            return INERTIA_NOT_PSD, f"not positive semi-definite (principal moments {moments_text})", float(-a)
        if a + b < c * (1 - 1e-9):
            text = f"principal moments violate the triangle inequality ({a:.3g} + {b:.3g} < {c:.3g})"
            return INERTIA_TRIANGLE, text, float(c - a - b)
        if c > 0 and a + b <= c * (1 + 1e-9):
            text = f"principal moments on the edge of the triangle inequality ({a:.3g} + {b:.3g} = {c:.3g})"
            return INERTIA_EDGE, text, 0.0
        return None


@dataclass
class Link:
    name: str
    inertial: Inertial | None


@dataclass
class Mimic:
    joint: str
    multiplier: float = 1.0
    offset: float = 0.0


@dataclass
class Joint:
    name: str
    type: str
    parent: str
    child: str
    xyz: np.ndarray
    rpy: np.ndarray
    axis: np.ndarray  # unit vector in the joint (= child link) frame
    axis_norm: float = 1.0  # norm of the axis as written (URDF expects 1)
    lower: float | None = None
    upper: float | None = None
    effort: float | None = None
    velocity: float | None = None
    damping: float | None = None
    friction: float | None = None
    mimic: Mimic | None = None
    ignored_range: tuple[float, float] | None = None  # lower/upper written on a continuous joint (spec: ignored)

    @property
    def movable(self) -> bool:
        return self.type in MOVABLE_TYPES

    @property
    def limited(self) -> bool:
        return self.type in ("revolute", "prismatic") and self.lower is not None and self.upper is not None

    @property
    def zero_width(self) -> bool:
        """``lower == upper`` (also what an omitted lower/upper defaults to)."""
        return self.limited and self.lower == self.upper

    @property
    def is_rotational(self) -> bool:
        return self.type in ("revolute", "continuous")

    def origin(self) -> Transform:
        return Transform.from_xyz_rpy(self.xyz, self.rpy)

    def motion(self, q: float) -> Transform:
        if self.is_rotational:
            return Transform(axis_angle_to_matrix(self.axis, q))
        if self.type == "prismatic":
            return Transform(p=self.axis * q)
        return Transform()


@dataclass
class DeclaredModel:
    name: str
    path: Path
    links: dict[str, Link]
    joints: dict[str, Joint]  # in declaration order
    root: str

    # ---- tree -------------------------------------------------------------------------------
    def child_joints(self, link: str, sort_by_name: bool = False) -> list[Joint]:
        children = [j for j in self.joints.values() if j.parent == link]
        return sorted(children, key=lambda j: j.name) if sort_by_name else children

    def parent_joint(self, link: str) -> Joint | None:
        return next((j for j in self.joints.values() if j.child == link), None)

    @property
    def movable_joints(self) -> list[str]:
        """Canonical joint order used in every report: depth-first, siblings in file order."""
        return self.joint_order_dfs()

    def carrier(self, link: str) -> str | None:
        """The nearest movable joint that moves ``link`` (None if it is rigidly attached to the root)."""
        joint = self.parent_joint(link)
        while joint is not None:
            if joint.movable:
                return joint.name
            joint = self.parent_joint(joint.parent)
        return None

    def links_without_inertial(self) -> list[str]:
        return [name for name, link in self.links.items() if link.inertial is None]

    # ---- joint orders (engines traverse the tree differently) -------------------------------
    def joint_order_dfs(self, sort_by_name: bool = False) -> list[str]:
        order: list[str] = []

        def visit(link: str) -> None:
            for joint in self.child_joints(link, sort_by_name):
                if joint.movable:
                    order.append(joint.name)
                visit(joint.child)

        visit(self.root)
        return order

    def joint_order_bfs(self, sort_by_name: bool = False) -> list[str]:
        order: list[str] = []
        queue = deque([self.root])
        while queue:
            for joint in self.child_joints(queue.popleft(), sort_by_name):
                if joint.movable:
                    order.append(joint.name)
                queue.append(joint.child)
        return order

    def joint_order_declared(self) -> list[str]:
        return [j.name for j in self.joints.values() if j.movable]

    def order_conventions(self) -> dict[str, list[str]]:
        """Known traversal conventions; used to name the convention each engine follows."""
        return {
            DFS_FILE_ORDER: self.joint_order_dfs(),
            DFS_SORTED: self.joint_order_dfs(sort_by_name=True),
            BFS_FILE_ORDER: self.joint_order_bfs(),
            BFS_SORTED: self.joint_order_bfs(sort_by_name=True),
            DECLARATION_ORDER: self.joint_order_declared(),
        }

    # ---- kinematics -------------------------------------------------------------------------
    def link_poses(self, q: dict[str, float] | None = None) -> dict[str, Transform]:
        """World pose of every link frame (fixed base at the origin)."""
        q = q or {}
        poses = {self.root: Transform()}

        def visit(link: str) -> None:
            for joint in self.child_joints(link):
                angle = float(q.get(joint.name, 0.0)) if joint.movable else 0.0
                poses[joint.child] = poses[link] @ joint.origin() @ joint.motion(angle)
                visit(joint.child)

        visit(self.root)
        return poses

    def joint_frames(self, q: dict[str, float] | None = None) -> dict[str, tuple[np.ndarray, np.ndarray, np.ndarray]]:
        """For each movable joint: (child frame position, child frame rotation, joint axis), world frame."""
        poses = self.link_poses(q)
        frames = {}
        for name in self.movable_joints:
            joint = self.joints[name]
            pose = poses[joint.child]
            frames[name] = (pose.p.copy(), pose.R.copy(), pose.R @ joint.axis)
        return frames

    # ---- mass properties --------------------------------------------------------------------
    def link_inertias(
        self, q: dict[str, float] | None = None
    ) -> list[tuple[str, str | None, float, np.ndarray, np.ndarray]]:
        """``(link, carrier, mass, com_world, inertia_world)`` for every link, from declared values.

        A link without ``<inertial>`` has zero mass, which is what the URDF specification defines.
        """
        poses = self.link_poses(q)
        rows = []
        for name, link in self.links.items():
            pose = poses.get(name)
            if pose is None:
                continue
            if link.inertial is None:
                rows.append((name, self.carrier(name), 0.0, pose.p.copy(), np.zeros((3, 3))))
                continue
            inertial = link.inertial
            inertia = pose.R @ inertial.inertia_in_link_frame() @ pose.R.T
            rows.append((name, self.carrier(name), float(inertial.mass), pose.apply(inertial.xyz), inertia))
        return rows

    def moving_bodies(
        self, q: dict[str, float] | None = None
    ) -> dict[str | None, tuple[float, np.ndarray, np.ndarray]]:
        """Composite ``(mass, com, inertia)`` per carrying joint (``None`` = the static root group)."""
        groups: dict[str | None, list] = {}
        for _, carrier, mass, com, inertia in self.link_inertias(q):
            groups.setdefault(carrier, []).append((mass, com, inertia))
        return {carrier: combine_inertias(parts) for carrier, parts in groups.items()}


# ----------------------------------------------------------------------------------------- parse
def _number(text: str, what: str) -> float:
    try:
        value = float(text)
    except ValueError:
        raise URDFError(f"{what}: {text!r} is not a number") from None
    if not np.isfinite(value):
        raise URDFError(f"{what}: {text!r} is not a finite number")
    return value


def _vector(text: str | None, n: int = 3, default: float = 0.0, what: str = "vector") -> np.ndarray:
    if text is None:
        return np.full(n, float(default))
    values = [_number(v, what) for v in text.split()]
    if len(values) != n:
        raise URDFError(f"{what}: expected {n} numbers separated by spaces, got {text!r}")
    return np.array(values)


def _attr(element: ET.Element | None, name: str, default: float | None = None) -> float | None:
    if element is None or element.get(name) is None:
        return default
    return _number(element.get(name), f"<{element.tag} {name}>")


def _parse_inertial(element: ET.Element | None) -> Inertial | None:
    if element is None:
        return None
    origin = element.find("origin")
    mass = _attr(element.find("mass"), "value", 0.0)
    tensor = element.find("inertia")
    components = {k: _attr(tensor, k, 0.0) for k in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")}
    return Inertial(
        mass=mass,
        xyz=_vector(origin.get("xyz") if origin is not None else None, what="<inertial><origin xyz>"),
        rpy=_vector(origin.get("rpy") if origin is not None else None, what="<inertial><origin rpy>"),
        inertia=inertia_matrix(**components),
    )


def _parse_link(element: ET.Element) -> Link:
    name = element.get("name")
    try:
        return Link(name=name, inertial=_parse_inertial(element.find("inertial")))
    except URDFError as e:
        raise URDFError(f"link {name!r}: {e}") from None


def _parse_joint(element: ET.Element) -> Joint:
    try:
        return _joint_from(element)
    except URDFError as e:
        raise URDFError(f"joint {element.get('name')!r}: {e}") from None


def _joint_from(element: ET.Element) -> Joint:
    name, jtype = element.get("name"), element.get("type")
    parent, child = element.find("parent"), element.find("child")
    if parent is None or child is None:
        raise URDFError("missing <parent> or <child>")
    origin = element.find("origin")
    axis_el = element.find("axis")
    axis = _vector(axis_el.get("xyz") if axis_el is not None else "1 0 0", what="<axis xyz>")
    axis_norm = float(np.linalg.norm(axis))
    limit = element.find("limit")
    dynamics = element.find("dynamics")
    mimic = element.find("mimic")
    joint = Joint(
        name=name,
        type=jtype,
        parent=parent.get("link"),
        child=child.get("link"),
        xyz=_vector(origin.get("xyz") if origin is not None else None, what="<origin xyz>"),
        rpy=_vector(origin.get("rpy") if origin is not None else None, what="<origin rpy>"),
        axis=axis / axis_norm if axis_norm > 0 else axis,
        axis_norm=axis_norm,
        effort=_attr(limit, "effort"),
        velocity=_attr(limit, "velocity"),
        damping=_attr(dynamics, "damping"),
        friction=_attr(dynamics, "friction"),
        mimic=Mimic(mimic.get("joint"), _attr(mimic, "multiplier", 1.0), _attr(mimic, "offset", 0.0))
        if mimic is not None
        else None,
    )
    if jtype in ("revolute", "prismatic") and limit is not None:
        # URDF: lower/upper default to 0 when <limit> is present
        joint.lower = _attr(limit, "lower", 0.0)
        joint.upper = _attr(limit, "upper", 0.0)
    elif jtype == "continuous" and limit is not None and (limit.get("lower") or limit.get("upper")):
        lower, upper = _attr(limit, "lower", 0.0), _attr(limit, "upper", 0.0)
        if lower > -UNBOUNDED or upper < UNBOUNDED:  # a range that could actually stop the joint
            joint.ignored_range = (lower, upper)
    return joint


def read_xml(path: str | Path) -> ET.ElementTree:
    """Parse XML like the engines' tolerant parsers do.

    Real URDFs often use namespace prefixes they never declare (``<sensor:camera>`` inside
    ``<gazebo>``). Engines ignore that; we declare the missing prefixes instead of failing.
    """
    try:
        return ET.parse(path)
    except ET.ParseError as e:
        if "unbound prefix" not in str(e):
            raise
    text = Path(path).read_text(encoding="utf-8", errors="replace")
    declared = set(re.findall(r"xmlns:([A-Za-z_][\w.-]*)", text))
    used = set(re.findall(r"</?([A-Za-z_][\w.-]*):[A-Za-z_]", text))  # element prefixes
    used |= set(re.findall(r"\s([A-Za-z_][\w.-]*):[\w.-]+=", text))  # attribute prefixes
    missing = sorted(used - declared - {"xmlns", "xml"})
    declarations = "".join(f' xmlns:{prefix}="urn:simcompat:undeclared:{prefix}"' for prefix in missing)
    text = re.sub(r"<robot\b", "<robot" + declarations, text, count=1)
    return ET.ElementTree(ET.fromstring(text))


def parse_urdf(path: str | Path) -> DeclaredModel:
    """Parse a plain URDF file. Raises :class:`URDFError` with an actionable message."""
    path = Path(path)
    try:
        root = read_xml(path).getroot()
    except ET.ParseError as e:
        raise URDFError(f"{path}: not valid XML ({e})") from e
    if root.tag != "robot":
        raise URDFError(f"{path}: root element is <{root.tag}>, expected <robot>")
    if any(isinstance(el.tag, str) and "xacro" in el.tag for el in root.iter()):
        raise URDFError(
            f"{path}: contains xacro macros; simcompat expands .xacro files itself "
            "(pip install xacrodoc) or run `xacro` first"
        )
    try:
        link_list = [_parse_link(el) for el in root.findall("link")]
        joint_list = [_parse_joint(el) for el in root.findall("joint")]
    except URDFError as e:
        raise URDFError(f"{path}: {e}") from None
    links = {link.name: link for link in link_list}
    joints = {joint.name: joint for joint in joint_list}
    for kind, items, unique in (("link", link_list, links), ("joint", joint_list, joints)):
        if len(unique) != len(items):
            duplicates = sorted({item.name for item in items if sum(i.name == item.name for i in items) > 1})
            raise URDFError(f"{path}: duplicate {kind} name(s) {duplicates}")
    for joint in joint_list:
        for role, link in (("parent", joint.parent), ("child", joint.child)):
            if link not in links:
                raise URDFError(f"{path}: joint {joint.name!r} names {role} link {link!r}, which is not declared")
    children = [j.child for j in joint_list]
    shared = sorted({c for c in children if children.count(c) > 1})
    if shared:
        raise URDFError(f"{path}: link(s) {shared} are the child of more than one joint (URDF must be a tree)")
    roots = [name for name in links if name not in set(children)]
    if len(roots) != 1:
        raise URDFError(f"{path}: expected exactly one root link, found {roots or 'none (the joints form a loop)'}")
    for joint in joint_list:
        if joint.type not in JOINT_TYPES:
            raise URDFError(f"{path}: joint {joint.name!r} has unknown type {joint.type!r}")
    return DeclaredModel(name=root.get("name", path.stem), path=path, links=links, joints=joints, root=roots[0])
