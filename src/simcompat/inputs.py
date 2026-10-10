"""Turning what the user typed into a URDF every engine can load.

Accepted robot specs:

* ``path/to/robot.urdf``            a plain URDF
* ``path/to/robot.urdf.xacro``      expanded with ``xacrodoc`` (``pip install xacrodoc``), no ROS needed
* ``path/to/my_robot_description``  a directory containing exactly one URDF or xacro file
* ``rd:go2`` / ``rd:go2_description``  any model from the ``robot_descriptions`` package
"""

from __future__ import annotations

import contextlib
import difflib
import hashlib
import io
import os
import re
import tempfile
import xml.etree.ElementTree as ET
from collections.abc import Callable, Iterator
from dataclasses import dataclass, field
from pathlib import Path
from types import ModuleType
from typing import Optional

from .declared import read_xml
from .errors import SimCompatError

ROBOT_DESCRIPTIONS_PREFIX = "rd:"
XACRO_SUFFIXES = (".xacro",)
XacroArgs = Optional[dict[str, str]]  # xacro arguments, NAME -> VALUE
_SKIP_DIRS = {".git", "build", "install", "log", "__pycache__", "node_modules"}


class InputError(SimCompatError):
    """The robot spec could not be turned into a URDF (the message says what to do)."""


@dataclass
class RobotInput:
    spec: str  # what the user typed
    urdf: Path  # plain URDF to check
    label: str  # short display name
    package_dirs: dict[str, str] = field(default_factory=dict)
    notes: list[str] = field(default_factory=list)  # how the spec was resolved (shown in reports)
    xacro_source: Path | None = None  # the xacro file ``urdf`` was expanded from, if any


def cache_dir() -> Path:
    """Where expanded xacro files are kept (``$SIMCOMPAT_CACHE`` or ``~/.cache/simcompat``)."""
    root = os.environ.get("SIMCOMPAT_CACHE") or os.path.join(
        os.environ.get("XDG_CACHE_HOME", os.path.expanduser("~/.cache")), "simcompat"
    )
    path = Path(root)
    path.mkdir(parents=True, exist_ok=True)
    return path


# ------------------------------------------------------------------------------------- resolve
def resolve(
    spec: str,
    package_dirs: dict[str, str] | None = None,
    xacro_args: dict[str, str] | None = None,
    log: Callable[[str], None] | None = None,
) -> RobotInput:
    """Resolve a robot spec (see module docstring) to a plain URDF."""
    log = log or (lambda _msg: None)
    package_dirs = _checked_package_dirs(package_dirs)
    if spec[: len(ROBOT_DESCRIPTIONS_PREFIX)].lower() == ROBOT_DESCRIPTIONS_PREFIX:
        found = _from_robot_descriptions(spec, package_dirs, xacro_args, log)
    elif Path(spec).expanduser().is_dir():
        found = _from_directory(spec, Path(spec).expanduser(), package_dirs, xacro_args)
    elif Path(spec).expanduser().is_file():
        found = _from_file(spec, Path(spec).expanduser(), package_dirs, xacro_args)
    else:
        hint = ""
        if "/" not in spec and not spec.endswith((".urdf", ".xacro")):
            hint = f" Did you mean the robot_descriptions model `{ROBOT_DESCRIPTIONS_PREFIX}{spec}`?"
        raise InputError(f"no such file or directory: {spec}.{hint}")
    if xacro_args:
        arguments = " ".join(f"{k}:={v}" for k, v in sorted(xacro_args.items()))
        if found.xacro_source is None:
            found.notes.append(f"ignored xacro arguments ({arguments}): the input is not a xacro file")
        else:
            found.notes.append(f"xacro arguments: {arguments}")
            unknown = _undeclared_xacro_args(found.xacro_source, xacro_args)
            if unknown:
                found.notes.append(unknown)
                log(f"warning: {unknown}")
    return found


def _undeclared_xacro_args(path: Path, xacro_args: dict[str, str]) -> str | None:
    """A warning if an argument is not declared by ``<xacro:arg>`` in the file (a likely typo)."""
    text = path.read_text(encoding="utf-8", errors="replace")
    declared = set(re.findall(r"<xacro:arg\s+name\s*=\s*[\"']([^\"']+)", text))
    unknown = sorted(set(xacro_args) - declared)
    if not unknown or not declared:  # no declarations at top level: arguments may live in included files
        return None
    known = ", ".join(sorted(declared))
    return f"xacro argument(s) {', '.join(unknown)} not declared in {path.name} (it declares {known})"


def _checked_package_dirs(package_dirs: dict[str, str] | None) -> dict[str, str]:
    """Copy of ``package_dirs`` with every path checked, so a typo is reported instead of ignored."""
    checked = {}
    for name, path in (package_dirs or {}).items():
        directory = Path(path).expanduser()
        if not directory.is_dir():
            raise InputError(f"--package-dir {name}={path}: no such directory")
        checked[name] = str(directory.resolve())
    return checked


def _from_file(spec: str, path: Path, package_dirs: dict[str, str], xacro_args: XacroArgs) -> RobotInput:
    if path.name.endswith(XACRO_SUFFIXES) or _uses_xacro(path):
        urdf = expand_xacro(path, package_dirs, xacro_args)
        return RobotInput(spec, urdf, _label(path), package_dirs, [f"expanded {path.name} with xacrodoc"], path)
    return RobotInput(spec, path, _label(path), package_dirs)


def _uses_xacro(path: Path) -> bool:
    """Some files named .urdf are really xacro templates (declaring the namespace alone does not count)."""
    text = path.read_text(encoding="utf-8", errors="replace")
    return re.search(r"<xacro:\w|\$\{|\$\((?:arg|find)\b", text) is not None


def _label(path: Path) -> str:
    name = path.name
    for suffix in (".urdf.xacro", ".xacro", ".urdf"):
        if name.endswith(suffix):
            return name[: -len(suffix)]
    return path.stem


def find_robot_files(directory: Path) -> list[Path]:
    """Plain URDF files under ``directory`` (or, if there are none, ``.urdf.xacro`` files), sorted."""
    urdfs, xacros = [], []
    for dirpath, dirnames, filenames in os.walk(directory):
        dirnames[:] = [d for d in dirnames if d not in _SKIP_DIRS and not d.startswith(".")]
        for name in filenames:
            if name.endswith(".urdf"):
                urdfs.append(Path(dirpath) / name)
            elif name.endswith(".urdf.xacro"):
                xacros.append(Path(dirpath) / name)
    return sorted(urdfs) or sorted(xacros)


def _from_directory(spec: str, directory: Path, package_dirs: dict[str, str], xacro_args: XacroArgs) -> RobotInput:
    candidates = find_robot_files(directory)
    if not candidates:
        raise InputError(f"no .urdf or .urdf.xacro file under {directory}")
    if len(candidates) > 1:
        listing = "\n  ".join(str(c) for c in candidates[:12])
        more = f"\n  ... and {len(candidates) - 12} more" if len(candidates) > 12 else ""
        raise InputError(f"{directory} contains {len(candidates)} robot files; pass one of them:\n  {listing}{more}")
    found = _from_file(spec, candidates[0], package_dirs, xacro_args)
    found.notes.insert(0, f"found {candidates[0].relative_to(directory)} in {directory}")
    return found


def _from_robot_descriptions(
    spec: str, package_dirs: dict[str, str], xacro_args: XacroArgs, log: Callable[[str], None]
) -> RobotInput:
    try:
        from robot_descriptions._descriptions import DESCRIPTIONS
    except ModuleNotFoundError as e:
        raise InputError("`rd:` specs need the robot_descriptions package: pip install robot_descriptions") from e
    name = spec[len(ROBOT_DESCRIPTIONS_PREFIX) :].strip().lower()
    urdf_names = sorted(n for n, d in DESCRIPTIONS.items() if d.has_urdf)
    if name not in DESCRIPTIONS and f"{name}_description" in DESCRIPTIONS:
        name = f"{name}_description"
    if name not in urdf_names:
        close = difflib.get_close_matches(name, urdf_names, n=4, cutoff=0.6)
        close += difflib.get_close_matches(short_name(name), [short_name(n) for n in urdf_names], n=4, cutoff=0.6)
        shown = list(dict.fromkeys(ROBOT_DESCRIPTIONS_PREFIX + short_name(c) for c in close))[:4]
        suggestion = f" Did you mean: {', '.join(shown)}?" if shown else ""
        kind = "has no URDF (MJCF-only)" if name in DESCRIPTIONS else "is not a robot_descriptions model"
        raise InputError(f"{name!r} {kind}.{suggestion} Run `simcompat robots` to list all.")

    if not _is_cached(DESCRIPTIONS[name].repository):
        log(f"downloading {name} via robot_descriptions (once; cached afterwards)...")
    import importlib

    try:
        with _silenced():  # robot_descriptions prints git clone progress bars
            module = importlib.import_module(f"robot_descriptions.{name}")
            urdf = _robot_descriptions_urdf(module, xacro_args)
    except InputError:
        raise
    except Exception as e:  # typically a failed download (git, network)
        first_line = str(e).strip().splitlines()[0] if str(e).strip() else type(e).__name__
        raise InputError(
            f"could not get {name} from robot_descriptions ({type(e).__name__}: {first_line}). "
            "The first use downloads the robot's repository: check the network connection."
        ) from e
    for attr in ("PACKAGE_PATH", "REPOSITORY_PATH"):
        if getattr(module, attr, None):
            package_dirs.setdefault(Path(getattr(module, attr)).name, getattr(module, attr))
    found = RobotInput(spec, Path(urdf), short_name(name), package_dirs, [f"robot_descriptions model {name}"])
    if not getattr(module, "URDF_PATH", None):
        found.xacro_source = Path(module.XACRO_PATH)
        found.notes.append(f"expanded {found.xacro_source.name} with xacrodoc")
    elif _uses_xacro(Path(urdf)):  # a few models ship xacro templates named .urdf
        found.xacro_source = Path(urdf)
        found.urdf = expand_xacro(found.xacro_source, package_dirs, xacro_args)
        found.notes.append(f"expanded {found.xacro_source.name} (a xacro template) with xacrodoc")
    return found


def _is_cached(repository: str) -> bool:
    """Whether robot_descriptions already cloned ``repository`` (so no download message is needed)."""
    try:
        from robot_descriptions._repositories import REPOSITORIES

        cache = os.environ.get("ROBOT_DESCRIPTIONS_CACHE", "~/.cache/robot_descriptions")
        return Path(cache).expanduser().joinpath(REPOSITORIES[repository].cache_path).is_dir()
    except Exception:  # private API: assume a download
        return False


def short_name(name: str) -> str:
    """``go2_description`` -> ``go2`` (what users type, and what suggestions show)."""
    return name[: -len("_description")] if name.endswith("_description") else name


def _robot_descriptions_urdf(module: ModuleType, xacro_args: XacroArgs) -> str:
    if getattr(module, "URDF_PATH", None):
        return module.URDF_PATH
    try:  # robot_descriptions >= 3 renders (and caches) xacro descriptions itself
        from robot_descriptions._xacro import get_urdf_path
    except ImportError:
        return str(expand_xacro(Path(module.XACRO_PATH), {}, xacro_args))
    try:
        return get_urdf_path(module, xacro_args=xacro_args)
    except ModuleNotFoundError as e:
        raise InputError(f"{module.__name__} is a xacro description: pip install xacrodoc") from e


@contextlib.contextmanager
def _working_directory(directory: Path) -> Iterator[None]:
    previous = Path.cwd()
    os.chdir(directory)
    try:
        yield
    finally:
        os.chdir(previous)


@contextlib.contextmanager
def _silenced() -> Iterator[None]:
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        yield


# --------------------------------------------------------------------------------------- xacro
def expand_xacro(
    path: Path, package_dirs: dict[str, str] | None = None, xacro_args: dict[str, str] | None = None
) -> Path:
    """Expand a .xacro file into a plain URDF (no ROS installation needed).

    The file is expanded on every call, because any included file may have changed; the result is
    written atomically to a path in the cache that depends on the input, the arguments and the
    package directories.
    """
    try:
        from xacrodoc import XacroDoc
        from xacrodoc import packages as xacro_packages
    except ModuleNotFoundError as e:
        raise InputError(
            f"{path.name} is a xacro file. Install the expander with `pip install xacrodoc`, "
            f"or run `xacro {path.name} > robot.urdf` and pass the result."
        ) from e
    if package_dirs:
        xacro_packages.update_package_cache(dict(package_dirs))
    path = Path(path).resolve()  # before changing directory for the includes
    key = f"{path}|{sorted((xacro_args or {}).items())}|{sorted((package_dirs or {}).items())}"
    out = cache_dir() / "xacro" / f"{_label(path)}-{hashlib.sha1(key.encode()).hexdigest()[:12]}.urdf"
    out.parent.mkdir(parents=True, exist_ok=True)
    handle, partial = tempfile.mkstemp(suffix=".urdf", dir=out.parent)
    os.close(handle)
    try:
        with _silenced(), _working_directory(path.parent):  # relative includes resolve from the file
            doc = XacroDoc.from_file(str(path), subargs=dict(xacro_args or {}))
            doc.to_urdf_file(partial)
        _absolutize_mesh_paths(Path(partial), relative_to=path.parent)
        os.replace(partial, out)  # atomic: a concurrent run never reads a half-written file
    except Exception as e:  # xacrodoc raises many exception types
        Path(partial).unlink(missing_ok=True)
        raise InputError(
            f"could not expand {path}: {e}. Missing packages can be given with "
            "--package-dir NAME=PATH; xacro arguments with --xacro-arg NAME:=VALUE."
        ) from e
    return out


def _absolutize_mesh_paths(urdf: Path, relative_to: Path) -> None:
    """The expanded file lives in the cache, so relative mesh paths must point back to the source."""
    tree = read_xml(urdf)
    changed = False
    for mesh in tree.getroot().iter("mesh"):
        uri = mesh.get("filename", "")
        if uri and "://" not in uri and not Path(uri).is_absolute():
            mesh.set("filename", str((relative_to / uri).resolve()))
            changed = True
    if changed:
        tree.write(urdf, encoding="unicode")


# ------------------------------------------------------------------------- loadable URDF copy
@dataclass
class PreparedURDF:
    """A self-contained copy of the URDF that every backend can load."""

    path: Path
    unresolved_meshes: list[str]
    stripped_visuals: int
    mesh_extensions: list[str]


def find_package(name: str, near: Path, package_dirs: dict[str, str]) -> Path | None:
    """Locate a ROS package directory for ``package://name/...`` without a ROS installation."""
    if name in package_dirs:
        return Path(package_dirs[name]).expanduser().resolve()
    candidates = []
    for ancestor in [near, *near.parents][:8]:
        if ancestor.name == name:
            candidates.append(ancestor)
        candidates.append(ancestor / name)
        manifest = ancestor / "package.xml"
        if manifest.exists():
            with contextlib.suppress(ET.ParseError):
                if (ET.parse(manifest).getroot().findtext("name") or "").strip() == name:
                    candidates.append(ancestor)
    for base in list(package_dirs.values()) + os.environ.get("ROS_PACKAGE_PATH", "").split(os.pathsep):
        if base:
            candidates += [Path(base) / name, Path(base).parent / name]
    return next((c.resolve() for c in candidates if c.is_dir()), None)


def resolve_uri(uri: str, near: Path, package_dirs: dict[str, str]) -> Path | None:
    """Absolute path for a mesh URI (``package://``, ``file://`` or relative), or None if missing."""
    if uri.startswith("package://"):
        package, _, rel = uri[len("package://") :].partition("/")
        base = find_package(package, near, package_dirs)
        path = base / rel if base else None
    else:
        path = Path(uri[len("file://") :] if uri.startswith("file://") else uri)
        if not path.is_absolute():
            path = near / path
    return path.resolve() if path is not None and path.exists() else None


def prepare_urdf(src: Path, out_dir: Path, package_dirs: dict[str, str] | None = None) -> PreparedURDF:
    """Write a loadable copy: absolute collision-mesh paths, ``<visual>`` stripped (it never affects physics)."""
    src = Path(src).resolve()
    tree = read_xml(src)
    unresolved, extensions, stripped = [], set(), 0
    for link in tree.getroot().findall("link"):
        for visual in link.findall("visual"):
            link.remove(visual)
            stripped += 1
        for collision in link.findall("collision"):
            mesh = collision.find("geometry/mesh")
            if mesh is None:
                continue
            uri = mesh.get("filename", "")
            extensions.add(Path(uri).suffix.lower())
            path = resolve_uri(uri, src.parent, package_dirs or {})
            if path is None:
                unresolved.append(uri)
                link.remove(collision)
            else:
                mesh.set("filename", str(path))
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    out = out_dir / "model.urdf"
    tree.write(out, encoding="unicode")
    return PreparedURDF(out, sorted(set(unresolved)), stripped, sorted(e for e in extensions if e))
