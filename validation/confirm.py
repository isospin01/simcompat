"""Counterfactual confirmation of simcompat's root-cause attributions on the corpus.

When simcompat says *why* an engine differs from the file, we can test the claim: change the one
thing it blames and re-run. If the attribution is right, the difference disappears.

* MuJoCo, links without <inertial>: "MuJoCo derives their mass from collision geometry". Re-run
  with MuJoCo told not to (``inertiafromgeom=false``, plus ``boundmass``/``boundinertia`` far below
  the tolerances and ``fusestatic=false`` so that massless bodies still load).
* MuJoCo, links with zero mass but non-zero inertia: "MuJoCo drops that inertia". Re-run with
  those links given a negligible mass (1e-9 kg).
* PyBullet (inertia from file), links without <inertial>: "PyBullet assigns 1 kg". Checked
  arithmetically: engine mass - declared mass must be exactly 1 kg per such link.
* PyBullet (inertia from file), links with a non-physical inertia tensor: "PyBullet sets it to
  zero". Re-run with those tensors made physical (eigenvalues clipped to the triangle inequality).

    python validation/confirm.py --corpus validation/results/corpus --out validation/results/confirm.jsonl
"""

from __future__ import annotations

import argparse
import json
import tempfile
import xml.etree.ElementTree as ET
from collections.abc import Callable
from pathlib import Path

import numpy as np

from simcompat.config import RunConfig
from simcompat.inputs import RobotInput, prepare_urdf, resolve
from simcompat.runner import run
from simcompat.spatial import inertia_matrix

CONFIG = RunConfig(samples=2, rollout_samples=1)
# fusestatic=false: MuJoCo cannot fuse massless static bodies; it does not change what is compared
NO_GEOMETRY = "mujoco:inertiafromgeom=false,fusestatic=false,boundmass=1e-13,boundinertia=1e-15"
NEGLIGIBLE_MASS = 1e-9  # kg


def claims(report: dict, backend: str, key: str) -> list[dict]:
    """inertia.clusters FAILs of ``backend`` that simcompat attributed to the links in ``data[key]``."""
    return [
        f
        for f in report["findings"]
        if f["check"] == "inertia.clusters"
        and f["backend"] == backend
        and f["severity"] == "FAIL"
        and f["data"].get(key)
    ]


def outcome(name: str, cause: str, found: list[dict], after, engine: str = "mujoco") -> dict:
    """Whether the counterfactual run made the claimed differences disappear."""
    testable = all(b.ok for b in after.backends)
    failing = {f.subject for f in after.findings if f.check == "inertia.clusters" and f.severity.name == "FAIL"}
    left = {f["subject"] for f in found} & failing if testable else set()
    return {
        "robot": name,
        "engine": engine,
        "cause": cause,
        "claims": len(found),
        "testable": testable,
        "confirmed": testable and not left,
        "remaining": sorted(left),
    }


def confirm_mujoco_geometry(name: str, report: dict) -> dict | None:
    found = claims(report, "mujoco", "links_without_inertial")
    if not found:
        return None
    after = run(f"rd:{name}", backends=["pinocchio", NO_GEOMETRY], only=["inertia.clusters"], config=CONFIG)
    return outcome(name, "links without <inertial>", found, after)


def rerun_with_changed_links(name: str, links: set[str], change: Callable[[ET.Element], None], backend: str):
    """Re-run inertia.clusters with ``change`` applied to the <inertial> of each of ``links``."""
    robot = resolve(f"rd:{name}")
    with tempfile.TemporaryDirectory() as workdir:
        prepared = prepare_urdf(robot.urdf, Path(workdir), robot.package_dirs)  # absolute mesh paths
        tree = ET.parse(prepared.path)
        for link in tree.getroot().findall("link"):
            inertial = link.find("inertial")
            if link.get("name") in links and inertial is not None:
                change(inertial)
        modified = Path(workdir) / "changed.urdf"
        tree.write(modified, encoding="unicode")
        changed = RobotInput(robot.spec, modified, robot.label, robot.package_dirs)
        return run(changed, backends=["pinocchio", backend], only=["inertia.clusters"], config=CONFIG)


def confirm_mujoco_massless(name: str, report: dict) -> dict | None:
    found = claims(report, "mujoco", "links_with_zero_mass")
    if not found:
        return None
    links = {link for f in found for link in f["data"]["links_with_zero_mass"]}
    after = rerun_with_changed_links(
        name, links, lambda el: el.find("mass").set("value", str(NEGLIGIBLE_MASS)), "mujoco"
    )
    return outcome(name, "links with zero mass but non-zero inertia", found, after)


def make_physical(inertial: ET.Element) -> None:
    """Replace the tensor by the nearest one that satisfies PSD and the triangle inequality (same axes)."""
    element = inertial.find("inertia")
    keys = ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")
    values = [float(element.get(k, 0.0)) for k in keys]
    moments, axes = np.linalg.eigh(inertia_matrix(*values))
    a, b = (max(m, 1e-12) for m in sorted(moments)[:2])
    c = min(max(sorted(moments)[2], b), (a + b) * (1 - 1e-6))
    order = np.argsort(moments)
    repaired = np.empty(3)
    repaired[order] = [a, b, max(c, b)]
    tensor = axes @ np.diag(repaired) @ axes.T
    for key, (i, j) in zip(keys, [(0, 0), (0, 1), (0, 2), (1, 1), (1, 2), (2, 2)], strict=True):
        element.set(key, repr(float(tensor[i, j])))


def confirm_pybullet_nonphysical(name: str, report: dict) -> dict | None:
    label = "pybullet[inertia_from_file=1]"
    others = ("links_without_inertial", "links_with_zero_mass")  # tested separately; would remain after the repair
    found = [
        f for f in claims(report, label, "links_with_nonphysical_inertia") if not any(f["data"][k] for k in others)
    ]
    if not found:
        return None
    links = {link for f in found for link in f["data"]["links_with_nonphysical_inertia"]}
    after = rerun_with_changed_links(name, links, make_physical, "pybullet:inertia_from_file=1")
    return outcome(name, "links with non-physical inertia", found, after, engine=label)


def confirm_pybullet(name: str, report: dict) -> dict | None:
    label = "pybullet[inertia_from_file=1]"
    found = claims(report, label, "links_without_inertial")
    if not found:
        return None
    exact = 0
    for f in found:
        extra = f["data"]["engine"]["mass"] - f["data"]["declared"]["mass"]
        exact += abs(extra - 1.0 * len(f["data"]["links_without_inertial"])) < 1e-6
    return {
        "robot": name,
        "engine": label,
        "cause": "links without <inertial>",
        "claims": len(found),
        "testable": True,
        "confirmed": exact == len(found),
        "exact_1kg_per_link": exact,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--corpus", required=True, type=Path)
    parser.add_argument("--out", required=True, type=Path)
    args = parser.parse_args()
    with args.out.open("w") as out:
        for path in sorted(args.corpus.glob("*_description.json")):
            report = json.loads(path.read_text())
            for confirm in (
                confirm_pybullet,
                confirm_pybullet_nonphysical,
                confirm_mujoco_geometry,
                confirm_mujoco_massless,
            ):
                try:
                    row = confirm(path.stem, report)
                except Exception as e:  # recorded, so one model cannot stop the run
                    row = {"robot": path.stem, "engine": confirm.__name__, "error": str(e)[:200]}
                if row:
                    out.write(json.dumps(row) + "\n")
                    print(row, flush=True)


if __name__ == "__main__":
    main()
