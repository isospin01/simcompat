"""Seeded importer-bug benchmark: does simcompat catch real classes of importer bugs on real robots?

For every robot in the corpus we load the model with MuJoCo, then *inject one importer bug* into
the compiled MuJoCo model (exactly where real importer bugs live) and run simcompat against the
Pinocchio reference. Each bug class mirrors a bug reported in the wild:

    mass_scale          link mass 5% off                    (wrong defaults; Go2/G1 cases)
    com_shift           COM moved 5 mm                      (inertial origin conversion)
    inertia_frame_lost  inertial rotation dropped           (MuJoCo #3559 report)
    inertia_scale       inertia tensor 10% off              (PyBullet recomputing inertia)
    axis_flip           joint axis reversed                 (sign convention bugs)
    axis_tilt           joint axis tilted 1 degree          (axis conversion bugs)
    origin_shift        joint origin moved 1 cm             (origin conversion bugs)
    damping_dropped     <dynamics damping> lost             (Isaac Sim #841)
    friction_dropped    <dynamics friction> lost            (Isaac Sim #841)
    armature_injected   0.01 armature added                 (Genesis #2398)
    limits_dropped      position limits lost
    order_bfs           joints exposed breadth-first        (Isaac Gym vs Isaac Sim ordering)

A bug counts as *detected* if the mutated engine gets a WARN/FAIL/ERROR finding the clean engine
did not have; *right check* if one of them comes from the check meant for that bug class;
*localized* if that finding names the mutated joint; *own problem* if the report lists it as a
root-cause problem of its own for the mutated engine (rather than only as a symptom).

    python validation/mutations.py --out validation/results/mutations.jsonl [--only go2 g1]
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
import time
import traceback
from multiprocessing import Pool
from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np

from simcompat import backends
from simcompat.backends.base import Option, SimSettings
from simcompat.backends.mujoco_backend import MuJoCoBackend
from simcompat.checks.base import Severity
from simcompat.config import RunConfig
from simcompat.declared import parse_urdf
from simcompat.inputs import prepare_urdf, resolve
from simcompat.runner import run

EXPECTED = {
    "mass_scale": "inertia.clusters",
    "com_shift": "inertia.clusters",
    "inertia_frame_lost": "inertia.clusters",
    "inertia_scale": "inertia.clusters",
    "axis_flip": "kinematics.frames",
    "axis_tilt": "kinematics.frames",
    "origin_shift": "kinematics.frames",
    "damping_dropped": "import.damping_friction",
    "friction_dropped": "import.damping_friction",
    "armature_injected": "import.armature",
    "limits_dropped": "import.limits",
    "order_bfs": "import.joint_order",
}
JOINT_LEVEL = {"order_bfs"}  # findings name the engine, not a joint
CONFIG = dict(samples=4, rollout_samples=1, duration=0.25)


@backends.register
class MutantMuJoCo(MuJoCoBackend):
    """MuJoCo with one importer bug injected into the compiled model."""

    name = "mutant"
    options_spec = {
        "mutation": Option("bug class (see module docstring)", kind="choice", choices=tuple(EXPECTED)),
        "target": Option("joint to mutate", kind="text"),
    }

    def load(self, prepared, declared, settings):
        options, self.options = self.options, {}  # MuJoCoBackend turns options into <compiler> attributes
        try:
            info = super().load(prepared, declared, settings)
        finally:
            self.options = options
        self._declared = declared
        apply_mutation(self, options["mutation"], options["target"])
        self._mj.mj_setConst(self.m, self.d)
        return info

    def joint_order(self):
        order = super().joint_order()
        if self.options["mutation"] == "order_bfs":
            bfs = self._declared.joint_order_bfs()
            return [n for n in bfs if n in order] + [n for n in order if n not in bfs]
        return order


def _rotate(vector: np.ndarray, degrees: float) -> np.ndarray:
    axis = np.cross(vector, [1.0, 0, 0] if abs(vector[0]) < 0.9 else [0, 1.0, 0])
    axis /= np.linalg.norm(axis)
    angle = np.radians(degrees)
    return (
        vector * np.cos(angle)
        + np.cross(axis, vector) * np.sin(angle)
        + axis * np.dot(axis, vector) * (1 - np.cos(angle))
    )


def apply_mutation(backend: MutantMuJoCo, mutation: str, target: str) -> None:
    m = backend.m
    j = m.joint(target).id
    body, dof = int(m.jnt_bodyid[j]), int(m.jnt_dofadr[j])
    if mutation == "mass_scale":
        m.body_mass[body] *= 1.05
        m.body_inertia[body] *= 1.05
    elif mutation == "com_shift":
        m.body_ipos[body] += [0.005, 0.0, 0.0]
    elif mutation == "inertia_frame_lost":
        m.body_iquat[body] = [1.0, 0.0, 0.0, 0.0]
    elif mutation == "inertia_scale":
        m.body_inertia[body] *= 1.1
    elif mutation == "axis_flip":
        m.jnt_axis[j] *= -1
    elif mutation == "axis_tilt":
        m.jnt_axis[j] = _rotate(np.array(m.jnt_axis[j]), 1.0)
    elif mutation == "origin_shift":
        m.body_pos[body] += [0.01, 0.0, 0.0]
    elif mutation == "damping_dropped":
        m.dof_damping[dof] = 0.0
    elif mutation == "friction_dropped":
        m.dof_frictionloss[dof] = 0.0
    elif mutation == "armature_injected":
        m.dof_armature[dof] = 0.01
    elif mutation == "limits_dropped":
        m.jnt_limited[j] = 0
    elif mutation != "order_bfs":
        raise ValueError(mutation)


def applicable(mutation: str, joint: str, declared, mujoco: MuJoCoBackend) -> bool:
    """Whether injecting ``mutation`` at ``joint`` changes anything (otherwise it is not a bug)."""
    spec = declared.joints[joint]
    m = mujoco.m
    body = int(m.jnt_bodyid[m.joint(joint).id])
    if mutation in ("mass_scale", "com_shift", "inertia_scale"):
        return m.body_mass[body] > 1e-6
    if mutation == "inertia_frame_lost":
        moments = np.array(m.body_inertia[body])
        rotated = not np.allclose(np.abs(m.body_iquat[body]), [1, 0, 0, 0], atol=1e-3)
        return m.body_mass[body] > 1e-6 and rotated and np.ptp(moments) > 1e-3 * max(moments.max(), 1e-12)
    if mutation == "damping_dropped":
        return (spec.damping or 0) > 0
    if mutation == "friction_dropped":
        return (spec.friction or 0) > 0
    if mutation == "limits_dropped":
        return spec.limited and not spec.zero_width
    if mutation == "order_bfs":
        return declared.joint_order_bfs() != mujoco.joint_order()
    return True


def _problems(report, label: str) -> set[tuple[str, str | None]]:
    return {(f.check, f.subject) for f in report.findings if f.backend == label and f.severity >= Severity.WARN}


def benchmark_robot(name: str) -> list[dict]:
    """Baseline + every applicable mutation for one robot."""
    rows = []
    try:
        robot = resolve(f"rd:{name}")
        declared = parse_urdf(robot.urdf)
        config = RunConfig(**CONFIG)
        baseline = run(robot, backends=["pinocchio", "mujoco"], config=config)
        if not all(b.ok for b in baseline.backends):
            return [{"robot": name, "skipped": "an engine failed to load the clean model"}]
        clean = _problems(baseline, "mujoco")
        probe = backends.create("mujoco")  # the clean compiled model, to test where a mutation applies
        with TemporaryDirectory() as workdir:
            probe.load(prepare_urdf(robot.urdf, Path(workdir), robot.package_dirs), declared, SimSettings())
            rng = np.random.default_rng(int(hashlib.sha1(name.encode()).hexdigest()[:8], 16))
            for mutation, check_id in EXPECTED.items():
                candidates = [
                    j
                    for j in probe.joint_order()
                    if j in declared.movable_joints
                    and applicable(mutation, j, declared, probe)
                    and not any(c == check_id and (s == j or mutation in JOINT_LEVEL) for c, s in clean)
                ]
                if not candidates:
                    rows.append({"robot": name, "mutation": mutation, "applicable": False})
                    continue
                target = str(rng.choice(candidates))
                label = f"mutant[mutation={mutation},target={target}]"
                started = time.perf_counter()
                report = run(
                    robot, backends=["pinocchio", f"mutant:mutation={mutation},target={target}"], config=config
                )
                new = _problems(report, label) - clean
                right = {(c, s) for c, s in new if c == check_id}
                own = any(p.check == check_id and label in p.engines for p in report.problems)
                rows.append(
                    {
                        "robot": name,
                        "mutation": mutation,
                        "applicable": True,
                        "target": target,
                        "detected": bool(new),
                        "right_check": bool(right),
                        "localized": bool(right) and (mutation in JOINT_LEVEL or any(s == target for _, s in right)),
                        "own_problem": own,
                        "new_checks": sorted({c for c, _ in new}),
                        "seconds": round(time.perf_counter() - started, 2),
                    }
                )
    except Exception as e:
        rows.append({"robot": name, "error": f"{type(e).__name__}: {e}", "trace": traceback.format_exc(limit=3)})
    return rows


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--only", nargs="*")
    parser.add_argument("--jobs", type=int, default=2)
    args = parser.parse_args()
    from corpus import corpus_names  # same corpus as validation/corpus.py

    names = (
        [n if n.endswith("_description") else f"{n}_description" for n in args.only] if args.only else corpus_names()
    )
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open("w") as out, Pool(args.jobs) as pool:
        for rows in pool.imap_unordered(benchmark_robot, names):
            for row in rows:
                out.write(json.dumps(row) + "\n")
            out.flush()
            done = [r for r in rows if r.get("applicable")]
            status = rows[0].get("error") or rows[0].get("skipped") or f"{sum(r['detected'] for r in done)}/{len(done)}"
            print(f"{rows[0]['robot']:40} {status}", flush=True)


if __name__ == "__main__":
    sys.path.insert(0, str(Path(__file__).parent))
    main()
