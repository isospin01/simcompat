"""Is simcompat's ground truth right? Compare its declared model with an independent URDF library.

simcompat's import, inertia and kinematics checks compare engines with its *own* reading of the
file. If that reading were wrong, every verdict would be wrong. This script checks it on the whole
corpus against yourdfpy (an independent URDF parser and forward-kinematics implementation):

* every link's mass,
* every link's world pose at random configurations (mimic joints follow their leaders).

Pinocchio provides a second, fully independent check of mass properties and kinematics inside the
corpus run itself (simcompat checks Pinocchio's import against the declared model like any engine).

    python validation/oracle.py --out validation/results/oracle.jsonl
"""

from __future__ import annotations

import argparse
import contextlib
import io
import json
import sys
from pathlib import Path

import numpy as np

from simcompat.declared import parse_urdf
from simcompat.inputs import resolve
from simcompat.spatial import rotation_angle

SAMPLES = 3


def _full_configuration(declared, actuated: dict[str, float]) -> dict[str, float]:
    """Add mimic joints, which follow ``multiplier * leader + offset`` (as yourdfpy does)."""
    q = dict(actuated)
    for _ in range(len(declared.joints)):  # mimic chains resolve in a few passes
        for joint in declared.joints.values():
            if joint.mimic is not None and joint.mimic.joint in q:
                q[joint.name] = joint.mimic.multiplier * q[joint.mimic.joint] + joint.mimic.offset
    return q


def compare(name: str) -> dict:
    import yourdfpy

    robot = resolve(f"rd:{name}")
    declared = parse_urdf(robot.urdf)
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        other = yourdfpy.URDF.load(
            str(robot.urdf), load_meshes=False, build_collision_scene_graph=False, load_collision_meshes=False
        )
    mass_errors = []
    for link_name, link in declared.links.items():
        theirs = other.link_map[link_name].inertial
        mass_theirs = float(theirs.mass) if theirs is not None and theirs.mass is not None else 0.0
        mass_ours = link.inertial.mass if link.inertial is not None else 0.0
        mass_errors.append(abs(mass_ours - mass_theirs))

    rng = np.random.default_rng(0)
    actuated = list(other.actuated_joint_names)
    position_error = angle_error = 0.0
    for _ in range(SAMPLES):
        values = {}
        for joint_name in actuated:
            joint = declared.joints[joint_name]
            low, high = (joint.lower, joint.upper) if joint.limited and joint.upper > joint.lower else (-1.0, 1.0)
            values[joint_name] = float(rng.uniform(low, high))
        other.update_cfg(np.array([values[j] for j in actuated]))
        ours = declared.link_poses(_full_configuration(declared, values))
        for link_name, pose in ours.items():
            theirs = other.get_transform(frame_to=link_name, frame_from=declared.root)
            position_error = max(position_error, float(np.linalg.norm(pose.p - theirs[:3, 3])))
            angle_error = max(angle_error, rotation_angle(pose.R, theirs[:3, :3]))
    return {
        "robot": name,
        "links": len(declared.links),
        "joints": len(declared.joints),
        "max_mass_error": max(mass_errors, default=0.0),
        "max_position_error_m": position_error,
        "max_angle_error_rad": angle_error,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--only", nargs="*")
    args = parser.parse_args()
    sys.path.insert(0, str(Path(__file__).parent))
    from corpus import corpus_names

    names = (
        corpus_names()
        if not args.only
        else [n if n.endswith("_description") else f"{n}_description" for n in args.only]
    )
    with args.out.open("w") as out:
        for name in names:
            try:
                row = compare(name)
            except Exception as e:  # recorded, so one model cannot stop the run
                row = {"robot": name, "error": f"{type(e).__name__}: {str(e).splitlines()[0][:200]}"}
            out.write(json.dumps(row) + "\n")
            out.flush()
            print(
                name,
                row.get("error")
                or f"mass {row['max_mass_error']:.1e}  pos {row['max_position_error_m']:.1e} m  "
                f"rot {row['max_angle_error_rad']:.1e} rad",
                flush=True,
            )


if __name__ == "__main__":
    main()
