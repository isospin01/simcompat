"""Run simcompat on every URDF model in robot_descriptions (the public corpus).

Each robot runs in its own subprocess, so an engine crash (or hang) on one model cannot take the
whole run down; that robustness is itself part of what we measure.

    python validation/corpus.py --out validation/results/corpus            # all models
    python validation/corpus.py --out /tmp/c --only go2 g1 panda            # a subset
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

ENGINES = ["pinocchio", "mujoco", "pybullet", "pybullet:inertia_from_file=1"]


def corpus_names() -> list[str]:
    from robot_descriptions._descriptions import DESCRIPTIONS

    return sorted(name for name, description in DESCRIPTIONS.items() if description.has_urdf)


def check_one(name: str, out: Path, timeout: int) -> dict:
    report = out / f"{name}.json"
    command = [
        sys.executable,
        "-m",
        "simcompat",
        "check",
        f"rd:{name}",
        "--quiet",
        "--no-step-summary",
        "-o",
        str(report),
    ]
    for engine in ENGINES:
        command += ["-b", engine]
    started = time.perf_counter()
    try:
        done = subprocess.run(command, capture_output=True, text=True, timeout=timeout)
        status = {0: "ok", 1: "differences", 2: "input-error"}.get(done.returncode, f"crash({done.returncode})")
        stderr = done.stderr.strip().splitlines()[-1:] if done.returncode not in (0, 1) else []
    except subprocess.TimeoutExpired:
        status, stderr = "timeout", []
    return {
        "name": name,
        "status": status,
        "seconds": round(time.perf_counter() - started, 2),
        "report": report.name if report.exists() else None,
        "error": stderr[0] if stderr else None,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--only", nargs="*", help="subset of robot_descriptions names (without rd:)")
    parser.add_argument("--jobs", type=int, default=2)
    parser.add_argument("--timeout", type=int, default=900)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    names = (
        [n if n.endswith("_description") else f"{n}_description" for n in args.only] if args.only else corpus_names()
    )
    rows = []
    with ThreadPoolExecutor(args.jobs) as pool:
        for row in pool.map(lambda n: check_one(n, args.out, args.timeout), names):
            print(f"{row['name']:40} {row['status']:12} {row['seconds']:7.1f}s {row['error'] or ''}", flush=True)
            rows.append(row)
    (args.out / "index.json").write_text(json.dumps({"engines": ENGINES, "robots": rows}, indent=2))


if __name__ == "__main__":
    main()
