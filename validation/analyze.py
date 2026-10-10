"""Aggregate the validation runs into validation/RESULTS.md (every number in VALIDATION.md comes from here).

python validation/analyze.py --results validation/results --out validation/RESULTS.md
"""

from __future__ import annotations

import argparse
import json
import statistics
from collections import Counter, defaultdict
from pathlib import Path

CORE_CHECKS = [
    "import.dofs",
    "import.limits",
    "inertia.clusters",
    "kinematics.frames",
    "dynamics.mass_matrix",
    "dynamics.gravity",
    "dynamics.coriolis",
    "rollout.rigid",
]
BAD = {"FAIL", "ERROR"}
STATUS = {"ok": "0 (every engine matches)", "differences": "1 (an engine differs)", "input-error": "2 (input error)"}


def load_jsonl(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()] if path.exists() else []


def pct(part: int, whole: int) -> str:
    return f"{part}/{whole} ({100 * part / whole:.0f}%)" if whole else "0/0"


def table(header: list[str], rows: list[list]) -> list[str]:
    return ["| " + " | ".join(header) + " |", "|" + "---|" * len(header)] + [
        "| " + " | ".join(str(c) for c in row) + " |" for row in rows
    ]


# ------------------------------------------------------------------------------------- sections
def robustness(index: dict, reports: dict[str, dict]) -> list[str]:
    rows = index["robots"]
    status = Counter(r["status"] for r in rows)
    seconds = sorted(r["seconds"] for r in rows if r["report"])
    crashes = [
        (n, f["check"], f["message"])
        for n, rep in reports.items()
        for f in rep["findings"]
        if f["severity"] == "ERROR" and f["message"].startswith("check crashed")
    ]
    lines = [
        "## 1. Robustness on the whole corpus",
        "",
        f"- Models in robot_descriptions with a URDF (plain or xacro): **{len(rows)}**",
        f"- Checked end to end (a report was produced): **{pct(len(seconds), len(rows))}**",
        "- Exit codes: "
        + ", ".join(f"{STATUS.get(k, k)}: {v}" for k, v in sorted(status.items(), key=lambda kv: -kv[1])),
        f"- Crashed checks inside a report: **{len(crashes)}**; process crashes or timeouts: "
        f"**{sum(v for k, v in status.items() if k.startswith(('crash', 'timeout')))}**",
        f"- Wall time per robot (4 engine configurations, incl. Python start-up): median "
        f"{statistics.median(seconds):.1f} s, 90th percentile {seconds[int(0.9 * (len(seconds) - 1))]:.1f} s, "
        f"max {seconds[-1]:.1f} s",
        "",
    ]
    errors = [r for r in rows if r["status"] == "input-error"]
    if errors:
        lines += ["Inputs that could not be turned into a URDF (reported to the user as errors):", ""]
        lines += [f"- `{r['name']}`: {r['error']}" for r in errors]
        lines.append("")
    engines = defaultdict(lambda: [0, 0])
    reasons = defaultdict(Counter)
    for rep in reports.values():
        for b in rep["backends"]:
            engines[b["label"]][0 if b["ok"] else 1] += 1
            if not b["ok"]:
                reasons[b["label"]][_load_reason(b["error"] or "")] += 1
    lines += table(
        ["Engine", "Loaded", "Refused the model", "Why"],
        [[k, v[0], v[1], "; ".join(f"{r} ({c})" for r, c in reasons[k].most_common())] for k, v in engines.items()],
    )
    return [*lines, ""]


def _load_reason(error: str) -> str:
    low = error.lower()
    if "inertia must satisfy" in low or "positive eigenvalues" in low:
        return "physically invalid inertia"
    if "mass and inertia of moving bodies" in low:
        return "zero-mass moving body"
    if "size 0 must be positive" in low:
        return "zero-size collision geometry"
    if "mesh" in low or "file" in low:
        return "mesh problem"
    return error.splitlines()[0][:60]


def ground_truth(oracle: list[dict], reports: dict[str, dict]) -> list[str]:
    ok = [r for r in oracle if "error" not in r]
    lines = [
        "## 2. Is simcompat's ground truth right?",
        "",
        "simcompat's own parser vs. yourdfpy (an independent URDF library), every link of every model:",
        "",
    ]
    if ok:
        lines += table(
            [
                "Models compared",
                "Links",
                "Max link-mass difference",
                "Max link position difference",
                "Max link rotation difference",
            ],
            [
                [
                    len(ok),
                    sum(r["links"] for r in ok),
                    f"{max(r['max_mass_error'] for r in ok):.1e} kg",
                    f"{max(r['max_position_error_m'] for r in ok):.1e} m",
                    f"{max(r['max_angle_error_rad'] for r in ok):.1e} rad",
                ]
            ],
        )
        lines.append("")
        lines.append("(Rotation differences at ~1e-8 rad are the floor of the arccos-based angle metric, i.e. zero.)")
    failed = [r for r in oracle if "error" in r]
    if failed:
        lines += [
            "",
            f"yourdfpy could not load {len(failed)} model(s): "
            + "; ".join(f"`{r['robot']}` ({r['error'][:80]})" for r in failed),
        ]
    agree = [
        n
        for n, rep in reports.items()
        if all(
            f["severity"] in ("PASS", "SKIP")
            for f in rep["findings"]
            if f["backend"] == "pinocchio" and f["check"] in ("inertia.clusters", "kinematics.frames")
        )
        and any(b["label"] == "pinocchio" and b["ok"] for b in rep["backends"])
    ]
    loaded = [n for n, rep in reports.items() if any(b["label"] == "pinocchio" and b["ok"] for b in rep["backends"])]
    lines += [
        "",
        f"Pinocchio (a second independent implementation) matches the declared mass properties and "
        f"kinematics on **{pct(len(agree), len(loaded))}** of the models it loads.",
        "",
    ]
    return lines


def agreement(reports: dict[str, dict]) -> list[str]:
    lines = [
        "## 3. Correct imports agree to machine precision",
        "",
        "For each engine: models where every core check passes (the engine simulates exactly what the file "
        "declares), and the mass-matrix error vs. Pinocchio on those models.",
        "",
    ]
    rows = []
    for engine in ("mujoco", "pybullet", "pybullet[inertia_from_file=1]"):
        loaded = [rep for rep in reports.values() if any(b["label"] == engine and b["ok"] for b in rep["backends"])]
        clean, errors = 0, []
        for rep in loaded:
            core = [f for f in rep["findings"] if f["backend"] == engine and f["check"] in CORE_CHECKS]
            if core and all(f["severity"] in ("PASS", "SKIP") for f in core):
                clean += 1
                errors += [
                    f["data"]["rel_err"]
                    for f in core
                    if f["check"] == "dynamics.mass_matrix" and "rel_err" in f["data"]
                ]
        rows.append(
            [
                engine,
                pct(clean, len(loaded)),
                f"{statistics.median(errors):.1e}" if errors else "-",
                f"{max(errors):.1e}" if errors else "-",
            ]
        )
    lines += table(["Engine", "All core checks pass", "M(q) rel. error, median", "M(q) rel. error, max"], rows)
    return [*lines, ""]


def landscape(reports: dict[str, dict]) -> list[str]:
    engines = ["mujoco", "pybullet", "pybullet[inertia_from_file=1]", "pinocchio", "file"]
    counts: dict[str, Counter] = defaultdict(Counter)
    for rep in reports.values():
        seen = set()
        for f in rep["findings"]:
            if f["severity"] in ("WARN", "FAIL", "ERROR"):
                seen.add((f["check"], f["backend"] or "file", f["severity"]))
        for check, column, severity in seen:
            counts[check][(column, severity)] += 1
    total = len(reports)
    rows = []
    for check in sorted(counts):
        cells = []
        for engine in engines:
            parts = [(counts[check][(engine, s)], s.lower()) for s in ("FAIL", "ERROR", "WARN")]
            cells.append(" / ".join(f"{n} {name}" for n, name in parts if n))
        rows.append([f"`{check}`", *cells])
    lines = [
        "## 4. What simcompat finds on public robots",
        "",
        f"Number of models (out of {total} checked) with at least one finding of each kind, per engine.",
        "",
    ]
    lines += table(["Check", *engines], rows)
    any_engine = sum(
        1
        for rep in reports.values()
        if any(
            f["severity"] in BAD and f["backend"] in ("mujoco", "pybullet[inertia_from_file=1]")
            for f in rep["findings"]
        )
    )
    lines += [
        "",
        f"Models where MuJoCo or PyBullet (with inertia from file) simulates something different from the "
        f"file (FAIL/ERROR): **{pct(any_engine, total)}**.",
        "",
    ]
    return lines


def explanations(reports: dict[str, dict], confirms: list[dict]) -> list[str]:
    unexplained, attributed, engine_failures = [], Counter(), Counter()
    for name, rep in reports.items():
        for p in rep["problems"]:
            if p["unexplained"]:
                unexplained.append((name, p["check"], ", ".join(p["engines"])))
            elif p["check"].startswith("engine.failure"):
                if p["explains"]:
                    engine_failures[name] += len(p["explains"])
            elif p["explains"]:
                attributed[p["check"]] += len(p["explains"])
    lines = [
        "## 5. Are the findings explained? (precision)",
        "",
        "Every dynamics or rollout difference must be explained by a root-cause finding for the same engine; "
        "anything left over is either a new kind of divergence or a simcompat false positive, and is listed "
        "for manual review.",
        "",
        f"- Symptoms (check, engine) attached to a root cause: **{sum(attributed.values())}** "
        f"({', '.join(f'{k}: {v}' for k, v in attributed.most_common())})",
        f"- Symptoms on an engine that failed to answer the checks that would explain them: "
        f"{sum(engine_failures.values())} ({', '.join(f'`{k}`' for k in sorted(engine_failures)) or 'none'})",
        f"- Unexplained symptoms: **{len(unexplained)}**",
        "",
    ]
    if unexplained:
        lines += table(["Model", "Check", "Engines"], [[f"`{n}`", c, e] for n, c, e in unexplained])
        lines.append("")
    if confirms:
        lines += ["Counterfactual confirmation of the attributions (change what simcompat blames, re-run):", ""]
        tests = {
            ("mujoco", "links without <inertial>"): "re-run with inertiafromgeom=false: the difference disappears",
            ("pybullet[inertia_from_file=1]", "links with non-physical inertia"): "re-run with those tensors made "
            "physical: the difference disappears",
            ("mujoco", "links with zero mass but non-zero inertia"): "re-run with those links at 1e-9 kg: the "
            "difference disappears",
            ("pybullet[inertia_from_file=1]", "links without <inertial>"): "engine mass - declared mass = exactly "
            "1 kg per link without <inertial>",
        }
        rows = []
        for (engine, cause), how in tests.items():
            mine = [c for c in confirms if c.get("engine") == engine and c.get("cause") == cause]
            testable = [c for c in mine if c["testable"]]
            rows.append(
                [
                    engine,
                    cause,
                    how,
                    pct(sum(c["confirmed"] for c in testable), len(testable)),
                    len(mine) - len(testable),
                ]
            )
        rows.append(["pybullet", *pybullet_default(reports)])
        lines += table(["Engine", "Attributed cause", "Test", "Models confirmed", "Not testable"], rows)
        errors = [c for c in confirms if "error" in c]
        if errors:
            lines += ["", f"Confirmation errors: {len(errors)}"]
        lines.append("")
    return lines


def pybullet_default(reports: dict[str, dict]) -> list:
    """PyBullet's default (inertia recomputed from collision shapes), confirmed by the corpus run itself.

    The same model also ran with URDF_USE_INERTIA_FROM_FILE. The attribution is confirmed when every
    difference that remains with the flag is one simcompat attributes to the file.
    """
    suspects = ("links_without_inertial", "links_with_nonphysical_inertia", "links_with_zero_mass")
    confirmed = tested = 0
    for rep in reports.values():
        fails = defaultdict(set)
        attributed = set()
        for f in rep["findings"]:
            if f["check"] == "inertia.clusters" and f["severity"] == "FAIL":
                fails[f["backend"]].add(f["subject"])
                if any(f["data"].get(key) for key in suspects):
                    attributed.add((f["backend"], f["subject"]))
        if fails["pybullet"] and any(
            b["label"] == "pybullet[inertia_from_file=1]" and b["ok"] for b in rep["backends"]
        ):
            tested += 1
            confirmed += all(
                ("pybullet[inertia_from_file=1]", s) in attributed for s in fails["pybullet[inertia_from_file=1]"]
            )
    return [
        "inertia recomputed from collision shapes (its default)",
        "same model with URDF_USE_INERTIA_FROM_FILE: only differences attributed to the file remain",
        pct(confirmed, tested),
        0,
    ]


def readability(reports: dict[str, dict]) -> list[str]:
    """How much a user has to read: problems (grouped by cause) vs. the raw findings behind them."""
    verdicts = Counter(rep["verdict"] for rep in reports.values())
    problems = sorted(len(rep["problems"]) for rep in reports.values())
    raw = sorted(
        sum(1 for f in rep["findings"] if f["severity"] in ("WARN", "FAIL", "ERROR")) for rep in reports.values()
    )
    lines = [
        "## 8. What a user reads",
        "",
        "Verdicts: " + ", ".join(f"{k} {v}" for k, v in sorted(verdicts.items())) + ".",
        "",
    ]
    lines += table(
        ["Per report", "Median", "90th percentile", "Max"],
        [
            ["WARN/FAIL/ERROR findings", statistics.median(raw), raw[int(0.9 * (len(raw) - 1))], raw[-1]],
            [
                "Problems shown (grouped by cause)",
                statistics.median(problems),
                problems[int(0.9 * (len(problems) - 1))],
                problems[-1],
            ],
        ],
    )
    return [*lines, ""]


def mutations(rows: list[dict]) -> list[str]:
    done = [r for r in rows if r.get("applicable")]
    by_class = defaultdict(list)
    for r in done:
        by_class[r["mutation"]].append(r)
    table_rows = []
    for mutation, items in by_class.items():
        n = len(items)
        table_rows.append(
            [
                mutation,
                n,
                pct(sum(r["detected"] for r in items), n),
                pct(sum(r["right_check"] for r in items), n),
                pct(sum(r["localized"] for r in items), n),
                pct(sum(r["own_problem"] for r in items), n),
            ]
        )
    n = len(done)
    table_rows.append(
        [
            "**all**",
            n,
            pct(sum(r["detected"] for r in done), n),
            pct(sum(r["right_check"] for r in done), n),
            pct(sum(r["localized"] for r in done), n),
            pct(sum(r["own_problem"] for r in done), n),
        ]
    )
    robots = {r["robot"] for r in done}
    errors = [r for r in rows if "error" in r]
    lines = [
        "## 6. Seeded importer bugs on real robots (recall)",
        "",
        f"{n} injected bugs on {len(robots)} robots (one random eligible joint per robot and bug class).",
        "",
    ]
    lines += table(["Bug class", "Injected", "Detected", "Right check", "Right joint", "Own problem"], table_rows)
    if errors:
        lines += [
            "",
            f"Benchmark errors: {len(errors)} ("
            + "; ".join(f"{e['robot']}: {e['error'][:60]}" for e in errors[:5])
            + ")",
        ]
    missed = [r for r in done if not r["right_check"]]
    if missed:
        lines += ["", "Missed by the intended check:", ""]
        lines += table(
            ["Robot", "Bug", "Joint", "Caught by"],
            [[r["robot"], r["mutation"], r["target"], ", ".join(r["new_checks"]) or "nothing"] for r in missed],
        )
    return [*lines, ""]


def orders(reports: dict[str, dict]) -> list[str]:
    follows = defaultdict(Counter)
    differ = 0
    for rep in reports.values():
        data = next(
            (f["data"] for f in rep["findings"] if f["check"] == "import.joint_order" and "follows" in f["data"]), None
        )
        if not data:
            continue
        for engine, convention in data["follows"].items():
            follows[engine][convention] += 1
        if len({tuple(o) for o in data["engines"].values()}) > 1:
            differ += 1
    lines = [
        "## 7. Joint-ordering conventions observed",
        "",
        f"Engines disagree on joint order for **{pct(differ, len(reports))}** of the models.",
        "",
    ]
    lines += table(
        ["Engine", "Convention (models)"],
        [[e, "; ".join(f"{c}: {n}" for c, n in counts.most_common())] for e, counts in follows.items()],
    )
    return [*lines, ""]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--results", type=Path, default=Path("validation/results"))
    parser.add_argument("--out", type=Path, default=Path("validation/RESULTS.md"))
    args = parser.parse_args()
    corpus = args.results / "corpus"
    index = json.loads((corpus / "index.json").read_text())
    reports = {r["name"]: json.loads((corpus / r["report"]).read_text()) for r in index["robots"] if r["report"]}
    versions = next(iter(reports.values()))["backends"]
    lines = [
        "# Validation results (generated)",
        "",
        "Generated by `validation/analyze.py` from the runs in `validation/results/`. Engines: "
        + ", ".join(f"{b['label']} {b['version']}" for b in versions)
        + ".",
        "",
    ]
    lines += robustness(index, reports)
    lines += ground_truth(load_jsonl(args.results / "oracle.jsonl"), reports)
    lines += agreement(reports)
    lines += landscape(reports)
    lines += explanations(reports, load_jsonl(args.results / "confirm.jsonl"))
    lines += mutations(load_jsonl(args.results / "mutations.jsonl"))
    lines += orders(reports)
    lines += readability(reports)
    args.out.write_text("\n".join(lines) + "\n")
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
