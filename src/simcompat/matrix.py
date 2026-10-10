"""Check many robots and summarize them as a compatibility matrix (Markdown, HTML or JSON).

Each cell names the *causes* that affect one engine on one robot (for example ``inertia.clusters``),
not every symptom they produce, so the matrix reads like the problem list of each report.
"""

from __future__ import annotations

import datetime as dt
import html
import json
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from pathlib import Path

from . import __version__
from .checks.base import Severity
from .config import RunConfig
from .diagnosis import ENGINE_FAILURE, FILE
from .errors import SimCompatError, UsageError
from .report.html import STYLE
from .report.markdown import escape
from .report.model import Report
from .report.wording import ICON

MATRIX_FORMATS = (".md", ".html", ".htm", ".json")


@dataclass
class Cell:
    """One engine (or the file) on one robot."""

    worst: Severity = Severity.PASS
    causes: list[str] = field(default_factory=list)  # problem check ids, worst first
    load_error: str | None = None

    @property
    def text(self) -> str:
        if self.load_error:
            return "failed to load: " + self.load_error.splitlines()[0][:70]
        return ", ".join("engine failure" if c.startswith(ENGINE_FAILURE) else c for c in self.causes)


@dataclass
class MatrixRow:
    robot: str  # the spec as given
    label: str = ""
    dofs: int = 0
    verdict: Severity = Severity.ERROR
    file: Cell = field(default_factory=Cell)  # problems found in the file itself
    engines: dict[str, Cell] = field(default_factory=dict)  # backend label -> cell
    seconds: float = 0.0
    error: str | None = None  # the robot could not be checked at all


def summarize(robot: str, report: Report) -> MatrixRow:
    """One matrix row from one report."""
    row = MatrixRow(robot, report.title, len(report.model["movable_joints"]), report.verdict)
    row.file = _cell(report, FILE)
    for result in report.backends:
        row.engines[result.label] = (
            _cell(report, result.label) if result.ok else Cell(Severity.ERROR, load_error=result.error or "")
        )
    return row


def _cell(report: Report, engine: str) -> Cell:
    if engine == FILE:
        affecting = [(p, p.own_severity) for p in report.problems if FILE in p.engines]
    else:
        affecting = [(p, p.severity_for(engine)) for p in report.problems]
    affecting = sorted(((p, s) for p, s in affecting if s is not None and s >= Severity.WARN), key=lambda x: -x[1])
    return Cell(max((s for _, s in affecting), default=Severity.PASS), [p.check for p, _ in affecting])


def build_matrix(
    robots: Sequence[str], *, json_dir: str | None = None, progress: Callable[[str], None] | None = None, **run_options
) -> tuple[list[MatrixRow], dict[str, str]]:
    """Run :func:`simcompat.runner.run` on every robot; one failing robot never stops the others.

    Options are checked once, before the first robot. ``progress`` also receives each run's log
    (downloads, xacro warnings). Returns the rows and the column titles (backend label -> "label version").
    """
    from .runner import prepare_options, run

    prepare_options(  # a mistake is reported once, before the first robot
        run_options.get("backends"),
        run_options.get("reference", "auto"),
        run_options.get("config") or RunConfig(),
        run_options.get("only"),
        run_options.get("skip"),
    )
    if json_dir:
        Path(json_dir).mkdir(parents=True, exist_ok=True)
    rows, titles = [], {}
    for number, robot in enumerate(robots, 1):
        started = time.perf_counter()
        try:
            report = run(robot, log=progress, **run_options)
        except SimCompatError as e:
            row = MatrixRow(robot=robot, error=str(e).splitlines()[0])
        except Exception as e:  # a simcompat bug on one robot must not stop the others
            row = MatrixRow(robot=robot, error=f"internal error: {type(e).__name__}: {e}")
        else:
            row = summarize(robot, report)
            for result in report.backends:
                titles.setdefault(result.label, result.title.replace(" (reference)", ""))
            if json_dir:
                safe = robot.replace(":", "_").replace("/", "_").strip("._")
                report.save(Path(json_dir, f"{safe}.json"))
        row.seconds = time.perf_counter() - started
        if progress:
            status = f"error: {row.error}" if row.error else row.verdict.name
            progress(f"[{number}/{len(robots)}] {row.label or robot}: {status} ({row.seconds:.1f}s)")
        rows.append(row)
    return rows, titles


def _intro() -> str:
    return (
        f"Generated {dt.date.today().isoformat()} by simcompat {__version__}. Each cell shows the worst result for "
        "that engine and the causes behind it. A problem in the file itself is listed in the file column, and also "
        "in an engine's cell when it changes that engine's results. `simcompat explain <check>` describes each cause."
    )


def render_markdown(rows: list[MatrixRow], titles: dict[str, str]) -> str:
    def cell(c: Cell | None) -> str:
        return f"{ICON[c.worst]} {escape(c.text)}".strip() if c is not None else ""

    labels = list(titles)
    lines = [
        "# SimCompat compatibility matrix",
        "",
        _intro(),
        "",
        "| Robot | DoF | file | " + " | ".join(escape(titles[label]) for label in labels) + " |",
        "|---|---|---|" + "---|" * len(labels),
    ]
    for row in rows:
        if row.error:
            lines.append(f"| `{escape(row.robot)}` | | 💥 {escape(row.error[:90])} |" + " |" * len(labels))
            continue
        cells = " | ".join(cell(row.engines.get(label)) for label in labels)
        lines.append(f"| `{escape(row.label)}` | {row.dofs} | {cell(row.file)} | {cells} |")
    return "\n".join(lines) + "\n"


def render_html(rows: list[MatrixRow], titles: dict[str, str]) -> str:
    def e(text: object) -> str:
        return html.escape(str(text))

    def cell(c: Cell | None) -> str:
        if c is None:
            return "<td></td>"
        return f"<td><span class='t-{c.worst.name}'>{c.worst.name}</span> {e(c.text)}</td>"

    labels = list(titles)
    head = "".join(f"<th>{e(titles[label])}</th>" for label in labels)
    body = []
    for row in rows:
        if row.error:
            body.append(
                f"<tr><td class='mono'>{e(row.robot)}</td><td></td>"
                f"<td colspan='{len(labels) + 1}' class='t-ERROR'>{e(row.error)}</td></tr>"
            )
            continue
        cells = "".join(cell(row.engines.get(label)) for label in labels)
        body.append(f"<tr><td class='mono'>{e(row.label)}</td><td>{row.dofs}</td>{cell(row.file)}{cells}</tr>")
    return (
        '<!doctype html><html lang="en"><head><meta charset="utf-8">'
        '<meta name="viewport" content="width=device-width,initial-scale=1"><title>SimCompat matrix</title>'
        f"<style>{STYLE}</style></head><body><main><h1>SimCompat compatibility matrix</h1>"
        f"<p class='muted'>{e(_intro())}</p><div class='scroll'><table><tr><th>Robot</th><th>DoF</th>"
        f"<th>file</th>{head}</tr>{''.join(body)}</table></div></main></body></html>"
    )


def to_json(rows: list[MatrixRow], titles: dict[str, str]) -> str:
    def cell(c: Cell) -> dict:
        return {"worst": c.worst.name, "causes": c.causes, "load_error": c.load_error}

    data = {
        "schema": "simcompat.matrix/v1",
        "tool_version": __version__,
        "engines": titles,
        "robots": [
            {
                "robot": r.robot,
                "label": r.label,
                "dofs": r.dofs,
                "verdict": r.verdict.name,
                "error": r.error,
                "seconds": round(r.seconds, 2),
                "file": cell(r.file),
                "engines": {label: cell(c) for label, c in r.engines.items()},
            }
            for r in rows
        ],
    }
    return json.dumps(data, indent=2)


def save(rows: list[MatrixRow], titles: dict[str, str], path: str | Path) -> Path:
    """Write the matrix; the format follows the extension (.md, .html, .json)."""
    path = Path(path)
    suffix = path.suffix.lower()
    if suffix not in MATRIX_FORMATS:
        raise UsageError(f"{path}: unknown matrix format {path.suffix!r}; use one of {', '.join(MATRIX_FORMATS)}")
    if suffix == ".json":
        text = to_json(rows, titles)
    elif suffix in (".html", ".htm"):
        text = render_html(rows, titles)
    else:
        text = render_markdown(rows, titles)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")
    return path
