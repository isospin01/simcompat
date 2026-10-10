"""Terminal rendering: problems first, in plain language, with the fix and where to read more."""

from __future__ import annotations

from ..checks.base import REGISTRY, Severity, Tier
from ..diagnosis import Problem
from .model import FILE_COLUMN, Report
from .wording import (
    MAX_LINES_PER_PROBLEM,
    SEVERITY_MEANING,
    SYMBOL,
    VERDICT_SENTENCE,
    explained_summary,
    ordering_columns,
    pluralize,
    problem_suffix,
    regraded_suffix,
)

_COLORS = {
    Severity.PASS: "32",
    Severity.SKIP: "90",
    Severity.INFO: "36",
    Severity.WARN: "33",
    Severity.FAIL: "31",
    Severity.ERROR: "35",
}


class _Style:
    def __init__(self, enabled: bool):
        self.enabled = enabled

    def sev(self, severity: Severity, text: str) -> str:
        return f"\033[{_COLORS[severity]}m{text}\033[0m" if self.enabled else text

    def bold(self, text: str) -> str:
        return f"\033[1m{text}\033[0m" if self.enabled else text

    def dim(self, text: str) -> str:
        return f"\033[2m{text}\033[0m" if self.enabled else text


def render(report: Report, color: bool = False, verbose: bool = False) -> str:
    style = _Style(color)
    lines = _header(report, style)
    limit = None if verbose else MAX_LINES_PER_PROBLEM
    for number, problem in enumerate(report.problems, 1):
        lines += _problem(number, problem, style, limit)
    lines.append(_tier_line(report, style))
    if report.regraded:
        lines.append(style.dim(f"{pluralize(len(report.regraded), 'finding')} re-graded by your profile"))
    if verbose:
        lines += _matrix(report, style) + _ordering(report) + _notes(report) + _regraded(report)
    else:
        lines.append(style.dim("Every finding, the check matrix and the joint ordering: --verbose, or -o report.html"))
    return "\n".join(lines)


def _header(report: Report, style: _Style) -> list[str]:
    engines = " · ".join(b.title if b.ok else f"{b.label} (failed to load)" for b in report.backends)
    verdict = report.verdict
    notes = report.model.get("notes", [])
    return [
        style.bold(f"SimCompat {report.tool_version} · {report.title}")
        + f" · {pluralize(len(report.model['movable_joints']), 'movable joint')}",
        f"Engines: {engines}",
        *([style.dim("Input: " + "; ".join(notes))] if notes else []),
        "",
        style.sev(verdict, f"{SYMBOL[verdict]} {verdict.name}") + f"  {VERDICT_SENTENCE[verdict]}",
        "",
    ]


def _problem(number: int, problem: Problem, style: _Style, limit: int | None) -> list[str]:
    marker = style.sev(problem.severity, SYMBOL[problem.severity])
    headline = problem.headline + problem_suffix(problem)
    lines = [f"{number}. {marker} {style.bold(headline)}"]
    engine_width = max(len(f.backend or FILE_COLUMN) for f in problem.findings)
    subject_width = max(len(f.subject or "") for f in problem.findings)
    for finding in problem.findings[:limit]:
        engine = (finding.backend or FILE_COLUMN).ljust(engine_width)
        subject = (finding.subject or "").ljust(subject_width)
        lines.append(f"   {style.dim(engine)}  {subject}  {finding.message}{regraded_suffix(finding)}".rstrip())
    hidden = len(problem.findings) - len(problem.findings[:limit])
    if hidden > 0:
        lines.append(style.dim(f"   ... and {hidden} more (all with --verbose)"))
    lines += [f"   Fix: {hint}" for hint in problem.hints[:2]]
    if problem.explains:
        lines.append(style.dim(f"   Also explains: {explained_summary(problem.explains)}"))
    if problem.check in REGISTRY:
        lines.append(style.dim(f"   More: simcompat explain {problem.check}"))
    return [*lines, ""]


def _tier_line(report: Report, style: _Style) -> str:
    """Worst result per tier, e.g. ``Checks: File ! · Import ✓ · Inertia ✗`` (with a short legend)."""
    worst: dict[Tier, Severity] = {}
    for finding in report.findings:
        spec = REGISTRY.get(finding.check)
        if spec is not None and finding.severity != Severity.SKIP:
            worst[spec.tier] = max(worst.get(spec.tier, Severity.PASS), finding.severity)
    parts = [f"{tier.label} {style.sev(sev, SYMBOL[sev])}" for tier, sev in sorted(worst.items())]
    legend = style.dim(f"   ({SYMBOL[Severity.PASS]} ok · i note · ! warning · {SYMBOL[Severity.FAIL]} differs)")
    return "Checks: " + " · ".join(parts) + legend


def _matrix(report: Report, style: _Style) -> list[str]:
    columns = report.columns()
    width = max(len(c) for c in [*report.check_ids(), "check"]) + 2
    widths = [max(len(col), 5) + 2 for col in columns]
    lines = ["", "check".ljust(width) + "".join(col.ljust(w) for col, w in zip(columns, widths, strict=True))]
    matrix = report.matrix()
    for check_id in report.check_ids():
        row = matrix.get(check_id, {})
        cells = [
            style.sev(row[column], row[column].name.ljust(w)) if column in row else ".".ljust(w)
            for column, w in zip(columns, widths, strict=True)
        ]
        lines.append(check_id.ljust(width) + "".join(cells))
    legend = " · ".join(f"{s.name} {meaning}" for s, meaning in SEVERITY_MEANING.items())
    return [*lines, style.dim(f"{legend} · '.' not run for this column")]


def _ordering(report: Report) -> list[str]:
    table = report.ordering_table()
    if not table:
        return []
    columns = ordering_columns(table)
    widths = [max(len(name), *(len(j) for j in order)) + 2 for name, order in columns.items()]
    lines = ["", "Joint ordering (index policy and controller vectors by joint name):"]
    lines.append("  #   " + "".join(name.ljust(w) for name, w in zip(columns, widths, strict=True)))
    for i in range(max(len(order) for order in columns.values())):
        orders = zip(columns.values(), widths, strict=True)
        cells = "".join((order[i] if i < len(order) else "").ljust(w) for order, w in orders)
        lines.append(f"  {i:<3} {cells}".rstrip())
    return lines + [f"  {label} follows {convention}" for label, convention in table["follows"].items()]


def _notes(report: Report) -> list[str]:
    if not report.notes:
        return []
    lines = ["", "Notes:"]
    for finding in report.notes:
        who = finding.backend or FILE_COLUMN
        subject = f" {finding.subject}:" if finding.subject else ""
        lines.append(f"  i {finding.check} [{who}]{subject} {finding.message}{regraded_suffix(finding)}")
    return lines


def _regraded(report: Report) -> list[str]:
    lowered = [f for f in report.regraded if f.severity < Severity.INFO]  # INFO ones are listed under notes
    if not lowered:
        return []
    lines = ["", "Re-graded to PASS by your profile:"]
    for finding in lowered:
        subject = f" {finding.subject}:" if finding.subject else ""
        lines.append(f"  {finding.check} [{finding.backend or FILE_COLUMN}]{subject} {finding.message}")
    return lines
