"""Markdown rendering (GitHub-flavoured): CI job summaries, PR comments, files in a repo."""

from __future__ import annotations

from ..checks.base import REGISTRY
from .model import FILE_COLUMN, Report
from .wording import (
    ICON,
    MAX_LINES_PER_PROBLEM,
    NO_PROBLEMS,
    SEVERITY_MEANING,
    VERDICT_SENTENCE,
    counts_line,
    explained_summary,
    method_note,
    ordering_columns,
    problem_suffix,
    regraded_suffix,
)


def escape(text) -> str:
    """Escape text so tags such as <inertial> survive GitHub's HTML sanitizer and tables stay intact."""
    return str(text).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;").replace("|", "\\|")


def render(report: Report) -> str:
    sections = [
        _header(report),
        _problems(report),
        _matrix(report),
        _ordering(report),
        _notes(report),
        _engines(report),
        _legend(report),
    ]
    return "\n".join(line for section in sections for line in section) + "\n"


def _header(report: Report) -> list[str]:
    model, verdict = report.model, report.verdict
    engines = ", ".join(escape(b.title) for b in report.backends)
    lines = [
        f"# SimCompat report: `{report.title}`",
        "",
        f"**{ICON[verdict]} {verdict.name}**: {VERDICT_SENTENCE[verdict]}",
        "",
        f"{counts_line(report)} · engines: {engines} · simcompat {report.tool_version} · {report.created}",
        "",
        f"Input `{escape(model.get('input', model['path']))}` · {model['links']} links · {model['joints']} joints "
        f"({len(model['movable_joints'])} movable)",
    ]
    lines += [f"- {escape(note)}" for note in model.get("notes", [])]
    return [*lines, ""]


def _problems(report: Report) -> list[str]:
    problems = report.problems
    lines = ["## Problems", ""]
    if not problems:
        return [*lines, NO_PROBLEMS, ""]
    for number, problem in enumerate(problems, 1):
        suffix = problem_suffix(problem)
        lines += [
            f"### {number}. {ICON[problem.severity]} {escape(problem.headline)}{suffix}",
            "",
            (f"`{problem.check}` · " if problem.check in REGISTRY else "") + ", ".join(problem.engines),
            "",
            "| Engine | Joint / link | Finding |",
            "|---|---|---|",
        ]
        for finding in problem.findings[:MAX_LINES_PER_PROBLEM]:
            message = finding.message + regraded_suffix(finding)
            lines.append(
                f"| {escape(finding.backend or FILE_COLUMN)} | {escape(finding.subject or '')} | {escape(message)} |"
            )
        hidden = len(problem.findings) - MAX_LINES_PER_PROBLEM
        if hidden > 0:
            lines.append(f"| | | ... and {hidden} more (all in the HTML or JSON report) |")
        lines.append("")
        lines += [f"- **Fix:** {escape(hint)}" for hint in problem.hints[:2]]
        if problem.explains:
            lines.append(f"- Also explains: {escape(explained_summary(problem.explains))}")
        lines.append("")
    return lines


def _matrix(report: Report) -> list[str]:
    columns = report.columns()
    titles = {FILE_COLUMN: FILE_COLUMN, **{b.label: b.title for b in report.backends}}
    lines = [
        "<details><summary><b>All checks</b></summary>",
        "",
        "| Tier | Check | " + " | ".join(escape(titles[c]) for c in columns) + " |",
        "|---|---|" + "---|" * len(columns),
    ]
    matrix = report.matrix()
    for check_id in report.check_ids():
        spec = REGISTRY.get(check_id)
        row = matrix.get(check_id, {})
        cells = " | ".join(f"{ICON[row[c]]} {row[c].name}" if c in row else "" for c in columns)
        lines.append(
            f"| {spec.tier.label if spec else ''} | {escape(spec.title if spec else check_id)} "
            f"<sub>`{check_id}`</sub> | {cells} |"
        )
    return [*lines, "", "</details>", ""]


def _ordering(report: Report) -> list[str]:
    table = report.ordering_table()
    if not table:
        return []
    shown = ordering_columns(table)
    rows = max(len(order) for order in shown.values())
    lines = [
        "<details><summary><b>Joint ordering</b></summary>",
        "",
        "Index policy and controller vectors by joint **name**. "
        + "; ".join(f"**{escape(k)}** follows {escape(v)}" for k, v in table["follows"].items())
        + ".",
        "",
        "| # | " + " | ".join(escape(k) for k in shown) + " |",
        "|---|" + "---|" * len(shown),
    ]
    for i in range(rows):
        lines.append(f"| {i} | " + " | ".join(f"`{o[i]}`" if i < len(o) else "" for o in shown.values()) + " |")
    return [*lines, "", "</details>", ""]


def _notes(report: Report) -> list[str]:
    if not report.notes:
        return []
    lines = ["<details><summary><b>Notes</b> (documented conventions)</summary>", ""]
    for finding in report.notes:
        subject = f" `{escape(finding.subject)}`" if finding.subject else ""
        lines.append(
            f"- **{finding.check}** ({escape(finding.backend or FILE_COLUMN)}){subject}: "
            f"{escape(finding.message + regraded_suffix(finding))}"
        )
    return [*lines, "", "</details>", ""]


def _engines(report: Report) -> list[str]:
    lines = ["<details><summary><b>Engines and settings</b></summary>", ""]
    for result in report.backends:
        lines += [f"**{escape(result.title)}**", ""]
        if not result.ok:
            lines += [f"Failed to load: {escape(result.error)}", ""]
            if result.hint:
                lines += [f"Hint: {escape(result.hint)}", ""]
            continue
        info = result.info
        lines += [f"- simcompat set: {escape(s)}" for s in info.settings]
        lines += [f"- engine default, {escape(k)}: {escape(v)}" for k, v in info.defaults.items()]
        lines += [f"- {escape(w)}" for w in info.warnings[:10]]
        lines.append("")
    return [*lines, "</details>", ""]


def _legend(report: Report) -> list[str]:
    legend = " ".join(f"{ICON[s]} {s.name}: {meaning}." for s, meaning in SEVERITY_MEANING.items())
    return [f"<sub>{escape(method_note(report))} {legend}</sub>"]
