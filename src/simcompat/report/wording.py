"""Wording shared by the renderers (console, Markdown, HTML), so all reports say the same thing."""

from __future__ import annotations

from typing import TYPE_CHECKING

from ..checks.base import Finding, Severity
from ..declared import BFS_FILE_ORDER, DFS_FILE_ORDER, DFS_SORTED

if TYPE_CHECKING:
    from .model import Report

ICON = {
    Severity.PASS: "✅",
    Severity.SKIP: "⏭️",
    Severity.INFO: "ℹ️",
    Severity.WARN: "⚠️",
    Severity.FAIL: "❌",
    Severity.ERROR: "💥",
}
SYMBOL = {  # terminal-safe
    Severity.PASS: "✓",
    Severity.SKIP: "-",
    Severity.INFO: "i",
    Severity.WARN: "!",
    Severity.FAIL: "✗",
    Severity.ERROR: "✗",
}
VERDICT_SENTENCE = {
    Severity.PASS: "Every engine simulates what the file declares, within tolerance.",
    Severity.WARN: "No engine contradicts the file, but some differences are likely to bite.",
    Severity.FAIL: "Some engines simulate something different from what the file declares.",
    Severity.ERROR: "Some engines could not load the model, or failed while checking it.",
}
SEVERITY_MEANING = {
    Severity.PASS: "matches",
    Severity.SKIP: "not applicable",
    Severity.INFO: "documented convention",
    Severity.WARN: "likely to bite",
    Severity.FAIL: "differs where it must match",
    Severity.ERROR: "could not run",
}
NO_PROBLEMS = "None. Every engine simulates what the file declares, within tolerance."
UNEXPLAINED = "nothing else explains it"
UNCHECKED = "its possible causes were not checked: run without --only/--skip"


def problem_suffix(problem) -> str:
    """`` (nothing else explains it)`` and similar notes after a problem's headline."""
    if problem.unexplained:
        return f" ({UNEXPLAINED})"
    if problem.causes_unchecked:
        return f" ({UNCHECKED})"
    return ""


MAX_LINES_PER_PROBLEM = 6


def pluralize(count: int, word: str) -> str:
    return f"{count} {word}{'' if count == 1 else 's'}"


def explained_summary(explains: list[tuple[str, str]]) -> str:
    """``[("dynamics.gravity", "mujoco"), ...]`` -> ``"dynamics.gravity (mujoco, pybullet), ..."``."""
    by_check: dict[str, list[str]] = {}
    for check_id, engine in explains:
        by_check.setdefault(check_id, []).append(engine)
    return ", ".join(f"{check_id} ({', '.join(engines)})" for check_id, engines in by_check.items())


def ordering_columns(table: dict) -> dict[str, list[str]]:
    """Columns of the joint-ordering table: the file's order under each convention, then each engine's."""
    conventions = table["conventions"]
    return {
        "declared": table["declared"],
        "depth-first": conventions[DFS_FILE_ORDER],
        "depth-first (sorted)": conventions[DFS_SORTED],
        "breadth-first": conventions[BFS_FILE_ORDER],
        **table["engines"],
    }


def regraded_suffix(finding: Finding) -> str:
    """`` (re-graded by profile; was FAIL)`` for a finding a project profile re-graded, else ``""``."""
    return f" (re-graded by profile; was {finding.regraded_from.name})" if finding.regraded_from is not None else ""


def counts_line(report: Report) -> str:
    """``2 problems · findings: 12 fail, 3 warn, 4 info`` (problems are findings grouped by cause)."""
    counts = report.counts()
    parts = [f"{counts['FAIL'] + counts['ERROR']} fail", f"{counts['WARN']} warn", f"{counts['INFO']} info"]
    return f"{pluralize(len(report.problems), 'problem')} · findings: {', '.join(parts)}"


def method_note(report: Report) -> str:
    """How the comparisons were made, for the footer of Markdown and HTML reports."""
    config, tol = report.config, report.config.tolerances
    return (
        "Import, inertia and kinematics checks compare each engine with the file as parsed by simcompat (no engine "
        f"involved). Dynamics checks compare with the reference at {pluralize(config.samples, 'sampled configuration')}"
        f" (fixed base, no contact; correct imports agree to about 1e-12; M(q) tolerance {tol.mass_matrix_rel:g}). "
        f"Rollouts: {pluralize(config.rollout_samples, 'random start')}, {config.duration:g} s at dt={config.dt:g}, "
        "zero torque, contacts off, compared until a joint nears a limit."
    )
