"""The report: everything a run found, renderable as text, Markdown, HTML or JSON."""

from __future__ import annotations

import dataclasses
import datetime as dt
import json
from dataclasses import dataclass, field
from functools import cached_property
from pathlib import Path
from typing import Any

import numpy as np

from ..backends.base import LoadResult
from ..checks.base import REGISTRY, Finding, Severity
from ..config import RunConfig
from ..diagnosis import FILE, Problem, diagnose
from ..errors import UsageError

FILE_COLUMN = FILE  # the column of findings about the file itself
SCHEMA = "simcompat.report/v1"
REPORT_FORMATS = (".html", ".htm", ".md", ".json", ".txt")


@dataclass(repr=False)
class Report:
    """Everything one run found. Build it once; ``problems`` is computed from ``findings`` on first use."""

    tool_version: str
    model: dict[str, Any]  # name, label, input, path, root, links, joints, movable_joints, notes
    backends: list[LoadResult]
    findings: list[Finding]
    config: RunConfig
    reference: str | None = None
    created: str = field(default_factory=lambda: dt.datetime.now(dt.timezone.utc).replace(microsecond=0).isoformat())

    @property
    def title(self) -> str:
        """The robot's short name, e.g. ``go2``."""
        return self.model.get("label") or self.model["name"]

    def __repr__(self) -> str:
        engines = ", ".join(b.label for b in self.backends)
        return f"<Report {self.title}: {self.verdict.name}, {len(self.problems)} problem(s); engines: {engines}>"

    # ---- verdict --------------------------------------------------------------------------------
    @property
    def verdict(self) -> Severity:
        """Worst problem severity (WARN, FAIL or ERROR), or PASS when there is no problem.

        Taken from the problems rather than the raw findings, so that symptoms of a cause that the
        project's profile accepted do not fail the run (see :mod:`simcompat.diagnosis`).
        """
        return max((p.severity for p in self.problems), default=Severity.PASS)

    @property
    def ok(self) -> bool:
        """True when no engine simulates something different from the file (no FAIL or ERROR)."""
        return self.verdict < Severity.FAIL

    def exit_code(self, strict: bool = False) -> int:
        """0 = ok, 1 = FAIL/ERROR (or WARN with ``strict``)."""
        return 1 if self.verdict >= (Severity.WARN if strict else Severity.FAIL) else 0

    def counts(self) -> dict[str, int]:
        counts = {s.name: 0 for s in Severity}
        for finding in self.findings:
            counts[finding.severity.name] += 1
        return counts

    # ---- views ----------------------------------------------------------------------------------
    @cached_property
    def problems(self) -> list[Problem]:
        """Findings grouped by cause, worst first (see :mod:`simcompat.diagnosis`)."""
        return diagnose(self.findings)

    @property
    def regraded(self) -> list[Finding]:
        """Findings whose severity a project profile changed."""
        return [f for f in self.findings if f.regraded_from is not None]

    @property
    def notes(self) -> list[Finding]:
        """INFO findings: documented conventions worth knowing."""
        return [f for f in self.findings if f.severity == Severity.INFO]

    def columns(self) -> list[str]:
        return [FILE_COLUMN] + [b.label for b in self.backends]

    def matrix(self) -> dict[str, dict[str, Severity]]:
        """check id -> column (backend label or "file") -> worst severity."""
        table: dict[str, dict[str, Severity]] = {}
        for finding in self.findings:
            row = table.setdefault(finding.check, {})
            column = finding.backend or FILE_COLUMN
            row[column] = max(row.get(column, finding.severity), finding.severity)
        return table

    def check_ids(self) -> list[str]:
        """Checks that ran, in tier order."""
        seen = list(dict.fromkeys(f.check for f in self.findings))
        return sorted(seen, key=lambda c: (REGISTRY[c].tier if c in REGISTRY else 99, seen.index(c)))

    def findings_for(self, check_id: str, column: str | None = None) -> list[Finding]:
        return [
            f for f in self.findings if f.check == check_id and (column is None or (f.backend or FILE_COLUMN) == column)
        ]

    def ordering_table(self) -> dict[str, Any] | None:
        """Joint orders per engine and per convention (from import.joint_order), if that check ran."""
        return next((f.data for f in self.findings if f.check == "import.joint_order" and "engines" in f.data), None)

    # ---- output ---------------------------------------------------------------------------------
    def to_dict(self) -> dict[str, Any]:
        return {
            "schema": SCHEMA,
            "tool_version": self.tool_version,
            "created": self.created,
            "verdict": self.verdict.name,
            "counts": self.counts(),
            "model": self.model,
            "reference": self.reference,
            "backends": [_backend_dict(b) for b in self.backends],
            "config": {
                "tolerances": dataclasses.asdict(self.config.tolerances),
                "samples": self.config.samples,
                "rollout_samples": self.config.rollout_samples,
                "seed": self.config.seed,
                "dt": self.config.dt,
                "duration": self.config.duration,
                "severity_overrides": self.config.severity_overrides,
            },
            "problems": [
                {
                    "check": p.check,
                    "severity": p.severity.name,
                    "headline": p.headline,
                    "engines": p.engines,
                    "hints": p.hints,
                    "explains": [list(e) for e in p.explains],
                    "unexplained": p.unexplained,
                    "causes_unchecked": p.causes_unchecked,
                }
                for p in self.problems
            ],
            "matrix": {check: {col: sev.name for col, sev in row.items()} for check, row in self.matrix().items()},
            "findings": [_finding_dict(f) for f in self.findings],
        }

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), indent=2)

    def to_text(self, color: bool = False, verbose: bool = False) -> str:
        from .console import render

        return render(self, color=color, verbose=verbose)

    def to_markdown(self) -> str:
        from .markdown import render

        return render(self)

    def to_html(self) -> str:
        from .html import render

        return render(self)

    def save(self, path: str | Path) -> Path:
        """Write the report; the format follows the extension (.html, .md, .json, .txt)."""
        path = Path(path)
        writers = {
            ".html": self.to_html,
            ".htm": self.to_html,
            ".md": self.to_markdown,
            ".json": self.to_json,
            ".txt": self.to_text,
        }
        if path.suffix.lower() not in writers:
            raise UsageError(f"{path}: unknown report format {path.suffix!r}; use one of {', '.join(REPORT_FORMATS)}")
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(writers[path.suffix.lower()](), encoding="utf-8")
        return path

    def __str__(self) -> str:
        return self.to_text()


def jsonable(value: Any) -> Any:
    """numpy-aware conversion to plain JSON types (NaN/inf become strings)."""
    if isinstance(value, dict):
        return {str(k): jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [jsonable(v) for v in value]
    if isinstance(value, np.ndarray):
        return jsonable(value.tolist())
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, (float, np.floating)):
        value = float(value)
        return value if np.isfinite(value) else str(value)
    return value


def _finding_dict(finding: Finding) -> dict[str, Any]:
    return {
        "check": finding.check,
        "severity": finding.severity.name,
        "backend": finding.backend,
        "subject": finding.subject,
        "message": finding.message,
        "hint": finding.hint,
        "score": jsonable(finding.score),
        "compared_with": finding.compared_with,
        "engine_failure": finding.engine_failure,
        "regraded_from": finding.regraded_from.name if finding.regraded_from is not None else None,
        "data": jsonable(finding.data),
    }


def _backend_dict(result: LoadResult) -> dict[str, Any]:
    info = result.info
    return {
        "label": result.label,
        "name": result.name,
        "ok": result.ok,
        "version": result.version,
        "error": result.error,
        "hint": result.hint,
        "is_simulator": result.is_simulator,
        "is_reference": result.is_reference,
        "settings": info.settings if info else [],
        "defaults": info.defaults if info else {},
        "warnings": info.warnings if info else [],
    }
