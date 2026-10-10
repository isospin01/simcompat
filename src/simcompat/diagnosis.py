"""From findings to problems: group what went wrong by cause, in plain language.

A dynamics or rollout difference is usually a *symptom*: if the same engine (or the engine it was
compared with, or the file) also has an inertia, kinematics or import problem, that problem explains
it, and the report says so instead of listing the symptom separately. Which causes can explain a
symptom is declared on the symptom's check (``@check(..., explained_by=...)``). A symptom that
nothing explains is reported on its own; it is the most interesting kind of finding.

A project profile can re-grade a cause (``[severity] "file.inertial" = "info"``). The symptoms
that cause explains follow it: they count at most as much as the cause itself, and disappear with
it when it is lowered below WARN. A known, accepted issue then stops failing the run through its
consequences.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from .checks.base import REGISTRY, Finding, Severity

FILE = "file"  # the "engine" of findings about the file itself
ENGINE_FAILURE = "engine.failure"  # pseudo check id: an engine failed to answer queries on this model


@dataclass
class Problem:
    check: str  # the root-cause check id (or ENGINE_FAILURE:<engine>)
    headline: str  # plain-language statement of the problem
    findings: list[Finding] = field(default_factory=list)  # evidence, largest difference first
    symptoms: list[Finding] = field(default_factory=list)  # dynamics/rollout differences it explains
    unexplained: bool = False  # a symptom with no root cause, although every possible cause was checked
    causes_unchecked: bool = False  # a symptom whose possible causes were not checked (--only/--skip)
    regraded: bool = False  # a profile re-graded this check: its symptoms count at most as much as it does

    @property
    def own_severity(self) -> Severity:
        return max((f.severity for f in self.findings), default=Severity.PASS)

    @property
    def severity(self) -> Severity:
        """The worst of its own findings and of the symptoms it explains."""
        return max([self.own_severity, *map(self._weight, self.symptoms)])

    @property
    def engines(self) -> list[str]:
        """Backend labels (or "file") of its own findings, in order of appearance."""
        return list(dict.fromkeys(f.backend or FILE for f in self.findings))

    @property
    def hints(self) -> list[str]:
        """How to fix it, without repeats."""
        return list(dict.fromkeys(f.hint for f in self.findings if f.hint))

    @property
    def explains(self) -> list[tuple[str, str]]:
        """``(symptom check id, engine)`` pairs, without repeats."""
        return list(dict.fromkeys((f.check, f.backend or FILE) for f in self.symptoms))

    @property
    def tier(self) -> int:
        spec = REGISTRY.get(self.check)
        return int(spec.tier) if spec else 0

    def severity_for(self, engine: str) -> Severity | None:
        """How badly this problem affects one engine, or None if it does not.

        It affects an engine where it has findings, and through the symptoms it explains there when it
        is a problem of the file. A symptom explained by the *other* engine of a comparison is that
        engine's consequence, not this one's.
        """
        own = [f.severity for f in self.findings if f.backend == engine]
        if own or FILE in self.engines:
            own += [self._weight(f) for f in self.symptoms if f.backend == engine]
        return max(own, default=None)

    def _weight(self, symptom: Finding) -> Severity:
        return min(symptom.severity, self.own_severity) if self.regraded else symptom.severity


def explained_by(check_id: str) -> tuple[str, ...]:
    """Root-cause checks that can explain ``check_id`` (empty for a root cause)."""
    spec = REGISTRY.get(check_id)
    return spec.explained_by if spec else ()


def _is_problem(finding: Finding) -> bool:
    return finding.severity >= Severity.WARN


def diagnose(findings: list[Finding]) -> list[Problem]:
    """Group WARN/FAIL/ERROR findings into problems, attaching explained symptoms to their cause."""
    problems: dict[str, Problem] = {}
    causes: dict[str, set[str]] = {}  # engine -> cause checks with a problem
    accepted: dict[str, set[str]] = {}  # engine -> cause checks a profile lowered below WARN
    unanswered: dict[str, set[str]] = {}  # engine -> checks the engine failed to answer
    ran = {f.check for f in findings}

    def problem_for(check_id: str) -> Problem:
        if check_id not in problems:
            spec = REGISTRY.get(check_id)
            problems[check_id] = Problem(check_id, spec.problem if spec else check_id)
        return problems[check_id]

    def failure_of(engine: str) -> Problem:  # one problem per engine, whichever checks it broke
        key = f"{ENGINE_FAILURE}:{engine}"
        if key not in problems:
            problems[key] = Problem(key, f"{engine} failed on this model while answering some checks")
        return problems[key]

    symptoms = []
    for finding in findings:
        engine = finding.backend or FILE
        if finding.engine_failure and _is_problem(finding):
            failure_of(engine).findings.append(finding)
            unanswered.setdefault(engine, set()).add(finding.check)
        elif explained_by(finding.check):
            if _is_problem(finding):
                symptoms.append(finding)
        elif _is_problem(finding):
            problem = problem_for(finding.check)
            problem.findings.append(finding)
            problem.regraded |= finding.regraded_from is not None
            causes.setdefault(engine, set()).add(finding.check)
        elif finding.accepted:
            accepted.setdefault(engine, set()).add(finding.check)

    for finding in symptoms:
        if finding.severity == Severity.ERROR:  # a crash is a problem of its own, never "explained"
            problem_for(finding.check).findings.append(finding)
            continue
        # A comparison is explained by a cause on either engine, or in the file.
        engines = {finding.backend or FILE, FILE, finding.compared_with}
        present = {c for e in engines for c in causes.get(e, ())}
        waived = {c for e in engines for c in accepted.get(e, ())}
        cause = next((c for c in explained_by(finding.check) if c in present or c in waived), None)
        if cause in present:
            problems[cause].symptoms.append(finding)
        elif cause is not None:
            pass  # explained by a cause the project accepted, so accepted with it
        elif set(explained_by(finding.check)) & unanswered.get(finding.backend, set()):
            failure_of(finding.backend).symptoms.append(finding)  # its possible causes could not be checked
        else:
            problem = problem_for(finding.check)
            problem.findings.append(finding)
            if set(explained_by(finding.check)) <= ran:
                problem.unexplained = True
            else:
                problem.causes_unchecked = True

    for problem in problems.values():
        problem.findings.sort(key=lambda f: (-int(f.severity), -f.score))
    return sorted(problems.values(), key=lambda p: (-int(p.severity), p.tier, p.check))
