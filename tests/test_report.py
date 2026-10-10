"""Diagnosis (grouping by cause) and the renderers."""

import json

import numpy as np

import simcompat.checks  # noqa: F401 - registers the checks diagnosis reads
from simcompat.checks.base import REGISTRY, Finding, Severity
from simcompat.config import RunConfig
from simcompat.diagnosis import diagnose
from simcompat.report.model import Report, jsonable

F = Severity.FAIL


EVERY_CHECK_RAN = [Finding(Severity.PASS, "ok", check=check_id) for check_id in REGISTRY]


def test_symptoms_attach_to_the_root_cause_of_the_same_engine():
    findings = [
        Finding(F, "mass differs", backend="a", subject="j1", check="inertia.clusters"),
        Finding(F, "g differs", backend="a", check="dynamics.gravity"),
        Finding(F, "g differs", backend="b", check="dynamics.gravity"),  # engine b has no root cause
    ]
    problems = diagnose(findings + EVERY_CHECK_RAN)
    assert [p.check for p in problems] == ["inertia.clusters", "dynamics.gravity"]
    assert problems[0].explains == [("dynamics.gravity", "a")]
    assert problems[1].unexplained and problems[1].engines == ["b"]
    # if the checks that could explain it did not run (--only), it is not called unexplained
    unchecked = diagnose(findings[2:])[0]
    assert unchecked.causes_unchecked and not unchecked.unexplained


def test_cross_engine_symptom_is_explained_by_the_baseline():
    findings = [
        Finding(Severity.WARN, "lost", backend="base", subject="j", check="import.mimic"),
        Finding(Severity.WARN, "deviates", backend="other", compared_with="base", check="rollout.passive"),
    ]
    problems = diagnose(findings)
    assert len(problems) == 1 and problems[0].explains == [("rollout.passive", "other")]


def test_file_level_causes_explain_every_engine():
    findings = [
        Finding(Severity.WARN, "norm 2", subject="j", check="file.axis"),
        Finding(F, "deviates", backend="pybullet", check="rollout.rigid"),
    ]
    assert diagnose(findings)[0].explains == [("rollout.rigid", "pybullet")]


def test_symptoms_whose_causes_could_not_be_checked_belong_to_the_engine_failure():
    failure = Finding(Severity.ERROR, "x", backend="a", engine_failure=True, check="inertia.clusters")
    symptom = Finding(F, "M differs", backend="a", check="dynamics.mass_matrix")
    problems = diagnose([failure, symptom])
    assert [p.check for p in problems] == ["engine.failure:a"] and problems[0].explains == [
        ("dynamics.mass_matrix", "a")
    ]


def test_findings_within_a_problem_are_ordered_largest_first():
    findings = [
        Finding(F, "small", backend="a", subject="j1", score=0.1, check="inertia.clusters"),
        Finding(F, "large", backend="a", subject="j2", score=5.0, check="inertia.clusters"),
        Finding(Severity.ERROR, "nan", backend="a", subject="j3", check="inertia.clusters"),
    ]
    assert [f.message for f in diagnose(findings)[0].findings] == ["nan", "large", "small"]


def test_a_cause_accepted_by_the_profile_takes_its_symptoms_with_it():
    accepted = Finding(Severity.INFO, "mass", backend="a", regraded_from=F, check="inertia.clusters")
    symptom = Finding(F, "deviates", backend="a", check="rollout.rigid")
    report = _report([accepted, symptom])
    assert report.problems == [] and report.verdict == Severity.PASS and report.regraded == [accepted]
    # without the profile, the same symptom fails the run through its cause
    cause = Finding(Severity.WARN, "no <inertial>", subject="l", check="file.inertial")
    problems = diagnose([cause, symptom])
    assert problems[0].check == "file.inertial" and problems[0].severity == F


def test_symptoms_count_no_more_than_a_cause_the_profile_regraded():
    cause = Finding(Severity.WARN, "mass", backend="a", regraded_from=F, check="inertia.clusters")
    symptom = Finding(F, "deviates", backend="a", check="rollout.rigid")
    report = _report([cause, symptom])
    assert report.verdict == Severity.WARN and report.problems[0].explains == [("rollout.rigid", "a")]


def test_hints_are_deduplicated():
    findings = [Finding(F, "x", backend="a", subject=s, hint="same fix", check="inertia.clusters") for s in "abc"]
    assert diagnose(findings)[0].hints == ["same fix"]


def _report(findings):
    return Report(
        tool_version="t",
        model={
            "name": "m",
            "label": "m",
            "input": "m.urdf",
            "path": "m.urdf",
            "root": "r",
            "links": 2,
            "joints": 1,
            "movable_joints": ["j"],
            "notes": [],
        },
        backends=[],
        findings=findings,
        config=RunConfig(),
    )


def test_verdict_and_exit_codes():
    assert _report([Finding(Severity.INFO, "i", check="import.mimic")]).verdict == Severity.PASS
    warn = _report([Finding(Severity.WARN, "w", check="import.mimic")])
    assert warn.ok and warn.exit_code() == 0 and warn.exit_code(strict=True) == 1
    assert _report([Finding(Severity.ERROR, "e", check="import.load")]).exit_code() == 1


def test_renderers_escape_markup():
    report = _report([Finding(Severity.WARN, "has no <inertial> | pipe", subject="l", check="file.inertial")])
    assert "&lt;inertial&gt;" in report.to_markdown() and "\\|" in report.to_markdown()
    assert "<inertial>" not in report.to_html().split("<main>")[1].replace("&lt;inertial&gt;", "")
    assert "has no <inertial>" in report.to_text()


def test_json_is_strict_json():
    data = {"value": np.float64("nan"), "arr": np.arange(3)}
    report = _report([Finding(F, "nan", data=data, check="inertia.clusters")])
    data = json.loads(report.to_json())
    assert data["findings"][0]["data"] == {"value": "nan", "arr": [0, 1, 2]}
    assert jsonable({1: np.int64(2)}) == {"1": 2}
