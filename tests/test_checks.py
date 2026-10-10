"""Every class of importer bug is caught and attributed - using the engine-free fake backend."""

from pathlib import Path

import pytest

from simcompat import backends
from simcompat.backends import BackendSpecError
from simcompat.config import RunConfig
from simcompat.runner import UsageError, run

from conftest import by, fixture, worst
from fake_backend import FakeBackend

backends.register(FakeBackend)
ARM = fixture("arm6.urdf")


def check(spec, only=None, **kwargs):
    return run(ARM, backends=["fake", spec], reference="none", only=only, **kwargs)


def failing_subjects(report, check_id, label):
    return {f.subject for f in by(report, check_id, label) if f.severity.name == "FAIL"}


def test_clean_import_passes_everything():
    report = run(ARM, backends=["fake"], reference="none")
    assert report.ok and not report.problems, [(f.check, f.message) for f in report.findings if f.severity >= 3]


@pytest.mark.parametrize("joint", ["shoulder_lift", "wrist_roll", "finger_slide"])
def test_reversed_axis_is_named(joint):
    report = check(f"fake:flip_axis={joint}", only=["kinematics.*"])
    finding = by(report, "kinematics.frames", f"fake[flip_axis={joint}]", joint)[0]
    assert finding.severity.name == "FAIL" and "REVERSED" in finding.message


def test_mass_error_is_localized_to_its_joint():
    report = check("fake:scale_mass=forearm@1.01", only=["inertia.*"])
    assert failing_subjects(report, "inertia.clusters", "fake[scale_mass=forearm@1.01]") == {"elbow"}


def test_mass_error_on_a_fused_fixed_child_is_attributed_to_the_carrier():
    report = check("fake:scale_mass=tool@2", only=["inertia.*"])
    assert failing_subjects(report, "inertia.clusters", "fake[scale_mass=tool@2]") == {"wrist_roll"}


def test_nan_is_a_failure_not_a_pass():
    report = check("fake:nan_mass=forearm", only=["inertia.*"])
    assert failing_subjects(report, "inertia.clusters", "fake[nan_mass=forearm]") == {"elbow"}


def test_origin_shift_moves_downstream_frames():
    report = check("fake:shift_origin=elbow", only=["kinematics.*"])
    assert {"elbow", "wrist_tilt"} <= failing_subjects(report, "kinematics.frames", "fake[shift_origin=elbow]")


def test_dropped_limits_and_damping():
    label = "fake[drop_damping=elbow,drop_limits=elbow]"
    report = check("fake:drop_limits=elbow,drop_damping=elbow", only=["import.*"])
    assert worst(report, "import.limits", label) == "FAIL"
    assert worst(report, "import.damping_friction", label) == "FAIL"


def test_reordered_joints_warn():
    assert (
        worst(check("fake:reverse_order=1", only=["import.joint_order"]), "import.joint_order", "fake[reverse_order=1]")
        == "WARN"
    )


def test_injected_armature_warns():
    assert (
        worst(check("fake:armature=0.01", only=["import.armature"]), "import.armature", "fake[armature=0.01]") == "WARN"
    )


def test_unknown_backend_or_option_is_a_usage_error_with_a_suggestion():
    with pytest.raises(BackendSpecError, match="Did you mean 'fake'"):
        run(ARM, backends=["fak"])
    with pytest.raises(BackendSpecError, match="Did you mean 'flip_axis'"):
        run(ARM, backends="fake:flip_axes=elbow")  # a single spec may be passed as a string


def test_option_values_are_validated_and_normalized():
    with pytest.raises(BackendSpecError, match="armature='lots' is not valid; expected a number"):
        run(ARM, backends="fake:armature=lots")
    report = run(ARM, backends="fake:reverse_order=yes", only=["import.load"])
    assert [b.label for b in report.backends] == ["fake[reverse_order=1]"]


def test_an_unknown_reference_is_a_usage_error():
    with pytest.raises(UsageError, match="--reference 'mujoco' is not one of the requested backends"):
        run(ARM, backends=["fake"], reference="mujoco")


def test_a_tier_name_selects_its_checks():
    report = run(ARM, backends="fake", reference="none", only="kinematics")  # a single pattern may be a string
    assert {f.check for f in report.findings} == {"import.load", "kinematics.frames"}


def test_the_same_backend_twice_is_a_usage_error():
    with pytest.raises(UsageError, match="requested twice"):
        run(ARM, backends=["fake:reverse_order=1", "fake:reverse_order=yes"])


def test_severity_overrides_accept_patterns_and_the_most_specific_wins():
    overrides = {"import.*": "WARN", "import.joint_order": "INFO"}
    report = check(
        "fake:reverse_order=1,armature=0.01", only=["import.*"], config=RunConfig(severity_overrides=overrides)
    )
    label = "fake[armature=0.01,reverse_order=1]"
    assert worst(report, "import.joint_order", label) == "INFO" and worst(report, "import.armature", label) == "WARN"


def test_load_errors_survive_only_filters():
    report = check("fake:fail_load=1", only=["kinematics.*"])
    error = by(report, "import.load", "fake[fail_load=1]")[0]
    assert error.severity.name == "ERROR" and error.hint == "(test hint)"
    assert report.exit_code() == 1


@pytest.mark.parametrize("option", ["only", "skip"])
def test_a_check_pattern_matching_nothing_is_an_error(option):
    with pytest.raises(UsageError, match=r"Did you mean 'dynamics\.gravity'"):
        run(ARM, backends=["fake"], reference="none", **{option: ["dynamics.gravty"]})


def test_checks_without_a_capable_engine_are_skipped():
    report = run(ARM, backends=["fake"], reference="none", only=["dynamics.*"])
    assert {f.severity.name for f in report.findings if f.check.startswith("dynamics.")} == {"SKIP"}


def test_one_engine_failing_is_reported_without_blaming_the_others():
    report = run(ARM, backends=["fake:fail_frames=1", "fake"], reference="fake[fail_frames=1]", only=["kinematics.*"])
    assert worst(report, "kinematics.frames", "fake[fail_frames=1]") == "ERROR"
    assert worst(report, "kinematics.frames", "fake") == "PASS"
    assert [p.check for p in report.problems] == ["engine.failure:fake[fail_frames=1]"]


def test_severity_overrides_regrade_but_never_hide_errors():
    config = RunConfig(severity_overrides={"import.joint_order": "INFO", "import.load": "PASS"})
    report = check("fake:reverse_order=1", only=["import.joint_order"], config=config)
    assert worst(report, "import.joint_order", "fake[reverse_order=1]") == "INFO" and report.exit_code() == 0
    report = check("fake:fail_load=1", only=["import.load"], config=config)
    assert any(f.severity.name == "ERROR" for f in by(report, "import.load"))


def test_non_unit_axis_warns(tmp_path):
    path = tmp_path / "axis.urdf"
    path.write_text(Path(ARM).read_text().replace('<axis xyz="0 0.6 0.8"/>', '<axis xyz="0 1.2 1.6"/>'))
    report = run(str(path), backends=["fake"], reference="none", only=["file.axis"])
    assert [f.subject for f in report.findings if f.severity.name == "WARN"] == ["shoulder_lift"]


def test_problems_group_findings_by_cause():
    report = check("fake:scale_mass=forearm@1.5")
    problem = report.problems[0]
    assert problem.check == "inertia.clusters" and problem.engines == ["fake[scale_mass=forearm@1.5]"]
    assert "mass" in problem.headline


def test_backends_from_other_packages_are_discovered(monkeypatch):
    class Plugged(FakeBackend):
        name = "plugged"

    class EntryPoint:
        name = "plugged"

        @staticmethod
        def load():
            return Plugged

    monkeypatch.setattr(backends, "_plugins_loaded", False)
    monkeypatch.setattr(backends.metadata, "entry_points", lambda **_: [EntryPoint])
    monkeypatch.delitem(backends.REGISTRY, "plugged", raising=False)
    assert "plugged" in backends.default_specs()
    assert run(ARM, backends=["plugged"], reference="none", only=["import.dofs"]).ok


def test_explained_by_must_name_root_causes():
    from simcompat.checks import REGISTRY, check, validate_registry
    from simcompat.checks.base import Tier

    check("test.symptom", Tier.ROLLOUT, "t", problem="p", explained_by=["rollout.rigid"])(lambda ctx: [])
    try:
        with pytest.raises(ValueError, match="symptoms themselves"):
            validate_registry()
    finally:
        del REGISTRY["test.symptom"]
