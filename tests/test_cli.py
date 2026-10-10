"""The command line: commands, outputs and exit codes."""

import json
from pathlib import Path

import pytest

from simcompat.cli import main

from conftest import fixture, installed, needs_engines

QUIET = ["--quiet", "--no-step-summary"]


@needs_engines
def test_check_writes_every_report_format(tmp_path):
    outputs = [tmp_path / f"report.{ext}" for ext in ("html", "md", "json", "txt")]
    args = ["check", fixture("arm6.urdf"), "-b", "pinocchio", "-b", "pybullet:inertia_from_file=1", *QUIET]
    for path in outputs:
        args += ["-o", str(path)]
    assert main(args) == 0
    assert json.loads(outputs[2].read_text())["verdict"] == "PASS"
    assert outputs[0].read_text().startswith("<!doctype html>")
    assert "SimCompat report" in outputs[1].read_text()


@needs_engines
def test_exit_code_is_1_when_an_engine_differs():
    assert main(["check", fixture("arm6.urdf"), "-b", "pinocchio", "-b", "pybullet", *QUIET]) == 1


@needs_engines
def test_console_output_leads_with_problems_and_fixes(capsys):
    main(["check", fixture("arm6.urdf"), "-b", "pinocchio", "-b", "pybullet", "--no-step-summary", "--no-color"])
    out = capsys.readouterr().out
    assert "FAIL" in out and "Fix:" in out and "simcompat explain inertia.clusters" in out


def test_missing_input_is_a_usage_error(capsys):
    assert main(["check", "does/not/exist.urdf", *QUIET]) == 2
    assert "no such file" in capsys.readouterr().err


@needs_engines
def test_reports_can_go_into_a_new_directory(tmp_path):
    path = tmp_path / "reports" / "arm6.html"
    assert main(["check", fixture("arm6.urdf"), "-b", "pinocchio", "-o", str(path), *QUIET]) == 0
    assert path.exists()


def test_bad_report_extension_is_a_usage_error(tmp_path):
    args = ["check", fixture("arm6.urdf"), "-b", "pinocchio", "-o", str(tmp_path / "r.pdf"), *QUIET]
    assert main(args) == 2


def test_explain_lists_and_explains_checks(capsys):
    assert main(["explain"]) == 0
    assert "inertia.clusters" in capsys.readouterr().out
    assert main(["explain", "inertia.clusters"]) == 0
    assert "Fix:" in capsys.readouterr().out
    assert main(["explain", "rollout.*"]) == 0
    assert "rollout.passive_effect" in capsys.readouterr().out
    assert main(["explain", "inertia.cluster"]) == 2
    assert "Did you mean" in capsys.readouterr().err


def test_backends_lists_options(capsys):
    assert main(["backends"]) == 0
    assert "pybullet:inertia_from_file" in capsys.readouterr().out


@pytest.mark.parametrize(
    "args, message",
    [
        (["-b", "mujoko"], "Did you mean 'mujoco'"),
        (["-b", "pybullet:inertia_from_fil=1"], "Did you mean 'inertia_from_file'"),
        (["--only", "dynamics.gravty"], "Did you mean 'dynamics.gravity'"),
        (["--skip", "nothing.*"], "matches no check"),
        (["--package-dir", "pkg=/no/such/dir"], "no such directory"),
        (["--package-dir", "missing-equals"], "expects NAME=PATH"),
        (["--profile", "/no/such/profile.toml"], "profile not found"),
        (["--samples", "0"], "samples must be >= 1"),
        (["--reference", "drake"], "not one of the requested backends"),
    ],
)
def test_mistakes_are_usage_errors_that_say_what_to_do(args, message, capsys):
    assert main(["check", fixture("arm6.urdf"), "-b", "pinocchio", *args, *QUIET]) == 2
    assert message in capsys.readouterr().err


@pytest.mark.parametrize(
    "profile, message",
    [
        ("[tolerence]\nmass_rel = 1", "unknown section 'tolerence'"),
        ("[run]\nsample = 3", "unknown [run] key 'sample'"),
        ('[severity]\n"import.joint_ordr" = "info"', "Did you mean 'import.joint_order'"),
        ('[severity]\n"import.joint_order" = "ignore"', "must be one of"),
    ],
)
def test_profile_mistakes_are_reported(profile, message, tmp_path, capsys):
    path = tmp_path / "simcompat.toml"
    path.write_text(profile)
    assert main(["check", fixture("arm6.urdf"), "-b", "pinocchio", "--profile", str(path), *QUIET]) == 2
    assert message in capsys.readouterr().err


def test_init_writes_profile_and_workflow(tmp_path, capsys):
    (tmp_path / "robot.urdf").write_text(Path(fixture("arm6.urdf")).read_text())
    assert main(["init", "--dir", str(tmp_path)]) == 0
    assert (tmp_path / "simcompat.toml").exists()
    workflow = (tmp_path / ".github/workflows/simcompat.yml").read_text()
    assert "simcompat check robot.urdf --profile simcompat.toml -o simcompat-report.html" in workflow
    assert main(["init", "--dir", str(tmp_path)]) == 0  # existing files are kept
    assert "kept" in capsys.readouterr().out


def test_init_paths_work_from_here_and_from_the_project(tmp_path, capsys):
    (tmp_path / "urdf").mkdir()
    robot = tmp_path / "urdf" / "arm.urdf"
    robot.write_text(Path(fixture("arm6.urdf")).read_text())
    assert main(["init", str(robot), "--dir", str(tmp_path)]) == 0
    assert "simcompat check urdf/arm.urdf" in (tmp_path / ".github/workflows/simcompat.yml").read_text()
    assert f"Next: simcompat check {robot}" in capsys.readouterr().out


def test_profile_from_init_is_valid(tmp_path):
    from simcompat.config import RunConfig
    from simcompat.scaffold import PROFILE

    path = tmp_path / "p.toml"
    path.write_text(PROFILE.format(robot="robot.urdf"))
    assert RunConfig.from_toml(path).tolerances.passive_rollout_abs == 0.01


@needs_engines
def test_matrix_in_every_format(tmp_path):
    outputs = [tmp_path / f"matrix.{ext}" for ext in ("md", "html", "json")]
    robots = [fixture("arm6.urdf"), fixture("zero_width.urdf"), fixture("divergent.urdf")]
    args = ["matrix", *robots, "-b", "pinocchio", "-b", "mujoco", "-b", "pybullet", "--quiet"]
    assert main(args + [arg for path in outputs for arg in ("-o", str(path))]) == 1  # divergent fails
    markdown = outputs[0].read_text()
    assert "| `arm6` |" in markdown and "| `zero_width` |" in markdown
    assert "inertia.clusters" in markdown and "rollout.rigid" not in markdown  # causes, not their symptoms
    html = outputs[1].read_text()
    assert html.startswith("<!doctype html>") and "<td><span class='t-PASS'>PASS</span>" in html
    data = json.loads(outputs[2].read_text())
    divergent = next(r for r in data["robots"] if r["label"] == "divergent")
    assert "inertia.clusters" in divergent["engines"]["pybullet"]["causes"]
    assert not {"rollout.rigid", "dynamics.mass_matrix"} & set(divergent["engines"]["pybullet"]["causes"])  # symptoms


def test_matrix_rejects_unknown_formats_before_running(capsys):
    assert main(["matrix", fixture("arm6.urdf"), "-o", "matrix.pdf", "--quiet"]) == 2
    assert ".html" in capsys.readouterr().err


@pytest.mark.skipif(not installed("robot_descriptions"), reason="robot_descriptions not installed")
def test_robots_filters_by_name_maker_and_tags(capsys):
    assert main(["robots", "go2"]) == 0
    assert "rd:go2" in capsys.readouterr().out
    assert main(["robots", "Unitree", "humanoid"]) == 0
    out = capsys.readouterr().out
    assert out.startswith("SPEC") and "rd:g1" in out and "rd:go2" not in out
