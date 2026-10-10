"""End-to-end against real engines.

These pin the engine behaviours simcompat was built to expose. If an engine release changes one of
them, a test fails - which is the point: the README's claims must stay true.
"""

import tempfile
from pathlib import Path

import pytest

from simcompat import backends
from simcompat.backends.base import SimSettings
from simcompat.declared import parse_urdf
from simcompat.inputs import PreparedURDF, prepare_urdf
from simcompat.runner import run

from conftest import by, fixture, needs_engines, needs_mujoco, worst

PB = "pybullet[inertia_from_file=1]"
ENGINES = ["pinocchio", "mujoco", "pybullet:inertia_from_file=1"]
CORE = [
    "import.dofs",
    "import.limits",
    "import.damping_friction",
    "inertia.clusters",
    "kinematics.frames",
    "dynamics.mass_matrix",
    "dynamics.gravity",
    "dynamics.coriolis",
    "rollout.rigid",
]

ONE_DOF = """<robot name="one"><link name="a"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1"
 ixy="0" ixz="0" iyz="0"/></inertial></link><link name="b"><inertial><origin xyz="0 0 -0.3"/><mass value="1"/>
 <inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/></inertial></link>
 <joint name="j" type="continuous"><parent link="a"/><child link="b"/><axis xyz="0 1 0"/></joint></robot>"""

STATIC = """<robot name="static"><link name="a"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1"
 ixy="0" ixz="0" iyz="0"/></inertial></link><link name="b"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.1"
 izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link><joint name="f" type="fixed"><parent link="a"/><child link="b"/>
 </joint></robot>"""


@needs_engines
def test_clean_arm_matches_in_every_engine():
    report = run(fixture("arm6.urdf"), backends=ENGINES)
    for check_id in CORE:
        for label in ("mujoco", PB):
            assert worst(report, check_id, label) == "PASS", (check_id, label, by(report, check_id, label))
    # correct imports agree to machine precision: the premise of strict tolerances
    assert by(report, "dynamics.mass_matrix", "mujoco")[0].data["rel_err"] < 1e-10
    assert report.ok


@needs_engines
def test_pybullet_default_ignores_urdf_inertia_tensors():
    report = run(fixture("arm6.urdf"), backends=["pinocchio", "pybullet"])
    assert worst(report, "inertia.clusters", "pybullet") == "FAIL"
    assert "URDF_USE_INERTIA_FROM_FILE" in by(report, "inertia.clusters", "pybullet")[0].hint
    assert worst(report, "dynamics.gravity", "pybullet") == "PASS"  # masses and COMs are kept


@needs_engines
def test_missing_inertial_gets_three_different_masses():
    report = run(fixture("divergent.urdf"), backends=ENGINES)
    masses = {}
    for label in ("mujoco", PB):
        finding = next(f for f in by(report, "inertia.clusters", label, "j3") if f.severity.name == "FAIL")
        assert finding.data["declared"]["mass"] == 0.0
        masses[label] = finding.data["engine"]["mass"]
    assert masses["mujoco"] == pytest.approx(0.1131, rel=1e-3)  # collision sphere at density 1000
    assert masses[PB] == pytest.approx(1.0)


@needs_engines
def test_pybullet_reads_but_ignores_joint_friction():
    report = run(fixture("friction_pendulum.urdf"), backends=ENGINES)
    assert all(f.severity.name == "PASS" for f in by(report, "rollout.passive_effect", "mujoco"))
    effect = {
        ("friction" if "friction" in f.message else "damping"): f.severity.name
        for f in by(report, "rollout.passive_effect", PB)
    }
    assert effect == {"damping": "PASS", "friction": "FAIL"}
    assert worst(report, "import.damping_friction", PB) == "PASS"  # read correctly ... but not simulated


@needs_engines
def test_friction_verdict_is_measured_on_a_friction_free_twin():
    declared = parse_urdf(fixture("friction_pendulum.urdf"))
    with tempfile.TemporaryDirectory() as workdir:
        backend = backends.create("pybullet:inertia_from_file=1")
        backend.load(prepare_urdf(declared.path, workdir), declared, SimSettings())
        assert backend._twin is not None and backend._twin[0] != backend.client
        backend.close()


@needs_engines
def test_zero_width_range_is_locked_in_one_engine_and_free_in_others():
    report = run(fixture("zero_width.urdf"), backends=ENGINES)
    readings = by(report, "import.limits", "mujoco")[0].data["interpretations"]
    assert readings == {"pinocchio": "unlimited", "mujoco": "unlimited", PB: "locked"}
    assert worst(report, "file.limits") == "WARN"


@needs_engines
def test_sibling_order_conventions_differ():
    report = run(fixture("hand.urdf"), backends=["pinocchio", "mujoco"])
    finding = by(report, "import.joint_order", "mujoco")[0]
    assert finding.severity.name == "WARN"
    assert finding.data["follows"] == {
        "pinocchio": "depth-first, siblings sorted by name",
        "mujoco": "depth-first, file order",
    }


@needs_mujoco
def test_mujoco_turns_mimic_into_an_equality_constraint():
    report = run(fixture("divergent.urdf"), backends=["mujoco"], reference="none", only=["import.mimic"])
    assert "equality" in by(report, "import.mimic", "mujoco")[0].message


@needs_mujoco
def test_package_uris_and_meshes_load():
    report = run(
        fixture("mesh_pkg/my_robot_description/urdf/robot.urdf"),
        backends=["mujoco"],
        reference="none",
        only=["import.load", "file.meshes"],
    )
    assert worst(report, "import.load", "mujoco") == "PASS"


@needs_engines
@pytest.mark.parametrize("urdf, expect_skip", [(ONE_DOF, False), (STATIC, True)])
def test_degenerate_models_do_not_crash(tmp_path, urdf, expect_skip):
    path = tmp_path / "model.urdf"
    path.write_text(urdf)
    report = run(str(path), backends=ENGINES)
    assert not [f for f in report.findings if f.severity.name in ("ERROR", "FAIL")]
    assert (worst(report, "dynamics.mass_matrix") == "SKIP") == expect_skip


@needs_engines
def test_symptoms_are_attributed_to_their_cause():
    report = run(fixture("arm6.urdf"), backends=["pinocchio", "pybullet"])
    problem = report.problems[0]
    assert problem.check == "inertia.clusters"
    assert ("dynamics.mass_matrix", "pybullet") in problem.explains
    assert not any(p.unexplained for p in report.problems)


@needs_engines
def test_a_range_on_a_continuous_joint_is_enforced_by_some_engines_only(tmp_path):
    path = tmp_path / "arm.urdf"
    text = Path(fixture("arm6.urdf")).read_text()
    path.write_text(
        text.replace('<limit effort="10" velocity="6.0"/>', '<limit lower="-1" upper="1" effort="10" velocity="6.0"/>')
    )
    report = run(str(path), backends=ENGINES, only=["*.limits"])
    readings = by(report, "import.limits", "mujoco", "wrist_roll")[0].data["interpretations"]
    assert readings == {"pinocchio": "unlimited", "mujoco": "limited", PB: "limited"}
    assert worst(report, "file.limits") == "WARN" and report.ok  # ambiguous, not a contradiction


@needs_engines
def test_dynamics_need_a_second_engine():
    report = run(fixture("arm6.urdf"), backends=["pinocchio"], only=["dynamics"])
    assert {f.severity.name for f in report.findings if f.check.startswith("dynamics.")} == {"SKIP"}


@needs_engines
def test_an_engine_that_refuses_the_file_can_still_be_closed():
    engine = backends.create("pybullet")
    with pytest.raises(backends.BackendLoadError):
        engine.load(PreparedURDF(Path("/no/such.urdf"), [], 0, []), None, SimSettings())
    engine.close()  # the runner always closes; this must not raise


def test_pybullet_importer_output_is_summarized_per_message():
    from simcompat.backends.pybullet_backend import _importer_messages

    text = (
        "b3Warning[a.cpp,1]:\nNo inertial data for link, using mass=1b3Warning[a.cpp,1]:\nfoot_l"
        "b3Warning[a.cpp,1]:\nNo inertial data for link, using mass=1b3Warning[a.cpp,1]:\nfoot_r"
        "b3Warning[b.cpp,2]:\nBad inertia tensor properties, setting inertia to zero for link: thigh\n"
    )
    assert _importer_messages(text) == [
        "importer: No inertial data for link, using mass=1: foot_l, foot_r",
        "importer: Bad inertia tensor properties, setting inertia to zero for link: thigh",
    ]
