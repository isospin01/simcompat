"""Turning user input into a loadable URDF: paths, directories, xacro, robot_descriptions names."""

import pytest

from simcompat.inputs import InputError, find_robot_files, prepare_urdf, resolve

from conftest import FIXTURES, installed, needs_xacrodoc

MESH_PKG = FIXTURES / "mesh_pkg" / "my_robot_description"


def test_plain_urdf():
    found = resolve(str(FIXTURES / "arm6.urdf"))
    assert found.urdf.name == "arm6.urdf" and found.label == "arm6"


def test_directory_with_one_robot():
    found = resolve(str(MESH_PKG))
    assert found.urdf.name == "robot.urdf" and "found urdf/robot.urdf" in found.notes[0]


def test_directory_with_several_robots_lists_them():
    with pytest.raises(InputError, match="pass one of them"):
        resolve(str(FIXTURES))


def test_missing_file_suggests_robot_descriptions():
    with pytest.raises(InputError, match="rd:go2"):
        resolve("go2")


def test_find_robot_files_prefers_plain_urdf():
    assert [p.name for p in find_robot_files(MESH_PKG)] == ["robot.urdf"]


def test_prepare_resolves_package_uris(tmp_path):
    prepared = prepare_urdf(MESH_PKG / "urdf" / "robot.urdf", tmp_path)
    text = prepared.path.read_text()
    assert "package://" not in text and str((MESH_PKG / "meshes" / "box.stl").resolve()) in text
    assert prepared.stripped_visuals == 1 and not prepared.unresolved_meshes


def test_prepare_reports_unresolved_meshes(tmp_path):
    path = tmp_path / "r.urdf"
    path.write_text(
        '<robot name="r"><link name="a"><collision><geometry><mesh filename="package://nope/x.stl"/>'
        "</geometry></collision></link></robot>"
    )
    assert prepare_urdf(path, tmp_path / "out").unresolved_meshes == ["package://nope/x.stl"]


@needs_xacrodoc
def test_xacro_is_expanded(tmp_path, monkeypatch):
    monkeypatch.setenv("SIMCOMPAT_CACHE", str(tmp_path / "cache"))
    found = resolve(str(FIXTURES / "xacro" / "pendulum.urdf.xacro"), xacro_args={"length": "0.7"})
    text = found.urdf.read_text()
    assert "xacro:" not in text and 'xyz="0 0 -0.7"' in text
    assert found.label == "pendulum" and "expanded" in found.notes[0]


@needs_xacrodoc
def test_a_misspelled_xacro_argument_is_reported(tmp_path, monkeypatch):
    monkeypatch.setenv("SIMCOMPAT_CACHE", str(tmp_path / "cache"))
    monkeypatch.chdir(FIXTURES)  # a relative path, as typed on the command line
    messages = []
    found = resolve("xacro/pendulum.urdf.xacro", xacro_args={"lenght": "0.7"}, log=messages.append)
    assert any("lenght not declared" in note and "length" in note for note in found.notes)
    assert messages and "lenght" in messages[0]


@needs_xacrodoc
def test_xacro_is_expanded_again_when_an_included_file_changes(tmp_path, monkeypatch):
    monkeypatch.setenv("SIMCOMPAT_CACHE", str(tmp_path / "cache"))
    (tmp_path / "part.xacro").write_text(
        '<robot xmlns:xacro="http://www.ros.org/wiki/xacro"><xacro:property name="m" value="1"/></robot>'
    )
    main = tmp_path / "robot.urdf.xacro"
    main.write_text(
        '<robot name="r" xmlns:xacro="http://www.ros.org/wiki/xacro"><xacro:include filename="part.xacro"/>'
        '<link name="a"><inertial><mass value="${m}"/><inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/>'
        "</inertial></link></robot>"
    )
    assert 'value="1"' in resolve(str(main)).urdf.read_text()
    (tmp_path / "part.xacro").write_text(
        '<robot xmlns:xacro="http://www.ros.org/wiki/xacro"><xacro:property name="m" value="5"/></robot>'
    )
    assert 'value="5"' in resolve(str(main)).urdf.read_text()


@pytest.mark.skipif(not installed("robot_descriptions"), reason="robot_descriptions not installed")
def test_unknown_robot_descriptions_name_gets_suggestions():
    with pytest.raises(InputError, match=r"Did you mean: rd:go2[,?]"):
        resolve("rd:go22")
    assert resolve("RD:Go2").label == "go2"  # names are not case-sensitive
