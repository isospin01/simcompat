"""The declared model: simcompat's own reading of the file (no engine involved)."""

import numpy as np
import pytest

from simcompat.declared import URDFError, parse_urdf
from simcompat.spatial import combine_inertias, rpy_to_matrix

from conftest import fixture


def test_rpy_follows_the_urdf_convention():
    # URDF rpy = fixed-axis rotations X, then Y, then Z  =>  R = Rz @ Ry @ Rx
    assert np.allclose(rpy_to_matrix([0, 0, np.pi / 2]) @ [1, 0, 0], [0, 1, 0])
    assert np.allclose(rpy_to_matrix([np.pi / 2, 0, 0]) @ [0, 1, 0], [0, 0, 1])


def test_parse_divergent_fixture():
    model = parse_urdf(fixture("divergent.urdf"))
    assert model.root == "base"
    assert model.movable_joints == ["j1", "j2", "j3", "jm"]
    assert model.joints["j2"].type == "continuous" and not model.joints["j2"].limited
    assert (model.joints["j1"].damping, model.joints["j1"].friction) == (0.5, 0.2)
    assert model.joints["jm"].mimic.joint == "j1" and model.joints["jm"].mimic.multiplier == -1
    assert model.links_without_inertial() == ["noinert"]
    assert model.carrier("tool") == "j2"  # a fixed child is moved by its parent's joint


def test_rotated_inertial_frame_is_applied():
    inertia = parse_urdf(fixture("divergent.urdf")).links["l1"].inertial.inertia_in_link_frame()
    assert np.allclose(np.diag(inertia), [0.02, 0.01, 0.025])  # rpy z = 90 deg swaps ixx and iyy


def test_moving_bodies_merge_fixed_children():
    mass, _, _ = parse_urdf(fixture("divergent.urdf")).moving_bodies()["j2"]
    assert mass == pytest.approx(0.8)  # l2 (0.5 kg) + the fixed tool (0.3 kg)


def test_parallel_axis_theorem():
    parts = [(1.0, np.array([1.0, 0, 0]), np.zeros((3, 3))), (1.0, np.array([-1.0, 0, 0]), np.zeros((3, 3)))]
    mass, com, inertia = combine_inertias(parts)
    assert mass == 2 and np.allclose(com, 0) and np.allclose(np.diag(inertia), [0, 2, 2])


def test_traversal_orders_of_a_branched_tree():
    model = parse_urdf(fixture("hand.urdf"))
    assert model.joint_order_dfs() == ["index_j1", "index_j2", "thumb_j1", "middle_j1"]
    assert model.joint_order_dfs(sort_by_name=True) == ["index_j1", "index_j2", "middle_j1", "thumb_j1"]
    assert model.joint_order_bfs() == ["index_j1", "thumb_j1", "middle_j1", "index_j2"]


def test_forward_kinematics_is_periodic():
    model = parse_urdf(fixture("arm6.urdf"))
    a, b = model.joint_frames({}), model.joint_frames({"shoulder_pan": 2 * np.pi})
    for joint in a:
        assert np.allclose(a[joint][0], b[joint][0]) and np.allclose(a[joint][2], b[joint][2])


def test_zero_width_range_and_axis_norm():
    joint = parse_urdf(fixture("zero_width.urdf")).joints["hinge"]
    assert joint.limited and joint.zero_width and joint.lower == joint.upper == 0
    assert joint.axis_norm == pytest.approx(1.0)


def test_explicit_zero_mimic_multiplier_is_kept(tmp_path):
    path = tmp_path / "m.urdf"
    path.write_text(
        '<robot name="m"><link name="a"/><link name="b"/><link name="c"/>'
        '<joint name="j" type="revolute"><parent link="a"/><child link="b"/><limit lower="-1" upper="1"/></joint>'
        '<joint name="k" type="revolute"><parent link="b"/><child link="c"/><limit lower="-1" upper="1"/>'
        '<mimic joint="j" multiplier="0"/></joint></robot>'
    )
    assert parse_urdf(path).joints["k"].mimic.multiplier == 0.0


@pytest.mark.parametrize(
    "text, message",
    [
        (
            '<robot name="r" xmlns:xacro="http://www.ros.org/wiki/xacro"><xacro:property name="a" value="1"/>'
            '<link name="a"/></robot>',
            "xacro",
        ),
        ('<robot name="r"><link name="a"/><link name="b"/></robot>', "exactly one root"),
        ("<sdf/>", "expected <robot>"),
        ("not xml", "not valid XML"),
    ],
)
def test_unreadable_files_have_actionable_errors(tmp_path, text, message):
    path = tmp_path / "bad.urdf"
    path.write_text(text)
    with pytest.raises(URDFError, match=message):
        parse_urdf(path)


@pytest.mark.parametrize(
    "body, message",
    [
        ('<link name="a"/><link name="a"/>', "duplicate link"),
        ('<link name="a"/><joint name="j" type="fixed"><parent link="a"/><child link="b"/></joint>', "not declared"),
        ('<link name="a"/><link name="b"/>', "exactly one root"),
        (
            '<link name="a"/><link name="b"/><joint name="j" type="hinge"><parent link="a"/><child link="b"/></joint>',
            "unknown type",
        ),
        ('<link name="a"><inertial><mass value="abc"/></inertial></link>', "link 'a': <mass value>: 'abc' is not"),
    ],
)
def test_malformed_files_are_rejected_with_a_reason(tmp_path, body, message):
    path = tmp_path / "bad.urdf"
    path.write_text(f'<robot name="r">{body}</robot>')
    with pytest.raises(URDFError, match=message):
        parse_urdf(path)


def test_inertia_defects_are_classified():
    from simcompat.declared import INERTIA_EDGE, INERTIA_NOT_PSD, INERTIA_TRIANGLE, Inertial
    from simcompat.spatial import inertia_matrix

    def defect(*moments):
        return Inertial(1.0, np.zeros(3), np.zeros(3), np.diag(moments)).defect()

    assert defect(1.0, 2.0, 2.5) is None
    assert defect(-1.0, 2.0, 2.5)[0] == INERTIA_NOT_PSD
    assert defect(1.0, 1.0, 3.0)[0] == INERTIA_TRIANGLE
    assert defect(1.0, 1.0, 2.0)[0] == INERTIA_EDGE  # a flat plate: possible, but fragile under rounding
    assert Inertial(1.0, np.zeros(3), np.zeros(3), inertia_matrix(1, 0, 0, 1, 0, 1)).defect() is None
