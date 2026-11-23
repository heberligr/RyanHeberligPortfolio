import numpy as np
import pytest

from segslicer.kinematics import (
    IKError,
    RotaryPose,
    check_linear_limits,
    from_commanded,
    inverse_kinematics,
    machine_to_part,
    part_to_machine,
    table_rotation,
    to_commanded,
    tool_direction_in_part,
)
from segslicer.transforms import direction_from_angles

Z = np.array([0.0, 0.0, 1.0])


def assert_points_up(d, pose):
    """The key IK property: after rotating the table, d points along machine +Z."""
    d = np.asarray(d, float) / np.linalg.norm(d)
    np.testing.assert_allclose(table_rotation(pose.a, pose.c) @ d, Z, atol=1e-9)


def test_vertical_gives_zero_tilt(machine):
    pose = inverse_kinematics([0, 0, 1], machine)
    assert pose.a == pytest.approx(0.0)
    assert pose.c == pytest.approx(0.0)


def test_vertical_keeps_previous_c(machine):
    pose = inverse_kinematics([0, 0, 1], machine, prev_pose=RotaryPose(30.0, 123.0))
    assert pose.a == pytest.approx(0.0)
    assert pose.c == pytest.approx(123.0)


@pytest.mark.parametrize("azimuth", [0, 45, 90, 135, 180, -135, -90, -45])
def test_45_degree_tilts_in_many_azimuths(machine, azimuth):
    d = direction_from_angles(45, azimuth)
    pose = inverse_kinematics(d, machine)
    assert abs(pose.a) == pytest.approx(45.0)
    assert_points_up(d, pose)
    np.testing.assert_allclose(tool_direction_in_part(pose), d, atol=1e-9)


def test_known_angles_tilt_toward_plus_x(machine):
    # d tilted 45 deg toward +X: rotate C by 90 so d lies toward +Y, then
    # tilt A by +45 about X. (Mirror solution A=-45, C=-90 is equally valid;
    # with prev C = 0 both are 90 deg away, the tie goes to the first one.)
    pose = inverse_kinematics([1, 0, 1], machine)
    assert (pose.a, pose.c) == pytest.approx((45.0, 90.0))


def test_known_angles_tilt_toward_minus_y(machine):
    # d tilted toward -Y needs no C turn at all: A = -45 brings it up.
    pose = inverse_kinematics([0, -1, 1], machine)
    assert (pose.a, pose.c) == pytest.approx((-45.0, 0.0))


@pytest.mark.parametrize("tilt", [10, 60, 89, 90])
@pytest.mark.parametrize("azimuth", [-170, -30, 0, 75, 160])
def test_ik_property_holds_over_range(machine, tilt, azimuth):
    d = direction_from_angles(tilt, azimuth)
    assert_points_up(d, inverse_kinematics(d, machine))


def test_unreachable_direction_raises(machine):
    with pytest.raises(IKError):
        inverse_kinematics(direction_from_angles(120, 0), machine)
    with pytest.raises(IKError):
        inverse_kinematics([0, 0, -1], machine)


def test_tight_a_limit_rejects_big_tilt(machine):
    machine.kinematics.a_axis.min_deg = -30
    machine.kinematics.a_axis.max_deg = 30
    with pytest.raises(IKError):
        inverse_kinematics(direction_from_angles(45, 0), machine)
    inverse_kinematics(direction_from_angles(25, 0), machine)  # fine


def test_one_sided_a_limit_picks_the_mirror_solution(machine):
    # Machine can only tilt A positive: IK must use the a > 0 solution.
    machine.kinematics.a_axis.min_deg = 0
    for az in (0, 90, 180, -90):
        d = direction_from_angles(40, az)
        pose = inverse_kinematics(d, machine)
        assert pose.a > 0
        assert_points_up(d, pose)


def test_c_continuity_avoids_full_turns(machine):
    d = direction_from_angles(30, 10)
    pose = inverse_kinematics(d, machine, prev_pose=RotaryPose(0, 720.0))
    assert abs(pose.c - 720.0) <= 180.0
    assert_points_up(d, pose)


def test_limited_c_stays_in_range(machine):
    c_axis = machine.kinematics.c_axis
    c_axis.continuous = False
    c_axis.min_deg, c_axis.max_deg = -180, 180
    for az in range(-180, 181, 30):
        d = direction_from_angles(50, az)
        pose = inverse_kinematics(d, machine, prev_pose=RotaryPose(0, 170))
        assert -180 - 1e-9 <= pose.c <= 180 + 1e-9
        assert_points_up(d, pose)


def test_signs_flip_commanded_angles(machine):
    machine.kinematics.a_axis.sign = -1
    pose = RotaryPose(30.0, 45.0)
    a_cmd, c_cmd = to_commanded(pose, machine)
    assert (a_cmd, c_cmd) == (-30.0, 45.0)
    assert from_commanded(a_cmd, c_cmd, machine) == pose


def test_sign_affects_limit_check(machine):
    # With a flipped A sign and only negative commanded A allowed, the math
    # angle must come out positive.
    machine.kinematics.a_axis.sign = -1
    machine.kinematics.a_axis.min_deg = -90
    machine.kinematics.a_axis.max_deg = 0
    pose = inverse_kinematics(direction_from_angles(45, 0), machine)
    assert pose.a > 0
    assert to_commanded(pose, machine)[0] < 0


# ---------------------------- pivot handling ----------------------------- #
PIVOT = np.array([12.0, -5.0, 40.0])


def test_pivot_point_does_not_move():
    pose = RotaryPose(37.0, -122.0)
    np.testing.assert_allclose(part_to_machine(PIVOT, pose, PIVOT), PIVOT, atol=1e-9)


def test_point_along_build_direction_ends_up_above_pivot(machine):
    d = direction_from_angles(45, 30)
    pose = inverse_kinematics(d, machine)
    p = PIVOT + 25.0 * d
    np.testing.assert_allclose(part_to_machine(p, pose, PIVOT), PIVOT + [0, 0, 25.0], atol=1e-9)


def test_zero_pose_is_identity():
    pts = np.array([[1.0, 2.0, 3.0], [-4.0, 0.0, 9.0]])
    np.testing.assert_allclose(part_to_machine(pts, RotaryPose(0, 0), PIVOT), pts, atol=1e-12)


def test_pure_c_rotation_about_offset_pivot():
    # C=90 turns (pivot + x_hat) into (pivot + y_hat): rotation is about the
    # pivot's vertical axis, not the machine origin.
    out = part_to_machine(PIVOT + [1, 0, 0], RotaryPose(0, 90), PIVOT)
    np.testing.assert_allclose(out, PIVOT + [0, 1, 0], atol=1e-9)


def test_part_machine_round_trip():
    rng = np.random.default_rng(1)
    pts = rng.uniform(-80, 80, size=(50, 3))
    pose = RotaryPose(-63.0, 211.0)
    back = machine_to_part(part_to_machine(pts, pose, PIVOT), pose, PIVOT)
    np.testing.assert_allclose(back, pts, atol=1e-9)


def test_linear_limit_check(machine):
    pts = np.array([[0, 0, 10], [500, 0, 10], [0, 0, -1]])
    np.testing.assert_array_equal(check_linear_limits(pts, machine), [False, True, True])
