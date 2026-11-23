import numpy as np
import pytest

from segslicer.transforms import (
    Z_AXIS,
    direction_from_angles,
    rotate_points,
    rotation_about_point,
    rotation_between,
    rotation_from_z,
    rotation_to_z,
)

# A spread of directions including the awkward ones (+Z, -Z, horizontal).
DIRECTIONS = [
    [0, 0, 1],
    [0, 0, -1],
    [1, 0, 0],
    [0, -1, 0],
    [1, 1, 1],
    [-0.3, 0.2, -0.9],
    direction_from_angles(45, 135),
    direction_from_angles(89.9, -60),
]


@pytest.mark.parametrize("d", DIRECTIONS)
def test_rotation_to_z_sends_direction_to_z(d):
    d = np.asarray(d, float)
    R = rotation_to_z(d)
    np.testing.assert_allclose(R @ (d / np.linalg.norm(d)), Z_AXIS, atol=1e-9)


@pytest.mark.parametrize("d", DIRECTIONS)
def test_rotation_is_proper(d):
    R = rotation_to_z(d)
    np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-9)  # orthonormal
    assert np.isclose(np.linalg.det(R), 1.0)  # not a mirror


@pytest.mark.parametrize("d", DIRECTIONS)
def test_to_z_and_back_round_trip(d):
    rng = np.random.default_rng(0)
    pts = rng.uniform(-50, 50, size=(20, 3))
    center = np.array([3.0, -7.0, 12.0])
    there = rotate_points(pts, rotation_to_z(d), center)
    back = rotate_points(there, rotation_from_z(d), center)
    np.testing.assert_allclose(back, pts, atol=1e-9)


def test_rotation_between_arbitrary_vectors():
    u, v = np.array([1.0, 2.0, 3.0]), np.array([-2.0, 0.5, 1.0])
    R = rotation_between(u, v)
    np.testing.assert_allclose(R @ (u / np.linalg.norm(u)), v / np.linalg.norm(v), atol=1e-9)


def test_rotation_about_point_keeps_center_fixed():
    c = np.array([10.0, 20.0, 30.0])
    T = rotation_about_point(rotation_to_z([1, 0, 0]), c)
    np.testing.assert_allclose(T @ np.append(c, 1.0), np.append(c, 1.0), atol=1e-9)


def test_direction_from_angles():
    np.testing.assert_allclose(direction_from_angles(0, 77), [0, 0, 1], atol=1e-12)
    np.testing.assert_allclose(direction_from_angles(90, 90), [0, 1, 0], atol=1e-12)
