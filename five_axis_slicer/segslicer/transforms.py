"""Basic rotation math used everywhere.

Conventions
-----------
* Rotations are 3x3 numpy arrays that act on COLUMN vectors: ``v_new = R @ v``.
* Arrays of points have shape (N, 3) - one point per row. To rotate them we use
  ``points @ R.T`` (which is the same as applying R to every row).
* Angles in degrees at the public API, radians internally.
"""

from __future__ import annotations

import numpy as np

Z_AXIS = np.array([0.0, 0.0, 1.0])

# Anything shorter than this is treated as a zero vector.
_EPS = 1e-12


def normalize(v: np.ndarray) -> np.ndarray:
    """Return v scaled to length 1. Raises ValueError for a zero vector."""
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v)
    if n < _EPS:
        raise ValueError("Cannot normalize a zero-length vector")
    return v / n


def rot_x(angle_deg: float) -> np.ndarray:
    """Right-hand-rule rotation about +X by angle_deg."""
    a = np.radians(angle_deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)


def rot_y(angle_deg: float) -> np.ndarray:
    """Right-hand-rule rotation about +Y by angle_deg."""
    a = np.radians(angle_deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)


def rot_z(angle_deg: float) -> np.ndarray:
    """Right-hand-rule rotation about +Z by angle_deg."""
    a = np.radians(angle_deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


def axis_angle_matrix(axis: np.ndarray, angle_rad: float) -> np.ndarray:
    """Rodrigues' formula: rotation by angle_rad about a unit axis."""
    k = normalize(axis)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(angle_rad) * K + (1 - np.cos(angle_rad)) * (K @ K)


def rotation_between(u: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Smallest rotation R with R @ u_hat == v_hat.

    Rotates about the axis u x v by the angle between them. If u and v point in
    exactly opposite directions there are infinitely many answers; we pick a
    180 degree turn about any axis perpendicular to u.
    """
    u, v = normalize(u), normalize(v)
    cos_t = float(np.clip(np.dot(u, v), -1.0, 1.0))
    axis = np.cross(u, v)
    if np.linalg.norm(axis) < 1e-9:
        if cos_t > 0:
            return np.eye(3)  # already aligned
        # Opposite: pick any axis perpendicular to u (cross with the least
        # aligned world axis so the cross product is never tiny).
        helper = np.eye(3)[np.argmin(np.abs(u))]
        return axis_angle_matrix(np.cross(u, helper), np.pi)
    return axis_angle_matrix(axis, np.arccos(cos_t))


def rotation_to_z(direction: np.ndarray) -> np.ndarray:
    """Rotation that takes ``direction`` onto +Z (used to lay a segment flat)."""
    return rotation_between(direction, Z_AXIS)


def rotation_from_z(direction: np.ndarray) -> np.ndarray:
    """Rotation that takes +Z onto ``direction`` (inverse of rotation_to_z)."""
    # The inverse of a rotation matrix is its transpose.
    return rotation_to_z(direction).T


def rotate_points(points: np.ndarray, R: np.ndarray, center: np.ndarray | None = None) -> np.ndarray:
    """Rotate (N, 3) points by R about ``center`` (origin if None)."""
    pts = np.asarray(points, dtype=float)
    if center is None:
        return pts @ R.T
    c = np.asarray(center, dtype=float)
    return (pts - c) @ R.T + c


def homogeneous(R: np.ndarray, t: np.ndarray | None = None) -> np.ndarray:
    """Pack a 3x3 rotation and a translation into a 4x4 matrix (for trimesh)."""
    T = np.eye(4)
    T[:3, :3] = R
    if t is not None:
        T[:3, 3] = t
    return T


def rotation_about_point(R: np.ndarray, center: np.ndarray) -> np.ndarray:
    """4x4 matrix for 'rotate by R about center':  p -> R (p - c) + c."""
    c = np.asarray(center, dtype=float)
    return homogeneous(R, c - R @ c)


def direction_from_angles(tilt_deg: float, azimuth_deg: float) -> np.ndarray:
    """Unit vector tilted ``tilt_deg`` away from +Z toward azimuth ``azimuth_deg``.

    azimuth 0 = toward +X, 90 = toward +Y. Handy for UI sliders and tests.
    """
    t, a = np.radians(tilt_deg), np.radians(azimuth_deg)
    return np.array([np.sin(t) * np.cos(a), np.sin(t) * np.sin(a), np.cos(t)])
