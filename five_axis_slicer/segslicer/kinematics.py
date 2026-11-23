"""Forward / inverse kinematics for a table-tilting A/C machine.

Machine layout (kinematics.type = "table_tilting_AC")
-----------------------------------------------------
The part sits on the C table. The C table is mounted on the A trunnion. So a
point on the part is first turned by C (about the table's own Z axis), then the
whole C table is tilted by A (about the machine X axis). Both axes pass through
the pivot point P (in machine coordinates):

    machine = R_A(a) @ R_C(c) @ (p - P) + P            (forward kinematics)
    p       = R_C(c).T @ R_A(a).T @ (machine - P) + P  (inverse)

where p is a point in the PART frame (= machine frame when a = c = 0), and
R_A = rot_x, R_C = rot_z in the right-hand-rule ("math") convention.

Angle conventions
-----------------
* ``a``, ``c`` inside this module are MATH angles (right-hand rule).
* The firmware may count the other way. machine.yaml has a ``sign`` per axis:
  ``commanded = sign * math``. Use :func:`to_commanded` right before writing
  G-code, and limits in machine.yaml are in commanded degrees.

Inverse kinematics problem
--------------------------
Each segment has a build direction d (unit vector in the part frame). We need
the table angles that make d point straight up (+Z) in the machine, so the
nozzle (which always points down) prints flat layers on that segment.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .config import MachineConfig
from .transforms import normalize, rot_x, rot_z


class IKError(ValueError):
    """Raised when no A/C pair within the axis limits can achieve a direction."""


@dataclass(frozen=True)
class RotaryPose:
    """A pair of rotary angles in MATH convention (degrees)."""

    a: float
    c: float


# --------------------------------------------------------------------------- #
# Forward kinematics
# --------------------------------------------------------------------------- #
def table_rotation(a_deg: float, c_deg: float) -> np.ndarray:
    """3x3 rotation applied to part-frame vectors: R = R_A(a) @ R_C(c)."""
    return rot_x(a_deg) @ rot_z(c_deg)


def part_to_machine(points: np.ndarray, pose: RotaryPose, pivot: np.ndarray) -> np.ndarray:
    """Map part-frame point(s) to machine XYZ for the given rotary pose.

    ``points`` may be a single (3,) point or an (N, 3) array.
    """
    R = table_rotation(pose.a, pose.c)
    P = np.asarray(pivot, dtype=float)
    pts = np.asarray(points, dtype=float)
    return (pts - P) @ R.T + P


def machine_to_part(points: np.ndarray, pose: RotaryPose, pivot: np.ndarray) -> np.ndarray:
    """Inverse of :func:`part_to_machine`."""
    R = table_rotation(pose.a, pose.c)
    P = np.asarray(pivot, dtype=float)
    pts = np.asarray(points, dtype=float)
    # Inverse rotation = transpose, and (x @ R.T).T = R x, so x @ R = R.T x.
    return (pts - P) @ R + P


def tool_direction_in_part(pose: RotaryPose) -> np.ndarray:
    """Which part-frame direction points straight up (+Z machine) at this pose.

    This is the inverse question to IK and is useful for checking it.
    """
    return table_rotation(pose.a, pose.c).T @ np.array([0.0, 0.0, 1.0])


# --------------------------------------------------------------------------- #
# Angle helpers
# --------------------------------------------------------------------------- #
def to_commanded(pose: RotaryPose, machine: MachineConfig) -> tuple[float, float]:
    """Convert math angles to the numbers that go after A and C in G-code."""
    k = machine.kinematics
    return k.a_axis.sign * pose.a, k.c_axis.sign * pose.c


def from_commanded(a_cmd: float, c_cmd: float, machine: MachineConfig) -> RotaryPose:
    """Convert G-code A/C numbers back into math angles."""
    k = machine.kinematics
    # sign is +1 or -1, so dividing and multiplying are the same thing.
    return RotaryPose(a=k.a_axis.sign * a_cmd, c=k.c_axis.sign * c_cmd)


def _closest_equivalent(angle: float, reference: float) -> float:
    """angle + k*360 that is closest to reference (avoids needless full turns)."""
    return angle + 360.0 * round((reference - angle) / 360.0)


def _pick_c(c_math: float, prev_c_math: float, machine: MachineConfig) -> float | None:
    """Choose the 360-degree-equivalent of c that is valid and nearest prev_c.

    Returns None if no equivalent is within the C limits.
    """
    axis = machine.kinematics.c_axis
    best = _closest_equivalent(c_math, prev_c_math)
    if axis.continuous:
        return best
    # Limited C: look at a few equivalents around the best one and keep the
    # in-limit candidate closest to the previous angle.
    candidates = [best + 360.0 * k for k in range(-2, 3)]
    valid = [c for c in candidates if axis.within_limits(axis.sign * c)]
    if not valid:
        return None
    return min(valid, key=lambda c: abs(c - prev_c_math))


# --------------------------------------------------------------------------- #
# Inverse kinematics
# --------------------------------------------------------------------------- #
def inverse_kinematics(
    direction: np.ndarray,
    machine: MachineConfig,
    prev_pose: RotaryPose | None = None,
    vertical_tol_deg: float = 1e-6,
) -> RotaryPose:
    """Find (a, c) so that table_rotation(a, c) @ direction == +Z.

    Derivation (see README for a sketch):
      Let d = (dx, dy, dz), s = sqrt(dx^2 + dy^2) (horizontal length),
      phi = atan2(dy, dx) (azimuth), theta = atan2(s, dz) (tilt from vertical).
      1. Rotate by C so d lies in the YZ plane:  R_C d = (0, +/-s, dz).
         For +s we need phi + c = 90 deg  ->  c = 90 - phi.
      2. Tilt by A about X to bring (0, s, dz) up to (0, 0, 1): a = +theta.
         (The mirror solution is c = -90 - phi, a = -theta.)

    Both solutions are tried; the one inside the axis limits with the smallest
    C change from ``prev_pose`` wins.

    Args:
        direction: build direction in the part frame (any length, not zero).
        machine: machine config (for limits and signs).
        prev_pose: previous rotary pose, used to avoid unnecessary C spins.
            Defaults to a = c = 0.
        vertical_tol_deg: tilts smaller than this are treated as vertical.

    Raises:
        IKError: if neither solution fits within the axis limits.
    """
    d = normalize(direction)
    prev = prev_pose or RotaryPose(0.0, 0.0)
    a_axis = machine.kinematics.a_axis

    s = float(np.hypot(d[0], d[1]))
    theta = float(np.degrees(np.arctan2(s, d[2])))  # tilt from +Z, 0..180

    if theta < vertical_tol_deg:
        # Pointing straight up: A = 0 and C can be anything. Keep C where it was
        # so the table doesn't spin for no reason.
        c = _pick_c(prev.c, prev.c, machine)
        if c is None or not a_axis.within_limits(0.0):
            raise IKError("Vertical direction is outside axis limits (check A/C limits)")
        return RotaryPose(0.0, c)

    phi = float(np.degrees(np.arctan2(d[1], d[0])))
    raw_solutions = [(theta, 90.0 - phi), (-theta, -90.0 - phi)]

    valid: list[RotaryPose] = []
    for a, c in raw_solutions:
        if not a_axis.within_limits(a_axis.sign * a):
            continue
        c_ok = _pick_c(c, prev.c, machine)
        if c_ok is None:
            continue
        valid.append(RotaryPose(a, c_ok))

    if not valid:
        raise IKError(
            f"Direction {np.round(d, 4).tolist()} needs a tilt of {theta:.1f} deg, "
            f"which is outside A limits [{a_axis.min_deg}, {a_axis.max_deg}] "
            "(or C limits) for both IK solutions."
        )
    # Prefer the smallest C move, then the smallest A move.
    return min(valid, key=lambda p: (abs(p.c - prev.c), abs(p.a - prev.a)))


def reachable(direction: np.ndarray, machine: MachineConfig) -> bool:
    """True if some A/C pair within limits points ``direction`` up."""
    try:
        inverse_kinematics(direction, machine)
        return True
    except IKError:
        return False


def check_linear_limits(points_machine: np.ndarray, machine: MachineConfig) -> np.ndarray:
    """Return a boolean mask of machine points that are OUTSIDE the XYZ limits."""
    pts = np.atleast_2d(np.asarray(points_machine, dtype=float))
    lim = machine.limits
    lo = np.array([lim.x[0], lim.y[0], lim.z[0]])
    hi = np.array([lim.x[1], lim.y[1], lim.z[1]])
    return np.any((pts < lo - 1e-9) | (pts > hi + 1e-9), axis=1)
