"""Overhang detection.

Definition (same in every milestone): with build direction d, a face is an
overhang if the angle between its normal and -d is less than (90 - threshold).
With the default threshold of 50 deg, a face counts as an overhang when it
faces within 40 deg of straight down - i.e. it leans more than 50 deg away
from vertical.

Faces lying ON the segment's base plane (the bed, or the cut face the segment
sits on) are never overhangs: they are supported.
"""

from __future__ import annotations

import numpy as np
import trimesh

from .transforms import normalize


def overhang_mask(
    mesh: trimesh.Trimesh,
    direction: np.ndarray,
    threshold_deg: float = 50.0,
    base_tol: float = 1e-3,
) -> np.ndarray:
    """Boolean mask over faces: True where the face is an overhang.

    Args:
        mesh: the segment.
        direction: its build direction (any length).
        threshold_deg: overhang threshold described in the module docstring.
        base_tol: faces within this distance of the lowest point (along d)
            count as "on the base" and are ignored.
    """
    d = normalize(direction)
    # cos of the angle between each face normal and -d
    cos_to_down = mesh.face_normals @ (-d)
    is_down = cos_to_down > np.cos(np.radians(90.0 - threshold_deg))

    heights = mesh.triangles @ d  # (F, 3) height of each vertex along d
    base = heights.min()
    on_base = np.all(heights < base + base_tol, axis=1)
    return is_down & ~on_base


def overhang_area(mesh: trimesh.Trimesh, direction: np.ndarray, threshold_deg: float = 50.0) -> float:
    """Total area (mm^2) of overhang faces."""
    return float(mesh.area_faces[overhang_mask(mesh, direction, threshold_deg)].sum())
