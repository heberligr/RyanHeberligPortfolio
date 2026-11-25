"""Test/demo parts generated in code (no STL needed).

All parts are built as unions of boxes, so they're guaranteed watertight, and
they sit with their lowest face at z = 0, centred on x = y = 0.
"""

from __future__ import annotations

import trimesh

from . import geometry


def _box(size: tuple[float, float, float], min_corner: tuple[float, float, float]) -> trimesh.Trimesh:
    """Axis-aligned box given its size and its minimum corner."""
    b = trimesh.creation.box(extents=size)
    b.apply_translation([min_corner[i] + size[i] / 2 for i in range(3)])
    return b


def cube(size: float = 20.0) -> trimesh.Trimesh:
    """Plain cube. Nothing overhangs, so no cuts are needed."""
    return _box((size, size, size), (-size / 2, -size / 2, 0))


def cube_with_arm(size: float = 20.0, arm_length: float = 25.0, arm_thickness: float = 8.0) -> trimesh.Trimesh:
    """Cube with a horizontal arm sticking out in +X near the top.

    The arm's underside is a 100% overhang when printed upright. Expected
    segmentation: 2 segments (cube, then the arm printed along +X).
    """
    body = cube(size)
    arm = _box(
        (arm_length + 1.0, arm_thickness, arm_thickness),  # +1 mm overlap into the cube
        (size / 2 - 1.0, -arm_thickness / 2, size - arm_thickness - 2.0),
    )
    return geometry.union([body, arm])


def t_shape(post: float = 10.0, height: float = 30.0, bar_length: float = 50.0, bar_height: float = 8.0) -> trimesh.Trimesh:
    """Upright post with a horizontal bar on top (a "T").

    Both halves of the bar overhang. Expected segmentation: 3 segments (post,
    then each side of the bar printed sideways).
    """
    stem = _box((post, post, height + 1.0), (-post / 2, -post / 2, 0))  # +1 overlap into the bar
    bar = _box((bar_length, post, bar_height), (-bar_length / 2, -post / 2, height))
    return geometry.union([stem, bar])


DEMO_MODELS = {
    "Cube": cube,
    "Cube with arm": cube_with_arm,
    "T-shape": t_shape,
}
