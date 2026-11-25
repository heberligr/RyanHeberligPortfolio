import numpy as np
import pytest
import trimesh

from segslicer.demo_models import cube, cube_with_arm
from segslicer.overhang import overhang_area, overhang_mask
from segslicer.transforms import rot_y

UP = np.array([0.0, 0.0, 1.0])


def test_cube_has_no_overhang():
    assert overhang_area(cube(), UP) == 0.0  # the bottom face is on the bed


def test_arm_underside_is_overhang():
    # 25 mm of arm outside the cube x 8 mm wide.
    assert overhang_area(cube_with_arm(), UP) == pytest.approx(25 * 8)


def test_arm_has_no_overhang_when_printed_sideways():
    # Printing along +X, the arm's base would be on the cube face, but here we
    # look at the whole part: only faces pointing toward -X are overhangs.
    m = cube_with_arm()
    mask = overhang_mask(m, [1, 0, 0])
    assert np.allclose(m.face_normals[mask] @ [-1, 0, 0], 1.0)


def test_threshold():
    # Box resting on an edge: its two bottom faces lean 45 deg from vertical.
    b = trimesh.creation.box(extents=[10, 10, 10])
    b.apply_transform(np.block([[rot_y(45), np.zeros((3, 1))], [np.zeros((1, 3)), 1]]))
    assert overhang_mask(b, UP, threshold_deg=50).sum() == 0  # 45 < 50: printable
    assert overhang_mask(b, UP, threshold_deg=40).sum() == 4  # 45 > 40: 2 faces x 2 triangles
