import numpy as np
import pytest
import trimesh

from segslicer import geometry


@pytest.fixture
def box():
    m = trimesh.creation.box(extents=[20, 10, 10])
    m.apply_translation([0, 0, 5])
    return m


def test_split_volumes_add_up(box):
    uppers, lower = geometry.split_by_plane(box, [4, 0, 0], [1, 0, 0])
    assert len(uppers) == 1
    assert uppers[0].volume == pytest.approx(6 * 10 * 10)
    assert lower.volume == pytest.approx(14 * 10 * 10)
    # Both sides are capped -> still closed solids.
    assert uppers[0].is_watertight and lower.is_watertight


def test_split_tilted_plane_is_watertight(box):
    uppers, lower = geometry.split_by_plane(box, [0, 0, 5], [1, 0.3, 0.8])
    assert all(u.is_watertight for u in uppers) and lower.is_watertight
    assert sum(u.volume for u in uppers) + lower.volume == pytest.approx(box.volume)


def test_plane_missing_the_mesh(box):
    uppers, lower = geometry.split_by_plane(box, [100, 0, 0], [1, 0, 0])
    assert uppers == [] and lower is not None


def test_union_restores_original(box):
    uppers, lower = geometry.split_by_plane(box, [4, 0, 0], [1, 0, 0])
    back = geometry.union([lower] + uppers)
    assert back.volume == pytest.approx(box.volume)
    assert len(geometry.components(back)) == 1


def test_winding_number_inside_outside_surface(box):
    assert geometry.winding_number(box, np.array([0, 0, 5.0])) == pytest.approx(1.0)
    assert geometry.winding_number(box, np.array([50, 0, 5.0])) == pytest.approx(0.0, abs=1e-9)
    assert geometry.contains(box, [9.9, 4.9, 0.1])
    assert not geometry.contains(box, [10.1, 0, 5])


def test_distance_to_surface(box):
    assert geometry.distance_to_surface(box, [15, 0, 5]) == pytest.approx(5.0)
    assert geometry.distance_to_surface(box, [0, 0, 5]) == pytest.approx(5.0)  # inside: to top/bottom/sides


def test_faces_on_plane(box):
    top = geometry.faces_on_plane(box, [0, 0, 10], [0, 0, 1], facing=+1)
    bottom = geometry.faces_on_plane(box, [0, 0, 0], [0, 0, 1], facing=-1)
    assert top.sum() == 2 and bottom.sum() == 2


def test_open_mesh_rejected():
    m = trimesh.creation.box()
    m.update_faces(np.arange(len(m.faces) - 1))  # remove a triangle -> hole
    with pytest.raises(ValueError, match="closed solid"):
        geometry.to_manifold(m)
