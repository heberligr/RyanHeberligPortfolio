import numpy as np
import pytest
import trimesh

from segslicer import geometry
from segslicer.demo_models import cube, cube_with_arm, t_shape
from segslicer.overhang import overhang_area
from segslicer.segments import Cut, Segment, _print_order, apply_cuts


def two_towers():
    """Slab with two posts at x = -15 and x = +15 (a "U" with square posts)."""
    slab = trimesh.creation.box(extents=[40, 10, 5])
    slab.apply_translation([0, 0, 2.5])
    posts = []
    for x in (-15, 15):
        p = trimesh.creation.box(extents=[10, 10, 21])
        p.apply_translation([x, 0, 14.5])  # z 4..25, overlapping the slab
        posts.append(p)
    return geometry.union([slab] + posts)


def assert_volume_conserved(part, seg):
    assert sum(s.mesh.volume for s in seg.segments) == pytest.approx(part.volume, rel=1e-6)
    assert all(s.mesh.is_watertight for s in seg.segments)


def test_no_cuts_one_segment(machine):
    seg = apply_cuts(cube(), [], machine)
    assert len(seg.segments) == 1 and seg.order == [0] and seg.valid


def test_arm_cut(machine):
    part = cube_with_arm()
    seg = apply_cuts(part, [Cut([10, 0, 14], [1, 0, 0])], machine)
    assert seg.valid, seg.warnings
    assert len(seg.segments) == 2
    assert seg.segments[1].supports == [0]
    assert seg.order == [0, 1]
    np.testing.assert_allclose(seg.segments[1].direction, [1, 0, 0])
    assert seg.segments[1].tilt_deg == pytest.approx(90)
    # The arm printed along +X has no overhang left.
    assert overhang_area(seg.segments[1].mesh, seg.segments[1].direction) == 0
    assert_volume_conserved(part, seg)


def test_t_shape_two_cuts(machine):
    part = t_shape()
    cuts = [Cut([5, 0, 34], [1, 0, 0]), Cut([-5, 0, 34], [-1, 0, 0])]
    seg = apply_cuts(part, cuts, machine)
    assert seg.valid, seg.warnings
    assert len(seg.segments) == 3
    assert seg.segments[1].supports == [0] and seg.segments[2].supports == [0]
    assert seg.order == [0, 1, 2]
    assert_volume_conserved(part, seg)


def test_local_cut_takes_only_nearest_piece(machine):
    part = two_towers()
    seg = apply_cuts(part, [Cut([15, 0, 10], [0, 0, 1], local=True)], machine)
    rep = seg.cut_reports[0]
    assert len(rep.component_centroids) == 2  # two posts above the plane
    new = seg.segments[1].mesh
    assert new.bounds[0][0] == pytest.approx(10)  # only the right post (x 10..20)
    assert len(geometry.components(seg.segments[0].mesh)) == 1  # left post glued back
    assert_volume_conserved(part, seg)


def test_local_cut_explicit_component(machine):
    part = two_towers()
    # Pieces are sorted by centroid x: piece 0 is the left post.
    seg = apply_cuts(part, [Cut([15, 0, 10], [0, 0, 1], local=True, components=[0])], machine)
    assert seg.segments[1].mesh.bounds[1][0] == pytest.approx(-10)


def test_non_local_cut_takes_all_pieces(machine):
    part = two_towers()
    seg = apply_cuts(part, [Cut([15, 0, 10], [0, 0, 1])], machine)
    assert len(geometry.components(seg.segments[1].mesh)) == 2
    assert seg.valid, seg.warnings


def test_chained_cut_targets_child_segment(machine):
    part = cube_with_arm()
    cuts = [Cut([10, 0, 14], [1, 0, 0]), Cut([25, 0, 14], [1, 0, 0])]  # second point is in the arm
    seg = apply_cuts(part, cuts, machine)
    assert seg.cut_reports[1].target_segment == 1
    assert seg.segments[2].supports == [1]
    assert seg.order == [0, 1, 2]


def test_dependencies_follow_later_cuts(machine):
    # Cut the arm off first, then cut the cube in half horizontally. The arm
    # (z 10..18) now rests on the UPPER cube half, so that must print first.
    part = cube_with_arm()
    cuts = [Cut([10, 0, 14], [1, 0, 0]), Cut([0, 0, 10], [0, 0, 1])]
    seg = apply_cuts(part, cuts, machine)
    assert seg.segments[1].supports == [2]
    assert seg.order == [0, 2, 1]
    assert seg.valid, seg.warnings


def test_reordering_cuts_changes_result(machine):
    part = cube_with_arm()
    a, b = Cut([10, 0, 14], [1, 0, 0]), Cut([0, 0, 10], [0, 0, 1])
    # Horizontal cut first: its upper half includes the arm, so the arm cut
    # then targets segment 1 instead of the root.
    seg = apply_cuts(part, [b, a], machine)
    assert seg.cut_reports[1].target_segment == 1
    assert seg.order == [0, 1, 2]


def test_point_outside_is_skipped(machine):
    seg = apply_cuts(cube(), [Cut([100, 0, 10], [1, 0, 0])], machine)
    assert not seg.cut_reports[0].applied
    assert len(seg.segments) == 1
    assert any("not inside any segment" in w for w in seg.warnings)


def test_plane_not_splitting_is_skipped(machine):
    # Point on the cube surface (accepted) but plane leaves nothing above.
    seg = apply_cuts(cube(), [Cut([10, 0, 10], [1, 0, 0])], machine)
    assert not seg.cut_reports[0].applied
    assert any("does not split" in w for w in seg.warnings)


def test_downward_cut_warns(machine):
    # Normal pointing down: the bottom half would become a segment printed
    # upside down and the top half would float above the bed.
    seg = apply_cuts(cube(), [Cut([0, 0, 10], [0, 0, -1])], machine)
    text = " ".join(seg.warnings)
    assert "mid-air" in text
    assert "outside the A-axis limits" in text


def test_cycle_detection():
    dummy = trimesh.creation.box()
    segs = [
        Segment(0, dummy, np.array([0, 0, 1.0]), np.zeros(3), None),
        Segment(1, dummy, np.array([1, 0, 0.0]), np.zeros(3), 0, supports=[2]),
        Segment(2, dummy, np.array([1, 0, 0.0]), np.zeros(3), 1, supports=[1]),
    ]
    order, warning = _print_order(segs)
    assert order[0] == 0 and sorted(order) == [0, 1, 2]
    assert warning and "loop" in warning
