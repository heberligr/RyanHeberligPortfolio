import numpy as np
import pytest
import trimesh

from segslicer.collision import (
    HotendModel,
    ObstacleSet,
    check_collision,
    check_many,
    sample_surface_grid,
    table_mesh,
)
from segslicer.transforms import direction_from_angles

UP = np.array([0.0, 0.0, 1.0])


@pytest.fixture
def hotend():
    # Cone 45 deg, 10 mm tall -> 10 mm radius at top; cylinder r=20 from z=10 to 40.
    return HotendModel(
        cone_half_angle_deg=45.0,
        cone_height=10.0,
        cylinder_radius=20.0,
        cylinder_start=10.0,
        cylinder_height=30.0,
        clearance=0.0,
        tip_ignore_height=0.3,
    )


def box(extents, center):
    m = trimesh.creation.box(extents=extents)
    m.apply_translation(center)
    return m


def obstacles(*meshes, spacing=0.5):
    return ObstacleSet.from_meshes(list(meshes), spacing)


def test_radius_profile(hotend):
    r = hotend.radius_at(np.array([0.0, 5.0, 10.0, 25.0, 40.0, 41.0]))
    np.testing.assert_allclose(r, [0, 5, 20, 20, 20, 0], atol=1e-9)


def test_mesh_matches_size(hotend):
    m = hotend.to_mesh([0, 0, 0], UP)
    np.testing.assert_allclose(m.bounds[0], [-20, -20, 0], atol=1e-6)
    np.testing.assert_allclose(m.bounds[1], [20, 20, 40], atol=1e-6)


def test_mesh_follows_direction(hotend):
    m = hotend.to_mesh([5, 5, 5], [1, 0, 0])
    assert m.bounds[0][0] == pytest.approx(5.0, abs=1e-6)  # tip end
    assert m.bounds[1][0] == pytest.approx(45.0, abs=1e-6)  # top of cylinder


def test_sampling_covers_surface():
    m = box([10, 10, 10], [0, 0, 0])
    pts = sample_surface_grid(m, 1.0)
    # All samples are on the surface (some coordinate is at +/-5).
    assert np.all(np.isclose(np.abs(pts).max(axis=1), 5.0))
    # Spacing is respected: every point of a fine reference grid on the top
    # face is within ~1 sample spacing of a sample.
    g = np.stack(np.meshgrid(np.linspace(-5, 5, 41), np.linspace(-5, 5, 41)), -1).reshape(-1, 2)
    top = pts[np.isclose(pts[:, 2], 5.0)][:, :2]
    nearest = np.min(np.linalg.norm(g[:, None] - top[None], axis=2), axis=1)
    assert nearest.max() <= 1.0


def test_tip_resting_on_box_is_not_a_collision(hotend):
    # Printing on top of a block, nozzle vertical: the surface under the tip
    # must not count as a hit.
    obs = obstacles(box([20, 20, 10], [0, 0, 5]))
    assert not check_collision(hotend, [0, 0, 10.2], UP, obs).hit


def test_box_beside_cone_clear_and_hit(hotend):
    # Tall wall 3 mm thick, from x=12..15. At 1 mm above the tip the cone is
    # only 1 mm wide, but higher up (cylinder r=20) it reaches the wall.
    wall = box([3, 40, 60], [13.5, 0, 30])
    obs = obstacles(wall)
    res = check_collision(hotend, [0, 0, 0], UP, obs)
    assert res.hit  # cylinder (r=20) sweeps into the wall
    # Move the wall far enough away: no hit.
    obs_far = obstacles(box([3, 40, 60], [25, 0, 30]))
    assert not check_collision(hotend, [0, 0, 0], UP, obs_far).hit


def test_short_obstacle_only_hits_cone_when_close(hotend):
    # A low block 4 mm tall: only the cone region (z 0..4, r <= 4) can reach it.
    def low_block_at(x):
        return obstacles(box([2, 10, 4], [x + 1, 0, 2]))  # block spans x..x+2

    assert check_collision(hotend, [0, 0, 0], UP, low_block_at(2.0)).hit  # r=2 at z>=2
    assert not check_collision(hotend, [0, 0, 0], UP, low_block_at(6.0)).hit  # r<=4 < 6


def test_tilting_toward_obstacle_causes_hit(hotend):
    # Tip just above a large flat block. Vertical tool: fine.
    # Tool tilted 60 deg: the 45 deg cone dips below horizontal and hits.
    obs = obstacles(box([60, 60, 10], [0, 0, -5]), spacing=1.0)
    tip = [0, 0, 0.5]
    assert not check_collision(hotend, tip, UP, obs).hit
    assert check_collision(hotend, tip, direction_from_angles(60, 0), obs).hit
    # 30 deg tilt: the cone clears, but the wide cylinder's low edge dips to
    # z = 0.5 + 10*cos30 - 20*sin30 = -0.84 -> still a hit (the cylinder matters!).
    assert check_collision(hotend, tip, direction_from_angles(30, 0), obs).hit
    # 20 deg tilt: lowest cylinder edge at 0.5 + 10*cos20 - 20*sin20 = +3.06 -> clear.
    assert not check_collision(hotend, tip, direction_from_angles(20, 0), obs).hit


def test_far_obstacle_is_skipped(hotend):
    obs = obstacles(box([5, 5, 5], [500, 500, 500]))
    assert not check_collision(hotend, [0, 0, 0], UP, obs).hit


def test_reports_which_obstacle(hotend):
    near = box([5, 5, 50], [8, 0, 25])
    far = box([5, 5, 5], [300, 0, 0])
    obs = ObstacleSet.from_meshes([far, near], 0.5, names=["far", "near"])
    res = check_collision(hotend, [0, 0, 0], UP, obs)
    assert res.hit and res.obstacle_name == "near"


def test_check_many_finds_first_hit(hotend):
    obs = obstacles(box([3, 40, 60], [13.5, 0, 30]))
    tips = np.array([[-60, 0, 0], [-40, 0, 0], [0, 0, 0], [5, 0, 0]])
    hits = check_many(hotend, tips, UP, obs)
    assert [i for i, _ in hits] == [2]


def test_clearance_makes_it_more_conservative(hotend):
    obs = obstacles(box([3, 40, 60], [22.5, 0, 30]))  # wall face at x=21
    assert not check_collision(hotend, [0, 0, 0], UP, obs).hit
    hotend.clearance = 1.0
    assert check_collision(hotend, [0, 0, 0], UP, obs).hit


def test_table_mesh_top_at_surface(machine):
    m = table_mesh(machine.table)
    assert m.bounds[1][2] == pytest.approx(machine.table.surface_z)
