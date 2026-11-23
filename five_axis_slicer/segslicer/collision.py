"""Hotend collision model: a cone at the nozzle tip plus a cylinder above it.

How the check works (kept simple on purpose)
--------------------------------------------
1. Every obstacle mesh (bed, already-printed segments) is turned ONCE into a
   cloud of points spread over its surface, about ``sample_spacing`` mm apart
   (:class:`ObstacleSet`).
2. To test a nozzle position we move those points into the TOOL frame
   (tip at the origin, tool axis = +Z pointing away from the part) and ask
   "is any point inside the hotend shape?" That is just a couple of numpy
   comparisons per point.
3. Because the samples have gaps, the hotend shape is inflated by about
   0.6 x sample_spacing (plus the configured clearance), so a surface can't
   sneak between two samples unnoticed. This makes the check slightly
   conservative (it may report a hit a little early), never optimistic.
4. A cheap bounding-box test skips obstacles that are nowhere near the tool.

Limitation: only surfaces are sampled, so a hotend buried completely inside
a solid with no surface nearby would be missed. That cannot happen for a real
nozzle printing on top of the part, so we accept it.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
import trimesh

from .config import HotendConfig, MachineConfig, TableConfig
from .transforms import homogeneous, normalize, rotation_from_z, rotation_to_z


# --------------------------------------------------------------------------- #
# Hotend shape
# --------------------------------------------------------------------------- #
@dataclass
class HotendModel:
    """Rotationally symmetric hotend envelope in the TOOL frame.

    z = distance from the nozzle tip along the tool axis (up, away from part).
      0 .. cone_height              -> cone, radius = z * tan(half_angle)
      cone_height .. cylinder_start -> neck, radius = cone top radius
      cylinder_start .. +height     -> cylinder, radius = cylinder_radius
    """

    cone_half_angle_deg: float
    cone_height: float
    cylinder_radius: float
    cylinder_start: float
    cylinder_height: float
    clearance: float = 0.0
    tip_ignore_height: float = 0.3

    @classmethod
    def from_config(cls, cfg: HotendConfig) -> "HotendModel":
        return cls(
            cone_half_angle_deg=cfg.cone_half_angle_deg,
            cone_height=cfg.cone_height,
            cylinder_radius=cfg.cylinder_radius,
            cylinder_start=cfg.cylinder_start,
            cylinder_height=cfg.cylinder_height,
            clearance=cfg.clearance,
            tip_ignore_height=cfg.tip_ignore_height,
        )

    @property
    def cone_top_radius(self) -> float:
        return self.cone_height * np.tan(np.radians(self.cone_half_angle_deg))

    @property
    def total_height(self) -> float:
        return max(self.cone_height, self.cylinder_start + self.cylinder_height)

    @property
    def max_radius(self) -> float:
        return max(self.cone_top_radius, self.cylinder_radius)

    def radius_at(self, z: np.ndarray) -> np.ndarray:
        """Envelope radius at height(s) z along the tool axis (no inflation).

        Returns 0 outside [0, total_height] - callers must also check z range.
        """
        z = np.asarray(z, dtype=float)
        tan_a = np.tan(np.radians(self.cone_half_angle_deg))
        r = np.zeros_like(z)
        in_cone = (z >= 0) & (z <= self.cone_height)
        r[in_cone] = z[in_cone] * tan_a
        in_neck = (z > self.cone_height) & (z < self.cylinder_start)
        r[in_neck] = self.cone_top_radius
        in_cyl = (z >= self.cylinder_start) & (z <= self.cylinder_start + self.cylinder_height)
        # Use the larger of the two where they overlap (cylinder_start < cone_height).
        r[in_cyl] = np.maximum(r[in_cyl], self.cylinder_radius)
        return r

    def contains_local(self, pts_local: np.ndarray, inflate: float = 0.0) -> np.ndarray:
        """Boolean mask: which TOOL-frame points are inside the (inflated) envelope.

        ``inflate`` grows the radius (and the top) by that many mm. The bottom is
        NOT grown below ``tip_ignore_height`` - see the module docstring.
        """
        pts = np.atleast_2d(pts_local)
        z = pts[:, 2]
        r = np.hypot(pts[:, 0], pts[:, 1])
        grow = inflate + self.clearance
        in_z = (z >= self.tip_ignore_height) & (z <= self.total_height + grow)
        # Clamp z so points slightly above the top are tested against the top radius.
        z_c = np.clip(z, 0.0, self.total_height)
        return in_z & (r <= self.radius_at(z_c) + grow)

    def to_mesh(self, tip: np.ndarray, direction: np.ndarray, sections: int = 32) -> trimesh.Trimesh:
        """Triangle mesh of the hotend placed at ``tip`` with axis ``direction``.

        For drawing only - the collision test does not use this mesh.
        """
        # Cone: trimesh builds it with the base at z=0 and apex at z=h. Flip it
        # so the apex sits at the tip and it opens upward.
        cone = trimesh.creation.cone(radius=self.cone_top_radius, height=self.cone_height, sections=sections)
        flip = homogeneous(np.diag([1.0, -1.0, -1.0]), np.array([0.0, 0.0, self.cone_height]))
        cone.apply_transform(flip)
        parts = [cone]
        neck = self.cylinder_start - self.cone_height
        if neck > 1e-6:
            n = trimesh.creation.cylinder(radius=self.cone_top_radius, height=neck, sections=sections)
            n.apply_translation([0, 0, self.cone_height + neck / 2])
            parts.append(n)
        cyl = trimesh.creation.cylinder(radius=self.cylinder_radius, height=self.cylinder_height, sections=sections)
        cyl.apply_translation([0, 0, self.cylinder_start + self.cylinder_height / 2])
        parts.append(cyl)
        mesh = trimesh.util.concatenate(parts)
        mesh.apply_transform(homogeneous(rotation_from_z(direction), np.asarray(tip, dtype=float)))
        return mesh


# --------------------------------------------------------------------------- #
# Obstacles
# --------------------------------------------------------------------------- #
def sample_surface_grid(mesh: trimesh.Trimesh, spacing: float) -> np.ndarray:
    """Deterministic points spread over a mesh surface, at most ~spacing apart.

    Each triangle is subdivided into an n x n barycentric grid, where n is
    chosen from the triangle's longest edge. No randomness, so tests are
    repeatable, and no extra dependencies.
    """
    tris = mesh.triangles  # (F, 3, 3)
    edges = np.stack(
        [tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 1], tris[:, 0] - tris[:, 2]], axis=1
    )
    longest = np.linalg.norm(edges, axis=2).max(axis=1)
    n_per_tri = np.maximum(1, np.ceil(longest / spacing).astype(int))

    out = [mesh.vertices]
    # Group triangles by subdivision count so each group is one vectorised op.
    for n in np.unique(n_per_tri):
        group = tris[n_per_tri == n]
        # Barycentric grid (i, j) with i + j <= n.
        ij = np.array([(i, j) for i in range(n + 1) for j in range(n + 1 - i)], dtype=float) / n
        w0 = 1.0 - ij[:, 0] - ij[:, 1]
        # points[f, k] = w0*v0 + i*v1 + j*v2
        pts = (
            w0[None, :, None] * group[:, None, 0]
            + ij[None, :, 0, None] * group[:, None, 1]
            + ij[None, :, 1, None] * group[:, None, 2]
        )
        out.append(pts.reshape(-1, 3))
    return np.unique(np.round(np.vstack(out), 6), axis=0)


@dataclass
class Obstacle:
    """One obstacle: a name, its surface samples, and its bounding box."""

    name: str
    points: np.ndarray  # (N, 3)
    bbox_min: np.ndarray
    bbox_max: np.ndarray


@dataclass
class ObstacleSet:
    """A group of obstacle meshes pre-sampled for fast collision checks."""

    spacing: float
    obstacles: list[Obstacle] = field(default_factory=list)

    @classmethod
    def from_meshes(
        cls, meshes: list[trimesh.Trimesh], spacing: float, names: list[str] | None = None
    ) -> "ObstacleSet":
        names = names or [f"obstacle_{i}" for i in range(len(meshes))]
        s = cls(spacing=spacing)
        for m, n in zip(meshes, names):
            s.add(m, n)
        return s

    def add(self, mesh: trimesh.Trimesh, name: str) -> None:
        pts = sample_surface_grid(mesh, self.spacing)
        self.obstacles.append(Obstacle(name, pts, pts.min(axis=0), pts.max(axis=0)))

    @property
    def inflate(self) -> float:
        """How much to grow the hotend so samples can't miss a surface.

        The farthest any surface point can be from its nearest sample is a bit
        under one spacing for our grid; 0.6 x spacing is a good compromise for
        roughly regular grids.
        """
        return 0.6 * self.spacing


# --------------------------------------------------------------------------- #
# The actual check
# --------------------------------------------------------------------------- #
@dataclass
class CollisionResult:
    hit: bool
    obstacle_name: str | None = None
    num_points: int = 0  # how many sample points were inside the hotend
    example_point: np.ndarray | None = None  # one offending point (part frame)


def check_collision(
    model: HotendModel,
    tip: np.ndarray,
    direction: np.ndarray,
    obstacles: ObstacleSet,
) -> CollisionResult:
    """Is the hotend, tip at ``tip`` and axis along ``direction``, hitting anything?

    ``tip`` and ``direction`` are in the same frame as the obstacle meshes (the
    part frame). ``direction`` is the build direction: the tool axis points
    along it, away from the part. Returns the FIRST obstacle hit.
    """
    tip = np.asarray(tip, dtype=float)
    d = normalize(direction)
    R = rotation_to_z(d)  # part frame -> tool frame (rotation part)
    grow = obstacles.inflate + model.clearance

    # Bounding sphere of the inflated hotend, for quick rejection.
    half_h = model.total_height / 2
    center = tip + d * half_h
    radius = np.hypot(half_h, model.max_radius) + grow

    for obs in obstacles.obstacles:
        # 1) Sphere vs. obstacle box: skip obstacles that are far away.
        closest = np.clip(center, obs.bbox_min, obs.bbox_max)
        if np.linalg.norm(closest - center) > radius:
            continue
        # 2) Keep only points inside the bounding sphere.
        near = obs.points[np.sum((obs.points - center) ** 2, axis=1) <= radius**2]
        if len(near) == 0:
            continue
        # 3) Exact test against the inflated envelope in the tool frame.
        local = (near - tip) @ R.T
        inside = model.contains_local(local, inflate=obstacles.inflate)
        if np.any(inside):
            return CollisionResult(True, obs.name, int(inside.sum()), near[np.argmax(inside)])
    return CollisionResult(False)


def check_many(
    model: HotendModel,
    tips: np.ndarray,
    direction: np.ndarray,
    obstacles: ObstacleSet,
    stop_at_first: bool = True,
) -> list[tuple[int, CollisionResult]]:
    """Check a list of tip points (same tool direction). Returns (index, result) hits."""
    hits = []
    for i, tip in enumerate(np.atleast_2d(tips)):
        res = check_collision(model, tip, direction, obstacles)
        if res.hit:
            hits.append((i, res))
            if stop_at_first:
                break
    return hits


# --------------------------------------------------------------------------- #
# Bed geometry
# --------------------------------------------------------------------------- #
def table_mesh(table: TableConfig) -> trimesh.Trimesh:
    """Mesh of the rotary table, top surface at z = table.surface_z (part frame)."""
    if table.shape == "round":
        m = trimesh.creation.cylinder(radius=table.diameter / 2, height=table.thickness, sections=96)
    else:
        m = trimesh.creation.box(extents=[table.size_x, table.size_y, table.thickness])
    m.apply_translation([0, 0, table.surface_z - table.thickness / 2])
    return m


def table_mesh_for_machine(machine: MachineConfig) -> trimesh.Trimesh:
    """Rotary table centred on the C axis (pivot X, Y)."""
    m = table_mesh(machine.table)
    p = machine.kinematics.pivot
    m.apply_translation([p[0], p[1], 0.0])
    return m
