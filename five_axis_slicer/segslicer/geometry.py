"""Mesh operations: plane cuts, connected components, unions, point queries.

The cutting itself is done by the ``manifold3d`` library. It works on
"Manifold" objects (always watertight solids), so cut results are capped
automatically. We convert to/from trimesh at the edges of this module, so the
rest of the code only ever sees ``trimesh.Trimesh``.
"""

from __future__ import annotations

import manifold3d as m3d
import numpy as np
import trimesh

from .transforms import normalize


# --------------------------------------------------------------------------- #
# trimesh <-> manifold
# --------------------------------------------------------------------------- #
def to_manifold(mesh: trimesh.Trimesh) -> m3d.Manifold:
    """Convert a trimesh to a manifold3d solid. Raises if it isn't watertight."""
    m = m3d.Manifold(
        m3d.Mesh64(
            vert_properties=np.ascontiguousarray(mesh.vertices, dtype=np.float64),
            tri_verts=np.ascontiguousarray(mesh.faces, dtype=np.uint64),
        )
    )
    if m.status() != m3d.Error.NoError:
        raise ValueError(
            f"Mesh is not a closed solid (manifold3d: {m.status()}). "
            "Repair it (e.g. in PrusaSlicer or Meshmixer) before cutting."
        )
    return m


def to_trimesh(m: m3d.Manifold) -> trimesh.Trimesh:
    """Convert a manifold3d solid back to a trimesh."""
    mesh = m.to_mesh64()
    verts = np.asarray(mesh.vert_properties, dtype=float)[:, :3]
    faces = np.asarray(mesh.tri_verts, dtype=np.int64)
    return trimesh.Trimesh(verts, faces, process=True)


# --------------------------------------------------------------------------- #
# Cutting
# --------------------------------------------------------------------------- #
def split_by_plane(
    mesh: trimesh.Trimesh, point: np.ndarray, normal: np.ndarray
) -> tuple[list[trimesh.Trimesh], trimesh.Trimesh | None]:
    """Cut a solid with a plane.

    Returns ``(upper_components, lower)``:
      * upper_components: list of separate solids on the side the normal
        points to (each capped and watertight), sorted in a stable order
      * lower: everything on the other side as one solid (None if empty)
    """
    n = normalize(normal)
    offset = float(np.dot(n, point))  # plane is {x : n.x = offset}
    upper, lower = to_manifold(mesh).split_by_plane(n.tolist(), offset)
    uppers = [to_trimesh(c) for c in upper.decompose() if not c.is_empty()]
    return sort_components(uppers), (None if lower.is_empty() else to_trimesh(lower))


def sort_components(meshes: list[trimesh.Trimesh]) -> list[trimesh.Trimesh]:
    """Sort meshes by centroid (x, then y, then z) so indices are repeatable."""
    return sorted(meshes, key=lambda m: tuple(np.round(m.centroid, 3)))


def union(meshes: list[trimesh.Trimesh]) -> trimesh.Trimesh:
    """Boolean union of solids (they may touch along a shared cut face)."""
    if len(meshes) == 1:
        return meshes[0]
    result = m3d.Manifold.batch_boolean([to_manifold(m) for m in meshes], m3d.OpType.Add)
    return to_trimesh(result)


def components(mesh: trimesh.Trimesh) -> list[trimesh.Trimesh]:
    """Split a solid into its disconnected pieces (stable order)."""
    return sort_components([to_trimesh(c) for c in to_manifold(mesh).decompose()])


# --------------------------------------------------------------------------- #
# Point queries (pure numpy, no extra dependencies)
# --------------------------------------------------------------------------- #
def winding_number(mesh: trimesh.Trimesh, point: np.ndarray) -> float:
    """Generalised winding number of a closed mesh around ``point``.

    About 1 if the point is inside, 0 if outside, 0.5 on the surface. Computed
    by adding up the solid angle of every triangle as seen from the point
    (Van Oosterom & Strackee formula), divided by 4*pi.
    """
    a = mesh.triangles[:, 0] - point
    b = mesh.triangles[:, 1] - point
    c = mesh.triangles[:, 2] - point
    la, lb, lc = (np.linalg.norm(v, axis=1) for v in (a, b, c))
    det = np.einsum("ij,ij->i", a, np.cross(b, c))
    denom = (
        la * lb * lc
        + np.einsum("ij,ij->i", a, b) * lc
        + np.einsum("ij,ij->i", b, c) * la
        + np.einsum("ij,ij->i", c, a) * lb
    )
    return float(np.sum(2.0 * np.arctan2(det, denom)) / (4.0 * np.pi))


def contains(mesh: trimesh.Trimesh, point: np.ndarray) -> bool:
    """True if ``point`` is inside the closed mesh."""
    return winding_number(mesh, np.asarray(point, dtype=float)) > 0.5


def distance_to_surface(mesh: trimesh.Trimesh, point: np.ndarray) -> float:
    """Shortest distance from ``point`` to the mesh surface (brute force, O(faces))."""
    p = np.asarray(point, dtype=float)
    tris = mesh.triangles
    closest = trimesh.triangles.closest_point(tris, np.repeat(p[None], len(tris), axis=0))
    return float(np.min(np.linalg.norm(closest - p, axis=1)))


def faces_on_plane(
    mesh: trimesh.Trimesh, point: np.ndarray, normal: np.ndarray, tol: float = 1e-3, facing: float = -1.0
) -> np.ndarray:
    """Mask of faces lying in a plane and facing a given way.

    ``facing = -1`` finds faces whose normal is opposite the plane normal (the
    bottom cap of a segment sitting on that plane); ``+1`` finds the top cap.
    """
    n = normalize(normal)
    offset = float(np.dot(n, point))
    dist = np.abs(mesh.triangles @ n - offset)  # (F, 3) distances of each vertex
    in_plane = np.all(dist < tol, axis=1)
    aligned = (mesh.face_normals @ n) * facing > 0.99
    return in_plane & aligned
