"""Loading STL files and placing them on the bed."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import trimesh

from .config import MachineConfig


@dataclass
class MeshInfo:
    """Summary shown in the UI after loading a model."""

    path: str
    triangles: int
    extents: np.ndarray  # size in x, y, z (mm)
    watertight: bool
    volume: float

    def lines(self) -> list[str]:
        e = self.extents
        out = [
            f"File: {Path(self.path).name}",
            f"Triangles: {self.triangles}",
            f"Size: {e[0]:.1f} x {e[1]:.1f} x {e[2]:.1f} mm",
            f"Volume: {self.volume / 1000:.2f} cm^3" if self.watertight else "Volume: n/a",
        ]
        if not self.watertight:
            out.append("WARNING: mesh is not watertight - cuts/slicing may fail.")
        return out


def load_mesh(path: str | Path) -> trimesh.Trimesh:
    """Load an STL (or any format trimesh knows) as a single triangle mesh.

    Units are assumed to be millimetres, as is standard for STL.
    Multi-body files are merged into one mesh.
    """
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f"No such file: {path}")
    mesh = trimesh.load(str(path), force="mesh")
    if not isinstance(mesh, trimesh.Trimesh) or len(mesh.faces) == 0:
        raise ValueError(f"{path} does not contain any triangles")
    # Merge duplicate vertices and fix face winding where possible. This is
    # cheap and makes STL files (which store every triangle separately) usable.
    mesh.merge_vertices()
    trimesh.repair.fix_normals(mesh)
    return mesh


def mesh_info(mesh: trimesh.Trimesh, path: str | Path = "") -> MeshInfo:
    return MeshInfo(
        path=str(path),
        triangles=len(mesh.faces),
        extents=mesh.extents.copy(),
        watertight=bool(mesh.is_watertight),
        volume=float(mesh.volume) if mesh.is_watertight else 0.0,
    )


def place_on_bed(mesh: trimesh.Trimesh, machine: MachineConfig) -> trimesh.Trimesh:
    """Return a copy moved so it sits on the table, centred on the C axis.

    The lowest point goes to table.surface_z and the XY centre of the bounding
    box goes to the pivot's XY (where the C axis is). Keeping the part near the
    rotary axes keeps tilted moves inside the XYZ travel limits.
    """
    out = mesh.copy()
    lo, hi = out.bounds
    pivot = machine.kinematics.pivot
    shift = np.array(
        [
            pivot[0] - (lo[0] + hi[0]) / 2,
            pivot[1] - (lo[1] + hi[1]) / 2,
            machine.table.surface_z - lo[2],
        ]
    )
    out.apply_translation(shift)
    return out
