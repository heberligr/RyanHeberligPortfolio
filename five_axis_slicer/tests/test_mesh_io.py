import numpy as np
import pytest
import trimesh

from segslicer.mesh_io import load_mesh, mesh_info, place_on_bed


def test_load_and_place(tmp_path, machine):
    m = trimesh.creation.box(extents=[10, 20, 30])
    m.apply_translation([100, -50, 7])
    path = tmp_path / "box.stl"
    m.export(path)

    loaded = load_mesh(path)
    assert loaded.is_watertight
    placed = place_on_bed(loaded, machine)
    lo, hi = placed.bounds
    assert lo[2] == pytest.approx(machine.table.surface_z)
    pivot = machine.kinematics.pivot
    np.testing.assert_allclose((lo[:2] + hi[:2]) / 2, pivot[:2], atol=1e-9)
    info = mesh_info(placed, path)
    assert info.triangles == 12
    np.testing.assert_allclose(info.extents, [10, 20, 30])


def test_missing_file():
    with pytest.raises(FileNotFoundError):
        load_mesh("does_not_exist.stl")
