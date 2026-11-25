import json

import numpy as np
import trimesh

from segslicer.project import Project, load_project, save_project
from segslicer.segments import Cut


def test_round_trip_stl(tmp_path, machine):
    stl = tmp_path / "models" / "box.stl"
    stl.parent.mkdir()
    trimesh.creation.box(extents=[10, 10, 10]).export(stl)
    proj = Project(
        stl_path=str(stl),
        overhang_angle_deg=45.0,
        cuts=[Cut([1, 2, 3], [0, 0, 2], local=True, components=[1], name="arm")],
    )
    path = tmp_path / "proj" / "p.json"
    save_project(proj, path)
    # Path is stored relative to the project file.
    assert json.loads(path.read_text())["model"]["stl"] == "../models/box.stl"

    back = load_project(path)
    assert back.stl_path == str(stl.resolve())
    assert back.overhang_angle_deg == 45.0
    c = back.cuts[0]
    np.testing.assert_allclose(c.point, [1, 2, 3])
    np.testing.assert_allclose(c.normal, [0, 0, 1])  # normalised
    assert c.local and c.components == [1] and c.name == "arm"
    assert back.load_part(machine).bounds[0][2] == machine.table.surface_z


def test_round_trip_demo(tmp_path, machine):
    path = tmp_path / "demo.json"
    save_project(Project(demo_name="T-shape"), path)
    back = load_project(path)
    assert back.demo_name == "T-shape" and back.stl_path is None
    assert back.load_part(machine).is_watertight
