"""Run the viewer with Polyscope's fake (mock) OpenGL backend - no window."""

import numpy as np
import pytest
import trimesh

ps = pytest.importorskip("polyscope")


@pytest.fixture
def viewer(machine):
    from segslicer.viewer import SlicerViewer

    v = SlicerViewer(machine)
    try:
        v.setup("openGL_mock")
    except Exception as exc:  # pragma: no cover - backend not available
        pytest.skip(f"Polyscope mock backend unavailable: {exc}")
    return v


def test_load_stl_and_ik_view(viewer, tmp_path):
    path = tmp_path / "cube.stl"
    trimesh.creation.box(extents=[20, 20, 20]).export(path)
    viewer.load(str(path))
    assert viewer.part is not None
    viewer.show_tool = True
    viewer.tilt_deg, viewer.azimuth_deg, viewer.machine_view = 45.0, 30.0, True
    viewer.refresh_scene()
    assert any(line.startswith("IK:") for line in viewer.status_lines)
    ps.frame_tick()


def test_demo_cut_edit_save_load(viewer, tmp_path):
    viewer.load_demo("Cube with arm")
    viewer.add_cut()
    cut = viewer.project.cuts[0]
    cut.point = np.array([10.0, 0, 14])
    viewer.cuts_changed()
    assert len(viewer.seg.segments) == 2 and viewer.seg.valid
    ps.frame_tick()  # draws all panels with a cut selected

    path = tmp_path / "p.json"
    viewer.save_project_file(str(path))
    viewer.load_demo("Cube")
    assert len(viewer.seg.segments) == 1
    viewer.load_project_file(str(path))
    assert len(viewer.project.cuts) == 1 and len(viewer.seg.segments) == 2

    viewer.delete_cut(0)
    assert len(viewer.seg.segments) == 1
    ps.frame_tick()
