"""Run the viewer with Polyscope's fake (mock) OpenGL backend - no window."""

import numpy as np
import pytest
import trimesh

ps = pytest.importorskip("polyscope")


def test_viewer_setup_and_frame(tmp_path, machine):
    from segslicer.viewer import SlicerViewer

    path = tmp_path / "cube.stl"
    trimesh.creation.box(extents=[20, 20, 20]).export(path)
    v = SlicerViewer(machine, str(path))
    try:
        v.setup("openGL_mock")
    except Exception as exc:  # pragma: no cover - backend not available
        pytest.skip(f"Polyscope mock backend unavailable: {exc}")
    assert v.part is not None
    v.tilt_deg, v.azimuth_deg, v.machine_view = 45.0, 30.0, True
    v.refresh_scene()
    assert any(line.startswith("IK:") for line in v.status_lines)
    ps.frame_tick()
