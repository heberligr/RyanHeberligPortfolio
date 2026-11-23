"""Polyscope 3D viewer with an ImGui side panel.

Milestone 1 features:
  * load an STL (path box or command line) and place it on the rotary table
  * show the table, the pivot point, and the machine axes
  * "Hotend / IK test": pick a build direction with tilt/azimuth sliders and a
    tip point; see the hotend model, the computed A/C angles, and whether the
    hotend collides with the table or the part
  * "Machine view": show everything rotated by A/C, the way the real machine
    would hold the part. The build direction should point straight up.

How Polyscope works, in one paragraph: you "register" meshes/points/curves
with names, then ``ps.show()`` runs the window loop. Every frame it calls our
``_ui_callback``; inside it we draw ImGui widgets (each widget call returns
whether the user changed it) and re-register any geometry that changed.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import polyscope as ps
import polyscope.imgui as psim
import trimesh

from .collision import HotendModel, ObstacleSet, check_collision, table_mesh_for_machine
from .config import MachineConfig
from .kinematics import IKError, RotaryPose, inverse_kinematics, part_to_machine, to_commanded
from .mesh_io import load_mesh, mesh_info, place_on_bed
from .transforms import direction_from_angles

# Colours (RGB 0..1)
COLOR_PART = (0.75, 0.75, 0.8)
COLOR_TABLE = (0.35, 0.35, 0.38)
COLOR_TOOL_OK = (0.2, 0.75, 0.3)
COLOR_TOOL_HIT = (0.9, 0.15, 0.15)
COLOR_PIVOT = (1.0, 0.6, 0.0)


class SlicerViewer:
    """Owns all app state for the viewer and draws the UI each frame."""

    def __init__(self, machine: MachineConfig, model_path: str | None = None) -> None:
        self.machine = machine
        self.hotend = HotendModel.from_config(machine.hotend)
        self.table = table_mesh_for_machine(machine)

        # Loaded model state
        self.path_text: str = model_path or ""
        self.part: trimesh.Trimesh | None = None
        self.info_lines: list[str] = ["No model loaded."]
        self.auto_place = True
        self.message = ""

        # Hotend / IK test state
        self.show_tool = True
        self.tilt_deg = 0.0
        self.azimuth_deg = 0.0
        self.tip = np.array([0.0, 0.0, machine.table.surface_z + 10.0])
        self.machine_view = False
        self._obstacles: ObstacleSet | None = None
        self.status_lines: list[str] = []

    # ------------------------------------------------------------------ #
    # Model loading
    # ------------------------------------------------------------------ #
    def load(self, path: str) -> None:
        """Load the STL at ``path``; on failure show the error in the panel."""
        try:
            mesh = load_mesh(path)
        except Exception as exc:  # show any loading problem in the UI
            self.message = f"Load failed: {exc}"
            return
        if self.auto_place:
            mesh = place_on_bed(mesh, self.machine)
        self.part = mesh
        self.info_lines = mesh_info(mesh, path).lines()
        self.message = f"Loaded {Path(path).name}"
        # Default tip: top centre of the part.
        lo, hi = mesh.bounds
        self.tip = np.array([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, hi[2]])
        self._obstacles = None  # must be rebuilt for the new part
        self.refresh_scene()

    def _get_obstacles(self) -> ObstacleSet:
        """Build (once per model) the sampled table + part for collision tests."""
        if self._obstacles is None:
            meshes, names = [self.table], ["table"]
            if self.part is not None:
                meshes.append(self.part)
                names.append("part")
            self._obstacles = ObstacleSet.from_meshes(
                meshes, self.machine.collision.sample_spacing, names
            )
        return self._obstacles

    # ------------------------------------------------------------------ #
    # Scene
    # ------------------------------------------------------------------ #
    def refresh_scene(self) -> None:
        """Re-register all geometry based on current state."""
        direction = direction_from_angles(self.tilt_deg, self.azimuth_deg)
        pivot = self.machine.kinematics.pivot

        # Work out the pose (needed for the machine view and the status text).
        pose: RotaryPose | None
        try:
            pose = inverse_kinematics(direction, self.machine)
        except IKError as exc:
            pose = None
            ik_text = f"IK: unreachable - {exc}"
        else:
            a_cmd, c_cmd = to_commanded(pose, self.machine)
            k = self.machine.kinematics
            ik_text = f"IK: {k.a_axis.letter}={a_cmd:.2f}  {k.c_axis.letter}={c_cmd:.2f}  (commanded)"

        def place(vertices: np.ndarray) -> np.ndarray:
            """Part frame -> what we draw (machine frame if machine view is on)."""
            if self.machine_view and pose is not None:
                return part_to_machine(vertices, pose, pivot)
            return vertices

        ps.register_surface_mesh("table", place(self.table.vertices), self.table.faces, color=COLOR_TABLE)
        ps.register_point_cloud("pivot", pivot[None, :], color=COLOR_PIVOT, radius=0.006)

        if self.part is not None:
            ps.register_surface_mesh("part", place(self.part.vertices), self.part.faces, color=COLOR_PART)
        elif ps.has_surface_mesh("part"):
            ps.remove_surface_mesh("part")

        lines = [ik_text]
        if self.show_tool:
            col = check_collision(self.hotend, self.tip, direction, self._get_obstacles())
            tool = self.hotend.to_mesh(self.tip, direction)
            ps.register_surface_mesh(
                "hotend",
                place(tool.vertices),
                tool.faces,
                color=COLOR_TOOL_HIT if col.hit else COLOR_TOOL_OK,
                transparency=0.5,
            )
            if col.hit:
                lines.append(f"COLLISION with '{col.obstacle_name}' ({col.num_points} sample points)")
            else:
                lines.append("No collision")
            if self.machine_view and pose is not None:
                # Sanity check shown to the user: where does the tip end up?
                tip_m = part_to_machine(self.tip, pose, pivot)
                lines.append(f"Tip in machine XYZ: {np.round(tip_m, 2).tolist()}")
        elif ps.has_surface_mesh("hotend"):
            ps.remove_surface_mesh("hotend")
        self.status_lines = lines

    # ------------------------------------------------------------------ #
    # UI
    # ------------------------------------------------------------------ #
    def _ui_callback(self) -> None:
        changed = False
        psim.PushItemWidth(220)

        # ---- Model ----
        if psim.CollapsingHeader("Model", psim.ImGuiTreeNodeFlags_DefaultOpen):
            _, self.path_text = psim.InputText("STL path", self.path_text)
            _, self.auto_place = psim.Checkbox("Place on table when loading", self.auto_place)
            if psim.Button("Load"):
                self.load(self.path_text.strip().strip('"'))
            for line in self.info_lines:
                psim.TextUnformatted(line)
            if self.message:
                psim.TextUnformatted(self.message)

        # ---- Hotend / IK test ----
        if psim.CollapsingHeader("Hotend / IK test", psim.ImGuiTreeNodeFlags_DefaultOpen):
            c, self.show_tool = psim.Checkbox("Show hotend", self.show_tool)
            changed |= c
            c, self.tilt_deg = psim.SliderFloat("Tilt from +Z (deg)", self.tilt_deg, 0.0, 120.0)
            changed |= c
            c, self.azimuth_deg = psim.SliderFloat("Azimuth (deg)", self.azimuth_deg, -180.0, 180.0)
            changed |= c
            c, tip = psim.InputFloat3("Tip point (part)", tuple(self.tip))
            if c:
                self.tip = np.array(tip, dtype=float)
                changed = True
            c, self.machine_view = psim.Checkbox("Machine view (apply A/C)", self.machine_view)
            changed |= c
            for line in self.status_lines:
                if line.startswith("COLLISION") or "unreachable" in line:
                    psim.TextColored((1.0, 0.3, 0.3, 1.0), line)
                else:
                    psim.TextUnformatted(line)

        psim.PopItemWidth()
        if changed:
            self.refresh_scene()

    # ------------------------------------------------------------------ #
    # Startup
    # ------------------------------------------------------------------ #
    def setup(self, backend: str | None = None) -> None:
        """Initialise Polyscope and the scene (separate from show() for tests)."""
        if backend:
            ps.set_allow_headless_backends(True)
            ps.init(backend)
        else:
            ps.init()
        ps.set_program_name("segslicer - 5-axis segmented slicer")
        ps.set_up_dir("z_up")
        ps.set_front_dir("neg_y_front")
        ps.set_ground_plane_mode("none")  # we draw our own table
        ps.set_user_callback(self._ui_callback)
        if self.path_text:
            self.load(self.path_text)
        else:
            self.refresh_scene()

    def run(self) -> None:
        self.setup()
        ps.show()
