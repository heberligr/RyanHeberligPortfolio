"""Polyscope 3D viewer with an ImGui side panel.

Panels:
  * Model     - load an STL or a built-in demo part
  * Project   - save / load the project (model + cuts) as JSON
  * Cuts      - add / delete / reorder cut planes and edit the selected one
  * Segments  - result: segments in print order, warnings
  * Hotend / IK test - the milestone 1 tool for checking IK and collisions

How Polyscope works, in one paragraph: you "register" meshes/points/curves
with names, then ``ps.show()`` runs the window loop. Every frame it calls our
``_ui_callback``; inside it we draw ImGui widgets (each widget call returns
whether the user changed it) and re-register any geometry that changed.
Clicking on a mesh in the 3D view "selects" it; we read that selection to let
you snap cut planes to faces.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import polyscope as ps
import polyscope.imgui as psim
import trimesh

from .collision import HotendModel, ObstacleSet, check_collision, table_mesh_for_machine
from .config import MachineConfig, Settings
from .demo_models import DEMO_MODELS
from .kinematics import IKError, RotaryPose, inverse_kinematics, part_to_machine, to_commanded
from .mesh_io import load_mesh, mesh_info, place_on_bed
from .overhang import overhang_area, overhang_mask
from .project import Project, load_project, save_project
from .segments import Cut, Segmentation, apply_cuts
from .transforms import direction_from_angles, normalize, rotation_from_z

# Colours (RGB 0..1)
COLOR_TABLE = (0.35, 0.35, 0.38)
COLOR_TOOL_OK = (0.2, 0.75, 0.3)
COLOR_TOOL_HIT = (0.9, 0.15, 0.15)
COLOR_PIVOT = (1.0, 0.6, 0.0)
COLOR_OVERHANG = (0.95, 0.1, 0.1)
COLOR_PLANE_OK = (0.2, 0.5, 1.0)
COLOR_PLANE_BAD = (1.0, 0.55, 0.0)
COLOR_PICK = (1.0, 1.0, 0.0)
RED_TEXT = (1.0, 0.35, 0.35, 1.0)
# One colour per segment (cycles if there are more segments than colours).
SEGMENT_COLORS = [
    (0.75, 0.75, 0.80),  # root: light grey
    (0.30, 0.65, 0.95),
    (0.40, 0.80, 0.40),
    (0.95, 0.75, 0.25),
    (0.75, 0.45, 0.90),
    (0.30, 0.85, 0.80),
    (0.95, 0.50, 0.65),
    (0.60, 0.55, 0.35),
]


def segment_color(i: int) -> tuple[float, float, float]:
    return SEGMENT_COLORS[i % len(SEGMENT_COLORS)]


def angles_of(direction: np.ndarray) -> tuple[float, float]:
    """(tilt from +Z, azimuth) in degrees - inverse of direction_from_angles."""
    d = normalize(direction)
    tilt = float(np.degrees(np.arccos(np.clip(d[2], -1, 1))))
    az = float(np.degrees(np.arctan2(d[1], d[0])))
    return tilt, az


@dataclass
class PickedFace:
    """A face the user clicked on in the 3D view."""

    segment_id: int
    face_index: int
    position: np.ndarray
    normal: np.ndarray
    is_overhang: bool


class SlicerViewer:
    """Owns all app state for the viewer and draws the UI each frame."""

    def __init__(
        self, machine: MachineConfig, settings: Settings | None = None, model_path: str | None = None
    ) -> None:
        self.machine = machine
        self.hotend = HotendModel.from_config(machine.hotend)
        self.table = table_mesh_for_machine(machine)

        # ---- model / project ----
        self.project = Project(overhang_angle_deg=settings.overhang_angle_deg if settings else 50.0)
        self.part: trimesh.Trimesh | None = None
        self.path_text: str = model_path or ""
        self.project_path_text: str = "project.json"
        self.demo_index = 0
        self.info_lines: list[str] = ["No model loaded."]
        self.message = ""

        # ---- segmentation ----
        self.seg: Segmentation | None = None
        self.selected_cut = -1
        self.live_update = True
        self.dirty = False            # cuts changed but not applied yet
        self.show_overhangs = True
        self.picked: PickedFace | None = None
        self._last_pick_key: tuple | None = None
        self._segment_names: set[str] = set()
        # Slider state for the selected cut's normal (kept separately so the
        # azimuth isn't lost while tilt is 0).
        self.edit_tilt = 90.0
        self.edit_az = 0.0

        # ---- hotend / IK test (milestone 1) ----
        self.show_tool = False
        self.tilt_deg = 0.0
        self.azimuth_deg = 0.0
        self.tip = np.array([0.0, 0.0, machine.table.surface_z + 10.0])
        self.machine_view = False
        self._obstacles: ObstacleSet | None = None
        self.tool_segment: int | None = None  # segment being "printed" in the test
        self.status_lines: list[str] = []

    # ================================================================== #
    # Loading
    # ================================================================== #
    def load(self, path: str) -> None:
        """Load an STL file as the part (clears the cuts)."""
        try:
            mesh = load_mesh(path)
        except Exception as exc:  # show any loading problem in the UI
            self.message = f"Load failed: {exc}"
            return
        self.project.stl_path, self.project.demo_name = str(Path(path).resolve()), None
        self.project.cuts = []
        self._set_part(mesh, path)

    def load_demo(self, name: str) -> None:
        """Load one of the built-in demo parts (clears the cuts)."""
        self.project.stl_path, self.project.demo_name = None, name
        self.project.cuts = []
        self._set_part(DEMO_MODELS[name](), f"demo: {name}")

    def load_project_file(self, path: str) -> None:
        try:
            proj = load_project(path)
            proj_part = proj.load_part(self.machine)
        except Exception as exc:
            self.message = f"Project load failed: {exc}"
            return
        self.project = proj
        self._set_part(proj_part, proj.stl_path or f"demo: {proj.demo_name}", already_placed=True)
        self.message = f"Loaded project {Path(path).name} ({len(proj.cuts)} cuts)"

    def save_project_file(self, path: str) -> None:
        if self.part is None:
            self.message = "Nothing to save - load a model first."
            return
        try:
            save_project(self.project, path)
            self.message = f"Saved {Path(path).resolve()}"
        except Exception as exc:
            self.message = f"Save failed: {exc}"

    def _set_part(self, mesh: trimesh.Trimesh, label: str, already_placed: bool = False) -> None:
        if self.project.place_on_bed and not already_placed:
            mesh = place_on_bed(mesh, self.machine)
        self.part = mesh
        self.info_lines = mesh_info(mesh, label).lines()
        self.message = f"Loaded {Path(label).name}"
        lo, hi = mesh.bounds
        self.tip = np.array([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, hi[2]])
        self.selected_cut = len(self.project.cuts) - 1
        self.picked = None
        self._sync_angles_from_cut()
        self.recompute()

    # ================================================================== #
    # Segmentation
    # ================================================================== #
    def recompute(self) -> None:
        """Re-apply all cuts and redraw."""
        if self.part is not None:
            self.seg = apply_cuts(self.part, self.project.cuts, self.machine)
        else:
            self.seg = None
        self.dirty = False
        self._obstacles = None
        self.refresh_scene()

    def cuts_changed(self) -> None:
        """Call after editing cuts: re-apply now (live) or later (Apply button)."""
        if self.live_update:
            self.recompute()
        else:
            self.dirty = True
            self.refresh_scene()  # still move the plane preview

    def add_cut(self) -> None:
        if self.part is None:
            return
        center = self.part.bounds.mean(axis=0)
        self.project.cuts.append(Cut(point=center, normal=[1.0, 0.0, 0.0]))
        self.selected_cut = len(self.project.cuts) - 1
        self._sync_angles_from_cut()
        self.cuts_changed()

    def delete_cut(self, i: int) -> None:
        del self.project.cuts[i]
        self.selected_cut = min(self.selected_cut, len(self.project.cuts) - 1)
        self._sync_angles_from_cut()
        self.cuts_changed()

    def move_cut(self, i: int, step: int) -> None:
        j = i + step
        cuts = self.project.cuts
        if 0 <= j < len(cuts):
            cuts[i], cuts[j] = cuts[j], cuts[i]
            self.selected_cut = j
            self.cuts_changed()

    def _current_cut(self) -> Cut | None:
        if 0 <= self.selected_cut < len(self.project.cuts):
            return self.project.cuts[self.selected_cut]
        return None

    def _sync_angles_from_cut(self) -> None:
        cut = self._current_cut()
        if cut is not None:
            tilt, az = angles_of(cut.normal)
            self.edit_tilt = tilt
            if tilt > 1e-6:  # azimuth is meaningless for a vertical normal
                self.edit_az = az

    # ================================================================== #
    # Picking faces in the 3D view
    # ================================================================== #
    def _poll_pick(self) -> None:
        """Turn a polyscope click on a segment face into self.picked."""
        if self.seg is None or not ps.have_selection():
            return
        sel = ps.get_selection()
        if not sel.is_hit or not sel.structure_name.startswith("segment_"):
            return
        if sel.structure_data.get("element_type") != "face":
            return
        key = (sel.structure_name, sel.structure_data["index"], tuple(sel.position))
        if key == self._last_pick_key:
            return
        self._last_pick_key = key
        if self.machine_view:
            self.message = "Turn off Machine view to pick faces for cuts."
            return
        seg_id = int(sel.structure_name.split("_")[1])
        if seg_id >= len(self.seg.segments):
            return
        seg = self.seg.segments[seg_id]
        face = int(sel.structure_data["index"])
        mask = overhang_mask(seg.mesh, seg.direction, self.project.overhang_angle_deg)
        self.picked = PickedFace(
            segment_id=seg_id,
            face_index=face,
            position=np.asarray(sel.position, dtype=float),
            normal=seg.mesh.face_normals[face].copy(),
            is_overhang=bool(mask[face]),
        )
        self.refresh_scene()

    # ================================================================== #
    # Scene
    # ================================================================== #
    def _pose_for_view(self) -> tuple[RotaryPose | None, str]:
        """Rotary pose from the Hotend/IK test direction, plus a status line."""
        direction = direction_from_angles(self.tilt_deg, self.azimuth_deg)
        try:
            pose = inverse_kinematics(direction, self.machine)
        except IKError as exc:
            return None, f"IK: unreachable - {exc}"
        a_cmd, c_cmd = to_commanded(pose, self.machine)
        k = self.machine.kinematics
        return pose, f"IK: {k.a_axis.letter}={a_cmd:.2f}  {k.c_axis.letter}={c_cmd:.2f}  (commanded)"

    def _obstacle_segments(self) -> list[int]:
        """Segments the hotend test checks against.

        If we're testing a specific segment (``tool_segment``), only the ones
        printed BEFORE it exist yet. Otherwise check against every segment.
        """
        if self.seg is None:
            return []
        if self.tool_segment is None or self.tool_segment not in self.seg.order:
            return [s.id for s in self.seg.segments]
        return self.seg.order[: self.seg.order.index(self.tool_segment)]

    def _get_obstacles(self) -> ObstacleSet:
        """Table + relevant segments, sampled once per change, for collision tests."""
        if self._obstacles is None:
            meshes, names = [self.table], ["table"]
            if self.seg is not None:
                for sid in self._obstacle_segments():
                    meshes.append(self.seg.segments[sid].mesh)
                    names.append(f"segment {sid}")
            self._obstacles = ObstacleSet.from_meshes(meshes, self.machine.collision.sample_spacing, names)
        return self._obstacles

    def refresh_scene(self) -> None:
        """Re-register all geometry based on current state."""
        pivot = self.machine.kinematics.pivot
        pose, ik_text = self._pose_for_view()

        def place(vertices: np.ndarray) -> np.ndarray:
            """Part frame -> what we draw (machine frame if machine view is on)."""
            if self.machine_view and pose is not None:
                return part_to_machine(vertices, pose, pivot)
            return vertices

        ps.register_surface_mesh("table", place(self.table.vertices), self.table.faces, color=COLOR_TABLE)
        ps.register_point_cloud("pivot", place(pivot[None, :]), color=COLOR_PIVOT, radius=0.006)

        # ---- segments, coloured, with overhang faces in red ----
        names_now: set[str] = set()
        if self.seg is not None:
            for s in self.seg.segments:
                name = f"segment_{s.id}"
                names_now.add(name)
                m = ps.register_surface_mesh(name, place(s.mesh.vertices), s.mesh.faces, color=segment_color(s.id))
                m.set_selection_mode("faces_only")
                if self.show_overhangs:
                    colors = np.tile(segment_color(s.id), (len(s.mesh.faces), 1))
                    colors[overhang_mask(s.mesh, s.direction, self.project.overhang_angle_deg)] = COLOR_OVERHANG
                    m.add_color_quantity("overhangs", colors, defined_on="faces", enabled=True)
        for stale in self._segment_names - names_now:
            ps.remove_surface_mesh(stale, error_if_absent=False)
        self._segment_names = names_now

        # ---- selected cut plane + normal arrow ----
        self._draw_cut_plane(place)

        # ---- picked face marker ----
        if self.picked is not None and not self.machine_view:
            ps.register_point_cloud("picked", self.picked.position[None, :], color=COLOR_PICK, radius=0.008)
        else:
            ps.remove_point_cloud("picked", error_if_absent=False)

        # ---- hotend test ----
        lines = [ik_text]
        if self.show_tool:
            direction = direction_from_angles(self.tilt_deg, self.azimuth_deg)
            col = check_collision(self.hotend, self.tip, direction, self._get_obstacles())
            tool = self.hotend.to_mesh(self.tip, direction)
            ps.register_surface_mesh(
                "hotend", place(tool.vertices), tool.faces,
                color=COLOR_TOOL_HIT if col.hit else COLOR_TOOL_OK, transparency=0.5,
            )
            if col.hit:
                lines.append(f"COLLISION with '{col.obstacle_name}' ({col.num_points} sample points)")
            else:
                lines.append("No collision")
            if self.machine_view and pose is not None:
                tip_m = part_to_machine(self.tip, pose, pivot)
                lines.append(f"Tip in machine XYZ: {np.round(tip_m, 2).tolist()}")
        else:
            ps.remove_surface_mesh("hotend", error_if_absent=False)
        self.status_lines = lines

    def _draw_cut_plane(self, place) -> None:
        cut = self._current_cut()
        if cut is None or self.part is None:
            ps.remove_surface_mesh("cut_plane", error_if_absent=False)
            ps.remove_point_cloud("cut_normal", error_if_absent=False)
            return
        half = 0.6 * float(np.linalg.norm(self.part.extents))
        corners = np.array([[-half, -half, 0], [half, -half, 0], [half, half, 0], [-half, half, 0]])
        corners = corners @ rotation_from_z(cut.normal).T + cut.point
        applied = True
        if self.seg is not None and self.selected_cut < len(self.seg.cut_reports):
            applied = self.seg.cut_reports[self.selected_cut].applied
        ps.register_surface_mesh(
            "cut_plane", place(corners), np.array([[0, 1, 2], [0, 2, 3]]),
            color=COLOR_PLANE_OK if applied and not self.dirty else COLOR_PLANE_BAD,
            transparency=0.35,
        ).set_back_face_policy("identical")
        pc = ps.register_point_cloud("cut_normal", place(cut.point[None, :]), color=COLOR_PLANE_OK, radius=0.006)
        arrow = cut.normal[None, :] * half
        if self.machine_view:
            pose, _ = self._pose_for_view()
            if pose is not None:
                arrow = part_to_machine(arrow, pose, np.zeros(3))
        pc.add_vector_quantity("normal", arrow, vectortype="ambient", enabled=True, color=COLOR_PLANE_OK)

    # ================================================================== #
    # UI
    # ================================================================== #
    def _ui_callback(self) -> None:
        self._poll_pick()
        psim.PushItemWidth(200)
        self._ui_model()
        self._ui_project()
        self._ui_cuts()
        self._ui_segments()
        self._ui_hotend()
        psim.PopItemWidth()

    def _ui_model(self) -> None:
        if not psim.CollapsingHeader("Model", psim.ImGuiTreeNodeFlags_DefaultOpen):
            return
        _, self.path_text = psim.InputText("STL path", self.path_text)
        psim.SameLine()
        if psim.Button("Load STL"):
            self.load(self.path_text.strip().strip('"'))
        names = list(DEMO_MODELS)
        _, self.demo_index = psim.Combo("Demo part", self.demo_index, names)
        psim.SameLine()
        if psim.Button("Load demo"):
            self.load_demo(names[self.demo_index])
        _, self.project.place_on_bed = psim.Checkbox("Place on table when loading", self.project.place_on_bed)
        for line in self.info_lines:
            psim.TextUnformatted(line)
        if self.message:
            psim.TextWrapped(self.message)

    def _ui_project(self) -> None:
        if not psim.CollapsingHeader("Project"):
            return
        _, self.project_path_text = psim.InputText("Project file", self.project_path_text)
        if psim.Button("Save project"):
            self.save_project_file(self.project_path_text.strip().strip('"'))
        psim.SameLine()
        if psim.Button("Load project"):
            self.load_project_file(self.project_path_text.strip().strip('"'))

    def _ui_cuts(self) -> None:
        if not psim.CollapsingHeader("Cuts", psim.ImGuiTreeNodeFlags_DefaultOpen):
            return
        if self.part is None:
            psim.TextUnformatted("Load a model first.")
            return

        # ---- overhang display ----
        c1, self.show_overhangs = psim.Checkbox("Show overhangs (red)", self.show_overhangs)
        c2, self.project.overhang_angle_deg = psim.SliderFloat(
            "Overhang threshold (deg)", self.project.overhang_angle_deg, 10.0, 89.0
        )
        if c1 or c2:
            self.refresh_scene()

        # ---- cut list ----
        cuts = self.project.cuts
        psim.Separator()
        if psim.Button("Add cut"):
            self.add_cut()
        psim.SameLine()
        c, self.live_update = psim.Checkbox("Live update", self.live_update)
        if self.dirty:
            psim.SameLine()
            if psim.Button("Apply cuts"):
                self.recompute()
        for i, cut in enumerate(cuts):
            report = self.seg.cut_reports[i] if self.seg and i < len(self.seg.cut_reports) else None
            status = "" if report is None or report.applied else "  [skipped]"
            local = " local" if cut.local else ""
            label = f"Cut {i + 1}{local}  n=({cut.normal[0]:.2f}, {cut.normal[1]:.2f}, {cut.normal[2]:.2f}){status}"
            psim.PushID(f"cut{i}")
            if psim.Selectable(label, i == self.selected_cut):
                self.selected_cut = i
                self._sync_angles_from_cut()
                self.refresh_scene()
            psim.PopID()

        cut = self._current_cut()
        if cut is None:
            psim.TextUnformatted("Click 'Add cut', or pick a cut above to edit it.")
            return
        i = self.selected_cut
        if psim.Button("Up"):
            self.move_cut(i, -1)
        psim.SameLine()
        if psim.Button("Down"):
            self.move_cut(i, +1)
        psim.SameLine()
        if psim.Button("Delete"):
            self.delete_cut(i)
            return
        self._ui_cut_editor(cut)

    def _ui_cut_editor(self, cut: Cut) -> None:
        """Widgets for editing the selected cut."""
        psim.Separator()
        psim.TextUnformatted(f"Editing cut {self.selected_cut + 1}")
        changed = False

        # Point
        c, p = psim.InputFloat3("Point", tuple(cut.point))
        if c:
            cut.point = np.array(p, dtype=float)
            changed = True
        offset = float(cut.normal @ cut.point)
        c, new_offset = psim.InputFloat("Slide along normal", offset, 0.5, 5.0, "%.2f")
        if c:
            cut.point = cut.point + (new_offset - offset) * cut.normal
            changed = True

        # Normal (= build direction of the new segment)
        c1, self.edit_tilt = psim.SliderFloat("Normal tilt from +Z", self.edit_tilt, 0.0, 180.0)
        c2, self.edit_az = psim.SliderFloat("Normal azimuth", self.edit_az, -180.0, 180.0)
        if c1 or c2:
            cut.normal = direction_from_angles(self.edit_tilt, self.edit_az)
            changed = True
        c, n = psim.InputFloat3("Normal (raw)", tuple(cut.normal))
        if c and np.linalg.norm(n) > 1e-9:
            cut.normal = normalize(np.array(n, dtype=float))
            self._sync_angles_from_cut()
            changed = True
        if psim.Button("Flip normal"):
            cut.normal = -cut.normal
            self._sync_angles_from_cut()
            changed = True

        # Snap to picked face
        if self.picked is None:
            psim.TextWrapped("Tip: click a face in the 3D view to pick it, then snap the plane to it.")
        else:
            pk = self.picked
            tag = " (OVERHANG)" if pk.is_overhang else ""
            psim.TextUnformatted(f"Picked: segment {pk.segment_id}, face {pk.face_index}{tag}")
            if psim.Button("Normal <- picked face"):
                cut.normal = pk.normal.copy()
                self._sync_angles_from_cut()
                changed = True
            psim.SameLine()
            if psim.Button("Point <- picked point"):
                cut.point = pk.position.copy()
                changed = True
            if psim.Button("Plane <- picked face (both)"):
                cut.normal, cut.point = pk.normal.copy(), pk.position.copy()
                self._sync_angles_from_cut()
                changed = True

        # Local cut and piece selection
        c, cut.local = psim.Checkbox("Local cut (only selected piece(s) move)", cut.local)
        changed |= c
        report = self.seg.cut_reports[self.selected_cut] if self.seg and self.selected_cut < len(self.seg.cut_reports) else None
        if report is not None:
            if report.applied:
                psim.TextWrapped(report.message)
            elif report.message:
                psim.TextColored(RED_TEXT, report.message)
            if cut.local and report.component_centroids:
                auto = cut.components is None
                c, auto = psim.Checkbox("Auto: piece nearest the cut point", auto)
                if c:
                    cut.components = None if auto else list(report.chosen_components)
                    changed = True
                for k, (cen, vol) in enumerate(zip(report.component_centroids, report.component_volumes)):
                    on = k in report.chosen_components
                    label = f"piece {k}: {vol / 1000:.2f} cm^3 at ({cen[0]:.0f}, {cen[1]:.0f}, {cen[2]:.0f})"
                    c, on = psim.Checkbox(label, on)
                    if c:
                        chosen = set(report.chosen_components)
                        chosen = chosen | {k} if on else chosen - {k}
                        cut.components = sorted(chosen)
                        changed = True
        if changed:
            self.cuts_changed()

    def _ui_segments(self) -> None:
        if not psim.CollapsingHeader("Segments", psim.ImGuiTreeNodeFlags_DefaultOpen) or self.seg is None:
            return
        k = self.machine.kinematics
        psim.TextUnformatted(f"{len(self.seg.segments)} segment(s). Print order:")
        for step, sid in enumerate(self.seg.order):
            s = self.seg.segments[sid]
            psim.ColorButton(f"##col{sid}", (*segment_color(sid), 1.0))
            psim.SameLine()
            try:
                a, c = to_commanded(inverse_kinematics(s.direction, self.machine), self.machine)
                ac = f"{k.a_axis.letter}{a:.1f} {k.c_axis.letter}{c:.1f}"
            except IKError:
                ac = "unreachable"
            rests = "bed" if s.is_root else ", ".join(str(x) for x in s.supports) or "nothing!"
            area = overhang_area(s.mesh, s.direction, self.project.overhang_angle_deg)
            psim.TextUnformatted(
                f"{step + 1}. seg {sid}: tilt {s.tilt_deg:.0f} deg ({ac}), on {rests}, "
                f"overhang {area:.0f} mm^2"
            )
        if self.seg.warnings:
            psim.Separator()
            for w in self.seg.warnings:
                psim.TextColored(RED_TEXT, "! " + w)
        else:
            psim.TextUnformatted("Cuts form a valid print order.")

    def _ui_hotend(self) -> None:
        if not psim.CollapsingHeader("Hotend / IK test"):
            return
        changed = False
        c, self.show_tool = psim.Checkbox("Show hotend", self.show_tool)
        changed |= c
        cut = self._current_cut()
        if cut is not None and psim.Button("Aim at selected cut"):
            # Put the nozzle on the cut point, pointing along the cut normal, and
            # only check against what is printed before that cut's segment.
            self.tilt_deg, self.azimuth_deg = angles_of(cut.normal)
            self.tip = cut.point.copy()
            self.show_tool = True
            report = self.seg.cut_reports[self.selected_cut] if self.seg else None
            self.tool_segment = report.new_segment if report else None
            self._obstacles = None
            changed = True
        if self.tool_segment is not None:
            psim.TextUnformatted(f"Checking against segments printed before segment {self.tool_segment}")
            psim.SameLine()
            if psim.Button("Check all"):
                self.tool_segment = None
                self._obstacles = None
                changed = True
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
                psim.TextColored(RED_TEXT, line)
            else:
                psim.TextUnformatted(line)
        if changed:
            self.refresh_scene()

    # ================================================================== #
    # Startup
    # ================================================================== #
    def setup(self, backend: str | None = None) -> None:
        """Initialise Polyscope and the scene (separate from run() for tests)."""
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
        if self.path_text.lower().endswith(".json"):
            self.project_path_text = self.path_text
            self.load_project_file(self.path_text)
        elif self.path_text:
            self.load(self.path_text)
        else:
            self.refresh_scene()

    def run(self) -> None:
        self.setup()
        ps.show()
