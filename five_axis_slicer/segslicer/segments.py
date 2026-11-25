"""Cuts and segments: turning a part + a list of cut planes into printable chunks.

Vocabulary
----------
* A **cut** is a plane (point + normal). The normal is the build direction of
  the new segment created on the side the normal points to.
* A **segment** is a watertight solid with a build direction. Segment 0 (the
  "root") is whatever is left on the bed and is printed along +Z.

How cuts are applied (:func:`apply_cuts`)
-----------------------------------------
Cuts are applied one at a time, in list order:

1. Find the segment that CONTAINS the cut's point. Only that segment is cut,
   so a plane that happens to pass through other segments doesn't touch them.
2. Split it with the plane. The side the normal points to ("upper") may fall
   apart into several connected pieces.
3. Normal cut: every upper piece becomes the new segment.
   LOCAL cut: only the selected upper pieces become the new segment (default:
   the piece nearest the cut point). Unselected pieces are glued back onto the
   segment that was cut.

Because each cut only depends on the cuts before it, reordering cuts can
change the result. That's what the Up/Down buttons in the UI are for.

Dependencies (which segment must be printed first)
--------------------------------------------------
After all cuts, every non-root segment has a flat base face (its "cap") lying
on its cut plane. We probe a tiny bit below each cap face and ask which
segment contains that point: that segment supports it and must print first.
This is purely geometric, so it stays correct after later cuts split a
supporting segment. The result is a graph; a topological sort gives the print
order. Problems (floating pieces, cycles, unreachable directions) become
human-readable warnings instead of exceptions.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
import trimesh

from . import geometry
from .config import MachineConfig
from .kinematics import IKError, inverse_kinematics
from .transforms import Z_AXIS, normalize

# Tolerances (mm)
ON_PLANE_TOL = 1e-3   # how close to the plane a cap face must be
PROBE_DEPTH = 0.02    # how far below a cap face we probe for the supporter
SURFACE_TOL = 0.5     # a cut point this close to a segment's surface still picks it
MAX_PROBES = 40       # cap faces probed per segment (largest first)


@dataclass
class Cut:
    """A user (or auto-segmenter) defined cut plane."""

    point: np.ndarray
    normal: np.ndarray
    local: bool = False
    # Local cuts only: which upper pieces to take (indices into the list the
    # UI shows). None = automatic (the piece nearest the cut point).
    components: list[int] | None = None
    name: str = ""

    def __post_init__(self) -> None:
        self.point = np.asarray(self.point, dtype=float)
        self.normal = normalize(self.normal)

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "point": self.point.tolist(),
            "normal": self.normal.tolist(),
            "local": self.local,
            "components": self.components,
        }

    @classmethod
    def from_dict(cls, d: dict) -> "Cut":
        return cls(
            point=d["point"],
            normal=d["normal"],
            local=bool(d.get("local", False)),
            components=d.get("components"),
            name=d.get("name", ""),
        )


@dataclass
class Segment:
    """One printable chunk."""

    id: int
    mesh: trimesh.Trimesh
    direction: np.ndarray        # build direction (unit vector, part frame)
    base_point: np.ndarray       # any point on the base plane
    cut_index: int | None        # cut that created it (None for the root)
    supports: list[int] = field(default_factory=list)  # segments it rests on

    @property
    def is_root(self) -> bool:
        return self.cut_index is None

    @property
    def tilt_deg(self) -> float:
        """Angle between the build direction and the bed normal."""
        return float(np.degrees(np.arccos(np.clip(self.direction @ Z_AXIS, -1, 1))))


@dataclass
class CutReport:
    """What happened when one cut was applied (shown in the UI)."""

    cut_index: int
    applied: bool
    target_segment: int | None = None
    new_segment: int | None = None
    component_centroids: list[np.ndarray] = field(default_factory=list)
    component_volumes: list[float] = field(default_factory=list)
    chosen_components: list[int] = field(default_factory=list)
    message: str = ""


@dataclass
class Segmentation:
    """Result of :func:`apply_cuts`."""

    segments: list[Segment]
    order: list[int]             # print order (segment ids)
    cut_reports: list[CutReport]
    warnings: list[str]

    @property
    def valid(self) -> bool:
        return not self.warnings


# --------------------------------------------------------------------------- #
# Applying cuts
# --------------------------------------------------------------------------- #
def find_target_segment(segments: list[Segment], point: np.ndarray) -> int | None:
    """Index of the segment containing ``point`` (or touching it, within SURFACE_TOL)."""
    windings = [geometry.winding_number(s.mesh, point) for s in segments]
    best = int(np.argmax(windings))
    if windings[best] > 0.5:
        return best
    # Point is not strictly inside anything: accept a segment whose surface
    # it is (almost) on - e.g. a cut point snapped to a picked face.
    dists = [geometry.distance_to_surface(s.mesh, point) for s in segments]
    best = int(np.argmin(dists))
    return best if dists[best] <= SURFACE_TOL else None


def _nearest_component(pieces: list[trimesh.Trimesh], point: np.ndarray) -> int:
    return int(np.argmin([geometry.distance_to_surface(p, point) for p in pieces]))


def apply_cuts(
    part: trimesh.Trimesh,
    cuts: list[Cut],
    machine: MachineConfig | None = None,
) -> Segmentation:
    """Apply ``cuts`` to ``part`` and work out the dependency order.

    ``machine`` is optional; if given, each build direction is checked for
    reachability with the A/C limits.
    """
    lowest = part.vertices[np.argmin(part.vertices[:, 2])]
    segments = [Segment(0, part.copy(), Z_AXIS.copy(), lowest.copy(), None)]
    reports: list[CutReport] = []
    warnings: list[str] = []

    for i, cut in enumerate(cuts):
        label = f"Cut {i + 1}" + (f" ({cut.name})" if cut.name else "")
        report = CutReport(cut_index=i, applied=False)
        reports.append(report)

        target = find_target_segment(segments, cut.point)
        if target is None:
            report.message = "point is not inside any segment - skipped"
            warnings.append(f"{label}: {report.message}")
            continue
        report.target_segment = target

        try:
            uppers, lower = geometry.split_by_plane(segments[target].mesh, cut.point, cut.normal)
        except ValueError as exc:
            report.message = f"cut failed: {exc}"
            warnings.append(f"{label}: {report.message}")
            continue
        report.component_centroids = [p.centroid for p in uppers]
        report.component_volumes = [float(p.volume) for p in uppers]
        if not uppers or lower is None:
            report.message = f"plane does not split segment {target} - skipped"
            warnings.append(f"{label}: {report.message}")
            continue

        # Which upper pieces become the new segment?
        if not cut.local:
            chosen = list(range(len(uppers)))
        else:
            chosen = [c for c in (cut.components or []) if 0 <= c < len(uppers)]
            if cut.components and len(chosen) != len(cut.components):
                warnings.append(f"{label}: some selected pieces no longer exist; using the rest")
            if not chosen:
                chosen = [_nearest_component(uppers, cut.point)]
        report.chosen_components = chosen

        leftover = [lower] + [p for k, p in enumerate(uppers) if k not in chosen]
        segments[target].mesh = geometry.union(leftover)
        new_id = len(segments)
        segments.append(
            Segment(
                id=new_id,
                mesh=geometry.union([uppers[k] for k in chosen]),
                direction=cut.normal.copy(),
                base_point=cut.point.copy(),
                cut_index=i,
            )
        )
        report.applied = True
        report.new_segment = new_id
        report.message = (
            f"split segment {target}: {len(uppers)} piece(s) above, "
            f"{len(chosen)} -> segment {new_id}"
        )

    warnings += _compute_supports(segments)
    order, cycle_warning = _print_order(segments)
    if cycle_warning:
        warnings.append(cycle_warning)
    warnings += _check_floating_pieces(segments)
    if machine is not None:
        for s in segments:
            try:
                inverse_kinematics(s.direction, machine)
            except IKError:
                warnings.append(
                    f"Segment {s.id}: build direction tilts {s.tilt_deg:.0f} deg - "
                    "outside the A-axis limits"
                )
    return Segmentation(segments, order, reports, warnings)


# --------------------------------------------------------------------------- #
# Dependencies
# --------------------------------------------------------------------------- #
def _compute_supports(segments: list[Segment]) -> list[str]:
    """Fill in ``Segment.supports`` by probing just below each base cap face."""
    warnings = []
    for seg in segments:
        if seg.is_root:
            continue
        caps = np.nonzero(geometry.faces_on_plane(seg.mesh, seg.base_point, seg.direction, ON_PLANE_TOL))[0]
        if len(caps) == 0:
            warnings.append(f"Segment {seg.id}: has no flat base face on its cut plane")
            continue
        # Probe the biggest cap faces (enough to find every supporter in practice).
        caps = caps[np.argsort(-seg.mesh.area_faces[caps])][:MAX_PROBES]
        probes = seg.mesh.triangles_center[caps] - PROBE_DEPTH * seg.direction
        supporters: set[int] = set()
        unsupported = 0
        for p in probes:
            hit = [o.id for o in segments if o.id != seg.id and geometry.contains(o.mesh, p)]
            if hit:
                supporters.update(hit)
            else:
                unsupported += 1
        seg.supports = sorted(supporters)
        if not supporters:
            warnings.append(f"Segment {seg.id}: its base does not rest on any other segment")
        elif unsupported:
            warnings.append(
                f"Segment {seg.id}: part of its base ({unsupported} probe(s)) rests on nothing"
            )
    return warnings


def _print_order(segments: list[Segment]) -> tuple[list[int], str | None]:
    """Topological sort: supporters before the segments they hold up.

    Ties are broken by segment id, so the order is stable and readable.
    Returns (order, warning-or-None).
    """
    remaining = {s.id: set(s.supports) for s in segments}
    order: list[int] = []
    while remaining:
        ready = sorted(i for i, deps in remaining.items() if not deps)
        if not ready:
            stuck = sorted(remaining)
            # Fall back to id order so the rest of the app still works.
            return order + stuck, (
                f"Segments {stuck} depend on each other in a loop - "
                "no valid print order. Reorder or change the cuts."
            )
        nxt = ready[0]
        order.append(nxt)
        del remaining[nxt]
        for deps in remaining.values():
            deps.discard(nxt)
    return order, None


def _check_floating_pieces(segments: list[Segment]) -> list[str]:
    """Every connected piece of a segment must touch that segment's base plane."""
    warnings = []
    for seg in segments:
        try:
            pieces = geometry.components(seg.mesh)
        except ValueError:
            continue
        base = float(seg.base_point @ seg.direction)
        for k, piece in enumerate(pieces):
            lowest = float((piece.vertices @ seg.direction).min())
            if lowest > base + ON_PLANE_TOL:
                where = "the bed" if seg.is_root else "its cut plane"
                warnings.append(
                    f"Segment {seg.id}: piece {k} starts {lowest - base:.1f} mm above "
                    f"{where} (it would print in mid-air)"
                )
    return warnings
