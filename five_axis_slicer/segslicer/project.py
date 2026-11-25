"""Save / load a project (which model + the list of cuts) as JSON.

Example file::

    {
      "version": 1,
      "model": {"stl": "parts/bracket.stl", "place_on_bed": true},
      "overhang_angle_deg": 50.0,
      "cuts": [
        {"name": "", "point": [10, 0, 14], "normal": [1, 0, 0],
         "local": false, "components": null}
      ]
    }

The model is either ``{"stl": path}`` (relative paths are relative to the
project file) or ``{"demo": "Cube with arm"}`` for a built-in demo part.
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass, field
from pathlib import Path

import trimesh

from .config import MachineConfig
from .demo_models import DEMO_MODELS
from .mesh_io import load_mesh, place_on_bed
from .segments import Cut

PROJECT_VERSION = 1


@dataclass
class Project:
    stl_path: str | None = None       # absolute path, or None for a demo model
    demo_name: str | None = None
    place_on_bed: bool = True
    overhang_angle_deg: float = 50.0
    cuts: list[Cut] = field(default_factory=list)

    def load_part(self, machine: MachineConfig) -> trimesh.Trimesh:
        """Load (or generate) the part this project refers to."""
        if self.demo_name:
            if self.demo_name not in DEMO_MODELS:
                raise ValueError(f"Unknown demo model '{self.demo_name}'")
            mesh = DEMO_MODELS[self.demo_name]()
        elif self.stl_path:
            mesh = load_mesh(self.stl_path)
        else:
            raise ValueError("Project has no model")
        return place_on_bed(mesh, machine) if self.place_on_bed else mesh


def save_project(project: Project, path: str | Path) -> None:
    path = Path(path).resolve()
    if project.demo_name:
        model = {"demo": project.demo_name}
    else:
        stl = Path(project.stl_path or "").resolve()
        # Store a relative path when possible so the project folder can be moved.
        try:
            stl_str = os.path.relpath(stl, path.parent)
        except ValueError:  # different drive on Windows
            stl_str = str(stl)
        model = {"stl": stl_str.replace("\\", "/")}
    model["place_on_bed"] = project.place_on_bed
    data = {
        "version": PROJECT_VERSION,
        "model": model,
        "overhang_angle_deg": project.overhang_angle_deg,
        "cuts": [c.to_dict() for c in project.cuts],
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2), encoding="utf-8")


def load_project(path: str | Path) -> Project:
    path = Path(path).resolve()
    data = json.loads(path.read_text(encoding="utf-8"))
    if data.get("version", 1) > PROJECT_VERSION:
        raise ValueError(f"{path.name} was saved by a newer version of segslicer")
    model = data.get("model", {})
    stl = model.get("stl")
    if stl is not None and not Path(stl).is_absolute():
        stl = str((path.parent / stl).resolve())
    return Project(
        stl_path=stl,
        demo_name=model.get("demo"),
        place_on_bed=bool(model.get("place_on_bed", True)),
        overhang_angle_deg=float(data.get("overhang_angle_deg", 50.0)),
        cuts=[Cut.from_dict(c) for c in data.get("cuts", [])],
    )
