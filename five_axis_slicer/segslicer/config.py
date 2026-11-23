"""Load machine.yaml and settings.yaml into typed Python objects.

The YAML files are the single source of truth for machine geometry. This module
turns them into plain dataclasses so the rest of the code can write
``machine.hotend.cone_height`` instead of ``cfg["hotend"]["cone_height"]``,
and so typos/missing values produce a clear error at startup instead of a
confusing crash later.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml

# Folder that holds the default machine.yaml / settings.yaml.
CONFIG_DIR = Path(__file__).resolve().parent.parent / "config"
DEFAULT_MACHINE_PATH = CONFIG_DIR / "machine.yaml"
DEFAULT_SETTINGS_PATH = CONFIG_DIR / "settings.yaml"


class ConfigError(ValueError):
    """Raised when a config file is missing a value or has a bad value."""


# --------------------------------------------------------------------------- #
# Small helpers for reading nested dicts with good error messages
# --------------------------------------------------------------------------- #
def _get(d: dict, key: str, where: str) -> Any:
    """Return d[key], or raise ConfigError naming the full path of the key."""
    if not isinstance(d, dict) or key not in d:
        raise ConfigError(f"Missing config value '{where}.{key}'")
    return d[key]


def _float(d: dict, key: str, where: str) -> float:
    try:
        return float(_get(d, key, where))
    except (TypeError, ValueError) as exc:
        raise ConfigError(f"'{where}.{key}' must be a number") from exc


def _range(d: dict, key: str, where: str) -> tuple[float, float]:
    """Read a [min, max] pair."""
    v = _get(d, key, where)
    if not isinstance(v, (list, tuple)) or len(v) != 2:
        raise ConfigError(f"'{where}.{key}' must be a [min, max] list")
    lo, hi = float(v[0]), float(v[1])
    if lo > hi:
        raise ConfigError(f"'{where}.{key}': min {lo} is greater than max {hi}")
    return lo, hi


# --------------------------------------------------------------------------- #
# Dataclasses
# --------------------------------------------------------------------------- #
@dataclass
class RotaryAxisConfig:
    """One rotary axis (A or C).

    Angles inside the code are always in the "math" convention (right-hand
    rule). ``sign`` converts math angle -> commanded G-code angle:
    ``gcode_angle = sign * math_angle``. Limits are in COMMANDED degrees.
    """

    letter: str
    sign: int
    min_deg: float
    max_deg: float
    continuous: bool = False

    def within_limits(self, commanded_deg: float, tol: float = 1e-6) -> bool:
        """True if a commanded angle is allowed (always True if continuous)."""
        if self.continuous:
            return True
        return self.min_deg - tol <= commanded_deg <= self.max_deg + tol


@dataclass
class KinematicsConfig:
    type: str
    pivot: np.ndarray  # shape (3,), machine coordinates
    a_axis: RotaryAxisConfig
    c_axis: RotaryAxisConfig


@dataclass
class LimitsConfig:
    x: tuple[float, float]
    y: tuple[float, float]
    z: tuple[float, float]


@dataclass
class TableConfig:
    shape: str  # "round" or "rect"
    diameter: float
    size_x: float
    size_y: float
    thickness: float
    surface_z: float


@dataclass
class HotendConfig:
    cone_half_angle_deg: float
    cone_height: float
    cylinder_radius: float
    cylinder_start: float
    cylinder_height: float
    clearance: float
    tip_ignore_height: float


@dataclass
class CollisionConfig:
    sample_spacing: float


@dataclass
class MotionConfig:
    safe_retract_distance: float
    safe_z: float
    travel_feedrate: float
    rotary_feedrate: float
    retract_length: float
    retract_feedrate: float


@dataclass
class MachineConfig:
    """Everything in machine.yaml."""

    name: str
    firmware: str
    kinematics: KinematicsConfig
    limits: LimitsConfig
    table: TableConfig
    hotend: HotendConfig
    collision: CollisionConfig
    motion: MotionConfig
    start_gcode: str
    end_gcode: str
    source_path: Path | None = None


@dataclass
class Settings:
    """Everything in settings.yaml."""

    prusaslicer_path: str
    overhang_angle_deg: float
    source_path: Path | None = None


# --------------------------------------------------------------------------- #
# Parsers
# --------------------------------------------------------------------------- #
SUPPORTED_FIRMWARE = ("marlin", "reprapfirmware", "klipper")
SUPPORTED_KINEMATICS = ("table_tilting_AC",)


def _parse_rotary(d: dict, where: str) -> RotaryAxisConfig:
    sign = int(_get(d, "sign", where))
    if sign not in (1, -1):
        raise ConfigError(f"'{where}.sign' must be 1 or -1, got {sign}")
    lo, hi = _float(d, "min_deg", where), _float(d, "max_deg", where)
    if lo > hi:
        raise ConfigError(f"'{where}': min_deg {lo} > max_deg {hi}")
    return RotaryAxisConfig(
        letter=str(_get(d, "letter", where)).upper(),
        sign=sign,
        min_deg=lo,
        max_deg=hi,
        continuous=bool(d.get("continuous", False)),
    )


def machine_from_dict(raw: dict, source_path: Path | None = None) -> MachineConfig:
    """Build a MachineConfig from an already-parsed YAML dict."""
    firmware = str(_get(raw, "firmware", "machine")).lower()
    if firmware not in SUPPORTED_FIRMWARE:
        raise ConfigError(f"firmware must be one of {SUPPORTED_FIRMWARE}, got '{firmware}'")

    k = _get(raw, "kinematics", "machine")
    ktype = str(_get(k, "type", "kinematics"))
    if ktype not in SUPPORTED_KINEMATICS:
        raise ConfigError(f"kinematics.type must be one of {SUPPORTED_KINEMATICS}, got '{ktype}'")
    pivot = np.asarray(_get(k, "pivot", "kinematics"), dtype=float)
    if pivot.shape != (3,):
        raise ConfigError("kinematics.pivot must be a list of 3 numbers [x, y, z]")
    kin = KinematicsConfig(
        type=ktype,
        pivot=pivot,
        a_axis=_parse_rotary(_get(k, "a_axis", "kinematics"), "kinematics.a_axis"),
        c_axis=_parse_rotary(_get(k, "c_axis", "kinematics"), "kinematics.c_axis"),
    )
    if kin.a_axis.letter == kin.c_axis.letter:
        raise ConfigError("A and C axes must use different G-code letters")

    lim = _get(raw, "limits", "machine")
    limits = LimitsConfig(
        x=_range(lim, "x", "limits"), y=_range(lim, "y", "limits"), z=_range(lim, "z", "limits")
    )

    t = _get(raw, "table", "machine")
    table = TableConfig(
        shape=str(_get(t, "shape", "table")).lower(),
        diameter=_float(t, "diameter", "table"),
        size_x=_float(t, "size_x", "table"),
        size_y=_float(t, "size_y", "table"),
        thickness=_float(t, "thickness", "table"),
        surface_z=_float(t, "surface_z", "table"),
    )
    if table.shape not in ("round", "rect"):
        raise ConfigError("table.shape must be 'round' or 'rect'")

    h = _get(raw, "hotend", "machine")
    hotend = HotendConfig(
        cone_half_angle_deg=_float(h, "cone_half_angle_deg", "hotend"),
        cone_height=_float(h, "cone_height", "hotend"),
        cylinder_radius=_float(h, "cylinder_radius", "hotend"),
        cylinder_start=_float(h, "cylinder_start", "hotend"),
        cylinder_height=_float(h, "cylinder_height", "hotend"),
        clearance=_float(h, "clearance", "hotend"),
        tip_ignore_height=_float(h, "tip_ignore_height", "hotend"),
    )
    if not 0.0 < hotend.cone_half_angle_deg < 90.0:
        raise ConfigError("hotend.cone_half_angle_deg must be between 0 and 90")

    c = _get(raw, "collision", "machine")
    collision = CollisionConfig(sample_spacing=_float(c, "sample_spacing", "collision"))
    if collision.sample_spacing <= 0:
        raise ConfigError("collision.sample_spacing must be > 0")

    m = _get(raw, "motion", "machine")
    motion = MotionConfig(
        safe_retract_distance=_float(m, "safe_retract_distance", "motion"),
        safe_z=_float(m, "safe_z", "motion"),
        travel_feedrate=_float(m, "travel_feedrate", "motion"),
        rotary_feedrate=_float(m, "rotary_feedrate", "motion"),
        retract_length=_float(m, "retract_length", "motion"),
        retract_feedrate=_float(m, "retract_feedrate", "motion"),
    )

    return MachineConfig(
        name=str(raw.get("name", "unnamed machine")),
        firmware=firmware,
        kinematics=kin,
        limits=limits,
        table=table,
        hotend=hotend,
        collision=collision,
        motion=motion,
        start_gcode=str(raw.get("start_gcode", "")),
        end_gcode=str(raw.get("end_gcode", "")),
        source_path=source_path,
    )


def load_machine(path: str | Path = DEFAULT_MACHINE_PATH) -> MachineConfig:
    """Read and validate machine.yaml."""
    path = Path(path)
    with open(path, "r", encoding="utf-8") as f:
        raw = yaml.safe_load(f)
    if not isinstance(raw, dict):
        raise ConfigError(f"{path} is empty or not a YAML mapping")
    return machine_from_dict(raw, source_path=path)


def load_settings(path: str | Path = DEFAULT_SETTINGS_PATH) -> Settings:
    """Read settings.yaml."""
    path = Path(path)
    with open(path, "r", encoding="utf-8") as f:
        raw = yaml.safe_load(f) or {}
    return Settings(
        prusaslicer_path=str(raw.get("prusaslicer_path", "prusa-slicer")),
        overhang_angle_deg=float(raw.get("overhang_angle_deg", 50.0)),
        source_path=path,
    )
