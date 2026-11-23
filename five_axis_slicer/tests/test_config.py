import pytest
import yaml

from segslicer.config import DEFAULT_MACHINE_PATH, ConfigError, load_settings, machine_from_dict


def _raw():
    with open(DEFAULT_MACHINE_PATH) as f:
        return yaml.safe_load(f)


def test_default_machine_loads(machine):
    assert machine.kinematics.a_axis.letter == "A"
    assert machine.kinematics.c_axis.letter == "C"
    assert machine.kinematics.pivot.shape == (3,)
    assert machine.hotend.cone_height > 0
    assert "G28" in machine.start_gcode


def test_settings_load():
    s = load_settings()
    assert s.overhang_angle_deg == 50.0


def test_missing_value_names_the_key():
    raw = _raw()
    del raw["hotend"]["cone_height"]
    with pytest.raises(ConfigError, match="hotend.cone_height"):
        machine_from_dict(raw)


def test_bad_sign_rejected():
    raw = _raw()
    raw["kinematics"]["a_axis"]["sign"] = 2
    with pytest.raises(ConfigError, match="sign"):
        machine_from_dict(raw)


def test_bad_firmware_rejected():
    raw = _raw()
    raw["firmware"] = "grbl"
    with pytest.raises(ConfigError, match="firmware"):
        machine_from_dict(raw)
