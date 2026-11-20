"""Configuration validation and unit handling."""

import json
import math

import pytest

from isaac_epics.config import (
    UNIT_PRESETS,
    AdapterConfig,
    AxisConfig,
    ConfigError,
    default_config,
)


def test_default_config_matches_the_prototype_stage():
    cfg = default_config()
    assert [a.joint for a in cfg.axes] == ["x_axis", "y_axis", "z_axis", "theta_axis"]


def test_rotary_axis_serves_degrees_over_a_radian_joint():
    """The context document flags this exact mismatch as a live bug source."""
    theta = default_config().axis("m4")
    assert theta.egu == "deg"
    assert theta.to_sim(180.0) == pytest.approx(math.pi)
    assert theta.to_egu(math.pi) == pytest.approx(180.0)


def test_linear_axis_serves_mm_over_a_metre_joint():
    x = default_config().axis("m1")
    assert x.to_sim(1.0) == pytest.approx(1e-3)


def test_unit_round_trip_for_every_preset():
    for name, scale in UNIT_PRESETS.items():
        axis = AxisConfig(motor="m", joint="j", egu=name, sim_units_per_egu=scale)
        assert axis.to_egu(axis.to_sim(7.5)) == pytest.approx(7.5)


def test_units_block_computes_the_ratio():
    axis = AxisConfig.from_dict({
        "motor": "m4", "joint": "theta", "units": {"sim": "rad", "egu": "deg"},
    })
    assert axis.sim_units_per_egu == pytest.approx(math.pi / 180.0)
    assert axis.egu == "deg"


def test_unknown_unit_rejected():
    with pytest.raises(ConfigError, match="unknown unit"):
        AxisConfig.from_dict({"motor": "m", "joint": "j",
                              "units": {"sim": "furlong", "egu": "mm"}})


def test_unknown_axis_key_rejected():
    """A typo in a config key must not be silently ignored."""
    with pytest.raises(ConfigError, match="unknown axis keys"):
        AxisConfig.from_dict({"motor": "m", "joint": "j", "velocty": 5.0})


@pytest.mark.parametrize("bad", [
    {"motor": "", "joint": "j"},
    {"motor": "m.1", "joint": "j"},
    {"motor": "m", "joint": ""},
    {"motor": "m", "joint": "j", "low_limit": 10.0, "high_limit": -10.0},
    {"motor": "m", "joint": "j", "velocity": 0.0},
    {"motor": "m", "joint": "j", "velocity": 10.0, "max_velocity": 1.0},
    {"motor": "m", "joint": "j", "retry_deadband": 0.0},
    {"motor": "m", "joint": "j", "sim_units_per_egu": 0.0},
    {"motor": "m", "joint": "j", "direction": "sideways"},
    {"motor": "m", "joint": "j", "max_retries": -1},
])
def test_invalid_axis_rejected(bad):
    with pytest.raises(ConfigError):
        AxisConfig.from_dict(bad)


def test_zero_deadband_rejected_with_a_useful_message():
    with pytest.raises(ConfigError, match="never be declared complete"):
        AxisConfig(motor="m", joint="j", retry_deadband=0.0)


def test_duplicate_motor_names_rejected():
    with pytest.raises(ConfigError, match="duplicate motor"):
        AdapterConfig(axes=[AxisConfig(motor="m1", joint="a"),
                            AxisConfig(motor="m1", joint="b")])


def test_two_axes_on_one_joint_rejected():
    """Joint ownership must be unambiguous (architecture decision 7.4)."""
    with pytest.raises(ConfigError, match="same joint"):
        AdapterConfig(axes=[AxisConfig(motor="m1", joint="x"),
                            AxisConfig(motor="m2", joint="x")])


def test_no_axes_rejected():
    with pytest.raises(ConfigError, match="at least one axis"):
        AdapterConfig(axes=[])


def test_empty_prefix_rejected():
    with pytest.raises(ConfigError, match="prefix is required"):
        AdapterConfig(prefix="", axes=[AxisConfig(motor="m", joint="j")])


# -- naming discipline (section 8) ---------------------------------------

def test_sim_prefix_produces_no_warnings():
    cfg = AdapterConfig(prefix="SIM:TWIN:", axes=[AxisConfig(motor="m", joint="j")])
    assert cfg.naming_warnings() == []


def test_ambiguous_prefix_warns():
    cfg = AdapterConfig(prefix="XSD:", axes=[AxisConfig(motor="m", joint="j")])
    warnings = cfg.naming_warnings()
    assert warnings
    assert any("SIM" in w for w in warnings)


def test_non_loopback_ca_interface_warns():
    cfg = AdapterConfig(prefix="SIM:", ca_interface="164.54.100.54",
                        axes=[AxisConfig(motor="m", joint="j")])
    assert any("visible to other hosts" in w for w in cfg.naming_warnings())


def test_loopback_is_the_default():
    cfg = AdapterConfig(axes=[AxisConfig(motor="m", joint="j")])
    assert cfg.ca_interface == "127.0.0.1"
    assert cfg.http_host == "127.0.0.1"


# -- file loading ---------------------------------------------------------

def test_round_trip_through_a_file(tmp_path):
    body = {
        "prefix": "SIM:TEST:",
        "bind": {"http_host": "127.0.0.1", "http_port": 9999},
        "axes": [{"motor": "m1", "joint": "x_axis",
                  "units": {"sim": "m", "egu": "mm"},
                  "low_limit": -10, "high_limit": 10}],
    }
    path = tmp_path / "cfg.json"
    path.write_text(json.dumps(body))
    cfg = AdapterConfig.from_file(str(path))
    assert cfg.prefix == "SIM:TEST:"
    assert cfg.http_port == 9999
    assert cfg.axis("m1").to_sim(1.0) == pytest.approx(1e-3)


def test_malformed_json_names_the_file(tmp_path):
    path = tmp_path / "bad.json"
    path.write_text("{not json")
    with pytest.raises(ConfigError, match="invalid JSON"):
        AdapterConfig.from_file(str(path))


def test_overrides_ignore_none():
    cfg = default_config()
    assert cfg.with_overrides(prefix=None).prefix == cfg.prefix
    assert cfg.with_overrides(prefix="OTHER:").prefix == "OTHER:"


# -- config file path handling -------------------------------------------

def test_missing_file_names_the_resolved_path(tmp_path):
    with pytest.raises(ConfigError, match="no such file"):
        AdapterConfig.from_file(str(tmp_path / "nope.json"))


def test_directory_is_rejected_clearly(tmp_path):
    with pytest.raises(ConfigError, match="not a regular file"):
        AdapterConfig.from_file(str(tmp_path))


def test_oversized_config_rejected(tmp_path):
    path = tmp_path / "huge.json"
    path.write_text("[" + "0," * 2_000_000 + "0]")
    with pytest.raises(ConfigError, match="larger than"):
        AdapterConfig.from_file(str(path))


def test_tilde_is_expanded(tmp_path, monkeypatch):
    monkeypatch.setenv("HOME", str(tmp_path))
    (tmp_path / "cfg.json").write_text(
        '{"prefix": "SIM:X:", "axes": [{"motor": "m1", "joint": "x"}]}'
    )
    assert AdapterConfig.from_file("~/cfg.json").prefix == "SIM:X:"
