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


# -- soft limits must stay inside the articulation's physical limits ------

#: Physical joint limits from digital-twin ``mechanism/beamline_stage.json``,
#: in the runtime units the Isaac Sim tensor API reports (metres, radians).
#: These are a property of the authored articulation, not of this adapter --
#: if the mechanism changes, this table and the config must change together.
MECHANISM_LIMITS_SIM_UNITS = {
    "x_axis": (-0.025, 0.025),
    "y_axis": (-0.025, 0.025),
    "z_axis": (0.0, 0.050),
    "theta_axis": (-math.pi, math.pi),
}


def test_soft_limits_sit_inside_the_physical_joint_limits():
    """Soft limits exist to keep the mechanism off its hard stops.

    If a soft limit were wider than the joint, the adapter would accept a
    move the articulation cannot physically make. The joint would stall
    against its PhysX limit and the move would surface as ``.MISS`` after
    retries -- indistinguishable, to a scan, from a hardware fault.
    """
    for axis in default_config().axes:
        lo_sim, hi_sim = MECHANISM_LIMITS_SIM_UNITS[axis.joint]
        assert axis.to_sim(axis.low_limit) >= lo_sim, (
            f"{axis.motor}: soft low limit {axis.low_limit} {axis.egu} is below "
            f"the {axis.joint} joint limit"
        )
        assert axis.to_sim(axis.high_limit) <= hi_sim, (
            f"{axis.motor}: soft high limit {axis.high_limit} {axis.egu} is above "
            f"the {axis.joint} joint limit"
        )


def test_z_deadband_exceeds_its_predicted_gravity_sag():
    """z is the only axis carrying a gravity load into a finite-stiffness drive.

    Load is z_carriage (3.0 kg) + theta_platter (1.5 kg) against a force
    drive of stiffness 300000 N/m, so the joint settles roughly
    4.5*9.81/300000 = 0.15 mm below target. A deadband tighter than that
    makes every z move report .MISS forever.
    """
    sag_mm = (3.0 + 1.5) * 9.81 / 300000.0 * 1000.0
    z = default_config().axis("m3")
    assert z.retry_deadband > sag_mm, (
        f"z deadband {z.retry_deadband} mm must exceed the ~{sag_mm:.3f} mm "
        "steady-state sag of its drive"
    )


def test_shipped_example_config_matches_default_config():
    """examples/beamline.json and default_config() must not drift apart.

    Compares every field rather than a hand-picked few. An earlier version
    checked only limits and joints, and silently missed that the shipped
    JSON had no console aliases while the built-in config did -- so
    ``%mov x 10`` worked with one and raised NameError with the other.
    """
    shipped = AdapterConfig.from_file("examples/beamline.json")
    built = default_config()
    fields = [f for f in AxisConfig.__dataclass_fields__]
    for a, b in zip(sorted(shipped.axes, key=lambda x: x.motor),
                    sorted(built.axes, key=lambda x: x.motor)):
        for field_name in fields:
            assert getattr(a, field_name) == getattr(b, field_name), (
                f"{a.motor}.{field_name}: example JSON has "
                f"{getattr(a, field_name)!r}, default_config has "
                f"{getattr(b, field_name)!r}"
            )


def test_console_aliases_are_usable_identifiers():
    """`%mov x 10` binds the alias as a Python name, so it must be one."""
    for axis in default_config().axes:
        assert axis.console_name.isidentifier()


def test_bad_alias_rejected():
    with pytest.raises(ConfigError, match="identifier"):
        AxisConfig(motor="m1", joint="j", alias="not a name")


# -- console RunEngine construction --------------------------------------

def test_console_run_engine_is_given_an_explicit_loop():
    """Regression: the console's RunEngine must own a loop.

    Under IPython the main thread can be left with no current event loop,
    and on Python 3.9 ``asyncio.Event()`` resolves its loop through
    ``get_event_loop()``. A RunEngine built without an explicit loop then
    raises "There is no current event loop" on the first ``%mov`` -- at use,
    not at construction. Bluesky's default ``BlueskyMagics.RE`` passes a
    fresh loop for this reason; ours must too.
    """
    import ast
    import pathlib

    source = pathlib.Path("src/isaac_epics/console.py").read_text()
    tree = ast.parse(source)
    found = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and getattr(node.func, "id", None) == "RunEngine":
            found.append({kw.arg for kw in node.keywords})
    assert found, "console.py no longer constructs a RunEngine"
    for kwargs in found:
        assert "loop" in kwargs, (
            "RunEngine must be constructed with an explicit loop= argument"
        )


def test_live_feedback_cannot_abort_a_plan():
    """A broken visualization callback must not halt a move.

    Regression: on Sentosa matplotlib imports but lacks get_backend, so
    BestEffortCallback raised AttributeError mid-plan and the RunEngine
    aborted a %mov. Diagnostics are not allowed to stop a motor.
    """
    from isaac_epics.console import _SafeCallback

    class Exploding:
        calls = 0

        def __call__(self, name, doc):
            Exploding.calls += 1
            raise AttributeError("module 'matplotlib' has no attribute 'get_backend'")

    cb = _SafeCallback(Exploding())
    cb("start", {})        # must not raise
    cb("event", {})        # must not raise
    cb("stop", {})
    assert Exploding.calls == 1, "callback must be disabled after its first failure"


def test_live_feedback_passes_documents_through_when_healthy():
    from isaac_epics.console import _SafeCallback

    seen = []
    cb = _SafeCallback(lambda name, doc: seen.append(name))
    cb("start", {})
    cb("event", {})
    assert seen == ["start", "event"]


def test_motors_carry_the_label_that_makes_wa_find_them():
    """`%wa` discovers devices through ophyd labels, not by duck-typing.

    An unlabelled motor is invisible to where-all: it prints nothing and
    looks broken rather than empty.
    """
    pytest.importorskip("ophyd")
    from isaac_epics.devices import make_stage_class

    cls = make_stage_class(default_config())
    for axis in default_config().axes:
        cpt = cls._sig_attrs[axis.console_name]
        labels = cpt.kwargs.get("labels", set())
        assert "motors" in labels, f"{axis.console_name} is not labelled 'motors'"
