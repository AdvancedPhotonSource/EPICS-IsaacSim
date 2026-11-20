"""Direction A (mirror) tests.

The property that makes direction A safe to point at a live beamline is that
it structurally cannot command one. That is worth testing explicitly.
"""

import ast
import math
import pathlib

import pytest

from isaac_epics.config import ConfigError
from isaac_epics.mirror import Mirror, MirrorMapping, load_mappings
from isaac_epics.plant import SharedPlant


def test_mirror_never_writes_to_a_pv():
    """Static guarantee: no write/put call anywhere in the module."""
    source = pathlib.Path("src/isaac_epics/mirror.py").read_text()
    tree = ast.parse(source)
    forbidden = {"write", "put", "caput", "write_notify"}
    called = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
            called.add(node.func.attr)
    assert not (called & forbidden), f"mirror mode must never write: found {called & forbidden}"


def test_unit_conversion_mm_to_metres():
    mapping = MirrorMapping(pv="XSD:m1.RBV", joint="x_axis", sim_units_per_egu=1e-3)
    assert mapping.to_sim(10.0) == pytest.approx(0.010)


def test_unit_conversion_degrees_to_radians():
    mapping = MirrorMapping.from_dict({
        "pv": "XSD:m4.RBV", "joint": "theta_axis",
        "units": {"sim": "rad", "egu": "deg"},
    })
    assert mapping.to_sim(180.0) == pytest.approx(math.pi)


def test_scale_and_offset_apply_in_egu():
    mapping = MirrorMapping(pv="p", joint="j", sim_units_per_egu=1.0,
                            scale=2.0, offset=1.0)
    assert mapping.to_sim(3.0) == pytest.approx(7.0)


def test_duplicate_joint_rejected():
    with pytest.raises(ConfigError, match="same joint"):
        Mirror([MirrorMapping(pv="a", joint="x"),
                MirrorMapping(pv="b", joint="x")], SharedPlant())


def test_empty_mapping_rejected():
    with pytest.raises(ConfigError):
        Mirror([], SharedPlant())


def test_unknown_key_rejected():
    with pytest.raises(ConfigError, match="unknown mirror keys"):
        MirrorMapping.from_dict({"pv": "a", "joint": "b", "untis": 1})


def test_missing_required_keys_rejected():
    with pytest.raises(ConfigError):
        MirrorMapping.from_dict({"pv": "a"})


def test_status_reports_unseen_pvs_as_stale():
    mirror = Mirror([MirrorMapping(pv="XSD:m1.RBV", joint="x_axis")], SharedPlant())
    status = mirror.status()
    assert status["XSD:m1.RBV"]["stale"] is True
    assert status["XSD:m1.RBV"]["value"] is None


def test_callback_drives_the_joint_and_clears_staleness():
    plant = SharedPlant()
    mapping = MirrorMapping(pv="XSD:m1.RBV", joint="x_axis", sim_units_per_egu=1e-3)
    mirror = Mirror([mapping], plant)

    class Response:
        data = [25.0]

    mirror._make_callback(mapping)(None, Response())
    assert plant.targets == {"x_axis": pytest.approx(0.025)}
    assert mirror.status()["XSD:m1.RBV"]["stale"] is False


def test_nan_readback_is_ignored():
    plant = SharedPlant()
    mapping = MirrorMapping(pv="p", joint="x_axis")
    mirror = Mirror([mapping], plant)

    class Response:
        data = [float("nan")]

    mirror._make_callback(mapping)(None, Response())
    assert plant.targets == {}, "a NaN readback must not reach a joint"


def test_load_mappings_from_file(tmp_path):
    path = tmp_path / "mirror.json"
    path.write_text('{"mirror": [{"pv": "XSD:m1.RBV", "joint": "x_axis", '
                    '"units": {"sim": "m", "egu": "mm"}}]}')
    mappings = load_mappings(str(path))
    assert len(mappings) == 1
    assert mappings[0].to_sim(1.0) == pytest.approx(1e-3)
