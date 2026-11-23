"""End-to-end tests: the real HTTP server against the real stdlib client.

These exercise the actual wire protocol and the actual transport, with the
Isaac Sim client code that will run inside the simulator. Only the physics is
faked. What is being tested is the control path -- which is precisely the part
this project exists to get right.
"""

import json
import math
import os
import sys
import time
import urllib.error
import urllib.request

import pytest

from isaac_epics.config import AdapterConfig, AxisConfig
from isaac_epics.ioc import build_ioc
from isaac_epics.plant import SharedPlant
from isaac_epics.transport import TickServer

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "isaacsim_client"))
from epics_tick import EpicsTickClient  # noqa: E402


@pytest.fixture
def server():
    plant = SharedPlant(stale_timeout=1.0)
    srv = TickServer(plant, host="127.0.0.1", port=0,
                     diagnostics=lambda: {"note": "test"}).start()
    try:
        yield srv, plant
    finally:
        srv.stop()


def post(url, body, raw=False):
    data = body if raw else json.dumps(body).encode()
    request = urllib.request.Request(
        url + "/tick", data=data, headers={"Content-Type": "application/json"},
    )
    with urllib.request.urlopen(request, timeout=5) as response:
        return response.status, json.loads(response.read())


# -- protocol over the wire ----------------------------------------------

def test_frame_exchange(server):
    srv, plant = server
    plant.set_targets({"x_axis": 0.25})
    status, body = post(srv.url, {
        "protocol": 1, "positions": {"x_axis": 0.1}, "t_sim": 1.0,
        "playing": True, "seq": 1,
    })
    assert status == 200
    assert body["targets"] == {"x_axis": 0.25}
    assert plant.get_state().positions == {"x_axis": 0.1}


def test_malformed_frame_is_rejected_whole(server):
    srv, plant = server
    post(srv.url, {"protocol": 1, "positions": {"x_axis": 0.1}, "t_sim": 1.0,
                   "playing": True, "seq": 1})
    good = plant.get_state().positions
    with pytest.raises(urllib.error.HTTPError) as exc:
        post(srv.url, {"protocol": 1, "positions": {"x_axis": "banana"},
                       "t_sim": 2.0, "playing": True, "seq": 2})
    assert exc.value.code == 422
    assert plant.get_state().positions == good, "a bad frame must not partly apply"


def test_missing_playing_rejected(server):
    srv, _ = server
    with pytest.raises(urllib.error.HTTPError) as exc:
        post(srv.url, {"protocol": 1, "positions": {}, "t_sim": 1.0})
    assert exc.value.code == 422


def test_invalid_json_rejected(server):
    srv, _ = server
    with pytest.raises(urllib.error.HTTPError) as exc:
        post(srv.url, b"{not json", raw=True)
    assert exc.value.code == 400


def test_oversized_body_gets_a_clean_413(server):
    """A frame slightly over the cap is refused with a status, not a reset."""
    srv, plant = server
    big = json.dumps({"protocol": 1, "playing": True, "t_sim": 0.0,
                      "positions": {f"j{i}": 0.0 for i in range(8000)}}).encode()
    assert len(big) > 64 * 1024
    with pytest.raises(urllib.error.HTTPError) as exc:
        post(srv.url, big, raw=True)
    assert exc.value.code == 413
    assert plant.get_state().frames == 0


def test_absurd_body_does_not_allocate_or_kill_the_server(server):
    """A multi-megabyte body is dropped; the server stays usable afterwards."""
    srv, plant = server
    huge = json.dumps({"protocol": 1, "playing": True, "t_sim": 0.0,
                       "positions": {f"j{i}": 0.0 for i in range(400000)}}).encode()
    assert len(huge) > 4 * 1024 * 1024
    with pytest.raises((urllib.error.HTTPError, urllib.error.URLError, OSError)):
        post(srv.url, huge, raw=True)
    # The contract that matters: nothing was accepted, and the next good
    # frame still works.
    assert plant.get_state().frames == 0
    status, _ = post(srv.url, {"protocol": 1, "positions": {"x_axis": 0.1},
                               "t_sim": 1.0, "playing": True, "seq": 1})
    assert status == 200
    assert plant.get_state().frames == 1


def test_unknown_endpoint_404(server):
    srv, _ = server
    request = urllib.request.Request(srv.url + "/exec", data=b"{}")
    with pytest.raises(urllib.error.HTTPError) as exc:
        urllib.request.urlopen(request, timeout=5)
    assert exc.value.code == 404


def test_state_endpoint_is_curl_friendly(server):
    srv, _ = server
    post(srv.url, {"protocol": 1, "positions": {"x_axis": 0.3}, "t_sim": 2.0,
                   "playing": True, "seq": 1})
    with urllib.request.urlopen(srv.url + "/state", timeout=5) as response:
        body = json.loads(response.read())
    assert body["link_state"] == "running"
    assert body["positions"]["x_axis"] == 0.3


def test_healthz_reports_503_when_no_simulator(server):
    srv, _ = server
    with pytest.raises(urllib.error.HTTPError) as exc:
        urllib.request.urlopen(srv.url + "/healthz", timeout=5)
    assert exc.value.code == 503, "adapter alive != adapter driving a simulation"


# -- the real stdlib client ----------------------------------------------

def test_stdlib_client_round_trip(server):
    srv, plant = server
    plant.set_targets({"x_axis": 0.75})
    client = EpicsTickClient(srv.url, rate_hz=50.0).start()
    try:
        deadline = time.time() + 5
        targets = {}
        while time.time() < deadline:
            targets = client.exchange({"x_axis": 0.2}, t_sim=1.0, playing=True)
            if targets:
                break
            time.sleep(0.02)
        assert targets == {"x_axis": 0.75}
        assert client.connected
        assert plant.get_state().positions == {"x_axis": 0.2}
    finally:
        client.stop()


def test_client_stops_posting_when_the_render_loop_stops(server):
    """The interlock that keeps a frozen simulator from looking healthy.

    Without it the worker thread would keep posting the last snapshot and the
    adapter would report a live link with a frozen readback -- the exact
    fabricated-telemetry failure the design forbids.
    """
    srv, plant = server
    client = EpicsTickClient(srv.url, rate_hz=50.0, max_frame_age=0.2).start()
    try:
        deadline = time.time() + 5
        while time.time() < deadline and not client.connected:
            client.exchange({"x_axis": 0.2}, t_sim=1.0, playing=True)
            time.sleep(0.02)
        assert plant.get_state().connected

        # The render loop stops. exchange() is never called again.
        time.sleep(1.6)
        assert not plant.get_state().connected, "stale frames must stop flowing"
        assert client.stats().suppressed > 0
    finally:
        client.stop()


def test_client_rejects_a_hostile_response():
    """A non-finite target must never reach an articulation."""
    from epics_tick import _parse_targets
    with pytest.raises(ValueError):
        _parse_targets({"protocol": 1, "targets": {"x": float("nan")}})
    with pytest.raises(ValueError):
        _parse_targets({"protocol": 1, "targets": {"x": "0.5"}})


# -- full loop: HTTP in, motor logic, targets out ------------------------

def test_full_loop_moves_a_joint(server):
    """caput-equivalent through the whole adapter, without Channel Access."""
    srv, plant = server
    config = AdapterConfig(
        prefix="SIM:TEST:",
        axes=[AxisConfig(motor="m1", joint="x_axis", sim_units_per_egu=1e-3,
                         low_limit=-50, high_limit=50, velocity=20.0,
                         max_velocity=50.0, accel_time=0.1, retry_deadband=1e-3)],
    )
    ioc = build_ioc(config, plant)
    axis = ioc.axes["m1"]

    assert axis.request_move(10.0)

    position = 0.0
    t_sim = 0.0
    for _ in range(4000):
        _, body = post(srv.url, {
            "protocol": 1, "positions": {"x_axis": position},
            "t_sim": t_sim, "playing": True, "seq": 1,
        })
        outputs = axis.tick(plant.get_state())
        if outputs.target_sim is not None:
            plant.set_targets({"x_axis": outputs.target_sim})
        target = body["targets"].get("x_axis", position)
        position += (target - position) * 0.3   # lagging drive
        t_sim += 1 / 60.0
        if outputs.done_moving and not axis.in_motion and t_sim > 0.2:
            break

    assert outputs.done_moving == 1
    assert outputs.readback == pytest.approx(10.0, abs=1e-3)
    assert position == pytest.approx(0.010, abs=1e-5)


# -- configuration check against the live articulation -------------------

def test_articulation_descriptor_reaches_the_adapter(server):
    srv, plant = server
    post(srv.url, {
        "protocol": 1, "positions": {"x_axis": 0.0}, "t_sim": 0.0,
        "playing": True, "seq": 1,
        "articulation": {
            "prim": "/World/stage/base", "meters_per_unit": 1.0,
            "joints": [{"name": "x_axis", "type": "prismatic",
                        "lower": -0.025, "upper": 0.025}],
        },
    })
    art = plant.articulation
    assert art is not None
    assert art.prim == "/World/stage/base"
    assert art.joint("x_axis").joint_type == "prismatic"


def test_config_check_catches_a_joint_that_does_not_exist():
    from isaac_epics.config import AdapterConfig, AxisConfig
    from isaac_epics.protocol import Articulation, JointInfo
    from isaac_epics.validate import check, errors

    config = AdapterConfig(prefix="SIM:T:", axes=[
        AxisConfig(motor="m1", joint="x_axis", sim_units_per_egu=1e-3,
                   low_limit=-10, high_limit=10)])
    art = Articulation(joints=(JointInfo("joint_1", "prismatic", -0.02, 0.02),))
    bad = errors(check(config, art))
    assert bad and "does not exist" in bad[0].message
    assert "joint_1" in bad[0].message, "must name what IS available"


def test_config_check_catches_soft_limits_outside_the_joint():
    from isaac_epics.config import AdapterConfig, AxisConfig
    from isaac_epics.protocol import Articulation, JointInfo
    from isaac_epics.validate import check, errors

    config = AdapterConfig(prefix="SIM:T:", axes=[
        AxisConfig(motor="m1", joint="x_axis", sim_units_per_egu=1e-3,
                   low_limit=-50, high_limit=50)])
    art = Articulation(joints=(JointInfo("x_axis", "prismatic", -0.025, 0.025),))
    bad = errors(check(config, art))
    assert len(bad) == 2
    assert all("stall" in f.message for f in bad)


def test_config_check_catches_a_scaled_stage():
    from isaac_epics.config import default_config
    from isaac_epics.protocol import Articulation
    from isaac_epics.validate import check, errors

    art = Articulation(meters_per_unit=0.001)
    bad = errors(check(default_config(), art))
    assert any("1000" in f.message for f in bad)


def test_config_check_warns_when_nothing_described_it():
    from isaac_epics.config import default_config
    from isaac_epics.validate import check

    findings = check(default_config(), None)
    assert findings and findings[0].severity == "warning"
    assert "could not be checked" in findings[0].message


def test_matching_config_produces_no_findings():
    import math

    from isaac_epics.config import default_config
    from isaac_epics.protocol import Articulation, JointInfo
    from isaac_epics.validate import check

    art = Articulation(meters_per_unit=1.0, joints=(
        JointInfo("x_axis", "prismatic", -0.025, 0.025),
        JointInfo("y_axis", "prismatic", -0.025, 0.025),
        JointInfo("z_axis", "prismatic", 0.0, 0.05),
        JointInfo("theta_axis", "revolute", -math.pi, math.pi),
    ))
    assert check(default_config(), art) == []


def test_declared_pv_binding_rides_the_existing_descriptor():
    """A joint's epics:motorPV reaches the adapter over the normal protocol."""
    from isaac_epics.protocol import Articulation

    art = Articulation.from_json({
        "prim": "/World/cradle",
        "meters_per_unit": 1.0,
        "joints": [
            {"name": "chi_axis", "type": "revolute", "lower": -1.7, "upper": 1.7,
             "motor_pv": "SIM:CRADLE:m1"},
            {"name": "phi_axis", "type": "revolute", "lower": -3.1, "upper": 3.1},
        ],
    })
    assert art.joints[0].motor_pv == "SIM:CRADLE:m1"
    assert art.joints[1].motor_pv is None, "a joint may decline to declare one"


def test_a_binding_that_disagrees_with_the_config_is_an_error():
    """The point of putting the binding in the model is catching this.

    Two statements of the same fact -- one in the asset, one in the config --
    can disagree. A joint name typed into a config is an agreement nobody
    re-reads; a declared binding is checked on the first frame of every run.
    """
    from isaac_epics.protocol import Articulation
    from isaac_epics.validate import check, errors

    config = AdapterConfig(
        prefix="SIM:CRADLE:",
        axes=[AxisConfig(motor="m7", joint="chi_axis", low_limit=-90, high_limit=90)],
    )
    art = Articulation.from_json({
        "prim": "/World/cradle", "meters_per_unit": 1.0,
        "joints": [{"name": "chi_axis", "type": "revolute",
                    "lower": -1.75, "upper": 1.75, "motor_pv": "SIM:CRADLE:m1"}],
    })
    bad = errors(check(config, art))
    assert any("SIM:CRADLE:m1" in f.message and "SIM:CRADLE:m7" in f.message
               for f in bad), f"the mismatch was not reported: {bad}"


def test_a_binding_for_a_joint_nobody_drives_is_a_warning():
    from isaac_epics.protocol import Articulation
    from isaac_epics.validate import check

    config = AdapterConfig(
        prefix="SIM:CRADLE:",
        axes=[AxisConfig(motor="m1", joint="chi_axis", low_limit=-90, high_limit=90)],
    )
    art = Articulation.from_json({
        "prim": "/World/cradle", "meters_per_unit": 1.0,
        "joints": [
            {"name": "chi_axis", "type": "revolute", "lower": -1.75, "upper": 1.75,
             "motor_pv": "SIM:CRADLE:m1"},
            {"name": "phi_axis", "type": "revolute", "lower": -3.1, "upper": 3.1,
             "motor_pv": "SIM:CRADLE:m2"},
        ],
    })
    findings = check(config, art)
    assert any("phi_axis" in f.message and "SIM:CRADLE:m2" in f.message
               for f in findings)


def test_matching_bindings_produce_no_findings():
    """A model and a config that agree must stay silent."""
    from isaac_epics.protocol import Articulation
    from isaac_epics.validate import check

    config = AdapterConfig(
        prefix="SIM:CRADLE:",
        axes=[AxisConfig(motor="m1", joint="chi_axis",
                         low_limit=-90, high_limit=90, egu="deg",
                         sim_units_per_egu=math.pi / 180.0)],
    )
    art = Articulation.from_json({
        "prim": "/World/cradle", "meters_per_unit": 1.0,
        "joints": [{"name": "chi_axis", "type": "revolute",
                    "lower": -1.8, "upper": 1.8, "motor_pv": "SIM:CRADLE:m1"}],
    })
    assert check(config, art) == []


def test_a_stage_with_no_bindings_is_not_nagged_about_it():
    """Every stage authored before this existed declares nothing."""
    from isaac_epics.protocol import Articulation
    from isaac_epics.validate import check

    config = AdapterConfig(
        prefix="SIM:CRADLE:",
        axes=[AxisConfig(motor="m1", joint="chi_axis",
                         low_limit=-90, high_limit=90, egu="deg",
                         sim_units_per_egu=math.pi / 180.0)],
    )
    art = Articulation.from_json({
        "prim": "/World/cradle", "meters_per_unit": 1.0,
        "joints": [{"name": "chi_axis", "type": "revolute",
                    "lower": -1.8, "upper": 1.8}],
    })
    assert check(config, art) == []
