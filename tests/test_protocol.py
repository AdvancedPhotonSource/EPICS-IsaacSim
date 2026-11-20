"""Protocol validation tests.

The theme: a frame we cannot fully trust is rejected outright, never partially
applied. A half-applied frame is indistinguishable from fabricated telemetry.
"""

import math

import pytest

from isaac_epics.protocol import (
    MAX_JOINTS,
    PROTOCOL_VERSION,
    ProtocolError,
    SimFrame,
    TargetFrame,
    describe_link_state,
    parse_target_frame,
)


def good(**overrides):
    frame = {"protocol": PROTOCOL_VERSION, "positions": {"x_axis": 1.5},
             "t_sim": 10.0, "playing": True, "seq": 3}
    frame.update(overrides)
    return frame


def test_round_trip():
    frame = SimFrame.from_json(good())
    assert frame.positions == {"x_axis": 1.5}
    assert frame.t_sim == 10.0
    assert frame.playing is True
    assert frame.seq == 3


def test_playing_is_mandatory():
    # Section 8: a paused sim reporting a frozen readback as a settled move is
    # the failure most likely to quietly corrupt a scan. It may not be omitted.
    body = good()
    del body["playing"]
    with pytest.raises(ProtocolError, match="playing"):
        SimFrame.from_json(body)


@pytest.mark.parametrize("bad_value", [float("nan"), float("inf"), float("-inf")])
def test_non_finite_positions_rejected(bad_value):
    # NaN propagates silently: `nan <= deadband` is False, so a NaN readback
    # would make a move never complete.
    with pytest.raises(ProtocolError):
        SimFrame.from_json(good(positions={"x_axis": bad_value}))


def test_non_finite_t_sim_rejected():
    with pytest.raises(ProtocolError):
        SimFrame.from_json(good(t_sim=float("nan")))


def test_booleans_are_not_numbers():
    with pytest.raises(ProtocolError):
        SimFrame.from_json(good(positions={"x_axis": True}))


def test_wrong_protocol_version_rejected():
    with pytest.raises(ProtocolError, match="unsupported protocol"):
        SimFrame.from_json(good(protocol=999))


def test_too_many_joints_rejected():
    positions = {f"j{i}": 0.0 for i in range(MAX_JOINTS + 1)}
    with pytest.raises(ProtocolError, match="too many joints"):
        SimFrame.from_json(good(positions=positions))


def test_negative_seq_rejected():
    with pytest.raises(ProtocolError, match="seq"):
        SimFrame.from_json(good(seq=-1))


def test_bool_seq_rejected():
    with pytest.raises(ProtocolError):
        SimFrame.from_json(good(seq=True))


@pytest.mark.parametrize("body", [[], "x", 42, None])
def test_non_object_rejected(body):
    with pytest.raises(ProtocolError):
        SimFrame.from_json(body)


def test_empty_joint_name_rejected():
    with pytest.raises(ProtocolError):
        SimFrame.from_json(good(positions={"": 1.0}))


def test_target_frame_serialises():
    payload = TargetFrame(targets={"x_axis": 0.5}).to_json()
    assert payload["protocol"] == PROTOCOL_VERSION
    assert payload["targets"] == {"x_axis": 0.5}


def test_client_rejects_non_finite_targets():
    # The client trusts the adapter no further than the adapter trusts the
    # client: an infinite target applied to a joint is a physics explosion.
    with pytest.raises(ProtocolError):
        parse_target_frame({"protocol": 1, "targets": {"x": float("inf")}})


def test_client_accepts_absent_targets():
    assert parse_target_frame({"protocol": 1}) == {}


def test_link_state_has_exactly_three_values():
    assert describe_link_state(False, True) == "disconnected"
    assert describe_link_state(False, False) == "disconnected"
    assert describe_link_state(True, False) == "paused"
    assert describe_link_state(True, True) == "running"


def test_disconnected_dominates_playing():
    # An unknown link state must report as disconnected, never as something
    # more reassuring.
    assert describe_link_state(False, None) == "disconnected"


def test_integers_accepted_as_floats():
    frame = SimFrame.from_json(good(positions={"x_axis": 2}))
    assert isinstance(frame.positions["x_axis"], float)
    assert not math.isnan(frame.positions["x_axis"])
