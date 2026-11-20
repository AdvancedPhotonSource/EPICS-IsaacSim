"""Plant seam behaviour, including staleness and thread safety."""

import threading

import pytest

from isaac_epics.plant import LoopbackPlant, SharedPlant
from isaac_epics.protocol import SimFrame


class Clock:
    def __init__(self):
        self.now = 1000.0

    def __call__(self):
        return self.now


def frame(**kw):
    defaults = dict(positions={"x_axis": 1.0}, t_sim=5.0, playing=True, seq=1)
    defaults.update(kw)
    return SimFrame(**defaults)


def test_starts_disconnected():
    plant = SharedPlant()
    state = plant.get_state()
    assert state.connected is False
    assert state.link_state == "disconnected"
    assert state.age == float("inf")


def test_frame_connects():
    clock = Clock()
    plant = SharedPlant(stale_timeout=1.0, clock=clock)
    plant.submit_frame(frame())
    state = plant.get_state()
    assert state.connected
    assert state.link_state == "running"
    assert state.positions == {"x_axis": 1.0}


def test_goes_stale_after_the_timeout():
    clock = Clock()
    plant = SharedPlant(stale_timeout=1.0, clock=clock)
    plant.submit_frame(frame())
    clock.now += 0.9
    assert plant.get_state().connected
    clock.now += 0.2
    state = plant.get_state()
    assert not state.connected
    assert state.link_state == "disconnected"


def test_stale_link_is_never_reported_as_playing():
    clock = Clock()
    plant = SharedPlant(stale_timeout=1.0, clock=clock)
    plant.submit_frame(frame(playing=True))
    clock.now += 5.0
    state = plant.get_state()
    assert state.playing is False, "a vanished sim must not be reported as playing"
    assert state.stepping is False


def test_paused_is_distinct_from_disconnected():
    clock = Clock()
    plant = SharedPlant(stale_timeout=1.0, clock=clock)
    plant.submit_frame(frame(playing=False))
    state = plant.get_state()
    assert state.connected
    assert state.link_state == "paused"
    assert state.stepping is False


def test_targets_round_trip():
    plant = SharedPlant()
    plant.set_targets({"x_axis": 0.5})
    assert plant.submit_frame(frame())["x_axis"] == 0.5


def test_full_target_map_is_returned_every_frame():
    """A delta protocol would leave a dropped response holding stale targets."""
    plant = SharedPlant()
    plant.set_targets({"x_axis": 0.5, "y_axis": 0.25})
    returned = plant.submit_frame(frame())
    assert returned == {"x_axis": 0.5, "y_axis": 0.25}
    assert plant.submit_frame(frame(seq=2)) == returned


def test_sequence_regression_is_tolerated():
    plant = SharedPlant()
    plant.submit_frame(frame(seq=100))
    plant.submit_frame(frame(seq=1))      # the client restarted
    assert plant.get_state().seq == 1


def test_non_monotonic_clock_does_not_produce_negative_age():
    clock = Clock()
    plant = SharedPlant(clock=clock)
    plant.submit_frame(frame())
    clock.now -= 10.0
    assert plant.get_state().age >= 0.0


def test_invalid_stale_timeout_rejected():
    with pytest.raises(ValueError):
        SharedPlant(stale_timeout=0.0)


def test_concurrent_access_is_safe():
    """The HTTP thread writes frames while the IOC thread reads state."""
    plant = SharedPlant()
    errors = []
    stop = threading.Event()

    def writer():
        try:
            n = 0
            while not stop.is_set():
                n += 1
                plant.submit_frame(frame(seq=n, positions={"x_axis": n * 0.001}))
        except Exception as exc:  # noqa: BLE001
            errors.append(exc)

    def reader():
        try:
            while not stop.is_set():
                state = plant.get_state()
                assert set(state.positions) <= {"x_axis"}
                plant.set_targets({"x_axis": 0.1})
        except Exception as exc:  # noqa: BLE001
            errors.append(exc)

    threads = [threading.Thread(target=writer) for _ in range(2)]
    threads += [threading.Thread(target=reader) for _ in range(2)]
    for t in threads:
        t.start()
    threading.Event().wait(0.3)
    stop.set()
    for t in threads:
        t.join(timeout=5)
    assert errors == []
    assert plant.get_state().frames > 0


def test_loopback_plant_is_marked_as_a_test_double():
    plant = LoopbackPlant()
    plant.set_targets({"j": 1.0})
    assert plant.get_state().positions["j"] == 1.0
    assert "never use it" in LoopbackPlant.__doc__.lower()
