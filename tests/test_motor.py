"""Motor record semantics.

Section 6 of the design document: a naive IOC exposing only .VAL and .RBV
"demos beautifully and then breaks scans in ways that look like beamline
faults". These tests are the guard against that.
"""

import math

import pytest
from helpers import FakePlant, run

from isaac_epics.config import AxisConfig
from isaac_epics.motor import (
    MSTA_COMM_ERR,
    MSTA_DONE,
    MSTA_MINUS_LS,
    MSTA_PLUS_LS,
    MSTA_PROBLEM,
    MotorAxis,
    Phase,
    Severity,
)

# -- basic motion ---------------------------------------------------------

def test_move_reaches_target(axis, plant):
    assert axis.request_move(10.0)
    outputs = run(axis, plant)
    assert outputs.done_moving == 1
    assert outputs.readback == pytest.approx(10.0, abs=1e-6)
    assert outputs.missed == 0


def test_units_are_converted_on_the_wire(axis, plant):
    """PVs are millimetres; the joint is metres."""
    axis.request_move(10.0)
    run(axis, plant)
    assert plant.position == pytest.approx(0.010, abs=1e-9)


def test_rotary_axis_converts_degrees_to_radians():
    cfg = AxisConfig(motor="m4", joint="theta_axis", egu="deg",
                     sim_units_per_egu=math.pi / 180.0,
                     low_limit=-180.0, high_limit=180.0, velocity=90.0,
                     max_velocity=180.0, accel_time=0.1, retry_deadband=1e-2)
    axis = MotorAxis(cfg)
    plant = FakePlant(joint="theta_axis")
    axis.request_move(90.0)
    run(axis, plant, seconds=10)
    assert plant.position == pytest.approx(math.pi / 2, abs=1e-6)


def test_dmov_drops_immediately_and_rises_at_the_end(axis, plant):
    axis.request_move(10.0)
    first = axis.tick(plant.state())
    assert first.done_moving == 0, "DMOV must drop before the next poll"
    assert first.moving == 1
    plant.advance(first.target_sim)
    final = run(axis, plant)
    assert final.done_moving == 1
    assert final.moving == 0


def test_velocity_is_honoured(axis, plant):
    """A 10 mm move at 10 mm/s with 0.2 s ramps takes ~1.2 s of sim time."""
    axis.request_move(10.0)
    run(axis, plant, seconds=10)
    assert plant.t_sim == pytest.approx(1.2, abs=0.1)


def test_slower_velocity_takes_longer(axis, plant):
    axis.velocity = 2.0
    axis.request_move(10.0)
    run(axis, plant, seconds=30)
    assert plant.t_sim > 4.0


# -- limits ---------------------------------------------------------------

def test_move_beyond_high_limit_is_refused(axis, plant):
    assert axis.request_move(999.0) is False
    outputs = axis.tick(plant.state())
    assert outputs.limit_violation == 1
    assert outputs.setpoint == 0.0, "a refused move must not change .VAL"


def test_move_beyond_low_limit_is_refused(axis, plant):
    assert axis.request_move(-999.0) is False
    assert axis.tick(plant.state()).limit_violation == 1


def test_refused_move_does_not_clamp(axis, plant):
    """Silently clamping would record a position that was never reached."""
    axis.request_move(999.0)
    run(axis, plant, seconds=1)
    assert plant.position == pytest.approx(0.0)


def test_lvio_clears_on_a_valid_move(axis, plant):
    axis.request_move(999.0)
    assert axis.tick(plant.state()).limit_violation == 1
    axis.request_move(5.0)
    assert run(axis, plant).limit_violation == 0


def test_limit_switches_assert_at_the_ends(axis, plant):
    axis.request_move(50.0)
    outputs = run(axis, plant, seconds=30)
    assert outputs.high_limit_switch == 1
    assert outputs.status & MSTA_PLUS_LS


def test_low_limit_switch(axis, plant):
    axis.request_move(-50.0)
    outputs = run(axis, plant, seconds=30)
    assert outputs.low_limit_switch == 1
    assert outputs.status & MSTA_MINUS_LS


def test_nan_target_refused(axis):
    assert axis.request_move(float("nan")) is False


# -- pause and disconnect: the section 8 requirements ---------------------

def test_a_move_cannot_complete_while_paused(axis, plant):
    """The single most important test in this file.

    A paused simulation holds a frozen readback. A frozen readback inside the
    deadband is not evidence that a move finished.
    """
    axis.request_move(10.0)
    run(axis, plant, seconds=0.3, stop_when_done=False)
    plant.playing = False
    outputs = run(axis, plant, seconds=60, stop_when_done=False)
    assert outputs.done_moving == 0
    assert outputs.severity == Severity.MAJOR
    assert "paused" in outputs.alarm_message


def test_pause_freezes_rather_than_skips(axis, plant):
    """Resuming continues the ramp; it does not snap to the destination."""
    axis.request_move(10.0)
    run(axis, plant, seconds=0.3, stop_when_done=False)
    mid = plant.position
    plant.playing = False
    run(axis, plant, seconds=10, stop_when_done=False)
    assert plant.position == pytest.approx(mid), "joints must not move while paused"
    plant.playing = True
    outputs = run(axis, plant, seconds=10)
    assert outputs.done_moving == 1
    assert outputs.readback == pytest.approx(10.0, abs=1e-6)


def test_paused_while_idle_is_only_a_minor_alarm(axis, plant):
    plant.playing = False
    outputs = axis.tick(plant.state())
    assert outputs.done_moving == 1, "nothing is moving, so .DMOV is honest"
    assert outputs.severity == Severity.MINOR


def test_disconnect_marks_everything_invalid(axis, plant):
    run(axis, plant, seconds=0.2, stop_when_done=False)
    plant.connected = False
    outputs = axis.tick(plant.state())
    assert outputs.severity == Severity.INVALID
    assert outputs.done_moving == 0, "a stale link must not assert .DMOV"
    assert outputs.status & MSTA_PROBLEM
    assert outputs.status & MSTA_COMM_ERR
    assert outputs.target_sim is None


def test_disconnect_message_names_the_cause(axis, plant):
    plant.connected = False
    assert "Isaac Sim" in axis.tick(plant.state()).alarm_message


def test_reconnect_recovers(axis, plant):
    plant.connected = False
    axis.tick(plant.state())
    plant.connected = True
    axis.request_move(5.0)
    outputs = run(axis, plant, seconds=10)
    assert outputs.severity == Severity.NO_ALARM
    assert outputs.done_moving == 1


# -- a lagging plant ------------------------------------------------------

def test_converges_against_a_lagging_drive():
    """A PhysX drive converges asymptotically; the ramp ending is not arrival."""
    cfg = AxisConfig(motor="m1", joint="x_axis", sim_units_per_egu=1e-3,
                     low_limit=-50, high_limit=50, velocity=10.0,
                     max_velocity=50.0, accel_time=0.2, retry_deadband=1e-3)
    axis = MotorAxis(cfg)
    plant = FakePlant(tau=0.05)
    axis.request_move(10.0)
    outputs = run(axis, plant, seconds=30)
    assert outputs.done_moving == 1
    assert outputs.missed == 0
    assert abs(outputs.readback - 10.0) <= cfg.retry_deadband


def test_unreachable_target_reports_missed_rather_than_hanging():
    """A stuck joint must raise .MISS, not block forever."""
    cfg = AxisConfig(motor="m1", joint="x_axis", sim_units_per_egu=1e-3,
                     low_limit=-50, high_limit=50, velocity=10.0,
                     max_velocity=50.0, accel_time=0.2, retry_deadband=1e-6,
                     max_retries=1, settle_timeout=0.5, stall_time=0.1)
    axis = MotorAxis(cfg)
    plant = FakePlant()
    plant.advance = lambda target: setattr(plant, "t_sim", plant.t_sim + plant.dt)
    axis.request_move(10.0)
    outputs = run(axis, plant, seconds=30)
    assert outputs.done_moving == 1, "must not hang"
    assert outputs.missed == 1
    assert outputs.severity == Severity.MINOR


# -- stop -----------------------------------------------------------------

def test_stop_halts_where_the_simulation_is(axis, plant):
    axis.request_move(40.0)
    run(axis, plant, seconds=1.0, stop_when_done=False)
    here = axis.readback
    axis.stop()
    outputs = run(axis, plant, seconds=5)
    assert outputs.done_moving == 1
    assert outputs.readback == pytest.approx(here, abs=0.5)
    assert outputs.readback < 40.0


def test_stop_does_not_creep_forward(axis, plant):
    axis.request_move(40.0)
    run(axis, plant, seconds=1.0, stop_when_done=False)
    axis.stop()
    run(axis, plant, seconds=2)
    settled = plant.position
    run(axis, plant, seconds=3)
    assert plant.position == pytest.approx(settled, abs=1e-9)


# -- dial / user coordinates ---------------------------------------------

def test_offset_shifts_user_coordinates(axis, plant):
    axis.offset = 100.0
    axis.high_limit = 150.0
    axis.low_limit = 50.0
    axis.request_move(110.0)
    run(axis, plant, seconds=10)
    assert plant.position == pytest.approx(0.010, abs=1e-6), "dial = user - offset"
    assert axis.readback == pytest.approx(110.0, abs=1e-5)


def test_negative_direction_inverts_the_joint(axis, plant):
    axis.direction = 1  # .DIR = Neg
    axis.request_move(10.0)
    run(axis, plant, seconds=10)
    assert plant.position == pytest.approx(-0.010, abs=1e-6)


def test_negative_direction_swaps_dial_limits(axis):
    axis.direction = 1
    axis.high_limit = 50.0
    axis.low_limit = -20.0
    assert axis.dial_high_limit == pytest.approx(20.0)
    assert axis.dial_low_limit == pytest.approx(-50.0)


def test_set_mode_redefines_position_without_moving(axis, plant):
    axis.request_move(10.0)
    run(axis, plant, seconds=10)
    before = plant.position
    axis.set_mode = 1
    axis.request_move(0.0)          # "this position is now zero"
    run(axis, plant, seconds=2)
    assert plant.position == pytest.approx(before, abs=1e-9), "SET must not move"
    assert axis.readback == pytest.approx(0.0, abs=1e-5)
    assert axis.offset == pytest.approx(-10.0, abs=1e-4)


def test_frozen_offset_redefines_dial_instead(axis, plant):
    axis.request_move(10.0)
    run(axis, plant, seconds=10)
    axis.set_mode = 1
    axis.freeze_offset = 1
    axis.request_move(0.0)
    assert axis.offset == 0.0, "a frozen offset must not change"


# -- backlash -------------------------------------------------------------

def test_backlash_approaches_from_a_consistent_direction(plant):
    cfg = AxisConfig(motor="m1", joint="x_axis", sim_units_per_egu=1e-3,
                     low_limit=-50, high_limit=50, velocity=10.0,
                     max_velocity=50.0, accel_time=0.1, retry_deadband=1e-3,
                     backlash_distance=1.0, backlash_velocity=2.0)
    axis = MotorAxis(cfg)
    axis.request_move(10.0)                       # positive move, no take-up
    run(axis, plant, seconds=20)
    assert axis.phase is Phase.IDLE

    axis.request_move(0.0)                        # negative: must take up
    seen_overshoot = False
    for _ in range(3000):
        outputs = axis.tick(plant.state())
        plant.advance(outputs.target_sim)
        if axis.readback < -0.5:
            seen_overshoot = True
        if outputs.done_moving and not axis.in_motion:
            break
    assert seen_overshoot, "a negative move must undershoot then approach upward"
    assert axis.readback == pytest.approx(0.0, abs=1e-3)


# -- tweak and home -------------------------------------------------------

def test_tweak_is_relative(axis, plant):
    axis.request_move(5.0)
    run(axis, plant, seconds=10)
    axis.request_relative(2.0)
    outputs = run(axis, plant, seconds=10)
    assert outputs.readback == pytest.approx(7.0, abs=1e-5)


def test_home_returns_to_dial_zero(axis, plant):
    axis.request_move(20.0)
    run(axis, plant, seconds=20)
    axis.home(forward=False)
    outputs = run(axis, plant, seconds=20)
    assert outputs.readback == pytest.approx(0.0, abs=1e-5)
    assert outputs.at_home == 1


# -- simulation clock handling -------------------------------------------

def test_sim_clock_going_backwards_does_not_integrate(axis, plant):
    axis.request_move(10.0)
    run(axis, plant, seconds=0.3, stop_when_done=False)
    before = plant.position
    plant.t_sim -= 100.0          # the stage was reloaded
    axis.tick(plant.state())
    assert plant.position == pytest.approx(before)


def test_large_sim_time_gap_is_clamped(axis, plant):
    """A stall while loading a stage must not teleport the setpoint."""
    axis.request_move(40.0)
    axis.tick(plant.state())
    plant.t_sim += 60.0           # 60 seconds of frozen renderer
    outputs = axis.tick(plant.state())
    assert outputs.target_sim is not None
    assert abs(axis.config.to_egu(outputs.target_sim)) < 5.0


def test_msta_done_bit_tracks_dmov(axis, plant):
    outputs = axis.tick(plant.state())
    assert outputs.status & MSTA_DONE
    axis.request_move(10.0)
    outputs = axis.tick(plant.state())
    assert not (outputs.status & MSTA_DONE)


# -- the pause gate, isolated from the sim-clock gate ---------------------

def test_pause_gate_holds_even_when_the_sim_clock_keeps_running(axis, plant):
    """Two independent mechanisms stop a paused sim completing a move.

    The obvious one is that a paused simulation's clock stops, so no
    trajectory time elapses. But that relies on the client freezing ``t_sim``
    when it pauses, which is a property of the client, not of this adapter.
    A client that reports ``playing=False`` while its clock keeps advancing --
    a wall-clock-driven client, or simply a buggy one -- must still not be
    able to complete a move.

    This test removes the clock protection so only the ``playing`` gate is
    left standing.
    """
    axis.request_move(10.0)
    run(axis, plant, seconds=0.3, stop_when_done=False)

    plant.playing = False
    # Keep the readback frozen but let the clock run, exactly as a client
    # that pauses physics without pausing its clock would report.
    for _ in range(4000):
        plant.t_sim += plant.dt
        outputs = axis.tick(plant.state())
        assert outputs.done_moving == 0, "a paused sim must never satisfy a move"
    assert outputs.severity == Severity.MAJOR


def test_frozen_clock_alone_also_blocks_completion(axis, plant):
    """The complementary case: client claims to be playing, clock is stuck."""
    axis.request_move(10.0)
    run(axis, plant, seconds=0.3, stop_when_done=False)
    frozen = plant.t_sim
    for _ in range(4000):
        plant.t_sim = frozen          # renderer wedged, still reporting playing
        outputs = axis.tick(plant.state())
    assert outputs.done_moving == 0


# -- exponential convergence with a steady-state floor -------------------

class SaggingPlant(FakePlant):
    """A drive that converges exponentially onto a floor it never leaves.

    This is what a gravity-loaded PhysX force drive does: it approaches the
    commanded position asymptotically and settles a fixed distance short,
    equal to load / stiffness. Measured on Sentosa's z axis as 0.144 mm.
    """

    def __init__(self, sag_sim, **kw):
        super().__init__(**kw)
        self.sag_sim = sag_sim

    def advance(self, target_sim):
        if not (self.playing and self.connected):
            return
        self.t_sim += self.dt
        if target_sim is None:
            return
        import math
        alpha = 1.0 - math.exp(-self.dt / max(self.tau, 1e-6))
        # The drive chases a point `sag` short of the commanded target.
        effective = target_sim - math.copysign(self.sag_sim, target_sim or 1.0)
        self.position += (effective - self.position) * alpha


def test_dmov_rises_on_entering_the_deadband_not_at_the_floor():
    """.DMOV means "inside .RDBD", not "finished settling".

    A finite-stiffness drive keeps creeping toward its floor after the move
    is complete, so the deadband -- not the floor -- sets how precisely a
    scan knows where the axis is. Measured on Sentosa: z entered a 0.5 mm
    deadband at 0.367 mm of error and went on settling to 0.144 mm. That is
    correct motor-record behaviour, and the reason to size .RDBD against the
    position uncertainty a scan can tolerate rather than as loosely as the
    drive allows.
    """
    cfg = AxisConfig(
        motor="m3", joint="x_axis", egu="mm", sim_units_per_egu=1e-3,
        low_limit=1.0, high_limit=49.0, velocity=2.0, max_velocity=10.0,
        accel_time=0.2, retry_deadband=0.4, settle_timeout=30.0, stall_time=0.3,
    )
    axis = MotorAxis(cfg)
    # tau=1.0 against a 1/60 s tick closes ~1.7% of the remaining error per
    # step, matching the slow tail the real z drive shows.
    plant = SaggingPlant(sag_sim=0.144e-3, tau=1.0)    # 0.144 mm floor

    axis.request_move(25.0)
    outputs = run(axis, plant, seconds=200)

    assert outputs.done_moving == 1
    assert outputs.missed == 0, "the floor is inside the deadband, so this is not a MISS"
    error = abs(outputs.readback - 25.0)
    assert error <= cfg.retry_deadband, "DMOV must not rise outside .RDBD"
    assert error > 0.144, "cannot be closer than the drive's own floor"


def test_tightening_the_deadband_tightens_dmov_position():
    """The deadband, not the drive, bounds how well a scan knows the position."""
    errors = {}
    for deadband in (0.4, 0.2):
        cfg = AxisConfig(
            motor="m3", joint="x_axis", egu="mm", sim_units_per_egu=1e-3,
            low_limit=1.0, high_limit=49.0, velocity=2.0, max_velocity=10.0,
            accel_time=0.2, retry_deadband=deadband, settle_timeout=30.0,
            stall_time=0.3,
        )
        axis = MotorAxis(cfg)
        plant = SaggingPlant(sag_sim=0.144e-3, tau=1.0)
        axis.request_move(25.0)
        outputs = run(axis, plant, seconds=200)
        assert outputs.done_moving == 1 and outputs.missed == 0
        errors[deadband] = abs(outputs.readback - 25.0)
    assert errors[0.2] < errors[0.4]


def test_a_floor_outside_the_deadband_still_reports_missed():
    """If the drive cannot physically reach the deadband, say so -- don't hang."""
    cfg = AxisConfig(
        motor="m3", joint="x_axis", egu="mm", sim_units_per_egu=1e-3,
        low_limit=1.0, high_limit=49.0, velocity=5.0, max_velocity=10.0,
        accel_time=0.2, retry_deadband=0.01, settle_timeout=5.0, stall_time=0.3,
        max_retries=1,
    )
    axis = MotorAxis(cfg)
    plant = SaggingPlant(sag_sim=0.5e-3, tau=0.05)     # 0.5 mm floor, deadband 0.01
    axis.request_move(25.0)
    outputs = run(axis, plant, seconds=120)
    assert outputs.done_moving == 1, "must not hang"
    assert outputs.missed == 1
