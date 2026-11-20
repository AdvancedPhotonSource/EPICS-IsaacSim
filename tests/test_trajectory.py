"""Trapezoidal profile tests."""

import math

import pytest

from isaac_epics.trajectory import plan


def test_trapezoid_duration_matches_analytic():
    # 100 units at 10 u/s with 0.2s ramps: 2*0.2 accel + 9.8 cruise.
    profile = plan(0.0, 100.0, velocity=10.0, accel_time=0.2)
    assert profile.duration == pytest.approx(10.2, abs=1e-9)
    assert profile.t_cruise == pytest.approx(9.8, abs=1e-9)


def test_endpoints_are_exact():
    profile = plan(3.0, 17.0, velocity=5.0, accel_time=0.3)
    assert profile.position(0.0) == 3.0
    assert profile.position(profile.duration) == pytest.approx(17.0, abs=1e-12)
    assert profile.position(profile.duration * 10) == 17.0


def test_profile_is_symmetric_about_midpoint():
    profile = plan(0.0, 100.0, velocity=10.0, accel_time=0.2)
    assert profile.position(profile.duration / 2) == pytest.approx(50.0, abs=1e-9)


def test_monotonic_and_never_overshoots():
    profile = plan(0.0, 42.0, velocity=7.0, accel_time=0.4)
    previous = -math.inf
    for i in range(2001):
        value = profile.position(profile.duration * i / 2000)
        assert value >= previous - 1e-9
        assert value <= 42.0 + 1e-9
        previous = value


def test_short_move_degenerates_to_triangle():
    profile = plan(0.0, 0.01, velocity=10.0, accel_time=0.2)
    assert profile.t_cruise == 0.0
    assert profile.velocity < 10.0  # never reaches cruise
    assert profile.position(profile.duration) == pytest.approx(0.01)


def test_negative_direction():
    profile = plan(5.0, -5.0, velocity=10.0, accel_time=0.2)
    assert profile.sign == -1.0
    assert profile.position(profile.duration) == pytest.approx(-5.0)
    assert profile.position(profile.duration / 2) == pytest.approx(0.0, abs=1e-9)


def test_zero_distance_is_immediately_done():
    profile = plan(3.0, 3.0, velocity=10.0, accel_time=0.2)
    assert profile.duration == 0.0
    assert profile.done(0.0)


def test_zero_accel_time_is_pure_cruise():
    profile = plan(0.0, 10.0, velocity=10.0, accel_time=0.0)
    assert profile.duration == pytest.approx(1.0)
    assert profile.position(0.5) == pytest.approx(5.0)


def test_base_velocity_sets_a_floor():
    slow = plan(0.0, 10.0, velocity=1.0, accel_time=0.0)
    floored = plan(0.0, 10.0, velocity=1.0, accel_time=0.0, base_velocity=5.0)
    assert floored.duration < slow.duration


def test_speed_starts_and_ends_at_zero():
    profile = plan(0.0, 100.0, velocity=10.0, accel_time=0.2)
    assert profile.speed(0.0) == 0.0
    assert profile.speed(profile.duration) == 0.0
    assert profile.speed(profile.duration / 2) == pytest.approx(10.0, abs=1e-9)


def test_rejects_nonsense_parameters():
    with pytest.raises(ValueError):
        plan(0.0, 1.0, velocity=0.0, accel_time=0.1)
    with pytest.raises(ValueError):
        plan(0.0, 1.0, velocity=1.0, accel_time=-1.0)
