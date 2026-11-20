"""Shared test fixtures."""

import pytest
from helpers import FakePlant

from isaac_epics.config import AxisConfig
from isaac_epics.motor import MotorAxis


@pytest.fixture
def linear_cfg():
    """A millimetre stage on a joint reported in metres."""
    return AxisConfig(
        motor="m1", joint="x_axis", egu="mm", sim_units_per_egu=1e-3,
        low_limit=-50.0, high_limit=50.0, velocity=10.0, max_velocity=50.0,
        accel_time=0.2, retry_deadband=1e-3, precision=4,
    )


@pytest.fixture
def axis(linear_cfg):
    return MotorAxis(linear_cfg)


@pytest.fixture
def plant():
    return FakePlant()
