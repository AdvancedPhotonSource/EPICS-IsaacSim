"""Trapezoidal velocity profile generation.

Why the adapter generates the trajectory rather than handing Isaac Sim a
destination:

* Section 7.6 requires that Isaac Sim know *nothing* about EPICS, and section
  7.5 keeps all motor semantics in the adapter.  ``.VELO`` and ``.ACCL`` are
  motor-record semantics.  If the adapter sent only a final destination, those
  fields would be decorative -- readable, writable, and with no effect on the
  motion, which is worse than not serving them.
* One of the stated reasons for this project is reproducing a collision by
  replaying a scan.  A joint teleported to its destination passes through no
  intermediate configuration and so collides with nothing.  The ramp *is* the
  fidelity.

The profile is the classic trapezoid: accelerate at ``a = velocity /
accel_time`` to ``velocity``, cruise, decelerate symmetrically.  When the move
is too short to reach cruise velocity it degenerates to a triangle.  Jerk is
not modelled -- the real motor record does not model it either.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

#: Moves shorter than this fraction of the deadband are treated as already
#: complete, avoiding a degenerate zero-distance profile.
_EPSILON = 1e-12


@dataclass(frozen=True)
class Trapezoid:
    """A time-parameterised 1-D motion profile from ``start`` to ``end``.

    All quantities are in engineering units and seconds.  Instances are
    immutable; evaluation is a pure function of elapsed time, which makes the
    state machine that uses them trivially restartable and testable.
    """

    start: float
    end: float
    velocity: float
    accel: float
    t_accel: float
    t_cruise: float
    duration: float

    @property
    def distance(self) -> float:
        return abs(self.end - self.start)

    @property
    def sign(self) -> float:
        return math.copysign(1.0, self.end - self.start) if self.distance > _EPSILON else 0.0

    def position(self, t: float) -> float:
        """Commanded position at ``t`` seconds after the move began."""
        if t <= 0.0:
            return self.start
        if t >= self.duration:
            return self.end
        a, ta, tc = self.accel, self.t_accel, self.t_cruise
        if t < ta:
            travelled = 0.5 * a * t * t
        elif t < ta + tc:
            travelled = 0.5 * a * ta * ta + self.velocity * (t - ta)
        else:
            td = t - ta - tc
            peak = a * ta
            travelled = (
                0.5 * a * ta * ta
                + self.velocity * tc
                + peak * td
                - 0.5 * a * td * td
            )
        return self.start + self.sign * min(travelled, self.distance)

    def speed(self, t: float) -> float:
        """Instantaneous speed magnitude at ``t``. Diagnostics only."""
        if t <= 0.0 or t >= self.duration:
            return 0.0
        a, ta, tc = self.accel, self.t_accel, self.t_cruise
        if t < ta:
            return a * t
        if t < ta + tc:
            return a * ta
        return max(0.0, a * ta - a * (t - ta - tc))

    def done(self, t: float) -> bool:
        return t >= self.duration


def plan(
    start: float,
    end: float,
    velocity: float,
    accel_time: float,
    base_velocity: float = 0.0,
) -> Trapezoid:
    """Plan a move from ``start`` to ``end``.

    Args:
        start: Current commanded position, engineering units.
        end: Destination, engineering units.
        velocity: Cruise speed (``.VELO``), EGU/s.  Must be positive.
        accel_time: Seconds from rest to ``velocity`` (``.ACCL``).  Zero means
            a step profile -- instantaneous acceleration, which is what the
            motor record does with ``.ACCL`` of zero.
        base_velocity: ``.VBAS``, a floor on the cruise speed.  A motor record
            will not run slower than its base velocity.

    Returns:
        A :class:`Trapezoid`.  A zero-distance move yields a zero-duration
        profile that is immediately ``done``.
    """
    if velocity <= 0:
        raise ValueError("velocity must be positive")
    if accel_time < 0:
        raise ValueError("accel_time must not be negative")

    cruise = max(float(velocity), float(base_velocity), _EPSILON)
    distance = abs(end - start)

    if distance <= _EPSILON:
        return Trapezoid(start, end, cruise, 0.0, 0.0, 0.0, 0.0)

    if accel_time <= 0.0:
        # Instantaneous acceleration: pure cruise.
        return Trapezoid(start, end, cruise, 0.0, 0.0, distance / cruise, distance / cruise)

    accel = cruise / accel_time
    d_accel = 0.5 * accel * accel_time * accel_time

    if 2.0 * d_accel >= distance:
        # Triangular: never reaches cruise speed.
        t_accel = math.sqrt(distance / accel)
        return Trapezoid(
            start=start, end=end, velocity=accel * t_accel, accel=accel,
            t_accel=t_accel, t_cruise=0.0, duration=2.0 * t_accel,
        )

    t_cruise = (distance - 2.0 * d_accel) / cruise
    return Trapezoid(
        start=start, end=end, velocity=cruise, accel=accel,
        t_accel=accel_time, t_cruise=t_cruise, duration=2.0 * accel_time + t_cruise,
    )
