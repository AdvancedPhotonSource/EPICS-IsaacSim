"""The plant seam (architecture decision 7.5).

The motor logic never talks to a transport.  It talks to a :class:`Plant`::

    plant.set_targets({"x_axis": 0.042})
    state = plant.get_state()

Today the only implementation is :class:`SharedPlant`, fed by the HTTP server
in :mod:`isaac_epics.transport`.  If the multi-IOC condition described in
section 7.4 ever arrives, the same interface is reimplemented as a client to a
bridge daemon: the motor-record code does not change, and Isaac Sim does not
change, because it still talks to exactly one endpoint.  That is the whole
point of paying for this abstraction on day one -- it is what makes "start
simple" honest rather than a thing said before a rewrite.

Threading
---------
:class:`SharedPlant` is the boundary between two threads: the HTTP server
thread writes frames and reads targets; the asyncio thread running the IOC
reads state and writes targets.  Every access is guarded by one lock held for
the shortest possible span, and :class:`PlantState` is an immutable snapshot so
callers can never observe a half-updated frame.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, Mapping, Optional

from .protocol import SimFrame, describe_link_state


@dataclass(frozen=True)
class PlantState:
    """An immutable snapshot of what the simulation last told us.

    Attributes:
        positions: Joint -> measured position in simulation units.
        t_sim: Simulation clock from the last frame.
        playing: Whether the simulation was stepping as of the last frame.
        connected: Whether a frame has arrived within the staleness timeout.
            ``False`` means the readback below is *old*, and the adapter must
            say so loudly rather than present it as live (section 8).
        age: Seconds since the last frame arrived, by wall clock.
        seq: Last frame sequence number seen.
        frames: Total frames accepted since start, for diagnostics.
    """

    positions: Mapping[str, float] = field(default_factory=dict)
    t_sim: float = 0.0
    playing: bool = False
    connected: bool = False
    age: float = float("inf")
    seq: int = 0
    frames: int = 0

    @property
    def link_state(self) -> str:
        """``"disconnected"`` / ``"paused"`` / ``"running"`` -- see section 8."""
        return describe_link_state(self.connected, self.playing)

    @property
    def stepping(self) -> bool:
        """True only when the simulation is genuinely advancing.

        Motor logic must gate on this, not on ``playing`` alone: a paused
        simulation and a vanished one both mean "this readback is not
        evidence of anything", and neither may be allowed to satisfy a move.
        """
        return self.connected and self.playing


class Plant:
    """Interface between motor logic and whatever is driving the geometry."""

    def set_targets(self, targets: Mapping[str, float]) -> None:
        """Command instantaneous joint setpoints, in simulation units."""
        raise NotImplementedError

    def get_state(self) -> PlantState:
        """Return an immutable snapshot of the most recent simulation state."""
        raise NotImplementedError


class SharedPlant(Plant):
    """Thread-safe plant backed by frames posted from Isaac Sim.

    Args:
        stale_timeout: Seconds without a frame after which the link is
            reported disconnected.  Should be a comfortable multiple of the
            client's tick period -- Isaac Sim's frame rate is not guaranteed,
            so a timeout near the frame period will flap.
        clock: Injectable wall clock, for tests.
    """

    def __init__(self, stale_timeout: float = 1.0, clock: Callable[[], float] = time.monotonic):
        if stale_timeout <= 0:
            raise ValueError("stale_timeout must be positive")
        self._stale_timeout = float(stale_timeout)
        self._clock = clock
        self._lock = threading.Lock()

        self._positions: Dict[str, float] = {}
        self._targets: Dict[str, float] = {}
        self._t_sim = 0.0
        self._playing = False
        self._seq = 0
        self._frames = 0
        self._last_rx: Optional[float] = None

    # -- driven by the transport (HTTP server thread) ---------------------

    def submit_frame(self, frame: SimFrame) -> Dict[str, float]:
        """Record a frame from Isaac Sim and return the targets it should apply.

        Returns the full target map rather than a delta.  A delta protocol
        would make a dropped response leave the simulation holding a stale
        setpoint with no way to notice; sending the whole map every frame
        costs a few hundred bytes and makes every exchange self-contained.
        """
        now = self._clock()
        with self._lock:
            if frame.seq and frame.seq < self._seq:
                # The client restarted, or a response was reordered.  Keep the
                # newest state but do not silently treat the regression as
                # normal -- the counter is reset so staleness stays honest.
                self._seq = 0
            self._positions = dict(frame.positions)
            self._t_sim = frame.t_sim
            self._playing = frame.playing
            self._seq = frame.seq
            self._frames += 1
            self._last_rx = now
            return dict(self._targets)

    # -- Plant interface ---------------------------------------------------

    def set_targets(self, targets: Mapping[str, float]) -> None:
        with self._lock:
            self._targets.update(targets)

    def get_state(self) -> PlantState:
        now = self._clock()
        with self._lock:
            if self._last_rx is None:
                age = float("inf")
            else:
                # A non-monotonic clock, or a frame timestamped microscopically
                # in the future, must not produce a negative age.
                age = max(0.0, now - self._last_rx)
            connected = age <= self._stale_timeout
            return PlantState(
                positions=dict(self._positions),
                t_sim=self._t_sim,
                playing=self._playing and connected,
                connected=connected,
                age=age,
                seq=self._seq,
                frames=self._frames,
            )

    # -- diagnostics -------------------------------------------------------

    @property
    def targets(self) -> Dict[str, float]:
        with self._lock:
            return dict(self._targets)


class LoopbackPlant(Plant):
    """A plant that instantly reports whatever it was told to do.

    For unit tests of code *around* the motor logic only.  It is deliberately
    not a simulation: it has no dynamics and always reports ``playing``, so it
    will make a broken state machine look healthy.  Never use it to claim the
    adapter works end to end -- use ``examples/fake_sim.py`` for that.
    """

    def __init__(self) -> None:
        self._state = PlantState(connected=True, playing=True, age=0.0)

    def set_targets(self, targets: Mapping[str, float]) -> None:
        merged = dict(self._state.positions)
        merged.update(targets)
        self._state = PlantState(
            positions=merged,
            t_sim=self._state.t_sim,
            playing=True,
            connected=True,
            age=0.0,
            seq=self._state.seq + 1,
            frames=self._state.frames + 1,
        )

    def get_state(self) -> PlantState:
        return self._state
