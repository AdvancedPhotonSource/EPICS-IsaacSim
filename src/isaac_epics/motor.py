"""The motor record state machine.

This module contains no EPICS code and performs no I/O.  It is a pure state
machine over :class:`~isaac_epics.plant.PlantState`, which is what makes it
possible to test motor semantics exhaustively without a Channel Access stack,
a simulator, or a GPU.  :mod:`isaac_epics.ioc` binds it to caproto.

Why this exists at all
----------------------
``caproto.server.records.MotorFields`` supplies all 153 motor record *fields*
-- correct names, types and enums, generated from the dbd.  It supplies no
*behaviour*: no ``.DMOV`` transitions, no backlash, no retries, no dial/user
conversion, no limit enforcement.  Section 6 of the design document predicted
exactly this, and it is the reason a naive IOC "demos beautifully and then
breaks scans in ways that look like beamline faults".  This file is the
missing half.

Two clocks, deliberately separated
----------------------------------
Section 8 requires an explicit decision about which clock is authoritative.

* **Simulation time** (``t_sim``) drives the trajectory.  The motion being
  commanded is motion *of the simulated articulation*, so pausing the
  simulation must freeze the ramp rather than let it run on and then snap.
* **Wall time** drives staleness detection only.  Whether the link is alive is
  a question about the network, not about the simulation.

A corollary that matters more than it looks: **a move can never complete while
the simulation is not stepping.**  A paused simulation holds a frozen readback,
and a frozen readback that happens to sit within the deadband is not evidence
that a move finished.  That is the single failure most likely to quietly
corrupt a scan, so it is blocked structurally rather than by convention.
"""

from __future__ import annotations

import enum
import math
from dataclasses import dataclass, field
from typing import Optional

from . import trajectory
from .config import AxisConfig
from .plant import PlantState

# ---------------------------------------------------------------------------
# .MSTA bit definitions, from the EPICS motor record reference.
# ---------------------------------------------------------------------------
MSTA_DIRECTION = 0x0001
MSTA_DONE = 0x0002
MSTA_PLUS_LS = 0x0004
MSTA_HOMELS = 0x0008
MSTA_SLIP_STALL = 0x0020
MSTA_HOME = 0x0040
MSTA_PRESENT = 0x0080
MSTA_PROBLEM = 0x0100
MSTA_MOVING = 0x0200
MSTA_GAIN_SUPPORT = 0x0400
MSTA_COMM_ERR = 0x0800
MSTA_MINUS_LS = 0x1000
MSTA_HOMED = 0x2000

#: Largest simulation-time step we will integrate in one tick.  Isaac Sim can
#: stall for seconds while loading a stage or compiling shaders; replaying that
#: gap as motion would teleport the commanded setpoint.
MAX_SIM_DT = 0.25


class Phase(enum.Enum):
    """Where a move currently is."""

    IDLE = "idle"
    MOVING = "moving"
    BACKLASH = "backlash"
    SETTLE = "settle"


class Severity(enum.IntEnum):
    """EPICS alarm severities."""

    NO_ALARM = 0
    MINOR = 1
    MAJOR = 2
    INVALID = 3


@dataclass
class AxisOutputs:
    """Everything the IOC layer should publish after a tick.

    Separating "what the state machine decided" from "how it reaches Channel
    Access" keeps the state machine testable and keeps the caproto binding
    free of logic worth arguing about.
    """

    readback: float = 0.0
    dial_readback: float = 0.0
    setpoint: float = 0.0
    dial_setpoint: float = 0.0
    done_moving: int = 1
    moving: int = 0
    direction_of_travel: int = 1
    limit_violation: int = 0
    high_limit_switch: int = 0
    low_limit_switch: int = 0
    missed: int = 0
    retry_count: int = 0
    status: int = 0
    at_home: int = 0
    severity: Severity = Severity.NO_ALARM
    #: Human-readable reason for the current alarm, surfaced for diagnostics.
    alarm_message: str = ""
    #: Commanded joint position in *simulation* units, or ``None`` when the
    #: axis has nothing to command (no frame yet).
    target_sim: Optional[float] = None
    link_state: str = "disconnected"


@dataclass
class MotorAxis:
    """One simulated motor, with motor-record semantics.

    The writable fields below mirror motor record fields of the same name and
    may be set directly by the IOC layer when a client writes to them.  Moves
    are started through :meth:`request_move`, never by assigning ``setpoint``.
    """

    config: AxisConfig

    # -- writable record fields -------------------------------------------
    offset: float = 0.0                 # .OFF
    direction: int = 0                  # .DIR  (0 = Pos, 1 = Neg)
    freeze_offset: int = 0              # .FOFF (0 = Variable, 1 = Frozen)
    set_mode: int = 0                   # .SET  (0 = Use, 1 = Set)
    velocity: float = 0.0               # .VELO
    base_velocity: float = 0.0          # .VBAS
    max_velocity: float = 0.0           # .VMAX
    accel_time: float = 0.0             # .ACCL
    high_limit: float = 0.0             # .HLM
    low_limit: float = 0.0              # .LLM
    retry_deadband: float = 0.0         # .RDBD
    max_retries: int = 0                # .RTRY
    backlash_distance: float = 0.0      # .BDST
    backlash_velocity: float = 0.0      # .BVEL
    backlash_accel: float = 0.0         # .BACC
    settle_time: float = 0.0            # .DLY
    settle_timeout: float = 2.0         # simulation-specific, see _advance_motion
    stall_time: float = 0.15            # simulation-specific, see _advance_motion
    stop_flag: int = 0                  # .STOP (momentary)

    # -- internal state ----------------------------------------------------
    _phase: Phase = field(default=Phase.IDLE, init=False)
    _dial_setpoint: float = field(default=0.0, init=False)
    _commanded_dial: float = field(default=0.0, init=False)
    _dial_readback: float = field(default=0.0, init=False)
    _traj: Optional[trajectory.Trapezoid] = field(default=None, init=False)
    _traj_t: float = field(default=0.0, init=False)
    _settle_t: float = field(default=0.0, init=False)
    _retry_count: int = field(default=0, init=False)
    _missed: int = field(default=0, init=False)
    _lvio: int = field(default=0, init=False)
    _tdir: int = field(default=1, init=False)
    _last_t_sim: Optional[float] = field(default=None, init=False)
    _have_readback: bool = field(default=False, init=False)
    _backlash_pending: bool = field(default=False, init=False)
    _done_moving: int = field(default=1, init=False)
    _best_error: float = field(default=float("inf"), init=False)
    _stall_t: float = field(default=0.0, init=False)

    def __post_init__(self) -> None:
        cfg = self.config
        self.offset = cfg.offset
        self.direction = 0 if cfg.direction == "Pos" else 1
        self.velocity = cfg.velocity
        self.base_velocity = cfg.base_velocity
        self.max_velocity = cfg.max_velocity
        self.accel_time = cfg.accel_time
        self.high_limit = cfg.high_limit
        self.low_limit = cfg.low_limit
        self.retry_deadband = cfg.retry_deadband
        self.max_retries = cfg.max_retries
        self.backlash_distance = cfg.backlash_distance
        self.backlash_velocity = cfg.backlash_velocity
        self.backlash_accel = cfg.accel_time
        self.settle_time = cfg.settle_time
        self.settle_timeout = cfg.settle_timeout
        self.stall_time = cfg.stall_time
        self._dial_setpoint = self._to_dial(cfg.initial)
        self._commanded_dial = self._dial_setpoint
        self._dial_readback = self._dial_setpoint

    # -- dial / user conversion -------------------------------------------

    @property
    def dir_sign(self) -> int:
        """+1 when ``.DIR`` is Pos, -1 when Neg."""
        return 1 if self.direction == 0 else -1

    def _to_user(self, dial: float) -> float:
        return dial * self.dir_sign + self.offset

    def _to_dial(self, user: float) -> float:
        return (user - self.offset) * self.dir_sign

    @property
    def dial_high_limit(self) -> float:
        """``.DHLM``.  User limits swap when ``.DIR`` is Neg."""
        a, b = self._to_dial(self.high_limit), self._to_dial(self.low_limit)
        return max(a, b)

    @property
    def dial_low_limit(self) -> float:
        """``.DLLM``."""
        a, b = self._to_dial(self.high_limit), self._to_dial(self.low_limit)
        return min(a, b)

    # -- public state ------------------------------------------------------

    @property
    def phase(self) -> Phase:
        return self._phase

    @property
    def setpoint(self) -> float:
        """``.VAL`` -- the user-units setpoint."""
        return self._to_user(self._dial_setpoint)

    @property
    def readback(self) -> float:
        """``.RBV`` -- the user-units readback."""
        return self._to_user(self._dial_readback)

    @property
    def in_motion(self) -> bool:
        return self._phase in (Phase.MOVING, Phase.BACKLASH)

    # -- commands ----------------------------------------------------------

    def request_move(self, user_target: float) -> bool:
        """Handle a write to ``.VAL``.

        Returns ``True`` if a move was started, ``False`` if it was rejected.
        A rejected move sets ``.LVIO`` and leaves the setpoint untouched --
        which is what the real motor record does, and matters because a scan
        that silently clamps a requested position records the position it
        asked for, not the one it reached.
        """
        if not math.isfinite(user_target):
            self._lvio = 1
            return False

        if self.set_mode:
            # .SET mode: redefine the current position rather than move to it.
            self._apply_set_mode(user_target)
            return False

        if user_target > self.high_limit or user_target < self.low_limit:
            self._lvio = 1
            return False

        self._lvio = 0

        # Re-commanding the position the axis is already holding starts no
        # move on a real motor record: .DMOV never drops. Measured on
        # motorSim with tools/motor_oracle.py.
        #
        # Without this the axis re-converges on whatever residual error it
        # was resting with -- legal, since it had settled inside .RDBD, but
        # it is real motion the control system never asked for. Measured at
        # 0.041 mm on a settled axis. In a twin that is a visible twitch, and
        # a scan that re-sends a position sees a .DMOV cycle where real
        # hardware gives it none.
        #
        # Exact equality on purpose: the client is writing back a value it
        # read from this same channel. A tolerance here would swallow real
        # short moves, which sub_deadband_move shows both records DO perform.
        if self._done_moving and not self.in_motion and user_target == self.setpoint:
            return True

        self._missed = 0
        self._retry_count = 0
        self._dial_setpoint = self._to_dial(user_target)
        self._start_motion(self._dial_setpoint)
        return True

    def request_relative(self, delta_user: float) -> bool:
        """Handle ``.TWF`` / ``.TWR`` / ``.RLV`` style relative moves."""
        return self.request_move(self.setpoint + delta_user)

    def _apply_set_mode(self, user_target: float) -> None:
        """``.SET`` = Set: redefine the present position to ``user_target``.

        With ``.FOFF`` = Variable the offset absorbs the change, leaving the
        dial coordinate (and therefore the simulation) untouched.  With
        ``.FOFF`` = Frozen the offset is fixed, so the dial coordinate itself
        is redefined.  No motion is commanded either way.
        """
        if self.freeze_offset == 0:
            self.offset = user_target - self._dial_readback * self.dir_sign
        else:
            self._dial_readback = self._to_dial(user_target)
            self._commanded_dial = self._dial_readback
        self._dial_setpoint = self._to_dial(user_target)
        self._traj = None
        self._phase = Phase.IDLE
        self._done_moving = 1

    def stop(self) -> None:
        """Handle ``.STOP``.

        Commands the axis to hold at wherever the simulation currently is, not
        at the position the ramp had reached.  Those differ whenever the
        simulation is lagging the commanded setpoint, and stopping at the
        commanded value would make the axis creep forward after a stop.
        """
        self._traj = None
        self._backlash_pending = False
        self._commanded_dial = self._dial_readback if self._have_readback else self._commanded_dial
        self._dial_setpoint = self._commanded_dial
        self._retry_count = 0
        self._missed = 0
        self._phase = Phase.SETTLE
        self._settle_t = 0.0
        self._best_error = float("inf")
        self._stall_t = 0.0

    def home(self, forward: bool) -> bool:
        """Handle ``.HOMF`` / ``.HOMR``.

        The simulation has no home switch, so homing is modelled as a move to
        dial zero in the requested direction.  ``.MSTA`` gains the HOMED bit on
        arrival.  This is a convenience, not a faithful homing sequence, and is
        documented as such.
        """
        target_user = self._to_user(0.0)
        if target_user > self.high_limit or target_user < self.low_limit:
            self._lvio = 1
            return False
        self._tdir = 1 if forward else 0
        return self.request_move(target_user)

    # -- motion planning ---------------------------------------------------

    def _effective_velocity(self) -> float:
        velo = self.velocity if self.velocity > 0 else self.config.velocity
        if self.max_velocity > 0:
            velo = min(velo, self.max_velocity)
        return max(velo, 1e-9)

    def _start_motion(self, target_dial: float) -> None:
        """Plan the move, inserting a backlash take-up leg when required."""
        start = self._commanded_dial
        bdst = self.backlash_distance
        move_sign = math.copysign(1.0, target_dial - start) if target_dial != start else 0.0
        self._tdir = 1 if move_sign >= 0 else 0

        needs_backlash = False
        if bdst != 0.0 and move_sign != 0.0:
            backlash_sign = math.copysign(1.0, bdst)
            # Backlash take-up is only needed when the final approach would
            # otherwise come from the wrong side.
            needs_backlash = move_sign != backlash_sign

        if needs_backlash:
            intermediate = target_dial - bdst
            self._backlash_pending = True
            self._phase = Phase.MOVING
            self._traj = trajectory.plan(
                start, intermediate, self._effective_velocity(),
                self.accel_time, self.base_velocity,
            )
        else:
            self._backlash_pending = False
            self._phase = Phase.MOVING
            self._traj = trajectory.plan(
                start, target_dial, self._effective_velocity(),
                self.accel_time, self.base_velocity,
            )
        self._traj_t = 0.0
        self._settle_t = 0.0
        self._done_moving = 0
        self._best_error = float("inf")
        self._stall_t = 0.0

    def _start_backlash_leg(self) -> None:
        """The slow, consistent-direction final approach."""
        self._backlash_pending = False
        self._phase = Phase.BACKLASH
        bvel = self.backlash_velocity if self.backlash_velocity > 0 else self._effective_velocity()
        self._traj = trajectory.plan(
            self._commanded_dial, self._dial_setpoint, bvel,
            self.backlash_accel, self.base_velocity,
        )
        self._traj_t = 0.0

    # -- the tick ----------------------------------------------------------

    def tick(self, state: PlantState) -> AxisOutputs:
        """Advance one step and return what the IOC should publish.

        Args:
            state: The latest snapshot from the plant.

        Returns:
            :class:`AxisOutputs`.
        """
        sim_dt = self._advance_sim_clock(state)

        # Absorb the measured position before deciding anything about it.
        raw = state.positions.get(self.config.joint)
        if raw is not None and state.connected:
            self._dial_readback = self.config.to_egu(raw)
            self._have_readback = True

        if not state.connected:
            return self._disconnected_outputs(state)

        if state.playing:
            self._advance_motion(sim_dt)
        # When connected but paused we deliberately do nothing: the trajectory
        # clock is frozen and no move may complete.  See the module docstring.

        return self._build_outputs(state)

    def _advance_sim_clock(self, state: PlantState) -> float:
        """Return the simulation-time delta since the last tick, clamped."""
        if self._last_t_sim is None or not state.connected:
            self._last_t_sim = state.t_sim if state.connected else None
            return 0.0
        dt = state.t_sim - self._last_t_sim
        self._last_t_sim = state.t_sim
        if dt < 0:
            # The simulation was reset or reloaded.  Do not integrate
            # backwards; treat it as a pause until the clock moves forward.
            return 0.0
        return min(dt, MAX_SIM_DT)

    def _advance_motion(self, dt: float) -> None:
        if self._phase is Phase.MOVING or self._phase is Phase.BACKLASH:
            traj = self._traj
            if traj is None:
                self._phase = Phase.SETTLE
                self._settle_t = 0.0
                return
            self._traj_t += dt
            self._commanded_dial = traj.position(self._traj_t)
            if traj.done(self._traj_t):
                if self._backlash_pending:
                    self._start_backlash_leg()
                else:
                    self._traj = None
                    self._phase = Phase.SETTLE
                    self._settle_t = 0.0
                    self._best_error = float("inf")
                    self._stall_t = 0.0
            return

        if self._phase is Phase.SETTLE:
            self._settle_t += dt
            error = abs(self._dial_readback - self._dial_setpoint)
            deadband = self._deadband()

            if error <= deadband:
                # Arrived.  Still honour .DLY before asserting .DMOV.
                if self._settle_t >= self.settle_time:
                    self._phase = Phase.IDLE
                    self._done_moving = 1
                    self._missed = 0
                return

            # Not there yet.  The ramp finishing and the joint arriving are
            # two different instants: Isaac Sim drives an articulation through
            # a PhysX drive with finite stiffness, so the measured position
            # approaches the commanded one asymptotically and always lags it
            # by the render/exchange pipeline.  Declaring the move complete
            # when the ramp ends would report a position the joint has not
            # reached -- fabricated telemetry, and the deadband would be
            # meaningless.  So wait while progress is still being made, and
            # only apply retry/.MISS once the readback has genuinely stalled
            # or the hard timeout fires.
            #
            # This path governs only moves that cannot reach the deadband:
            # a move that does reach it is completed by the check above, on
            # the tick it arrives.  Progress is judged *relatively* rather
            # than as a fraction of the deadband, because an exponentially
            # converging drive improves by a roughly constant fraction of
            # its remaining error each tick.  An absolute threshold is
            # scale-dependent -- a wide deadband makes it give up early and
            # a narrow one makes it wait far longer than necessary -- while
            # a relative one stops when the error genuinely stops shrinking,
            # which is the drive's steady-state floor however far away that
            # floor happens to be.
            if not math.isfinite(self._best_error):
                self._best_error = error
                self._stall_t = 0.0
            elif self._best_error - error > max(self._best_error, deadband) * 1e-3:
                self._best_error = error
                self._stall_t = 0.0
            else:
                self._stall_t += dt

            if self._stall_t >= self.stall_time or self._settle_t >= self.settle_timeout:
                self._finish_or_retry()

    def _deadband(self) -> float:
        """``.RDBD``, falling back to the configured value if cleared."""
        return self.retry_deadband if self.retry_deadband > 0 else self.config.retry_deadband

    def _finish_or_retry(self) -> None:
        """Decide whether the move landed, needs a retry, or missed."""
        error = abs(self._dial_readback - self._dial_setpoint)
        deadband = self._deadband()

        if error <= deadband:
            self._phase = Phase.IDLE
            self._done_moving = 1
            self._missed = 0
            return

        if self._retry_count < self.max_retries:
            self._retry_count += 1
            # Retry from where the simulation actually is, not from the
            # commanded value -- otherwise the retry repeats the same move.
            self._commanded_dial = self._dial_readback
            self._backlash_pending = False
            self._phase = Phase.MOVING
            self._traj = trajectory.plan(
                self._commanded_dial, self._dial_setpoint,
                self._effective_velocity(), self.accel_time, self.base_velocity,
            )
            self._traj_t = 0.0
            self._done_moving = 0
            self._best_error = float("inf")
            self._stall_t = 0.0
            return

        # Out of retries.  The motor record declares the move done and raises
        # .MISS; it does not hang.  A scan sees DMOV rise with MISS set, which
        # is the honest report: motion stopped, the destination was not reached.
        self._phase = Phase.IDLE
        self._done_moving = 1
        self._missed = 1

    # -- output construction -----------------------------------------------

    def _disconnected_outputs(self, state: PlantState) -> AxisOutputs:
        """Outputs for a dead link.

        Section 8: staleness must be loud.  The readback keeps its last value
        because discarding it would be its own kind of lie, but it is flagged
        INVALID, and ``.DMOV`` is **not** asserted.  A scan waiting on ``.DMOV``
        then blocks with an alarm raised, which is the correct outcome: a
        visible hang beats a fabricated completion.
        """
        status = MSTA_PROBLEM | MSTA_COMM_ERR | MSTA_PRESENT
        if self._tdir:
            status |= MSTA_DIRECTION
        return AxisOutputs(
            readback=self.readback,
            dial_readback=self._dial_readback,
            setpoint=self.setpoint,
            dial_setpoint=self._dial_setpoint,
            done_moving=0,
            moving=0,
            direction_of_travel=self._tdir,
            limit_violation=self._lvio,
            missed=self._missed,
            retry_count=self._retry_count,
            status=status,
            severity=Severity.INVALID,
            alarm_message=(
                f"no frame from Isaac Sim for {state.age:.1f}s "
                f"(stale link; readback is not live)"
            ),
            target_sim=None,
            link_state=state.link_state,
        )

    def _build_outputs(self, state: PlantState) -> AxisOutputs:
        moving = 1 if self.in_motion else 0
        hls = 1 if self._dial_readback >= self.dial_high_limit else 0
        lls = 1 if self._dial_readback <= self.dial_low_limit else 0

        status = MSTA_PRESENT | MSTA_GAIN_SUPPORT
        if self._done_moving:
            status |= MSTA_DONE
        if moving:
            status |= MSTA_MOVING
        if self._tdir:
            status |= MSTA_DIRECTION
        if hls:
            status |= MSTA_PLUS_LS
        if lls:
            status |= MSTA_MINUS_LS
        at_home = 1 if abs(self._dial_readback) <= max(self.retry_deadband, 1e-12) else 0
        if at_home:
            status |= MSTA_HOMED | MSTA_HOME

        severity = Severity.NO_ALARM
        message = ""
        if not state.playing:
            # Connected but paused.  Not an error -- but it must be visible,
            # and it must never look like a settled move in progress.
            severity = Severity.MINOR
            message = "simulation is paused"
            if self.in_motion or not self._done_moving:
                severity = Severity.MAJOR
                message = "simulation paused mid-move; this move cannot complete"
        elif self._missed:
            severity = Severity.MINOR
            message = "move finished outside the retry deadband (.MISS)"
        elif self._lvio:
            severity = Severity.MINOR
            message = "requested position violates a soft limit (.LVIO)"
        elif hls or lls:
            severity = Severity.MINOR
            message = "at a limit"

        return AxisOutputs(
            readback=self.readback,
            dial_readback=self._dial_readback,
            setpoint=self.setpoint,
            dial_setpoint=self._dial_setpoint,
            done_moving=self._done_moving,
            moving=moving,
            direction_of_travel=self._tdir,
            limit_violation=self._lvio,
            high_limit_switch=hls,
            low_limit_switch=lls,
            missed=self._missed,
            retry_count=self._retry_count,
            status=status,
            at_home=at_home,
            severity=severity,
            alarm_message=message,
            target_sim=self.config.to_sim(self._commanded_dial),
            link_state=state.link_state,
        )
