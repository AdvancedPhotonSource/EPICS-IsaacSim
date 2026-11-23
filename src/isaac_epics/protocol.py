"""Wire protocol between Isaac Sim (client) and the adapter (server).

Design constraints, from the architecture decisions in EPICS-INTEGRATION-CONTEXT.md:

* Isaac Sim is the *client* (section 7.2).  It opens one outbound connection to
  one endpoint and therefore structurally cannot multiplex.
* The Isaac Sim side uses only the Python standard library (section 7.6).  That
  rules out protobuf/msgpack/ZMQ; JSON over HTTP it is.
* The protocol is deliberately dumb.  Isaac Sim knows *nothing* about EPICS.
  Every motor-record semantic -- deadband, ``.DMOV``, soft limits, offsets,
  engineering units -- lives in the adapter.

One request/response per simulation frame carries both directions::

    Isaac Sim  -->  {"protocol": 1, "positions": {...}, "t_sim": 1234.5,
                     "playing": true, "seq": 42}
               <--  {"protocol": 1, "targets": {"x_axis": 0.042}, ...}

Units on the wire are always *simulation* units: metres for prismatic joints,
radians for revolute joints, matching what the Isaac Sim runtime reports.  The
conversion to engineering units (mm, degrees) is the adapter's job -- see
:mod:`isaac_epics.config`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Tuple

PROTOCOL_VERSION = 1

#: Upper bound on a decoded request body.  A frame carries one float per joint;
#: a beamline stage has single-digit joint counts.  This is a sanity bound to
#: keep a malformed or hostile client from forcing a large allocation.
MAX_BODY_BYTES = 64 * 1024

#: Upper bound on the number of joints accepted in one frame.
MAX_JOINTS = 256


class ProtocolError(ValueError):
    """Raised when a peer sends a frame that cannot be trusted.

    Always treated as a hard rejection: a frame we cannot parse is never
    partially applied, because a partially applied frame is indistinguishable
    from fabricated telemetry.
    """


@dataclass(frozen=True)
class JointInfo:
    """What the simulation reports about one joint of its articulation.

    Sent so the adapter can check its configuration against the mechanism
    that actually exists, rather than trusting that a human kept two files
    in step. A ``joint`` name that matches nothing drives nothing, and a
    soft limit outside the physical joint produces ``.MISS`` after retries --
    both read as faults rather than as the configuration errors they are.
    """

    name: str
    joint_type: str = "unknown"      # prismatic / revolute / unknown
    lower: Optional[float] = None    # simulation units
    upper: Optional[float] = None
    #: PV this joint declares itself bound to, read from an ``epics:motorPV``
    #: attribute on the joint prim in USD. Optional: a stage that carries no
    #: bindings simply does not set it, and the adapter falls back to the
    #: joint names in its own configuration, as before.
    #:
    #: When it IS set, the model is the source of truth. A hand-typed joint
    #: name in a config file is an agreement between two files that nobody
    #: re-checks; a binding authored next to the joint it names travels with
    #: the asset and cannot drift away from it.
    motor_pv: Optional[str] = None

    @staticmethod
    def from_json(obj: Any) -> "JointInfo":
        if not isinstance(obj, dict) or not isinstance(obj.get("name"), str):
            raise ProtocolError("each joint descriptor needs a string 'name'")
        lower = obj.get("lower")
        upper = obj.get("upper")
        motor_pv = obj.get("motor_pv")
        if motor_pv is not None and not isinstance(motor_pv, str):
            raise ProtocolError("'motor_pv' must be a string when present")
        return JointInfo(
            name=obj["name"],
            joint_type=str(obj.get("type", "unknown")),
            lower=None if lower is None else _finite_float(lower, "joint lower"),
            upper=None if upper is None else _finite_float(upper, "joint upper"),
            motor_pv=motor_pv,
        )


@dataclass(frozen=True)
class Articulation:
    """A description of the articulation the simulation is driving.

    Optional: an older client simply does not send it, and the adapter then
    has nothing to check against and says so rather than assuming agreement.
    """

    prim: str = ""
    meters_per_unit: float = 1.0
    joints: Tuple[JointInfo, ...] = ()

    @staticmethod
    def from_json(obj: Any) -> "Articulation":
        if not isinstance(obj, dict):
            raise ProtocolError("'articulation' must be a JSON object")
        raw = obj.get("joints", [])
        if not isinstance(raw, list):
            raise ProtocolError("'articulation.joints' must be a list")
        if len(raw) > MAX_JOINTS:
            raise ProtocolError("too many joints in articulation descriptor")
        return Articulation(
            prim=str(obj.get("prim", "")),
            meters_per_unit=_finite_float(obj.get("meters_per_unit", 1.0),
                                          "meters_per_unit"),
            joints=tuple(JointInfo.from_json(j) for j in raw),
        )

    def joint(self, name: str) -> Optional[JointInfo]:
        for info in self.joints:
            if info.name == name:
                return info
        return None


@dataclass(frozen=True)
class SimFrame:
    """One frame of state reported *by* Isaac Sim.

    Attributes:
        positions: Joint name -> measured position, in simulation units
            (metres / radians).
        t_sim: The simulation clock, in seconds.  Not wall clock, and not
            guaranteed to advance at wall-clock rate -- Isaac Sim is not hard
            real-time (section 8).
        playing: Whether the simulation is actually stepping.  A paused sim
            reporting a frozen readback as a settled move is the failure most
            likely to quietly corrupt a scan, which is why this is mandatory
            rather than inferred.
        seq: Monotonic frame counter from the client, used to detect restarts
            and reordering.
        meta: Free-form client diagnostics.  Never interpreted by motor logic.
        articulation: Optional description of the mechanism, used to check
            the adapter's configuration against what actually exists.
    """

    positions: Dict[str, float]
    t_sim: float
    playing: bool
    seq: int = 0
    meta: Dict[str, Any] = field(default_factory=dict)
    articulation: Optional["Articulation"] = None

    @staticmethod
    def from_json(obj: Any) -> "SimFrame":
        """Parse and validate a decoded JSON object into a :class:`SimFrame`.

        Every field is validated.  We refuse NaN and infinity: they propagate
        silently through comparisons (``nan <= deadband`` is ``False``, so a
        NaN readback would make a move never complete) and must not reach the
        motor state machine.
        """
        if not isinstance(obj, dict):
            raise ProtocolError(f"frame must be a JSON object, got {type(obj).__name__}")

        version = obj.get("protocol", PROTOCOL_VERSION)
        if not isinstance(version, int) or version != PROTOCOL_VERSION:
            raise ProtocolError(
                f"unsupported protocol version {version!r}; this adapter speaks {PROTOCOL_VERSION}"
            )

        raw_positions = obj.get("positions")
        if not isinstance(raw_positions, dict):
            raise ProtocolError("'positions' must be a JSON object of joint -> value")
        if len(raw_positions) > MAX_JOINTS:
            raise ProtocolError(f"too many joints in frame: {len(raw_positions)} > {MAX_JOINTS}")

        positions: Dict[str, float] = {}
        for joint, value in raw_positions.items():
            if not isinstance(joint, str) or not joint:
                raise ProtocolError(f"joint name must be a non-empty string, got {joint!r}")
            positions[joint] = _finite_float(value, f"positions[{joint!r}]")

        playing = obj.get("playing")
        if not isinstance(playing, bool):
            raise ProtocolError("'playing' is required and must be a boolean")

        t_sim = _finite_float(obj.get("t_sim", 0.0), "t_sim")

        seq = obj.get("seq", 0)
        if not isinstance(seq, int) or isinstance(seq, bool) or seq < 0:
            raise ProtocolError(f"'seq' must be a non-negative integer, got {seq!r}")

        meta = obj.get("meta", {})
        if not isinstance(meta, dict):
            raise ProtocolError("'meta' must be a JSON object when present")

        raw_articulation = obj.get("articulation")
        articulation = (
            Articulation.from_json(raw_articulation)
            if raw_articulation is not None else None
        )

        return SimFrame(positions=positions, t_sim=t_sim, playing=playing, seq=seq,
                        meta=meta, articulation=articulation)


@dataclass(frozen=True)
class TargetFrame:
    """Commanded joint targets sent *to* Isaac Sim, in simulation units.

    These are *instantaneous* setpoints, not final destinations.  The adapter
    runs the velocity profile (see :mod:`isaac_epics.trajectory`) and streams
    the ramped position each frame.  That keeps ``.VELO`` / ``.ACCL`` meaningful
    without teaching Isaac Sim anything about the motor record, and it is what
    makes replaying a scan to reproduce a collision worth doing -- a teleport to
    the destination would skip the collision entirely.
    """

    targets: Dict[str, float]
    t_wall: float = 0.0
    #: Per-joint adapter diagnostics, for `curl` debugging only.
    axes: Dict[str, Any] = field(default_factory=dict)

    def to_json(self) -> Dict[str, Any]:
        return {
            "protocol": PROTOCOL_VERSION,
            "targets": self.targets,
            "t_wall": self.t_wall,
            "axes": self.axes,
        }


def _finite_float(value: Any, where: str) -> float:
    """Coerce to float, rejecting bools, non-numbers, NaN and infinity."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ProtocolError(f"{where} must be a number, got {value!r}")
    result = float(value)
    if not math.isfinite(result):
        raise ProtocolError(f"{where} must be finite, got {result!r}")
    return result


def parse_target_frame(obj: Any) -> Dict[str, float]:
    """Client-side counterpart to :meth:`SimFrame.from_json`.

    Used by the Isaac Sim client to validate what the adapter sent back before
    applying it to an articulation.  The client trusts the adapter no further
    than the adapter trusts the client: a bad target silently applied to a
    joint is a physical-looking fault that is painful to trace.
    """
    if not isinstance(obj, dict):
        raise ProtocolError("response must be a JSON object")
    version = obj.get("protocol", PROTOCOL_VERSION)
    if version != PROTOCOL_VERSION:
        raise ProtocolError(f"unsupported protocol version {version!r}")
    raw = obj.get("targets")
    if raw is None:
        return {}
    if not isinstance(raw, dict):
        raise ProtocolError("'targets' must be a JSON object")
    if len(raw) > MAX_JOINTS:
        raise ProtocolError(f"too many joints in response: {len(raw)}")
    return {str(k): _finite_float(v, f"targets[{k!r}]") for k, v in raw.items()}


def describe_link_state(connected: bool, playing: Optional[bool]) -> str:
    """Render the three states section 8 requires to be distinguishable.

    Returns one of ``"disconnected"``, ``"paused"``, ``"running"``.  There is
    deliberately no fourth value and no default: an unknown link state must be
    reported as disconnected, never as something more reassuring.
    """
    if not connected:
        return "disconnected"
    return "running" if playing else "paused"
