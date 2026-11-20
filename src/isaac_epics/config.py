"""Adapter configuration: axis definitions, PV naming, bind addresses.

Unit handling deserves a note, because it is a live source of bugs in this
specific system.  The context document records that the existing twin reports
rotary joints in *radians* from the runtime while the USD authors joint limits
in *degrees*.  EPICS clients, meanwhile, want engineering units -- mm and
degrees -- because that is what the real beamline's motor records serve and
what every scan file and screen is written against.

So there are exactly two unit systems, and one conversion, in one place:

* **simulation units** -- metres, radians.  Used on the wire and nowhere else.
* **engineering units** -- whatever ``egu`` says, typically mm or degrees.
  Used by every PV.

``sim_units_per_egu`` converts between them: ``sim = egu * sim_units_per_egu``.
For a linear stage reading metres and serving mm that is ``0.001``; for a
rotary joint reading radians and serving degrees it is ``pi/180``.
"""

from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass, field, replace
from typing import Any, Dict, List, Optional

#: Conversion presets, so configs do not carry hand-typed magic numbers.
UNIT_PRESETS: Dict[str, float] = {
    "m": 1.0,
    "mm": 1e-3,
    "um": 1e-6,
    "micron": 1e-6,
    "rad": 1.0,
    "mrad": 1e-3,
    "deg": math.pi / 180.0,
    "degrees": math.pi / 180.0,
}

#: Sanity bound on a configuration file. An axis entry is a few hundred bytes.
MAX_CONFIG_BYTES = 1024 * 1024

#: PV prefixes that must never be served by this adapter.  Section 8 requires
#: naming discipline: once the simulation is indistinguishable to the control
#: system, it becomes possible to lose track of which one you are connected to.
#: These are the APS beamline prefix shapes that would be genuinely ambiguous.
_REAL_PREFIX_HINTS = ("XSD:", "ID:", "S:", "FE:", "PV:", "BL:")


class ConfigError(ValueError):
    """Raised for a configuration that would be unsafe or ambiguous to serve."""


def read_config_text(path: str) -> str:
    """Resolve and read a configuration file, validating the path first.

    The path comes from an operator's command line, so this is not a privilege
    boundary -- they could read the file anyway. It is still worth resolving
    and checking: it turns the confusing failures (a directory, a dangling
    symlink, a typo that happens to name a device node) into one clear error
    naming the resolved path, which is what you want when an IOC refuses to
    start and you are reading a log.
    """
    try:
        resolved = os.path.realpath(os.path.expanduser(str(path)))
    except (OSError, ValueError) as exc:
        raise ConfigError(f"{path!r}: cannot be resolved to a path: {exc}") from exc

    if not os.path.exists(resolved):
        raise ConfigError(f"{path!r}: no such file (resolved to {resolved})")
    if not os.path.isfile(resolved):
        raise ConfigError(f"{path!r}: not a regular file (resolved to {resolved})")

    try:
        with open(resolved, encoding="utf-8") as handle:
            return handle.read(MAX_CONFIG_BYTES + 1)
    except OSError as exc:
        raise ConfigError(f"{path!r}: cannot be read: {exc}") from exc


def load_config_json(path: str) -> Any:
    """Read and parse a JSON configuration file."""
    text = read_config_text(path)
    if len(text) > MAX_CONFIG_BYTES:
        raise ConfigError(f"{path!r}: configuration larger than {MAX_CONFIG_BYTES} bytes")
    try:
        return json.loads(text)
    except json.JSONDecodeError as exc:
        raise ConfigError(f"{path}: invalid JSON: {exc}") from exc


@dataclass(frozen=True)
class AxisConfig:
    """One simulated motor: a joint in Isaac Sim exposed as a motor record.

    Attributes:
        motor: PV name suffix.  Full PV is ``prefix + motor`` (e.g. ``m1``).
        joint: Joint name as Isaac Sim reports it (e.g. ``x_axis``).
        desc: ``.DESC``.
        egu: ``.EGU`` -- engineering units string shown to clients.
        sim_units_per_egu: Multiply engineering units by this to get
            simulation units.  See the module docstring.
        low_limit / high_limit: ``.LLM`` / ``.HLM``, engineering units.
        velocity: ``.VELO``, EGU/s.
        base_velocity: ``.VBAS``, EGU/s.
        max_velocity: ``.VMAX``, EGU/s.  Writes to ``.VELO`` are clamped here.
        accel_time: ``.ACCL``, seconds from rest to ``.VELO``.
        retry_deadband: ``.RDBD``.  A move is done when the readback is within
            this of the target.  Must be larger than the simulation's own
            position noise or moves will never complete.
        max_retries: ``.RTRY``.
        backlash_distance: ``.BDST``.  Non-zero means every move finishes by
            approaching from a consistent direction.
        backlash_velocity: ``.BVEL``, EGU/s for the final approach.
        settle_time: ``.DLY``, seconds to wait after motion before ``.DMOV``.
        settle_timeout: Hard cap on how long the axis waits for the readback
            to converge after the ramp ends, before applying retry/``.MISS``
            logic.  Bounds the worst case so a move can never hang forever.
        stall_time: How long the readback must stop making progress before
            the axis concludes it has arrived as close as it is going to get.
            A physics drive converges asymptotically, so "the ramp finished"
            and "the joint got there" are different instants.
        precision: ``.PREC``.
        resolution: ``.MRES``, EGU per step.
        direction: ``.DIR`` -- ``"Pos"`` or ``"Neg"``.
        offset: ``.OFF`` -- user = dial * dir_sign + offset.
        initial: Where the adapter assumes the axis is before the first frame
            arrives.  Only used to seed setpoints; never reported as a
            readback (an unconnected axis reports an invalid readback).
    """

    motor: str
    joint: str
    desc: str = ""
    egu: str = "mm"
    sim_units_per_egu: float = 1e-3
    low_limit: float = -100.0
    high_limit: float = 100.0
    velocity: float = 10.0
    base_velocity: float = 0.0
    max_velocity: float = 50.0
    accel_time: float = 0.2
    retry_deadband: float = 1e-3
    max_retries: int = 1
    backlash_distance: float = 0.0
    backlash_velocity: float = 1.0
    settle_time: float = 0.0
    settle_timeout: float = 2.0
    stall_time: float = 0.15
    precision: int = 4
    resolution: float = 1e-4
    direction: str = "Pos"
    offset: float = 0.0
    initial: float = 0.0

    def __post_init__(self) -> None:
        if not self.motor or "." in self.motor:
            raise ConfigError(f"motor name {self.motor!r} must be non-empty and contain no '.'")
        if not self.joint:
            raise ConfigError(f"axis {self.motor!r}: 'joint' is required")
        if self.low_limit >= self.high_limit:
            raise ConfigError(
                f"axis {self.motor!r}: low_limit {self.low_limit} must be below "
                f"high_limit {self.high_limit}"
            )
        if self.sim_units_per_egu == 0 or not math.isfinite(self.sim_units_per_egu):
            raise ConfigError(f"axis {self.motor!r}: sim_units_per_egu must be non-zero and finite")
        if self.velocity <= 0:
            raise ConfigError(f"axis {self.motor!r}: velocity must be positive")
        if self.max_velocity < self.velocity:
            raise ConfigError(
                f"axis {self.motor!r}: max_velocity {self.max_velocity} is below "
                f"velocity {self.velocity}"
            )
        if self.accel_time < 0:
            raise ConfigError(f"axis {self.motor!r}: accel_time must not be negative")
        if self.retry_deadband <= 0:
            raise ConfigError(
                f"axis {self.motor!r}: retry_deadband must be positive, or moves can never "
                "be declared complete"
            )
        if self.direction not in ("Pos", "Neg"):
            raise ConfigError(f"axis {self.motor!r}: direction must be 'Pos' or 'Neg'")
        if self.max_retries < 0:
            raise ConfigError(f"axis {self.motor!r}: max_retries must not be negative")
        if self.settle_timeout <= 0:
            raise ConfigError(f"axis {self.motor!r}: settle_timeout must be positive")
        if self.stall_time < 0:
            raise ConfigError(f"axis {self.motor!r}: stall_time must not be negative")

    @property
    def dir_sign(self) -> int:
        """+1 for ``.DIR`` = Pos, -1 for Neg."""
        return 1 if self.direction == "Pos" else -1

    def to_sim(self, egu_value: float) -> float:
        """Engineering units -> simulation units."""
        return egu_value * self.sim_units_per_egu

    def to_egu(self, sim_value: float) -> float:
        """Simulation units -> engineering units."""
        return sim_value / self.sim_units_per_egu

    @staticmethod
    def from_dict(obj: Dict[str, Any]) -> "AxisConfig":
        data = dict(obj)
        # Allow `{"units": {"sim": "rad", "egu": "deg"}}` instead of a raw ratio.
        units = data.pop("units", None)
        if units is not None:
            if not isinstance(units, dict) or "sim" not in units or "egu" not in units:
                raise ConfigError("'units' must be an object with 'sim' and 'egu' keys")
            sim_name, egu_name = str(units["sim"]), str(units["egu"])
            for name in (sim_name, egu_name):
                if name not in UNIT_PRESETS:
                    raise ConfigError(
                        f"unknown unit {name!r}; known units: {sorted(UNIT_PRESETS)}"
                    )
            data["sim_units_per_egu"] = UNIT_PRESETS[egu_name] / UNIT_PRESETS[sim_name]
            data.setdefault("egu", egu_name)
        known = {f for f in AxisConfig.__dataclass_fields__}
        unknown = set(data) - known
        if unknown:
            raise ConfigError(f"unknown axis keys: {sorted(unknown)}")
        return AxisConfig(**data)


@dataclass(frozen=True)
class AdapterConfig:
    """Top-level adapter configuration.

    Attributes:
        prefix: PV prefix for every record this adapter serves.  Must be
            distinct from any real beamline prefix (section 8).
        axes: The simulated motors.
        http_host / http_port: Where Isaac Sim connects.  Defaults to loopback;
            Isaac Sim's container runs with ``--network=host`` on these hosts,
            so loopback is reachable from inside the container without
            exposing the endpoint to the network.
        ca_interface: Interface the Channel Access server binds.  Defaults to
            loopback so that starting the adapter cannot accidentally serve
            simulated motors onto the controls subnet.  Overriding this is a
            deliberate act and is logged as such.
        stale_timeout: Seconds without a frame before the link is declared
            dead and readbacks are marked invalid.
        update_rate: Hz at which the motor state machines run.
    """

    prefix: str = "SIM:TWIN:"
    axes: List[AxisConfig] = field(default_factory=list)
    http_host: str = "127.0.0.1"
    http_port: int = 8227
    ca_interface: str = "127.0.0.1"
    stale_timeout: float = 1.0
    update_rate: float = 20.0

    def __post_init__(self) -> None:
        if not self.prefix:
            raise ConfigError(
                "a PV prefix is required; serving motor records with no prefix risks "
                "colliding with real beamline PVs"
            )
        if not self.axes:
            raise ConfigError("at least one axis must be configured")
        names = [a.motor for a in self.axes]
        dupes = {n for n in names if names.count(n) > 1}
        if dupes:
            raise ConfigError(f"duplicate motor names: {sorted(dupes)}")
        joints = [a.joint for a in self.axes]
        dupe_joints = {j for j in joints if joints.count(j) > 1}
        if dupe_joints:
            raise ConfigError(
                f"two axes map to the same joint {sorted(dupe_joints)}; joint ownership "
                "must be unambiguous"
            )
        if self.update_rate <= 0:
            raise ConfigError("update_rate must be positive")
        if self.stale_timeout <= 0:
            raise ConfigError("stale_timeout must be positive")
        if not (0 < self.http_port < 65536):
            raise ConfigError(f"http_port {self.http_port} out of range")

    def naming_warnings(self) -> List[str]:
        """Return human-readable warnings about a prefix that may be unsafe.

        This cannot be made an error: a site may legitimately use any string.
        It is surfaced loudly at startup instead, because the failure it guards
        against -- not knowing whether you are driving the twin or the beamline
        -- is silent by nature.
        """
        warnings: List[str] = []
        upper = self.prefix.upper()
        if not any(tag in upper for tag in ("SIM", "TWIN", "FAKE", "TEST", "DEV")):
            warnings.append(
                f"PV prefix {self.prefix!r} does not contain SIM/TWIN/FAKE/TEST/DEV. "
                "Section 8 of the design requires a prefix that cannot be confused with "
                "a real beamline IOC."
            )
        for hint in _REAL_PREFIX_HINTS:
            if upper.startswith(hint) and "SIM" not in upper and "TWIN" not in upper:
                warnings.append(
                    f"PV prefix {self.prefix!r} looks like a real APS prefix ({hint}...)."
                )
        if self.ca_interface not in ("127.0.0.1", "localhost", "::1"):
            warnings.append(
                f"Channel Access is bound to {self.ca_interface}, not loopback. Simulated "
                "motor records will be visible to other hosts. Confirm this is not the "
                "controls subnet."
            )
        return warnings

    def axis(self, motor: str) -> Optional[AxisConfig]:
        for a in self.axes:
            if a.motor == motor:
                return a
        return None

    @staticmethod
    def from_dict(obj: Dict[str, Any]) -> "AdapterConfig":
        if not isinstance(obj, dict):
            raise ConfigError("configuration must be a JSON object")
        data = dict(obj)
        data.pop("$schema", None)
        data.pop("comment", None)
        raw_axes = data.pop("axes", None)
        if not isinstance(raw_axes, list):
            raise ConfigError("'axes' must be a list")
        axes = [AxisConfig.from_dict(a) for a in raw_axes]
        bind = data.pop("bind", None)
        if bind is not None:
            if not isinstance(bind, dict):
                raise ConfigError("'bind' must be an object")
            for key in ("http_host", "http_port", "ca_interface"):
                if key in bind:
                    data[key] = bind[key]
            unknown_bind = set(bind) - {"http_host", "http_port", "ca_interface"}
            if unknown_bind:
                raise ConfigError(f"unknown bind keys: {sorted(unknown_bind)}")
        known = {f for f in AdapterConfig.__dataclass_fields__}
        unknown = set(data) - known
        if unknown:
            raise ConfigError(f"unknown configuration keys: {sorted(unknown)}")
        return AdapterConfig(axes=axes, **data)

    @staticmethod
    def from_file(path: str) -> "AdapterConfig":
        return AdapterConfig.from_dict(load_config_json(path))

    def with_overrides(self, **kwargs: Any) -> "AdapterConfig":
        """Return a copy with non-``None`` keyword overrides applied."""
        updates = {k: v for k, v in kwargs.items() if v is not None}
        return replace(self, **updates) if updates else self


def default_config() -> AdapterConfig:
    """The 4-DOF stage from the digital-twin prototype.

    Limits match ``mechanism/beamline_stage.json``, the authoritative
    description the articulation is actually built from:

    ======  =========  ===========================  ==================
    Joint   Type       Physical limit (runtime)     Soft limit served
    ======  =========  ===========================  ==================
    x_axis  prismatic  -0.025 .. 0.025 m            -24 .. 24 mm
    y_axis  prismatic  -0.025 .. 0.025 m            -24 .. 24 mm
    z_axis  prismatic  0.0 .. 0.050 m               1 .. 49 mm
    theta   revolute   -180 .. 180 deg              -179 .. 179 deg
    ======  =========  ===========================  ==================

    The soft limits sit *inside* the physical ones, which is how a real
    beamline is configured: soft limits exist to keep the mechanism off its
    hard stops. Serving soft limits wider than the joint would let a client
    command a position the articulation physically cannot reach -- the move
    would stall against the PhysX limit and surface as ``.MISS`` after
    retries, which reads like a hardware fault rather than the configuration
    error it is.

    ``retry_deadband`` on ``z_axis`` is deliberately the loosest. It is the
    only axis carrying a gravity load, and its drive is a force drive with
    finite stiffness (300000 N/m against roughly 44 N), so it settles a
    predictable distance below its target -- of order 0.15 mm. A deadband
    tighter than that steady-state error would make every z move report
    ``.MISS`` forever. These values are starting points to be replaced by
    measurements from the running articulation.
    """
    return AdapterConfig(
        prefix=os.environ.get("ISAAC_EPICS_PREFIX", "SIM:TWIN:"),
        axes=[
            AxisConfig(
                motor="m1", joint="x_axis", desc="Sample X", egu="mm",
                sim_units_per_egu=UNIT_PRESETS["mm"],
                low_limit=-24.0, high_limit=24.0, velocity=5.0, max_velocity=20.0,
                accel_time=0.2, retry_deadband=0.05, precision=4,
                settle_timeout=3.0, stall_time=0.2,
            ),
            AxisConfig(
                motor="m2", joint="y_axis", desc="Sample Y", egu="mm",
                sim_units_per_egu=UNIT_PRESETS["mm"],
                low_limit=-24.0, high_limit=24.0, velocity=5.0, max_velocity=20.0,
                accel_time=0.2, retry_deadband=0.05, precision=4,
                settle_timeout=3.0, stall_time=0.2,
            ),
            AxisConfig(
                motor="m3", joint="z_axis", desc="Sample Z", egu="mm",
                sim_units_per_egu=UNIT_PRESETS["mm"],
                low_limit=1.0, high_limit=49.0, velocity=2.0, max_velocity=10.0,
                accel_time=0.2, retry_deadband=0.5, precision=4,
                settle_timeout=3.0, stall_time=0.2,
            ),
            AxisConfig(
                motor="m4", joint="theta_axis", desc="Sample Theta", egu="deg",
                # Runtime reports radians; the USD authors limits in degrees.
                sim_units_per_egu=UNIT_PRESETS["deg"],
                low_limit=-179.0, high_limit=179.0, velocity=30.0, max_velocity=90.0,
                accel_time=0.3, retry_deadband=0.1, precision=3,
                settle_timeout=3.0, stall_time=0.2,
            ),
        ],
    )
