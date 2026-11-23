"""Direction A: mirror a real beamline into the twin, read-only.

Section 4 of the design document describes three directions with very
different risk profiles, and is emphatic that they must never be conflated:

* **A -- Mirror** (this module): subscribe to real motor ``.RBV`` values and
  drive the simulated articulation to match.  No writes to hardware.  Works
  through a read-only CA Gateway.  Lowest risk, nothing to negotiate.
* **B -- Twin as the IOC** (:mod:`isaac_epics.app`): expose the simulation as
  motor records so real scan code drives the twin.
* **C -- Twin commands hardware**: deliberately not implemented here.  It is
  a controls-safety and access-security question, not a software one.

This module **never writes to a PV.**  It opens read-only subscriptions and
nothing else.  There is no code path from this process to a ``caput``, and
that is a property worth preserving: the whole reason direction A is safe to
run against a live beamline is that it structurally cannot command one.

Timestamps, not wall clock
--------------------------
Section 8 requires that mirror mode be driven by PV timestamps rather than
free-running.  A readback that stops updating must be visible as stale rather
than silently held: if the beamline's IOC dies, the twin must stop pretending
it knows where the motor is.
"""

from __future__ import annotations

import argparse
import logging
import sys
import threading
import time
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional

from .config import UNIT_PRESETS, ConfigError, load_config_json
from .plant import SharedPlant
from .transport import TickServer

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class MirrorMapping:
    """One real PV mirrored onto one simulated joint.

    Attributes:
        pv: The real readback PV, typically ``<motor>.RBV``.
        joint: Isaac Sim joint name.
        sim_units_per_egu: Multiply the PV's engineering value by this to get
            simulation units.  The real motor serves mm or degrees; the joint
            wants metres or radians.
        scale: Extra gain, for mirroring onto a joint with different kinematics.
        offset: Applied in engineering units before conversion.
    """

    pv: str
    joint: str
    sim_units_per_egu: float = 1e-3
    scale: float = 1.0
    offset: float = 0.0

    def to_sim(self, egu_value: float) -> float:
        return (egu_value * self.scale + self.offset) * self.sim_units_per_egu

    @staticmethod
    def from_dict(obj: Dict[str, object]) -> "MirrorMapping":
        data = dict(obj)
        units = data.pop("units", None)
        if units is not None:
            if not isinstance(units, dict) or "sim" not in units or "egu" not in units:
                raise ConfigError("'units' needs 'sim' and 'egu'")
            sim_name, egu_name = str(units["sim"]), str(units["egu"])
            for name in (sim_name, egu_name):
                if name not in UNIT_PRESETS:
                    raise ConfigError(f"unknown unit {name!r}")
            data["sim_units_per_egu"] = UNIT_PRESETS[egu_name] / UNIT_PRESETS[sim_name]
        known = set(MirrorMapping.__dataclass_fields__)
        unknown = set(data) - known
        if unknown:
            raise ConfigError(f"unknown mirror keys: {sorted(unknown)}")
        if "pv" not in data or "joint" not in data:
            raise ConfigError("each mapping needs 'pv' and 'joint'")
        return MirrorMapping(**data)  # type: ignore[arg-type]


class Mirror:
    """Subscribes to real readbacks and drives the simulated joints.

    Args:
        mappings: What to mirror where.
        plant: The plant whose targets are driven.
        stale_after: Seconds without a monitor update before a channel is
            treated as stale and its joint is left alone.
    """

    def __init__(self, mappings: List[MirrorMapping], plant: SharedPlant,
                 stale_after: float = 5.0) -> None:
        if not mappings:
            raise ConfigError("at least one mapping is required")
        joints = [m.joint for m in mappings]
        duplicates = {j for j in joints if joints.count(j) > 1}
        if duplicates:
            raise ConfigError(f"two mappings drive the same joint: {sorted(duplicates)}")

        self.mappings = mappings
        self.plant = plant
        self.stale_after = stale_after
        self._lock = threading.Lock()
        self._last_update: Dict[str, float] = {}
        self._last_value: Dict[str, float] = {}
        self._subscriptions: List[object] = []
        # Strong references to the subscription callbacks.
        #
        # caproto's threading client stores callbacks under weakref.ref (see
        # CallbackHandler.add_callback). A closure passed straight into
        # add_callback has no other referent, so CPython collects it before
        # the first monitor arrives and the subscription silently delivers
        # nothing -- it connects, reports healthy, and never fires. In mirror
        # mode that failure is indistinguishable from a beamline that is not
        # moving, which is the worst shape a bug can take here.
        self._callbacks: List[Callable] = []
        self._context = None

    def start(self) -> None:
        """Open read-only subscriptions. Never writes."""
        from caproto.threading.client import Context

        self._context = Context()
        names = [m.pv for m in self.mappings]
        logger.info("subscribing read-only to %d PVs", len(names))
        channels = self._context.get_pvs(*names)
        for mapping, channel in zip(self.mappings, channels):
            subscription = channel.subscribe(data_type="time")
            callback = self._make_callback(mapping)
            # Hold the callback alive for the life of the Mirror; caproto will
            # not. See the note in __init__.
            self._callbacks.append(callback)
            subscription.add_callback(callback)
            self._subscriptions.append(subscription)

    def _make_callback(self, mapping: MirrorMapping):
        def callback(subscription, response):
            try:
                value = float(response.data[0])
            except (TypeError, ValueError, IndexError):
                logger.warning("%s: unusable value %r", mapping.pv, response.data)
                return
            if value != value:  # NaN
                logger.warning("%s: NaN readback ignored", mapping.pv)
                return
            with self._lock:
                self._last_update[mapping.pv] = time.monotonic()
                self._last_value[mapping.pv] = value
            self.plant.set_targets({mapping.joint: mapping.to_sim(value)})
        return callback

    def status(self) -> Dict[str, object]:
        """Per-PV freshness, for logging and the ``/state`` endpoint."""
        now = time.monotonic()
        out: Dict[str, object] = {}
        with self._lock:
            for mapping in self.mappings:
                last = self._last_update.get(mapping.pv)
                age = None if last is None else now - last
                out[mapping.pv] = {
                    "joint": mapping.joint,
                    "value": self._last_value.get(mapping.pv),
                    "age": None if age is None else round(age, 3),
                    # A readback that stopped updating is stale. Saying so is
                    # the whole difference between a mirror and a lie.
                    "stale": age is None or age > self.stale_after,
                }
        return out

    def stop(self) -> None:
        for subscription in self._subscriptions:
            try:
                subscription.clear()
            except Exception:  # noqa: BLE001
                pass
        self._subscriptions.clear()
        self._callbacks.clear()
        if self._context is not None:
            try:
                self._context.disconnect()
            except Exception:  # noqa: BLE001
                pass
            self._context = None


def load_mappings(path: str) -> List[MirrorMapping]:
    obj = load_config_json(path)
    raw = obj.get("mirror") if isinstance(obj, dict) else obj
    if not isinstance(raw, list):
        raise ConfigError("expected a list of mappings, or an object with a 'mirror' key")
    return [MirrorMapping.from_dict(item) for item in raw]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="isaac-epics-mirror",
        description=(
            "Direction A: drive an Isaac Sim articulation from real beamline "
            "readbacks. Read-only - this program never writes to a PV."
        ),
    )
    parser.add_argument("-c", "--config", required=True,
                        help="JSON file with a 'mirror' list of {pv, joint, units}")
    parser.add_argument("--http-host", default="127.0.0.1")
    parser.add_argument("--http-port", type=int, default=8227)
    parser.add_argument("--stale-after", type=float, default=5.0)
    parser.add_argument("-v", "--verbose", action="store_true")
    return parser


def main(argv: Optional[List[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s", datefmt="%H:%M:%S",
    )
    logging.getLogger("caproto").setLevel(logging.WARNING)

    try:
        mappings = load_mappings(args.config)
    except (ConfigError, OSError) as exc:
        print(f"configuration error: {exc}", file=sys.stderr)
        return 2

    plant = SharedPlant(stale_timeout=2.0)
    mirror = Mirror(mappings, plant, stale_after=args.stale_after)
    server = TickServer(plant, host=args.http_host, port=args.http_port,
                        diagnostics=mirror.status).start()

    logger.info("=" * 72)
    logger.info("MIRROR MODE - read-only. Real beamline -> Isaac Sim.")
    logger.info("This process never writes to a PV.")
    logger.info("=" * 72)
    for mapping in mappings:
        logger.info("  %-30s -> joint %-14s (x %g)",
                    mapping.pv, mapping.joint, mapping.sim_units_per_egu)
    logger.info("Isaac Sim tick endpoint: %s/tick", server.url)
    logger.info("=" * 72)

    mirror.start()
    try:
        while True:
            time.sleep(5.0)
            stale = [pv for pv, info in mirror.status().items() if info["stale"]]
            if stale:
                logger.warning("stale readbacks (joints not being driven): %s",
                               ", ".join(stale))
    except KeyboardInterrupt:
        logger.info("interrupted; shutting down")
    finally:
        mirror.stop()
        server.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
