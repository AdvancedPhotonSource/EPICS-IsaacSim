"""One process: HTTP endpoint + Channel Access server (architecture 7.3).

The motor record is a state machine with two inputs -- a ``.VAL`` write starts
a move, ``.DMOV`` drops, the readback converges, backlash may fire, a limit may
trip, ``.DMOV`` rises.  It needs command input and position feedback in the
same place, synchronously.  Split it across processes and you must decide which
side owns ``.DMOV``; the other side reads a stale copy.  That is the classic
distributed-state bug, landed on the exact signal a step scan's pace depends
on.  So: one process.

Concurrency
-----------
Two threads, one shared object:

* the **asyncio thread** runs the caproto server and the periodic tick that
  advances every motor state machine;
* the **HTTP thread** accepts frames from Isaac Sim;
* :class:`~isaac_epics.plant.SharedPlant` is the only shared mutable state,
  and every access to it is guarded by one lock.

No motor state is touched from the HTTP thread, and no socket is touched from
the asyncio thread.
"""

from __future__ import annotations

import asyncio
import logging
import time
from typing import Any, Optional

from caproto.asyncio.server import Context as AsyncioContext

from .config import AdapterConfig
from .ioc import build_ioc
from .plant import SharedPlant
from .transport import TickServer, port_is_free

logger = logging.getLogger(__name__)


class Adapter:
    """Owns the plant, the HTTP endpoint and the IOC for one simulation."""

    def __init__(self, config: AdapterConfig) -> None:
        self.config = config
        self.plant = SharedPlant(stale_timeout=config.stale_timeout)
        self.ioc = build_ioc(config, self.plant)
        self.server: Optional[TickServer] = None
        self._tick_period = 1.0 / config.update_rate
        self._ticks = 0
        self._last_warn = 0.0

    # -- lifecycle ---------------------------------------------------------

    def start_http(self) -> TickServer:
        host, port = self.config.http_host, self.config.http_port
        if port != 0 and not port_is_free(host, port):
            raise RuntimeError(
                f"cannot bind {host}:{port} - already in use. Another adapter may be "
                f"running; two adapters driving one simulation would fight over joint "
                f"ownership (see architecture decision 7.4)."
            )
        self.server = TickServer(
            self.plant, host=host, port=port, diagnostics=self.ioc.diagnostics,
        ).start()
        return self.server

    async def tick_forever(self) -> None:
        """Advance every motor state machine at the configured rate."""
        loop = asyncio.get_running_loop()
        next_at = loop.time()
        while True:
            next_at += self._tick_period
            try:
                await self.ioc.tick_all()
            except Exception:  # noqa: BLE001 - a bad tick must not kill the IOC
                logger.exception("tick failed")
            self._ticks += 1
            delay = next_at - loop.time()
            if delay < -self._tick_period:
                # We are not keeping up. Resynchronise rather than spiral, and
                # say so -- a silently slow control loop is a lie about rate.
                now = time.monotonic()
                if now - self._last_warn > 10.0:
                    logger.warning(
                        "tick loop is behind by %.3fs; motor updates are slower than the "
                        "configured %.1f Hz", -delay, self.config.update_rate,
                    )
                    self._last_warn = now
                next_at = loop.time()
                delay = 0.0
            await asyncio.sleep(max(0.0, delay))

    async def run(self) -> None:
        """Run the CA server and the tick loop until cancelled."""
        context = AsyncioContext(self.ioc.pvdb, interfaces=[self.config.ca_interface])
        tick_task = asyncio.ensure_future(self.tick_forever())
        try:
            await context.run(log_pv_names=False)
        finally:
            tick_task.cancel()
            try:
                await tick_task
            except asyncio.CancelledError:
                pass

    def stop(self) -> None:
        if self.server is not None:
            self.server.stop()
            self.server = None


def log_startup_banner(config: AdapterConfig, server: TickServer) -> None:
    """Print what is being served, and shout about anything unsafe.

    Section 8 calls for naming discipline because once the simulation is
    indistinguishable to the control system, it becomes possible to lose track
    of which one you are connected to.  The banner exists so that nobody has to
    guess, and so that an unusual bind address is noticed at the moment it is
    chosen rather than during an incident.
    """
    logger.info("=" * 72)
    logger.info("Isaac Sim EPICS adapter - SIMULATED MOTORS, NOT REAL HARDWARE")
    logger.info("=" * 72)
    logger.info("PV prefix        : %s", config.prefix)
    logger.info("Channel Access   : bound to %s", config.ca_interface)
    logger.info("Isaac Sim tick   : %s/tick", server.url)
    logger.info("Update rate      : %.1f Hz   stale after %.2fs",
                config.update_rate, config.stale_timeout)
    logger.info("-" * 72)
    for axis in config.axes:
        logger.info(
            "  %-18s joint=%-12s %7.3f .. %-7.3f %-4s  (1 %s = %g sim units)",
            config.prefix + axis.motor, axis.joint,
            axis.low_limit, axis.high_limit, axis.egu,
            axis.egu, axis.sim_units_per_egu,
        )
    logger.info("-" * 72)
    for warning in config.naming_warnings():
        logger.warning("SAFETY: %s", warning)
    logger.info(
        "Until Isaac Sim connects, every readback is INVALID and .DMOV is not "
        "asserted. That is intentional: a frozen readback must never look settled."
    )
    logger.info("=" * 72)


def configure_logging(verbose: bool = False) -> None:
    logging.basicConfig(
        level=logging.DEBUG if verbose else logging.INFO,
        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )
    # caproto's server logs every search at INFO, which drowns everything else.
    logging.getLogger("caproto").setLevel(logging.WARNING)


def run_adapter(config: AdapterConfig, verbose: bool = False) -> None:
    """Blocking entry point: start everything and serve until interrupted."""
    configure_logging(verbose)
    adapter = Adapter(config)
    server = adapter.start_http()
    log_startup_banner(config, server)
    try:
        asyncio.run(adapter.run())
    except KeyboardInterrupt:
        logger.info("interrupted; shutting down")
    finally:
        adapter.stop()


def _unused(*args: Any) -> None:
    """Placeholder to keep linters quiet about optional imports."""
