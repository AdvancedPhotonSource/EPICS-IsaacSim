"""Isaac Sim side of the EPICS adapter link.

**This file uses the Python standard library only.**  That is a hard
requirement, not a preference (architecture decision 7.6).  Isaac Sim's
embedded interpreter has pinned versions that Kit, USD and warp were built
against -- numpy especially -- and a resolver pulling a different numpy breaks
the simulator in ways that present as simulator bugs, in an environment
NVIDIA's troubleshooting assumes is stock.  Nothing here may import anything
outside the stdlib.  No caproto, no requests, no numpy, no ZMQ.

Drop this single file next to your standalone script, or copy it into the
container.  It has no package dependencies and no installation step.

Usage
-----
::

    from epics_tick import EpicsTickClient

    client = EpicsTickClient("http://127.0.0.1:8227", rate_hz=30.0)
    client.start()

    while simulation_app.is_running():
        world.step(render=True)
        targets = client.exchange(
            positions={"x_axis": float(dof[0]), ...},
            t_sim=world.current_time,
            playing=world.is_playing(),
        )
        for joint, value in targets.items():
            ...  # apply to the articulation

``exchange()`` never blocks and never raises: it swaps the latest state into a
background worker and returns the most recently received targets.  A 60 Hz
render loop cannot afford a synchronous round trip, and a control link that
can stall the renderer is a control link that will be removed by whoever is
trying to get a frame rate.

The client knows nothing about EPICS, motors, limits or engineering units.  It
reports joint positions in simulation units and applies joint targets in
simulation units.  Everything else is the adapter's problem.
"""

import json
import threading
import time

try:  # Python 3
    import http.client as httplib
    from urllib.parse import urlparse
except ImportError as exc:  # pragma: no cover - Python 2 is not supported
    raise RuntimeError("epics_tick requires Python 3") from exc

PROTOCOL_VERSION = 1

__all__ = ["EpicsTickClient", "LinkStats"]


class LinkStats(object):
    """Snapshot of link health, for display in the simulator's UI or logs."""

    __slots__ = ("connected", "sent", "received", "errors", "last_error",
                 "last_rx_age", "round_trip", "suppressed")

    def __init__(self, connected, sent, received, errors, last_error,
                 last_rx_age, round_trip, suppressed=0):
        self.connected = connected
        self.sent = sent
        self.received = received
        self.errors = errors
        self.last_error = last_error
        self.last_rx_age = last_rx_age
        self.round_trip = round_trip
        self.suppressed = suppressed

    def __repr__(self):
        state = "connected" if self.connected else "DISCONNECTED"
        return ("<LinkStats %s sent=%d recv=%d err=%d suppressed=%d rtt=%.1fms "
                "last_error=%r>"
                % (state, self.sent, self.received, self.errors, self.suppressed,
                   self.round_trip * 1e3, self.last_error))


class EpicsTickClient(object):
    """Non-blocking client that exchanges joint state for joint targets.

    Args:
        url: Base URL of the adapter, e.g. ``http://127.0.0.1:8227``.
        rate_hz: How often the worker thread posts a frame.  Motor readback
            rates are ~10 Hz; 30 Hz is ample and keeps the adapter's
            trajectory streaming smooth.  Posting at full render rate wastes
            effort -- the adapter's state machine runs at its own rate.
        timeout: Per-request socket timeout in seconds.
        stale_after: Seconds without a successful exchange before
            :attr:`connected` reports ``False``.
        max_frame_age: The worker refuses to post a frame older than this.
            This is a safety interlock, not a tuning knob -- see below.

    Why ``max_frame_age`` exists
    ----------------------------
    The worker posts on its own schedule, independently of the render loop.
    If the simulation's loop stops calling :meth:`exchange` -- the script
    crashed, the stage is reloading, Kit is wedged, a physics step hung --
    the worker would happily keep posting the last snapshot it was given.
    The adapter would see frames arriving on time and report a healthy link,
    while the readback it serves is frozen.

    That is the exact failure the design forbids: a frozen readback presented
    as live.  Worse, it defeats the adapter's own staleness detection, because
    from the adapter's side the link genuinely is alive.  Only the client can
    tell the difference between "the simulation is paused" (the loop is
    running and reporting ``playing=False``) and "the loop is gone".

    So the worker stops posting once the newest frame exceeds this age, and
    the adapter's staleness logic does the rest.  Keep it comfortably below
    the adapter's ``stale_timeout``.
    """

    def __init__(self, url="http://127.0.0.1:8227", rate_hz=30.0, timeout=0.5,
                 stale_after=2.0, max_frame_age=0.5):
        parsed = urlparse(url)
        if parsed.scheme != "http":
            raise ValueError("only http:// is supported, got %r" % (url,))
        if not parsed.hostname:
            raise ValueError("could not parse a host from %r" % (url,))
        if rate_hz <= 0:
            raise ValueError("rate_hz must be positive")

        self._host = parsed.hostname
        self._port = parsed.port or 80
        self._path = (parsed.path.rstrip("/") or "") + "/tick"
        self._period = 1.0 / float(rate_hz)
        self._timeout = float(timeout)
        self._stale_after = float(stale_after)
        if max_frame_age <= 0:
            raise ValueError("max_frame_age must be positive")
        self._max_frame_age = float(max_frame_age)

        self._lock = threading.Lock()
        self._outbound = None          # None until the loop supplies a frame
        self._frame_time = None        # wall clock when it was supplied
        self._suppressed = 0           # frames withheld as too old
        self._targets = {}
        self._seq = 0
        self._sent = 0
        self._received = 0
        self._errors = 0
        self._last_error = None
        self._last_rx = None
        self._round_trip = 0.0

        self._conn = None
        self._stop = threading.Event()
        self._thread = None

    # -- lifecycle ---------------------------------------------------------

    def start(self):
        """Start the background worker.  Returns ``self`` for chaining."""
        if self._thread is not None:
            raise RuntimeError("client already started")
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="epics-tick")
        self._thread.daemon = True  # never block simulator shutdown
        self._thread.start()
        return self

    def stop(self, timeout=2.0):
        """Stop the worker and close the connection."""
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=timeout)
            self._thread = None
        self._close()

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.stop()

    # -- per-frame API -----------------------------------------------------

    def exchange(self, positions, t_sim, playing):
        """Publish the latest joint state and return the latest targets.

        This is O(number of joints) and takes a lock for a few microseconds.
        It performs no I/O, never blocks on the network, and never raises.

        Args:
            positions: Mapping of joint name -> position in simulation units
                (metres for prismatic joints, radians for revolute).
            t_sim: Simulation time in seconds.
            playing: Whether the simulation is actually stepping.  Report this
                honestly: the adapter uses it to refuse to complete a move
                against a frozen readback.

        Returns:
            dict: joint name -> commanded position in simulation units.  Empty
            until the first successful exchange.  Joints absent from the
            mapping should be left alone, not zeroed.
        """
        snapshot = {}
        for key, value in positions.items():
            try:
                snapshot[str(key)] = float(value)
            except (TypeError, ValueError):
                continue  # a non-numeric joint value is dropped, not guessed
        with self._lock:
            self._outbound = {
                "positions": snapshot,
                "t_sim": float(t_sim),
                "playing": bool(playing),
            }
            self._frame_time = time.time()
            return dict(self._targets)

    @property
    def connected(self):
        """Whether a successful exchange happened recently."""
        with self._lock:
            if self._last_rx is None:
                return False
            return (time.time() - self._last_rx) <= self._stale_after

    def stats(self):
        """Return a :class:`LinkStats` snapshot."""
        with self._lock:
            age = None if self._last_rx is None else (time.time() - self._last_rx)
            return LinkStats(
                connected=(age is not None and age <= self._stale_after),
                sent=self._sent, received=self._received, errors=self._errors,
                last_error=self._last_error,
                last_rx_age=age, round_trip=self._round_trip,
                suppressed=self._suppressed,
            )

    # -- worker ------------------------------------------------------------

    def _run(self):
        backoff = self._period
        while not self._stop.is_set():
            started = time.time()
            try:
                self._exchange_once()
                backoff = self._period
            except Exception as exc:  # noqa: BLE001 - the link must never kill the sim
                with self._lock:
                    self._errors += 1
                    self._last_error = "%s: %s" % (type(exc).__name__, exc)
                self._close()
                # Back off on a dead adapter so a stopped IOC does not spin
                # the CPU inside the simulator process.
                backoff = min(backoff * 2.0, 2.0)
            elapsed = time.time() - started
            self._stop.wait(max(0.0, backoff - elapsed))

    def _connect(self):
        if self._conn is None:
            self._conn = httplib.HTTPConnection(
                self._host, self._port, timeout=self._timeout,
            )
        return self._conn

    def _close(self):
        conn, self._conn = self._conn, None
        if conn is not None:
            try:
                conn.close()
            except Exception:  # noqa: BLE001
                pass

    def _exchange_once(self):
        with self._lock:
            frame, supplied = self._outbound, self._frame_time
            if frame is None or supplied is None:
                return  # nothing to report yet; the loop has not ticked once
            age = time.time() - supplied
            if age > self._max_frame_age:
                # The render loop has stopped feeding us. Go quiet rather than
                # heartbeat a frozen position: the adapter must be allowed to
                # notice, raise alarms and stop asserting .DMOV.
                self._suppressed += 1
                self._close()
                return
            self._seq += 1
            payload = {
                "protocol": PROTOCOL_VERSION,
                "positions": frame["positions"],
                "t_sim": frame["t_sim"],
                "playing": frame["playing"],
                "seq": self._seq,
            }
        body = json.dumps(payload).encode("utf-8")

        started = time.time()
        conn = self._connect()
        conn.request("POST", self._path, body=body, headers={
            "Content-Type": "application/json",
            "Content-Length": str(len(body)),
        })
        response = conn.getresponse()
        raw = response.read()
        if response.status != 200:
            raise RuntimeError("adapter returned HTTP %d: %s"
                               % (response.status, raw[:200].decode("utf-8", "replace")))
        decoded = json.loads(raw.decode("utf-8"))
        targets = _parse_targets(decoded)
        rtt = time.time() - started

        with self._lock:
            self._targets = targets
            self._sent += 1
            self._received += 1
            self._last_rx = time.time()
            self._last_error = None
            self._round_trip = rtt


def _parse_targets(decoded):
    """Validate the adapter's response before letting it touch a joint.

    The client trusts the adapter no further than the adapter trusts the
    client.  A NaN or infinite target silently applied to an articulation
    produces a physics explosion that looks like a simulation bug and is
    painful to trace back to a control message.
    """
    if not isinstance(decoded, dict):
        raise ValueError("response was not a JSON object")
    version = decoded.get("protocol", PROTOCOL_VERSION)
    if version != PROTOCOL_VERSION:
        raise ValueError("adapter speaks protocol %r, client speaks %r"
                         % (version, PROTOCOL_VERSION))
    raw = decoded.get("targets") or {}
    if not isinstance(raw, dict):
        raise ValueError("'targets' was not a JSON object")
    targets = {}
    for key, value in raw.items():
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError("target %r is not a number: %r" % (key, value))
        value = float(value)
        # NaN != NaN, and infinity fails every range check downstream.
        if value != value or value in (float("inf"), float("-inf")):
            raise ValueError("target %r is not finite: %r" % (key, value))
        targets[str(key)] = value
    return targets
