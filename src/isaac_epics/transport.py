"""HTTP transport: the endpoint Isaac Sim posts frames to.

Isaac Sim is the client and this is the server (section 7.2).  The adapter
never opens a connection *into* the simulator, which matters for a component
whose existing IPC surface (``isaac-sim-remote`` on port 8226) executes
arbitrary Python by design.  Section 8 is explicit that that trust model must
not be copied: this server parses JSON into a fixed schema and can do exactly
two things -- record joint positions and hand back joint targets.  There is no
code path from a request to an interpreter.

Why stdlib ``http.server``
--------------------------
Section 7.6 pushes the "no third-party packages" principle hard on the Isaac
Sim side.  On the adapter side we already depend on caproto, so the constraint
is weaker -- but at motor readback rates (~10-60 Hz, single-digit joints) a
threaded stdlib server is not the bottleneck, and ``curl`` working as a
debugger is worth more here than throughput.
"""

from __future__ import annotations

import json
import logging
import socket
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Callable, Dict, Optional, Tuple

from .plant import SharedPlant
from .protocol import MAX_BODY_BYTES, PROTOCOL_VERSION, ProtocolError, SimFrame, TargetFrame

logger = logging.getLogger(__name__)

#: Per-request socket timeout.  A wedged client must not hold a worker thread.
REQUEST_TIMEOUT = 10.0

#: How much of an over-sized body we are willing to read and throw away so
#: that the client receives a clean 413 rather than a broken pipe.  A client
#: that is mid-upload when we reply gets its write refused, which surfaces as
#: a connection error and hides the status code we chose.  Draining a bounded
#: amount fixes the realistic case (a slightly-too-big frame) without letting
#: an absurd body dictate how long we spend on it.
MAX_DRAIN_BYTES = 1024 * 1024


class _Handler(BaseHTTPRequestHandler):
    """Request handler.  One instance per request, per stdlib convention."""

    protocol_version = "HTTP/1.1"  # keep-alive: the client ticks continuously
    server_version = "isaac-epics"
    sys_version = ""  # do not advertise the Python version

    # Injected by the server factory below.
    plant: SharedPlant
    diagnostics: Callable[[], Dict[str, Any]]

    def log_message(self, fmt: str, *args: Any) -> None:
        # BaseHTTPRequestHandler logs every request to stderr by default; at
        # 60 Hz that is unusable. Route to the logger at DEBUG instead.
        logger.debug("%s - %s", self.address_string(), fmt % args)

    # -- helpers -----------------------------------------------------------

    def _send_json(self, status: int, payload: Dict[str, Any]) -> None:
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _read_body(self) -> Optional[bytes]:
        """Read the request body, enforcing the size cap.

        Returns ``None`` after sending an error response.

        Every rejection path here closes the connection. The body was never
        read, so the socket still holds the rest of it; replying on a
        keep-alive connection would make the next request parse from the
        middle of this one's payload, and the client would see a reset rather
        than the status code we carefully chose.
        """
        raw_length = self.headers.get("Content-Length")
        if raw_length is None:
            self.close_connection = True
            self._send_json(411, {"error": "Content-Length required"})
            return None
        try:
            length = int(raw_length)
        except ValueError:
            self.close_connection = True
            self._send_json(400, {"error": "malformed Content-Length"})
            return None
        if length < 0 or length > MAX_BODY_BYTES:
            self.close_connection = True
            self._drain(length)
            self._send_json(413, {"error": f"body too large (max {MAX_BODY_BYTES} bytes)"})
            return None
        body = self.rfile.read(length)
        if len(body) != length:
            self.close_connection = True
            self._send_json(400, {"error": "truncated request body"})
            return None
        return body

    def _drain(self, length: int) -> None:
        """Read and discard up to :data:`MAX_DRAIN_BYTES` of a rejected body."""
        remaining = min(max(length, 0), MAX_DRAIN_BYTES)
        while remaining > 0:
            chunk = self.rfile.read(min(remaining, 65536))
            if not chunk:
                break
            remaining -= len(chunk)

    # -- routes ------------------------------------------------------------

    def do_POST(self) -> None:  # noqa: N802 - stdlib naming
        path = self.path.split("?", 1)[0].rstrip("/") or "/"
        if path != "/tick":
            self._send_json(404, {"error": f"no such endpoint: {path}"})
            return

        body = self._read_body()
        if body is None:
            return

        try:
            decoded = json.loads(body.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            self._send_json(400, {"error": f"invalid JSON: {exc}"})
            return

        try:
            frame = SimFrame.from_json(decoded)
        except ProtocolError as exc:
            # Rejected outright. A frame we cannot fully trust is never
            # partially applied.
            logger.warning("rejected frame from %s: %s", self.address_string(), exc)
            self._send_json(422, {"error": str(exc)})
            return

        targets = self.plant.submit_frame(frame)
        response = TargetFrame(targets=targets, t_wall=0.0, axes=self.diagnostics())
        self._send_json(200, response.to_json())

    def do_GET(self) -> None:  # noqa: N802 - stdlib naming
        """Read-only introspection, so ``curl`` is a usable debugger."""
        path = self.path.split("?", 1)[0].rstrip("/") or "/"
        if path in ("/", "/state"):
            state = self.plant.get_state()
            self._send_json(200, {
                "protocol": PROTOCOL_VERSION,
                "link_state": state.link_state,
                "connected": state.connected,
                "playing": state.playing,
                "age": round(state.age, 4) if state.age != float("inf") else None,
                "t_sim": state.t_sim,
                "seq": state.seq,
                "frames": state.frames,
                "positions": dict(state.positions),
                "targets": self.plant.targets,
                "axes": self.diagnostics(),
            })
            return
        if path == "/healthz":
            state = self.plant.get_state()
            # 503 when the simulator link is down, so an external check can
            # distinguish "adapter running" from "adapter actually driving a
            # simulation". Those are different facts and must not be conflated.
            self._send_json(200 if state.connected else 503,
                            {"link_state": state.link_state, "frames": state.frames})
            return
        self._send_json(404, {"error": f"no such endpoint: {path}"})


class _Server(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True
    timeout = REQUEST_TIMEOUT

    def handle_error(self, request: Any, client_address: Any) -> None:
        # The default prints a traceback to stderr for every dropped
        # connection, which is noise when a simulator restarts.
        logger.debug("connection error from %s", client_address, exc_info=True)


class TickServer:
    """Runs the HTTP endpoint on its own thread.

    Args:
        plant: The shared plant that frames are written into.
        host: Bind address.  Loopback by default.
        port: Bind port.  Pass ``0`` to let the OS choose (used by tests);
            the chosen port is then available as :attr:`port`.
        diagnostics: Callable returning a JSON-serialisable dict of per-axis
            diagnostics, echoed back to the client and exposed on ``/state``.
    """

    def __init__(
        self,
        plant: SharedPlant,
        host: str = "127.0.0.1",
        port: int = 8227,
        diagnostics: Optional[Callable[[], Dict[str, Any]]] = None,
    ) -> None:
        self._plant = plant
        self._diagnostics = diagnostics or (lambda: {})

        handler = type("_BoundHandler", (_Handler,), {
            "plant": plant,
            "diagnostics": staticmethod(self._diagnostics),
        })
        self._httpd = _Server((host, port), handler)
        self._thread: Optional[threading.Thread] = None

    @property
    def address(self) -> Tuple[str, int]:
        sockname = self._httpd.socket.getsockname()
        return sockname[0], sockname[1]

    @property
    def port(self) -> int:
        return self.address[1]

    @property
    def url(self) -> str:
        host, port = self.address
        if host in ("0.0.0.0", "::"):
            host = "127.0.0.1"
        if ":" in host:
            host = f"[{host}]"
        return f"http://{host}:{port}"

    def start(self) -> "TickServer":
        if self._thread is not None:
            raise RuntimeError("server already started")
        self._thread = threading.Thread(
            target=self._httpd.serve_forever, name="isaac-epics-http", daemon=True,
        )
        self._thread.start()
        logger.info("tick endpoint listening on %s/tick", self.url)
        return self

    def stop(self) -> None:
        self._httpd.shutdown()
        self._httpd.server_close()
        if self._thread is not None:
            self._thread.join(timeout=5.0)
            self._thread = None

    def __enter__(self) -> "TickServer":
        return self.start()

    def __exit__(self, *exc: Any) -> None:
        self.stop()


def port_is_free(host: str, port: int) -> bool:
    """Check a bind address before starting, for a clearer startup error."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            probe.bind((host, port))
        except OSError:
            return False
    return True
