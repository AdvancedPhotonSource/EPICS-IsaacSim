"""Watch live EPICS PVs drive a model's joints. No beamline, no GPU, one command.

    python examples/mirror_demo.py

This is **direction A** -- mirror mode. Real hardware's readbacks drive the
simulated articulation, read-only, so a 3D model moves because the instrument
is moving. It is the opposite direction from the rest of the README, where the
twin stands in for hardware so scan code can be tested against it.

What this script stands up, all on loopback:

    stand-in "beamline" IOC  (SIM:REAL:)   <- caput, as if an operator moved it
            |
            |  real Channel Access monitors
            v
    isaac-epics-mirror   read-only subscriptions to SIM:REAL:*.RBV
            |
            |  HTTP tick: joint targets in simulation units
            v
    stand-in "twin"      (fake_sim, where Isaac Sim would be)

Two of those four are stand-ins for things you would not have on a laptop. In
a real deployment the first is your beamline's own IOC -- this script fakes it
with a second adapter instance, because the demo needs *some* source of
live PVs -- and the last is Isaac Sim running your USD articulation.

**The middle piece is the real thing.** `isaac-epics-mirror` here is exactly
the process you would run against a beamline, driven by real Channel Access
monitors over a real network stack. Nothing about it is mocked.

What it demonstrates, in order:

1. A move on a "real" motor appears as a joint target in simulation units,
   converted from engineering units (10 mm -> 0.010 m, 45 deg -> 0.785 rad).
2. The twin's joints follow.
3. When the source IOC dies, every channel is reported **stale** rather than
   holding its last value. A model frozen at the last known position, with no
   indication it is frozen, is the failure this refuses to produce.
"""

import argparse
import json
import os
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO, "src"))

# Channel Access is loopback-only here, and this must be set before caproto
# builds a client Context -- it reads the address list at construction. It
# applies to this process as well as the ones it starts: everything the demo
# serves is bound to 127.0.0.1, so a client left on the default broadcast
# address list simply never finds it.
os.environ.setdefault("EPICS_CA_ADDR_LIST", "127.0.0.1")
os.environ.setdefault("EPICS_CA_AUTO_ADDR_LIST", "NO")

#: What the stand-in beamline is asked to do, and which joint it should drive.
MOVES = [
    ("m1", 10.0, "mm", "x_axis"),
    ("m3", 5.0, "mm", "z_axis"),
    ("m4", 45.0, "deg", "theta_axis"),
]


def free_port():
    """Ask the OS for a port nothing is using.

    Fixed ports are how a demo fails on someone else's machine: an adapter
    left running from an earlier session already owns 8227, the second bind
    fails, and the symptom is a mirror that connects to the wrong process.
    """
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        probe.bind(("127.0.0.1", 0))
        return probe.getsockname()[1]


class Stack:
    """Every process this demo starts, and how to take them all down again."""

    def __init__(self, verbose=False):
        self.procs = []
        self.verbose = verbose
        self.env = dict(os.environ, PYTHONUNBUFFERED="1")

    def spawn(self, name, argv):
        out = None if self.verbose else subprocess.DEVNULL
        proc = subprocess.Popen(argv, cwd=REPO, env=self.env,
                                stdout=out, stderr=subprocess.STDOUT)
        self.procs.append((name, proc))
        return proc

    def stop(self, *names):
        for name, proc in self.procs:
            if (not names or name in names) and proc.poll() is None:
                proc.terminate()
        deadline = time.time() + 5
        for name, proc in self.procs:
            if not names or name in names:
                while proc.poll() is None and time.time() < deadline:
                    time.sleep(0.05)
                if proc.poll() is None:
                    proc.kill()


def wait_for_http(port, timeout=30):
    """Block until the tick endpoint answers, instead of guessing with sleep."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            with urllib.request.urlopen(
                    f"http://127.0.0.1:{port}/state", timeout=1) as response:
                json.loads(response.read().decode())
                return True
        except (urllib.error.URLError, OSError, ValueError):
            time.sleep(0.2)
    raise TimeoutError(f"tick endpoint on :{port} never came up")


def state(port):
    with urllib.request.urlopen(f"http://127.0.0.1:{port}/state", timeout=3) as response:
        return json.loads(response.read().decode())


def wait_for_frames(port, timeout=30):
    """Block until something is actually posting frames to that endpoint."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if state(port).get("connected"):
            return True
        time.sleep(0.2)
    raise TimeoutError(f"nothing connected to the tick endpoint on :{port}")


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Mirror mode end to end, with no beamline and no GPU.")
    parser.add_argument("--prefix", default="SIM:REAL:",
                        help="PV prefix for the stand-in beamline IOC")
    parser.add_argument("-v", "--verbose", action="store_true",
                        help="Show the output of every process started")
    args = parser.parse_args(argv)

    try:
        from caproto.threading.client import Context
    except ImportError:
        print("caproto is required: pip install -e .", file=sys.stderr)
        return 2

    beamline_tick, mirror_tick = free_port(), free_port()
    stack = Stack(verbose=args.verbose)
    config = os.path.join(HERE, "mirror.json")

    # The mapping file drives both the mirror and this script's reporting, so
    # the demo cannot describe something different from what it ran.
    mappings = {entry["pv"]: entry for entry in json.load(open(config))["mirror"]}

    print(__doc__.split("What it demonstrates")[0].rstrip())
    print("=" * 72)

    try:
        print(f"\n[1/4] stand-in beamline IOC on {args.prefix} (tick :{beamline_tick})")
        stack.spawn("beamline", [sys.executable, "-m", "isaac_epics.cli",
                                 "--prefix", args.prefix,
                                 "--http-port", str(beamline_tick)])
        wait_for_http(beamline_tick)
        stack.spawn("beamline_sim", [sys.executable, os.path.join(HERE, "fake_sim.py"),
                                     "--url", f"http://127.0.0.1:{beamline_tick}",
                                     "--quiet"])
        wait_for_frames(beamline_tick)
        print("      up: these PVs now behave like real motors")

        print(f"\n[2/4] mirror mode -- read-only subscriptions (tick :{mirror_tick})")
        stack.spawn("mirror", [sys.executable, "-m", "isaac_epics.mirror",
                               "-c", config, "--http-port", str(mirror_tick)])
        wait_for_http(mirror_tick)
        for pv, entry in mappings.items():
            print(f"      {pv:<22} -> joint {entry['joint']}")

        print("\n[3/4] twin stand-in attached (Isaac Sim would be here)")
        stack.spawn("twin", [sys.executable, os.path.join(HERE, "fake_sim.py"),
                             "--url", f"http://127.0.0.1:{mirror_tick}", "--quiet"])
        wait_for_frames(mirror_tick)

        context = Context()
        pvs = {}
        for motor, _, _, _ in MOVES:
            pv, = context.get_pvs(args.prefix + motor)
            pv.wait_for_connection(timeout=20)
            pvs[motor] = pv

        print("\n[4/4] moving the 'real' motors\n")
        for motor, target, egu, _ in MOVES:
            print(f"      caput {args.prefix}{motor:<4} {target:7.1f} {egu}")
            pvs[motor].write((target,), wait=True)

        # Settle: the real motor ramps, the monitor fires, the joint follows.
        time.sleep(6)

        snapshot = state(mirror_tick)
        channels = snapshot.get("axes", {})
        positions = snapshot.get("positions", {})

        print("\n" + "-" * 72)
        print(f"{'real PV':<22} {'value':>12} {'joint':>13} "
              f"{'twin (sim units)':>16} {'stale':>7}")
        print("-" * 72)
        for pv in sorted(channels):
            info = channels[pv]
            joint = info["joint"]
            value = info.get("value")
            measured = positions.get(joint)
            shown = "-" if value is None else f"{value:.4f}"
            twin = "-" if measured is None else f"{measured:.6f}"
            print(f"{pv:<22} {shown:>12} {joint:>13} {twin:>16} "
                  f"{str(info.get('stale')):>7}")
        print("-" * 72)
        print("mm -> metres and degrees -> radians, converted by the 'units' in")
        print(f"{os.path.relpath(config, REPO)}. The twin never sees an engineering unit.")

        print("\n[extra] killing the beamline IOC -- a frozen model must say so")
        stack.stop("beamline", "beamline_sim")
        time.sleep(7)
        stale = state(mirror_tick).get("axes", {})
        flags = {pv.rsplit(":", 1)[-1]: info["stale"] for pv, info in sorted(stale.items())}
        print(f"        stale flags: {flags}")
        if all(flags.values()):
            print("        every channel reported stale, as required: the joints hold")
            print("        their last position, and the staleness says not to trust it.")
        else:
            print("        WARNING: a channel did not go stale. That is a bug.")
            return 1

        print("\nNext: replace the stand-in beamline with your real PV names in")
        print(f"{os.path.relpath(config, REPO)}, and the twin stand-in with Isaac Sim.")
        print("See docs/mirror.md.")
        return 0
    except KeyboardInterrupt:
        print("\ninterrupted")
        return 130
    finally:
        stack.stop()


if __name__ == "__main__":
    raise SystemExit(main())
