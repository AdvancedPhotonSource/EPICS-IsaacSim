"""Run a real Bluesky scan against the Isaac Sim twin.

This is the milestone the design document names in section 10: if a Bluesky
``scan`` steps correctly against the simulation, direction A has no
architectural unknowns left and direction B reduces to motor-record
completeness.

Nothing here is twin-aware.  ``EpicsMotor`` is ophyd's stock device, the plan
is bluesky's stock ``scan``, and the only thing that identifies the simulation
is the PV prefix.  That is the entire point of section 5: the experiment runs
unmodified, so what you test is the scan, not a reimplementation of it.

Requires the adapter and a simulator (real or ``examples/fake_sim.py``)::

    isaac-epics-ioc &
    python examples/fake_sim.py &
    python examples/bluesky_scan.py
"""

import argparse
import os
import sys

os.environ.setdefault("OPHYD_CONTROL_LAYER", "caproto")
os.environ.setdefault("EPICS_CA_ADDR_LIST", "127.0.0.1")
os.environ.setdefault("EPICS_CA_AUTO_ADDR_LIST", "NO")

import bluesky.plans as bp  # noqa: E402
from bluesky import RunEngine  # noqa: E402
from ophyd import Component as Cpt  # noqa: E402
from ophyd import Device, EpicsMotor, EpicsSignalRO  # noqa: E402


class SimStage(Device):
    """The 4-DOF stage, as an ordinary ophyd device.

    Note what is *not* here: no simulation awareness, no special handling, no
    twin-specific signals in the motors themselves.  These are plain
    ``EpicsMotor`` components.
    """

    x = Cpt(EpicsMotor, "m1")
    y = Cpt(EpicsMotor, "m2")
    z = Cpt(EpicsMotor, "m3")
    theta = Cpt(EpicsMotor, "m4")

    #: The one twin-specific signal, deliberately outside the motors so it can
    #: never be mistaken for a motor record field. Worth checking before a run:
    #: it distinguishes a live simulation from a paused or vanished one.
    link = Cpt(EpicsSignalRO, "LINK", string=True, kind="config")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--prefix", default="SIM:TWIN:")
    parser.add_argument("--start", type=float, default=-5.0)
    parser.add_argument("--stop", type=float, default=5.0)
    parser.add_argument("--points", type=int, default=6)
    parser.add_argument("--timeout", type=float, default=30.0)
    args = parser.parse_args(argv)

    print(f"connecting to {args.prefix} ...")
    stage = SimStage(args.prefix, name="stage")
    try:
        stage.wait_for_connection(timeout=args.timeout)
    except Exception as exc:
        print(f"could not connect: {exc}", file=sys.stderr)
        print("is the adapter running?  isaac-epics-ioc &", file=sys.stderr)
        return 1

    link = stage.link.get()
    print(f"link state: {link}")
    if link != "running":
        # Refusing to scan a paused or dead simulation is the honest default.
        # A scan against a frozen readback produces a dataset that looks
        # complete and means nothing.
        print(f"refusing to scan: simulation is '{link}', not 'running'", file=sys.stderr)
        return 2

    readings = []

    def collect(name, doc):
        if name == "event":
            readings.append(doc["data"])

    engine = RunEngine({})
    print(f"\nscan: stage.x from {args.start} to {args.stop} in {args.points} points")
    print("-" * 64)
    engine(bp.scan([], stage.x, args.start, args.stop, args.points), collect)

    print(f"{'requested':>12} {'readback':>14} {'error':>12}")
    print("-" * 64)
    step = (args.stop - args.start) / max(args.points - 1, 1)
    worst = 0.0
    for index, data in enumerate(readings):
        want = args.start + index * step
        rbv = data["stage_x"]
        worst = max(worst, abs(rbv - want))
        print(f"{want:12.4f} {rbv:14.6f} {rbv - want:+12.2e}")
    print("-" * 64)
    print(f"{len(readings)} points collected, worst readback error {worst:.3e} mm")
    return 0 if len(readings) == args.points else 3


if __name__ == "__main__":
    raise SystemExit(main())
