"""Command line entry point for the adapter."""

from __future__ import annotations

import argparse
import sys
from typing import List, Optional

from .app import run_adapter
from .config import AdapterConfig, ConfigError, default_config


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="isaac-epics-ioc",
        description=(
            "Serve an Isaac Sim articulation as EPICS motor records. "
            "Runs the HTTP tick endpoint and the Channel Access server in one process."
        ),
        epilog=(
            "Safety: this serves SIMULATED motors. Keep the PV prefix distinct from any "
            "real beamline IOC, and do not bind Channel Access to a controls subnet."
        ),
    )
    parser.add_argument(
        "-c", "--config", metavar="FILE",
        help="JSON configuration file. Without one, the built-in 4-DOF x/y/z/theta "
             "stage is served.",
    )
    parser.add_argument("--prefix", help="Override the PV prefix (e.g. SIM:TWIN:)")
    parser.add_argument("--http-host", help="Bind address for the tick endpoint")
    parser.add_argument("--http-port", type=int, help="Bind port for the tick endpoint")
    parser.add_argument(
        "--ca-interface",
        help="Interface for the Channel Access server. Defaults to loopback; setting "
             "anything else makes simulated motors visible to other hosts.",
    )
    parser.add_argument("--rate", type=float, dest="update_rate",
                        help="Motor update rate in Hz (default 20)")
    parser.add_argument("--stale-timeout", type=float,
                        help="Seconds without a frame before readbacks go INVALID")
    parser.add_argument("--list-pvs", action="store_true",
                        help="Print every PV that would be served, then exit")
    parser.add_argument("-v", "--verbose", action="store_true", help="Debug logging")
    return parser


def main(argv: Optional[List[str]] = None) -> int:
    args = build_parser().parse_args(argv)

    try:
        config = (
            AdapterConfig.from_file(args.config) if args.config else default_config()
        )
        config = config.with_overrides(
            prefix=args.prefix,
            http_host=args.http_host,
            http_port=args.http_port,
            ca_interface=args.ca_interface,
            update_rate=args.update_rate,
            stale_timeout=args.stale_timeout,
        )
    except (ConfigError, OSError) as exc:
        print(f"configuration error: {exc}", file=sys.stderr)
        return 2

    if args.list_pvs:
        from .ioc import build_ioc
        from .plant import SharedPlant
        ioc = build_ioc(config, SharedPlant())
        for name in sorted(ioc.pvdb):
            print(name)
        print(f"\n{len(ioc.pvdb)} PVs (plus 153 motor record fields per axis "
              f"served via caproto's record support)", file=sys.stderr)
        return 0

    run_adapter(config, verbose=args.verbose)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
