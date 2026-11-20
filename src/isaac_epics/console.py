"""Interactive instrument-control console for the Isaac Sim twin.

This deliberately **invents no command language.** Bluesky already ships the
SPEC-style vocabulary that beamline users know, as IPython magics:

======================  ==================  ===============================
Command                 SPEC equivalent     Meaning
======================  ==================  ===============================
``%mov x 10``           ``mv``              move to an absolute position
``%movr x 2``           ``mvr``             move relative
``%wa``                 ``wa``              where-all: list every positioner
``%ct``                 ``ct``              count the detectors
======================  ==================  ===============================

Writing a bespoke CLI would mean beamline staff learning a second vocabulary
that works only against the twin -- which is precisely the kind of
"reimplementation you end up testing instead of the real thing" that section 5
of the design document warns about. The same argument that says the twin
should speak the motor record says the console should speak Bluesky.

So all this module does is assemble: build the ophyd device from the adapter
config, bind each axis to its alias in the namespace, point the magics at one
shared RunEngine, attach the live table/plot callback, and report the link
state honestly before handing over the prompt.

Launch it with ``isaac-epics-console``.
"""

from __future__ import annotations

import argparse
import os
import sys
from typing import Any, Dict, List, Optional

from .config import AdapterConfig, ConfigError, default_config

BANNER = r"""
  Isaac Sim EPICS console - SIMULATED MOTORS, NOT REAL HARDWARE
  ------------------------------------------------------------------------
  %mov x 10          move x to 10 (absolute)      %wa      where-all
  %movr x -2         move x by -2 (relative)      %ct      count detectors
  x.position         read a value                 status() link + positions
  RE(bp.scan([], x, -10, 10, 11))                 run a scan
  ------------------------------------------------------------------------
"""


def build_namespace(config: AdapterConfig, timeout: float = 20.0) -> Dict[str, Any]:
    """Assemble the console namespace. Importable, so it can be tested."""
    # ophyd defaults to pyepics, which is absent on APS hosts; caproto is
    # already a hard dependency of the adapter.
    os.environ.setdefault("OPHYD_CONTROL_LAYER", "caproto")

    import bluesky.plan_stubs as bps
    import bluesky.plans as bp
    from bluesky import RunEngine

    from .devices import axis_table, connect_stage

    stage = connect_stage(config, timeout=timeout)

    engine = RunEngine({})
    engine.subscribe(_live_callback())

    def status() -> None:
        """Print link state and every axis position."""
        try:
            link = stage.link.get()
        except Exception as exc:  # noqa: BLE001
            link = f"<unreadable: {exc}>"
        print(f"link: {link}")
        if link != "running":
            print("  WARNING: the simulation is not stepping. Moves will not "
                  "complete and .DMOV will not be asserted.")
        print(axis_table(config, stage))

    namespace: Dict[str, Any] = {
        "stage": stage,
        "RE": engine,
        "bp": bp,
        "bps": bps,
        "config": config,
        "status": status,
        "axis_table": lambda: print(axis_table(config, stage)),
    }
    # Bind each axis to its alias so `%mov x 10` resolves.
    for axis in config.axes:
        namespace[axis.console_name] = getattr(stage, axis.console_name)
    return namespace


def _live_callback() -> Any:
    """Return the best live-feedback callback available.

    ``BestEffortCallback`` gives a live table *and* live plots, but it imports
    matplotlib at module scope. Requiring a plotting stack to move a motor
    from a text console is a poor trade on a headless beamline host, so fall
    back to ``LiveTable``, which needs nothing extra and still prints every
    point as it is collected.
    """
    try:
        from bluesky.callbacks.best_effort import BestEffortCallback
        return BestEffortCallback()
    except ImportError:
        from bluesky.callbacks.core import LiveTable

        class _AutoTable:
            """LiveTable that discovers its columns from the run's hints."""

            def __init__(self) -> None:
                self._table: Optional[Any] = None

            def __call__(self, name: str, doc: Dict[str, Any]) -> None:
                if name == "descriptor":
                    fields = list(doc.get("data_keys", {}))
                    self._table = LiveTable(fields)
                    self._table("start", getattr(self, "_start", {}) or {})
                if name == "start":
                    self._start = doc
                    return
                if self._table is not None:
                    self._table(name, doc)

        return _AutoTable()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="isaac-epics-console",
        description=(
            "Interactive console for the simulated stage. Uses Bluesky's "
            "standard %mov / %movr / %wa magics - no bespoke command language."
        ),
    )
    parser.add_argument("-c", "--config", help="Adapter JSON configuration file")
    parser.add_argument("--prefix", help="Override the PV prefix")
    parser.add_argument("--timeout", type=float, default=20.0,
                        help="Seconds to wait for PVs to connect")
    parser.add_argument("--no-banner", action="store_true")
    return parser


def main(argv: Optional[List[str]] = None) -> int:
    args = build_parser().parse_args(argv)

    try:
        config = AdapterConfig.from_file(args.config) if args.config else default_config()
        config = config.with_overrides(prefix=args.prefix)
    except (ConfigError, OSError) as exc:
        print(f"configuration error: {exc}", file=sys.stderr)
        return 2

    try:
        import IPython
        from traitlets.config import Config
    except ImportError:
        print("IPython is required for the console.\n"
              "  pip install 'isaac-epics[bluesky]'", file=sys.stderr)
        return 2

    print(f"connecting to {config.prefix}* ...")
    try:
        namespace = build_namespace(config, timeout=args.timeout)
    except TimeoutError as exc:
        print(str(exc), file=sys.stderr)
        return 1
    except ImportError as exc:
        print(str(exc), file=sys.stderr)
        return 2

    if not args.no_banner:
        print(BANNER)
    namespace["status"]()

    shell_config = Config()
    shell_config.InteractiveShellApp.exec_lines = [
        "from bluesky.magics import BlueskyMagics",
        # One RunEngine for both the magics and explicit RE(...) calls. The
        # magics class otherwise creates its own on a separate event loop,
        # and two engines mean two sets of state to reason about -- the
        # confusion lands exactly when a move is interrupted.
        "BlueskyMagics.RE = RE",
        "get_ipython().register_magics(BlueskyMagics)",
    ]
    shell_config.TerminalInteractiveShell.banner1 = ""
    shell_config.TerminalInteractiveShell.confirm_exit = False

    IPython.start_ipython(argv=[], user_ns=namespace, config=shell_config)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
