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
import asyncio
import os
import sys
from typing import Any, Dict, List, Optional

from .config import AdapterConfig, ConfigError, default_config

BANNER = r"""
  Isaac Sim EPICS console - SIMULATED MOTORS, NOT REAL HARDWARE
  ------------------------------------------------------------------------
  %mov x 10          move x to 10 (absolute)      %wa      where-all
  %movr x -2         move x by -2 (relative)      %ct      count detectors
  %mv / %mvr         SPEC spellings of the above  (plain `mv` is the shell!)
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
    from bluesky.utils import DuringTask

    from .devices import axis_table, connect_stage

    stage = connect_stage(config, timeout=timeout)

    # The loop must be explicit. Under IPython the main thread can be left
    # with no current event loop between cells, and on Python 3.9
    # asyncio.Event() resolves its loop through get_event_loop(), which then
    # raises "There is no current event loop in thread 'MainThread'" from
    # inside RunEngine._clear_call_cache -- i.e. on the first %mov, not at
    # construction, so it looks like a magics bug rather than a setup one.
    # Bluesky's own default BlueskyMagics.RE passes a fresh loop for exactly
    # this reason; replacing it with one that does not regresses that fix.
    # during_task=DuringTask() rather than bluesky's default.
    #
    # DefaultDuringTask exists to keep a Qt GUI event loop spinning while a
    # plan runs. It decides whether to do so by checking whether "matplotlib"
    # is in sys.modules -- and IPython puts a partial matplotlib entry there
    # even when the real package is not installed. bluesky then takes the
    # "matplotlib + Qt" branch and calls matplotlib.get_backend() on a stub,
    # raising AttributeError from inside the RunEngine. The plan is halted,
    # so on Sentosa the first %mov aborted mid-move with PlanHalt.
    #
    # A text console has no Qt loop to keep alive, so the plain DuringTask --
    # which simply waits on the event -- is both correct here and immune to
    # whatever the ambient plotting stack happens to be.
    engine = RunEngine({}, loop=asyncio.new_event_loop(), during_task=DuringTask())
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


class _SafeCallback:
    """Wrap a callback so a failure in live feedback cannot abort a run.

    Live tables and plots are diagnostics. They must never be able to halt a
    motor move -- which is not hypothetical: on Sentosa matplotlib imports
    but has no ``get_backend``, so ``BestEffortCallback`` raised
    ``AttributeError`` partway through the plan and the RunEngine aborted a
    ``%mov``. A broken plotting library stopping a stage mid-travel is a bad
    trade in any control system, and a worse one in a twin that is supposed
    to behave like a beamline.

    The first failure is reported, the callback is then disabled, and the run
    continues. Silence would be worse than noise here, so the warning is
    printed rather than swallowed.
    """

    def __init__(self, inner: Any) -> None:
        self._inner = inner
        self._failed = False

    def __call__(self, name: str, doc: Dict[str, Any]) -> None:
        if self._failed:
            return
        try:
            self._inner(name, doc)
        except Exception as exc:  # noqa: BLE001 - diagnostics must not halt a plan
            self._failed = True
            print(f"\n[live feedback disabled: {type(exc).__name__}: {exc}]\n"
                  f"[the plan continues; this affects display only]")


def _plotting_is_usable() -> bool:
    """Whether matplotlib is healthy enough for BestEffortCallback.

    Importability is not enough. A partial install can satisfy ``import
    matplotlib`` and still lack ``get_backend``, which is what
    ``BestEffortCallback`` reaches for.
    """
    try:
        import matplotlib
        matplotlib.get_backend()
    except Exception:  # noqa: BLE001
        return False
    return True


def _live_callback() -> Any:
    """Return the best live-feedback callback that actually works here.

    ``BestEffortCallback`` gives a live table *and* live plots, but it needs
    a working matplotlib. Requiring a plotting stack to move a motor from a
    text console is a poor trade on a headless beamline host, so fall back to
    ``LiveTable``, which needs nothing extra and still prints every point as
    it is collected.
    """
    if _plotting_is_usable():
        try:
            from bluesky.callbacks.best_effort import BestEffortCallback
            return _SafeCallback(BestEffortCallback())
        except Exception:  # noqa: BLE001
            pass

    from bluesky.callbacks.core import LiveTable

    class _AutoTable:
        """LiveTable that discovers its columns from the run's descriptor."""

        def __init__(self) -> None:
            self._table: Optional[Any] = None
            self._start: Dict[str, Any] = {}

        def __call__(self, name: str, doc: Dict[str, Any]) -> None:
            if name == "start":
                self._start = doc
                return
            if name == "descriptor":
                self._table = LiveTable(list(doc.get("data_keys", {})))
                self._table("start", self._start)
            if self._table is not None:
                self._table(name, doc)

    return _SafeCallback(_AutoTable())


def install_event_loop_guard(shell: Any, engine: Any) -> bool:
    """Keep a current event loop on the main thread for every cell.

    IPython runs each prompt under asyncio and leaves the main thread with
    no current event loop afterwards. On Python 3.9 ``asyncio.Event()``
    resolves its loop through ``get_event_loop()``, which then raises
    ``RuntimeError: There is no current event loop in thread 'MainThread'``.

    bluesky constructs exactly such an Event inside
    ``RunEngine._clear_call_cache``. It would pass the loop explicitly if
    ``_loop_for_kwargs`` were populated, but on this version that dict is
    empty, so the Event falls back to the ambient loop. The failure
    therefore lands on the user's first ``%mov`` rather than at startup, and
    presents as a bug in the magics.

    Passing ``loop=`` to the RunEngine is not sufficient on its own, because
    it is the *ambient* loop that Event consults. Re-establishing the
    engine's own loop before each cell is the narrow fix, and binds the
    Event to the loop the RunEngine actually runs on -- which is where
    bluesky sets and clears it.

    Returns True if the guard was installed.
    """
    loop = getattr(engine, "loop", None)
    if loop is None:  # pragma: no cover - defensive
        return False

    def _ensure(*_args: Any, **_kwargs: Any) -> None:
        try:
            asyncio.get_event_loop()
        except RuntimeError:
            asyncio.set_event_loop(loop)

    _ensure()
    try:
        shell.events.register("pre_run_cell", _ensure)
    except Exception:  # noqa: BLE001 - older IPython without the events API
        return False
    return True


def install_spec_aliases(shell: Any) -> List[str]:
    """Bind the SPEC spellings, and disarm the shell alias that shadows them.

    This is a safety fix before it is a convenience. IPython auto-aliases
    common shell commands, ``mv`` among them. SPEC trains beamline users to
    type ``mv x 8`` to move a motor, so in a console that looks like a
    beamline they will -- and get ``/bin/mv``. If a file named ``x`` happens
    to exist in the working directory it is silently renamed to ``8``. The
    command fails in the wrong domain, quietly, and nothing moves.

    So ``mv`` is removed from the alias table and bound, with ``mvr``,
    ``umv`` and ``umvr``, to the corresponding Bluesky magics. Bluesky's
    ``%mov`` already prints a live progress bar, so the SPEC distinction
    between ``mv`` and the "updated" ``umv`` does not apply here; both are
    accepted because both are muscle memory.

    Returns the names bound, for the banner.
    """
    bound: List[str] = []
    try:
        line_magics = shell.magics_manager.magics["line"]
    except Exception:  # noqa: BLE001 - never block startup over aliases
        return bound

    for spec_name, magic_name in (("mv", "mov"), ("umv", "mov"),
                                  ("mvr", "movr"), ("umvr", "movr")):
        func = line_magics.get(magic_name)
        if func is None:
            continue
        # Drop any shell alias of the same name first, or it wins.
        try:
            shell.alias_manager.undefine_alias(spec_name)
        except Exception:  # noqa: BLE001 - absent alias is fine
            pass
        try:
            shell.register_magic_function(func, "line", spec_name)
            bound.append(spec_name)
        except Exception:  # noqa: BLE001
            pass
    return bound


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
        "from isaac_epics.console import install_event_loop_guard",
        "install_event_loop_guard(get_ipython(), RE)",
        "from bluesky.magics import BlueskyMagics",
        # One RunEngine for both the magics and explicit RE(...) calls. The
        # magics class otherwise creates its own on a separate event loop,
        # and two engines mean two sets of state to reason about -- the
        # confusion lands exactly when a move is interrupted.
        "BlueskyMagics.RE = RE",
        "get_ipython().register_magics(BlueskyMagics)",
        "from isaac_epics.console import install_spec_aliases",
        "_spec = install_spec_aliases(get_ipython())",
        "print('  SPEC spellings bound: ' + ', '.join('%' + n for n in _spec)) "
        "if _spec else None",
        "del _spec",
    ]
    shell_config.TerminalInteractiveShell.banner1 = ""
    shell_config.TerminalInteractiveShell.confirm_exit = False

    IPython.start_ipython(argv=[], user_ns=namespace, config=shell_config)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
