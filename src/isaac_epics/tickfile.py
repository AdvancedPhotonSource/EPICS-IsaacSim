"""Hand out the Isaac Sim side of the link: ``isaac-epics-tick``.

Why this command exists
-----------------------
The adapter is useless on its own. Something has to post joint positions to
it from inside the simulator, and that something is a single stdlib-only file
(architecture decision 7.6). Until this command existed the file lived outside
the packaged tree, so ``pip install isaac-epics`` produced an adapter that
*structurally could not* be connected to Isaac Sim -- the only way to obtain
the missing half was to clone the repository.

Shipping the file inside the wheel and emitting it on demand also closes a
version-skew hazard. A copy made by hand is a fork from the moment it is
made: the adapter can be upgraded underneath it for a year before anyone
notices. ``PROTOCOL_VERSION`` catches a mismatch at runtime, but catching it
is worse than not causing it. The file emitted here always matches the
adapter that emitted it.

Usage::

    isaac-epics-tick --install /path/next/to/your/standalone_script
    isaac-epics-tick --print > epics_tick.py
    isaac-epics-tick --path

The emitted file is byte-identical to the packaged one, on every platform, so
re-running ``--install`` is a safe no-op and a changed copy is detectable by
comparison rather than by reading it.
"""

from __future__ import annotations

import argparse
import sys
from typing import List, Optional

#: Name the emitted file must have. ``epics_tick`` is the module name the
#: Isaac Sim side imports, so this is part of the interface, not a default.
TICK_FILENAME = "epics_tick.py"


def tick_source() -> bytes:
    """Return the packaged ``epics_tick.py`` verbatim.

    Bytes rather than text, deliberately. Text mode would rewrite the line
    endings on Windows -- where a good share of the engineers using this run
    Isaac Sim natively -- and an emitted file that differs from the packaged
    one by invisible bytes defeats the "has this been modified?" comparison
    that :func:`install` relies on.
    """
    from importlib.resources import files

    return (files("isaac_epics.isaacsim_client") / TICK_FILENAME).read_bytes()


def tick_path() -> str:
    """Return the on-disk location of the packaged file.

    Useful when someone would rather mount or symlink it than copy it, and as
    the thing to paste into a bug report.
    """
    from importlib.resources import as_file, files

    with as_file(files("isaac_epics.isaacsim_client") / TICK_FILENAME) as path:
        return str(path)


def install(directory: str, force: bool = False) -> int:
    """Write ``epics_tick.py`` into *directory*.

    Returns a process exit status. An existing identical file is reported and
    left alone; an existing *different* file is refused unless *force*, on the
    grounds that it may be a copy someone edited, and silently overwriting
    local changes is a poor way to repay a user for upgrading.
    """
    import os

    source = tick_source()
    target = os.path.join(directory, TICK_FILENAME)

    if not os.path.isdir(directory):
        print(f"not a directory: {directory}", file=sys.stderr)
        return 2

    if os.path.exists(target):
        try:
            with open(target, "rb") as handle:
                existing = handle.read()
        except OSError as exc:
            print(f"cannot read existing {target}: {exc}", file=sys.stderr)
            return 1
        if existing == source:
            print(f"{target} is already up to date")
            return 0
        if not force:
            print(
                f"{target} exists and differs from the packaged copy.\n"
                f"Re-run with --force to overwrite it, or diff it first against\n"
                f"  {tick_path()}",
                file=sys.stderr,
            )
            return 1

    try:
        with open(target, "wb") as handle:
            handle.write(source)
    except OSError as exc:
        print(f"cannot write {target}: {exc}", file=sys.stderr)
        return 1

    from . import __version__
    from .protocol import PROTOCOL_VERSION

    print(f"wrote {target}")
    print(f"  isaac-epics {__version__}, wire protocol {PROTOCOL_VERSION}")
    print("  import it from your standalone script: from epics_tick import EpicsTickClient")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="isaac-epics-tick",
        description=(
            "Emit the Isaac Sim side of the link: a single stdlib-only file to drop "
            "next to your standalone script. It is not installed into Isaac Sim and "
            "has no dependencies."
        ),
        epilog=(
            "The emitted file always matches the installed adapter, so re-run this "
            "after upgrading rather than keeping an old copy."
        ),
    )
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument(
        "--install", metavar="DIR",
        help=f"Write {TICK_FILENAME} into DIR",
    )
    action.add_argument(
        "--print", action="store_true", dest="to_stdout",
        help="Write the file to stdout",
    )
    action.add_argument(
        "--path", action="store_true",
        help="Print where the packaged file lives on disk",
    )
    parser.add_argument(
        "--force", action="store_true",
        help="With --install, overwrite an existing file that differs",
    )
    return parser


def main(argv: Optional[List[str]] = None) -> int:
    args = build_parser().parse_args(argv)

    if args.path:
        print(tick_path())
        return 0

    if args.to_stdout:
        # Binary, so the bytes land exactly as packaged even on Windows, where
        # text-mode stdout would translate the newlines on the way through.
        sys.stdout.buffer.write(tick_source())
        return 0

    return install(args.install, force=args.force)


if __name__ == "__main__":
    raise SystemExit(main())
