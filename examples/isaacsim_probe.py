"""Probe Isaac Sim's API surface. No EPICS, no adapter, no network.

Why this exists
---------------
``examples/isaacsim_standalone.py`` is the one component of this project that
has never been executed -- it was written against documented Isaac Sim 6.2.0
API on a machine with no Isaac Sim and no GPU. Its import paths and
articulation calls are therefore *unverified guesses*, and Isaac Sim has moved
these between releases: ``omni.isaac.kit`` became ``isaacsim``,
``omni.isaac.core`` became ``isaacsim.core.api``, and the articulation classes
moved and changed shape conventions.

Running the full bridge to discover that would bury the real error under a
stack trace from inside Kit. This probe isolates the question: it tries each
API in turn, reports which spelling works on *this* build, and never fails
fast. Every check is independent, so one failure still leaves you with a
complete picture.

It touches no sockets and imports nothing from this project. A clean run here
means the standalone script's assumptions are correct; a failure here names
precisely the line to change.

Usage, inside the container::

    ./python.sh /workspace/epics/examples/isaacsim_probe.py
    ./python.sh /workspace/epics/examples/isaacsim_probe.py --usd /path/stage.usd --prim /World/Stage
"""

import argparse
import sys
import traceback

RESULTS = []


def probe(label):
    """Decorator-ish helper: run a thunk, record PASS/FAIL, never raise."""
    def run(thunk):
        try:
            detail = thunk()
            RESULTS.append((True, label, detail if detail is not None else ""))
            print(f"  PASS  {label}" + (f"   {detail}" if detail else ""))
            return True
        except Exception as exc:  # noqa: BLE001 - diagnosis is the whole point
            RESULTS.append((False, label, f"{type(exc).__name__}: {exc}"))
            print(f"  FAIL  {label}   {type(exc).__name__}: {exc}")
            return False
    return run


def try_imports(candidates, what):
    """Return the first importable (module_path, attr) of several spellings."""
    errors = []
    for module_path, attr in candidates:
        try:
            module = __import__(module_path, fromlist=[attr])
            obj = getattr(module, attr)
            print(f"  PASS  {what}: {module_path}.{attr}")
            RESULTS.append((True, what, f"{module_path}.{attr}"))
            return obj, f"{module_path}.{attr}"
        except Exception as exc:  # noqa: BLE001
            errors.append(f"{module_path}.{attr} -> {type(exc).__name__}: {exc}")
    print(f"  FAIL  {what}: none of the known spellings worked")
    for line in errors:
        print(f"           {line}")
    RESULTS.append((False, what, "; ".join(errors)))
    return None, None


def main(argv=None):
    parser = argparse.ArgumentParser(description="Probe Isaac Sim's API surface")
    parser.add_argument("--usd", help="USD stage to open (optional)")
    parser.add_argument("--prim", default="/World/Stage", help="Articulation prim path")
    parser.add_argument("--headless", action="store_true", default=True)
    args = parser.parse_args(argv)

    failed = []

    print("\n=== interpreter ===")
    print(f"  python   {sys.version.split()[0]}  ({sys.executable})")

    print("\n=== SimulationApp ===")
    SimulationApp, _ = try_imports(
        [("isaacsim", "SimulationApp"),            # 4.x / 5.x / 6.x
         ("omni.isaac.kit", "SimulationApp")],     # legacy
        "SimulationApp import",
    )
    if SimulationApp is None:
        print("\nCannot continue without SimulationApp.")
        return 1

    app = SimulationApp({"headless": True})

    try:
        print("\n=== version ===")
        probe("isaacsim version")(lambda: _version())

        print("\n=== core API ===")
        World, world_path = try_imports(
            [("isaacsim.core.api", "World"),
             ("omni.isaac.core", "World")],
            "World import",
        )
        Articulation, art_path = try_imports(
            [("isaacsim.core.prims", "Articulation"),
             ("isaacsim.core.prims", "SingleArticulation"),
             ("omni.isaac.core.articulations", "Articulation"),
             ("omni.isaac.core.articulations", "ArticulationView")],
            "Articulation import",
        )

        if args.usd:
            print("\n=== stage ===")
            def _open():
                from isaacsim.core.utils.stage import open_stage
                open_stage(args.usd)
                return args.usd
            probe("open_stage")(_open)

        if World is None:
            print("\nNo World class; stopping before articulation checks.")
            return 1

        print("\n=== world ===")
        world_box = {}

        def _make_world():
            world_box["w"] = World(stage_units_in_meters=1.0)
            return None

        if not probe("World(stage_units_in_meters=1.0)")(_make_world):
            return 1
        world = world_box["w"]
        probe("world.reset()")(lambda: world.reset())

        if Articulation is None:
            print("\nNo Articulation class; cannot inspect joints.")
            return 1

        print("\n=== articulation ===")
        art_box = {}

        def _make():
            # The view-style API takes prim_paths_expr; the single-prim API
            # takes prim_path. Try both rather than assume.
            try:
                art = Articulation(prim_paths_expr=args.prim, name="probe")
                art_box["style"] = "view (prim_paths_expr)"
            except TypeError:
                art = Articulation(prim_path=args.prim, name="probe")
                art_box["style"] = "single (prim_path)"
            art_box["a"] = art
            return art_box["style"]

        if not probe(f"Articulation({args.prim})")(_make):
            print("\n  Hint: is the prim path right? Try --prim with the actual path,")
            print("        or --usd to open a stage that contains it.")
            return 1

        art = art_box["a"]
        def _add():
            world.scene.add(art)
            return None

        probe("world.scene.add(articulation)")(_add)
        probe("world.reset() after add")(lambda: world.reset())

        print("\n=== joints ===")
        probe("dof_names")(lambda: list(art.dof_names))

        def _positions():
            pos = art.get_joint_positions()
            shape = getattr(pos, "shape", None)
            return f"shape={shape}  value[0]={pos[0] if shape and len(shape) > 1 else pos}"
        probe("get_joint_positions()")(_positions)

        def _targets():
            import numpy as np
            pos = art.get_joint_positions()
            target = np.array(pos[0] if getattr(pos, "ndim", 1) > 1 else pos, dtype=float)
            try:
                art.set_joint_position_targets(np.expand_dims(target, axis=0))
                return "accepted (N, dof) - batched"
            except Exception:
                art.set_joint_position_targets(target)
                return "accepted (dof,) - unbatched"
        probe("set_joint_position_targets()")(_targets)

        probe("world.current_time")(lambda: world.current_time)
        probe("world.is_playing()")(lambda: world.is_playing())

        print("\n=== expected joints for the default adapter config ===")
        try:
            names = list(art.dof_names)
            wanted = ["x_axis", "y_axis", "z_axis", "theta_axis"]
            missing = [n for n in wanted if n not in names]
            print(f"  articulation reports: {names}")
            if missing:
                print(f"  NOTE  default config expects {wanted}")
                print(f"        missing: {missing}")
                print("        -> edit examples/beamline.json 'joint' fields to match,")
                print("           or pass --joints to the standalone script.")
            else:
                print("  PASS  all four default joints present")
        except Exception as exc:  # noqa: BLE001
            print(f"  (could not compare: {exc})")

    except Exception:  # noqa: BLE001
        print("\nunexpected error:")
        traceback.print_exc()
    finally:
        print("\n=== summary ===")
        failed[:] = [(label, detail) for ok, label, detail in RESULTS if not ok]
        passed = sum(1 for ok, _, _ in RESULTS if ok)
        print(f"  {passed} passed, {len(failed)} failed")
        for label, detail in failed:
            print(f"  FAILED: {label}\n          {detail}")
        app.close()
    return 0 if not failed else 1


def _version():
    try:
        import isaacsim
        return getattr(isaacsim, "__version__", "unknown")
    except Exception:
        from omni.isaac.version import get_version
        return get_version()[0]


if __name__ == "__main__":
    raise SystemExit(main())
