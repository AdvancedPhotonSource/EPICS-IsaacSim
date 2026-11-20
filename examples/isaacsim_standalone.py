"""Isaac Sim standalone script: drive an articulation from EPICS.

**Verification status.** Everything else in this repository has been run and
tested. This file has **not** been executed against a real Isaac Sim -- the
development machine has no Isaac Sim install and no NVIDIA GPU. The adapter
side, the wire protocol and ``epics_tick.py`` are all covered by tests that do
run (see ``tests/test_integration.py``), and ``examples/fake_sim.py`` exercises
the identical client against the identical protocol. What is unverified here is
only the Isaac Sim API usage: the joint-name lookup and the articulation
read/write calls. Treat those as a starting point to confirm on Sentosa, not
as a tested path. The project's standing rule is that simulator state is
authoritative and claimed validation must be real, so this notice stays until
someone runs it.

What this does
--------------
Adds a tick to the per-frame loop that posts joint positions to the adapter
and applies the targets it returns. The simulation is the *client*
(architecture decision 7.2): it opens one outbound connection to one endpoint,
never listens, and knows nothing about EPICS.

Usage::

    # on the host, outside the container
    isaac-epics-ioc &

    # inside Isaac Sim
    ./python.sh examples/isaacsim_standalone.py --usd /path/to/stage.usd

The container runs with ``--network=host`` on these hosts (required for WebRTC
streaming), so ``127.0.0.1`` reaches the adapter on the host with no extra
plumbing.
"""

import argparse
import os
import sys

# --------------------------------------------------------------------------
# SimulationApp must be constructed before any other Isaac Sim import. This is
# a hard ordering requirement of Kit, not a style choice.
# --------------------------------------------------------------------------
from isaacsim import SimulationApp  # noqa: E402  (must come first)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Isaac Sim <-> EPICS adapter bridge")
    parser.add_argument("--usd", help="USD stage to open. Omit to build a bare stage.")
    parser.add_argument("--prim", default="/World/Stage",
                        help="Articulation prim path")
    parser.add_argument("--url", default="http://127.0.0.1:8227",
                        help="Adapter tick endpoint")
    parser.add_argument("--rate", type=float, default=30.0,
                        help="Exchange rate with the adapter (Hz)")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--joints", default="",
                        help="Comma-separated joint names. Default: every DOF "
                             "the articulation reports.")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)

    simulation_app = SimulationApp({"headless": args.headless})

    # Isaac Sim imports must follow SimulationApp construction.
    import numpy as np  # noqa: E402
    from isaacsim.core.api import World  # noqa: E402
    from isaacsim.core.prims import Articulation  # noqa: E402

    sys.path.insert(
        0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "isaacsim_client")
    )
    from epics_tick import EpicsTickClient  # noqa: E402

    world = World(stage_units_in_meters=1.0)
    if args.usd:
        from isaacsim.core.utils.stage import open_stage
        open_stage(args.usd)
        world = World(stage_units_in_meters=1.0)
    world.reset()

    articulation = Articulation(prim_paths_expr=args.prim, name="stage")
    world.scene.add(articulation)
    world.reset()

    dof_names = list(articulation.dof_names)
    if args.joints:
        wanted = [name.strip() for name in args.joints.split(",") if name.strip()]
        missing = [name for name in wanted if name not in dof_names]
        if missing:
            # Fail loudly rather than silently mirroring the wrong joints. A
            # typo here would drive a different axis than the PV claims, which
            # is the most dangerous kind of quiet bug in this system.
            print(f"ERROR: joints not found on {args.prim}: {missing}", file=sys.stderr)
            print(f"       available: {dof_names}", file=sys.stderr)
            simulation_app.close()
            return 2
    else:
        wanted = dof_names

    indices = [dof_names.index(name) for name in wanted]
    print(f"EPICS bridge: {args.prim}")
    print(f"  joints  : {wanted}")
    print(f"  adapter : {args.url}")
    print("  units   : metres for prismatic joints, radians for revolute "
          "(engineering units are the adapter's problem)")

    client = EpicsTickClient(args.url, rate_hz=args.rate).start()

    # Seed the commanded targets from the current pose so the first exchange
    # cannot jolt the articulation toward a stale setpoint.
    try:
        last_targets = np.array(articulation.get_joint_positions()[0], dtype=float)
    except Exception:
        last_targets = np.zeros(len(dof_names), dtype=float)

    frames = 0
    try:
        while simulation_app.is_running():
            world.step(render=not args.headless)
            frames += 1

            positions = np.array(articulation.get_joint_positions()[0], dtype=float)
            reported = {name: float(positions[i]) for name, i in zip(wanted, indices)}

            targets = client.exchange(
                positions=reported,
                t_sim=float(world.current_time),
                playing=bool(world.is_playing()),
            )

            if targets:
                changed = False
                for name, value in targets.items():
                    if name in wanted:
                        last_targets[dof_names.index(name)] = value
                        changed = True
                if changed:
                    articulation.set_joint_position_targets(
                        np.expand_dims(last_targets, axis=0)
                    )

            if frames % 600 == 0:
                print(f"  {client.stats()}")
    except KeyboardInterrupt:
        pass
    finally:
        client.stop()
        simulation_app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
