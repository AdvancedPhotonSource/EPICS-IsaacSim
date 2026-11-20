"""Isaac Sim standalone script: drive the beamline stage articulation from EPICS.

Adds an EPICS tick to the per-frame loop. The simulation is the *client*
(architecture decision 7.2): it opens one outbound connection to one endpoint,
never listens, and knows nothing about EPICS -- it reports joint positions in
simulation units and applies joint targets in simulation units. Everything
else is the adapter's problem.

API provenance
--------------
The Isaac Sim calls here are **not guesses**. They mirror
``digital-twin/prototype/beamline_stage_authored.py``, which is known to run
on Sentosa, and were confirmed by ``examples/isaacsim_probe.py`` against Isaac
Sim 6.2.0 (embedded Python 3.12.14). Specifically:

* ``isaacsim.core.experimental.prims.Articulation``, constructed with a
  positional prim path -- **not** ``isaacsim.core.prims.Articulation`` with
  ``prim_paths_expr``. Both exist in 6.2.0; the experimental one is what the
  working prototype uses.
* No ``World``. Stage setup is ``stage_utils.create_new_stage`` plus
  ``SimulationManager.setup_simulation``, and stepping is
  ``simulation_app.update()``.
* ``get_dof_positions()`` / ``set_dof_position_targets()`` return and accept
  warp arrays shaped ``(num_envs, num_dofs)``; hence ``.numpy()[0]`` and
  ``np.array([targets])``.
* Play state and simulation time come from ``omni.timeline``.

The stage is authored at runtime from the mechanism JSON, because this project
has no ``.usd`` on disk -- geometry, masses and joint limits live in
``mechanism/beamline_stage.json`` and are applied by ``build_mechanism.build``.
That builder is imported read-only from the digital-twin checkout; nothing in
this script writes to it.

Usage (inside the container)::

    /isaac-sim/python.sh /workspace/epics/examples/isaacsim_standalone.py \
        --mechanism-root /workspace/twin \
        --description /workspace/twin/mechanism/beamline_stage.json \
        --url http://127.0.0.1:8227

Requires two bind mounts: this repo, and the digital-twin prototype directory
that provides ``mechanism/``.
"""

import argparse
import json
import os
import sys

# --------------------------------------------------------------------------
# SimulationApp must be constructed before any other Isaac Sim import. A hard
# ordering requirement of Kit, not a style choice.
# --------------------------------------------------------------------------
from isaacsim import SimulationApp  # noqa: E402

parser = argparse.ArgumentParser(description="Isaac Sim <-> EPICS adapter bridge")
parser.add_argument("--url", default="http://127.0.0.1:8227",
                    help="Adapter tick endpoint")
parser.add_argument("--rate", type=float, default=30.0,
                    help="Exchange rate with the adapter (Hz). Motor readback "
                         "rates are ~10 Hz; posting at full render rate is waste.")
parser.add_argument("--mechanism-root", default="/workspace/twin",
                    help="Directory containing the 'mechanism' package")
parser.add_argument("--description", default=None,
                    help="Mechanism JSON (default: <mechanism-root>/mechanism/"
                         "beamline_stage.json)")
parser.add_argument("--root", default="/World/stage", help="Stage root prim")
parser.add_argument("--physics-dt", type=float, default=1.0 / 240.0)
parser.add_argument("--device", default="cpu", help="Physics device: cpu or cuda")
parser.add_argument("--livestream", action="store_true")
parser.add_argument("--status-every", type=int, default=600,
                    help="Frames between link status lines (0 to disable)")
args, _ = parser.parse_known_args()

CFG = {"headless": True, "hide_ui": False}
if args.livestream:
    CFG.update({"width": 1280, "height": 720, "window_width": 1920,
                "window_height": 1080, "renderer": "RaytracedLighting",
                "display_options": 3286})

simulation_app = SimulationApp(CFG)

# Isaac Sim imports must follow SimulationApp construction.
import isaacsim.core.experimental.utils.stage as stage_utils  # noqa: E402
import numpy as np  # noqa: E402
import omni.timeline  # noqa: E402
from isaacsim.core.experimental.objects import DomeLight, GroundPlane  # noqa: E402
from isaacsim.core.experimental.prims import Articulation  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402

if args.livestream:
    from isaacsim.core.experimental.utils.app import enable_extension
    simulation_app.set_setting("/app/window/drawMouse", True)
    enable_extension("omni.kit.livestream.app")

# The mechanism builder lives in the digital-twin checkout and is used
# read-only. The EPICS client is a single stdlib-only file in this repo.
sys.path.insert(0, os.path.abspath(args.mechanism_root))
from mechanism.build_mechanism import build  # noqa: E402

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "isaacsim_client"))
from epics_tick import EpicsTickClient  # noqa: E402


def log(payload):
    print("[epics-bridge] " + json.dumps(payload), flush=True)


def main():
    description = args.description or os.path.join(
        args.mechanism_root, "mechanism", "beamline_stage.json")

    stage_utils.create_new_stage(template="sunlight")
    usd_stage = stage_utils.get_current_stage()
    GroundPlane("/World/GroundPlane")
    DomeLight("/World/DomeLight").set_intensities(np.array([1200.0]))

    report = build(usd_stage, description, root=args.root)
    log({"event": "mechanism_built",
         **{k: v for k, v in report.items() if k != "body_paths"}})
    simulation_app.update()

    SimulationManager.setup_simulation(dt=args.physics_dt, device=args.device)
    robot = Articulation(report["articulation_roots"][0])
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    simulation_app.update()

    dof_names = list(robot.dof_names)
    lower, upper = robot.get_dof_limits()
    lower, upper = lower.numpy()[0], upper.numpy()[0]
    index = {name: i for i, name in enumerate(dof_names)}

    log({"event": "articulation", "dofs": dof_names, "num_dofs": robot.num_dofs})
    for name, lo, hi in zip(dof_names, lower, upper):
        # Runtime units: metres for prismatic, radians for revolute. The
        # adapter's soft limits must sit INSIDE these, exactly as a real
        # beamline's soft limits protect its hard limits.
        log({"event": "dof_limit_runtime_units", "joint": name,
             "lower": round(float(lo), 6), "upper": round(float(hi), 6)})

    client = EpicsTickClient(args.url, rate_hz=args.rate).start()
    log({"event": "epics_client_started", "url": args.url, "rate_hz": args.rate})

    # Seed targets from the current pose so the first exchange cannot jolt the
    # articulation toward a stale setpoint.
    targets = robot.get_dof_positions().numpy()[0].copy()
    frames = 0

    try:
        while simulation_app.is_running():
            simulation_app.update()
            frames += 1

            measured = robot.get_dof_positions().numpy()[0]
            reported = {name: float(measured[i]) for name, i in index.items()}

            returned = client.exchange(
                positions=reported,
                t_sim=float(timeline.get_current_time()),
                playing=bool(timeline.is_playing()),
            )

            changed = False
            for name, value in returned.items():
                i = index.get(name)
                if i is not None:
                    # Clamp to the articulation's own limits. The adapter
                    # should never command outside them, but a target the
                    # physics cannot honour would stall the joint and surface
                    # as .MISS -- better to refuse it here and keep the
                    # failure legible.
                    targets[i] = float(np.clip(value, lower[i], upper[i]))
                    changed = True
            if changed:
                robot.set_dof_position_targets(np.array([targets], dtype=np.float32))

            if args.status_every and frames % args.status_every == 0:
                stats = client.stats()
                log({"event": "link", "connected": stats.connected,
                     "sent": stats.sent, "errors": stats.errors,
                     "suppressed": stats.suppressed,
                     "rtt_ms": round(stats.round_trip * 1e3, 2),
                     "last_error": stats.last_error,
                     "measured": {n: round(float(measured[i]), 6)
                                  for n, i in index.items()}})
    except KeyboardInterrupt:
        pass
    finally:
        client.stop()
        log({"status": "DONE", "frames": frames})
        simulation_app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
