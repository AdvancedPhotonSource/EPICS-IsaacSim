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
parser.add_argument("--usd", default=None,
                    help="Open an existing USD stage instead of authoring one "
                         "from a mechanism table. Requires --prim.")
parser.add_argument("--prim", default=None,
                    help="Articulation root prim to drive, e.g. /World/stage/base. "
                         "Required with --usd. A stage may contain several "
                         "articulations and guessing wrong looks like a physics "
                         "bug, so it must be named.")
parser.add_argument("--mechanism-root", default="/workspace/twin",
                    help="Directory containing the 'mechanism' package")
parser.add_argument("--description", default=None,
                    help="Mechanism JSON (default: <mechanism-root>/mechanism/"
                         "beamline_stage.json)")
parser.add_argument("--root", default="/World/stage", help="Stage root prim")
parser.add_argument("--emit-config", default=None, metavar="FILE",
                    help="Write an adapter configuration derived from the live "
                         "articulation and exit. Joint names, types and limits "
                         "come from the mechanism itself, so they cannot be "
                         "mistyped.")
parser.add_argument("--prefix", default="SIM:TWIN:",
                    help="PV prefix to write into an emitted configuration")
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

# The EPICS client is a single stdlib-only file in this repo.
#
# The mechanism builder is NOT imported here. It lives in the digital-twin
# checkout, and importing it at module scope made --usd -- the documented
# route for bringing your own model -- fail with ModuleNotFoundError for
# anyone who does not have that checkout mounted. A bring-your-own-asset
# path that requires someone else's package is not one. It is imported
# inside the branch that actually builds a mechanism.
# The tick client ships inside the adapter package so that a pip install
# obtains it; this repo-relative path is how the copy mounted into the
# container is found. `isaac-epics-tick --install` is the other route.
sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "src", "isaac_epics", "isaacsim_client"))
from epics_tick import EpicsTickClient  # noqa: E402


def log(payload):
    print("[epics-bridge] " + json.dumps(payload), flush=True)


def discover_pv_bindings(usd_stage):
    """Read `epics:motorPV` off every joint prim that declares one.

    The model gets to say which PV drives it, instead of that living only in
    a config file the asset knows nothing about.

    Two details that are easy to get wrong and silent when wrong:

    * Traverse with instance proxies. An instanced prim is invisible to a
      plain Traverse(), so bindings on instanced joints would simply not be
      found -- and no error would be raised, because finding nothing looks
      exactly like declaring nothing. (Instanced prims cannot carry
      per-instance attributes at all, which is a separate reason the CAD
      conversion disables instancing.)
    * Match on the prim NAME, because that is what the articulation reports
      as its dof name. Prim paths differ; dof names are the join.
    """
    bindings = {}
    try:
        from pxr import Usd
        predicate = Usd.TraverseInstanceProxies(Usd.PrimDefaultPredicate)
        for prim in usd_stage.Traverse(predicate):
            attr = prim.GetAttribute("epics:motorPV")
            if attr and attr.HasAuthoredValue():
                value = attr.Get()
                if value:
                    bindings[prim.GetName()] = str(value)
    except Exception as exc:  # noqa: BLE001 - bindings are optional
        log({"event": "pv_binding_scan_failed", "error": str(exc)})
    return bindings


def describe_articulation(robot, prim, meters_per_unit, bindings=None):
    """Build the descriptor the adapter checks its configuration against."""
    lower, upper = robot.get_dof_limits()
    lower, upper = lower.numpy()[0], upper.numpy()[0]
    types = joint_types(robot)
    bindings = bindings or {}
    joints = []
    for i, name in enumerate(robot.dof_names):
        entry = {"name": name, "type": types.get(name, "unknown"),
                 "lower": float(lower[i]), "upper": float(upper[i])}
        if name in bindings:
            entry["motor_pv"] = bindings[name]
        joints.append(entry)
    return {"prim": str(prim), "meters_per_unit": float(meters_per_unit),
            "joints": joints}


def joint_types(robot):
    """Best-effort prismatic/revolute classification.

    Isaac Sim exposes this differently across versions, so every lookup is
    guarded. An unknown type is reported honestly as "unknown" rather than
    guessed -- the adapter only uses it to warn about unit mismatches, and a
    wrong guess there would be worse than no opinion.
    """
    out = {}
    names = list(robot.dof_names)
    for attr in ("dof_types", "joint_types"):
        values = getattr(robot, attr, None)
        if values is None:
            continue
        try:
            for name, value in zip(names, list(values)):
                text = str(value).lower()
                if "prismatic" in text or "linear" in text or "translation" in text:
                    out[name] = "prismatic"
                elif "revolute" in text or "angular" in text or "rotation" in text:
                    out[name] = "revolute"
            if out:
                return out
        except Exception:  # noqa: BLE001
            continue
    return out


def split_pv(pv):
    """Split "SIM:CRADLE:m1" into ("SIM:CRADLE:", "m1")."""
    head, sep, tail = pv.rpartition(":")
    if not sep or not tail:
        return None, pv
    return head + sep, tail


def resolve_prefix(descriptor, fallback):
    """Work out the PV prefix from what the model declares.

    A config serves exactly one prefix, so joints declaring PVs under
    different prefixes cannot be served by one adapter. That is a real
    configuration error -- two instruments in one stage want two adapters --
    and it is worth refusing clearly rather than silently serving the first
    one and leaving the rest inert.
    """
    declared = [j["motor_pv"] for j in descriptor["joints"] if j.get("motor_pv")]
    if not declared:
        return fallback, []
    prefixes = {split_pv(pv)[0] for pv in declared}
    prefixes.discard(None)
    if len(prefixes) > 1:
        raise SystemExit(
            "ERROR: the stage declares PVs under more than one prefix: %s\n"
            "       One adapter serves one prefix. Split the stage, or drive "
            "it with one adapter per prefix." % ", ".join(sorted(prefixes))
        )
    resolved = prefixes.pop()
    notes = []
    if fallback and resolved != fallback:
        notes.append("--prefix %r was overridden by the prefix the model "
                     "declares, %r" % (fallback, resolved))
    return resolved, notes


def emit_config(path, descriptor, prefix):
    """Write an adapter configuration derived from the real articulation.

    This removes the error that costs the most time when bringing up a new
    model: a joint name that does not match anything, which drives nothing
    while every PV looks healthy. Names, types and limits are taken from the
    mechanism, so they cannot be mistyped.

    Where a joint declares `epics:motorPV`, the motor name and the prefix come
    from the model too, so the generated config agrees with the asset by
    construction rather than by someone keeping two files in step. Joints that
    declare nothing fall back to m1, m2, ... under --prefix, which is what
    every stage authored before bindings existed does.

    What it cannot know is still left marked. Velocities are a guess scaled to
    the travel, and the deadband is a placeholder, because the steady-state
    error of a drive depends on the mass it carries and the gains it was
    given -- it has to be measured on the running model.
    """
    import math as _math

    prefix, prefix_notes = resolve_prefix(descriptor, prefix)
    for note in prefix_notes:
        log({"event": "prefix_from_model", "note": note})

    axes = []
    for index, joint in enumerate(descriptor["joints"], start=1):
        rotary = joint["type"] == "revolute"
        lo, hi = joint.get("lower"), joint.get("upper")
        if lo is None or hi is None or not (hi > lo):
            lo, hi = (-_math.pi, _math.pi) if rotary else (-0.01, 0.01)
        scale = (_math.pi / 180.0) if rotary else 1e-3     # per EGU, in sim units
        lo_egu, hi_egu = lo / scale, hi / scale
        # Keep soft limits inside the joint, as on a real beamline.
        margin = (hi_egu - lo_egu) * 0.02
        span = hi_egu - lo_egu
        declared_pv = joint.get("motor_pv")
        if declared_pv:
            declared_prefix, motor = split_pv(declared_pv)
            # The prefix is already resolved and identical across joints, so
            # only the suffix varies here.
            basis = "declared by the model as %s" % declared_pv
        else:
            motor = "m%d" % index
            basis = ("no epics:motorPV on this joint; name generated. Author "
                     "one on the joint prim to make the asset self-describing.")

        axes.append({
            "motor": motor,
            "_pv_basis": basis,
            "joint": joint["name"],
            "alias": joint["name"].replace("-", "_"),
            "desc": joint["name"],
            "units": {"sim": "rad" if rotary else "m", "egu": "deg" if rotary else "mm"},
            "low_limit": round(lo_egu + margin, 6),
            "high_limit": round(hi_egu - margin, 6),
            "velocity": round(max(span / 5.0, 1e-3), 6),
            "max_velocity": round(max(span / 2.0, 1e-2), 6),
            "accel_time": 0.2,
            "retry_deadband": round(max(span * 1e-3, 1e-4), 6),
            "settle_timeout": 5.0,
            "stall_time": 0.3,
            "precision": 3 if rotary else 4,
        })

    config = {
        "comment": ("Generated from the live articulation at %s. Joint names, "
                    "types and limits come from the mechanism and are correct. "
                    "VELOCITY and RETRY_DEADBAND are placeholders: the deadband "
                    "must exceed the drive's measured steady-state error, which "
                    "depends on mass and gains and can only be measured on the "
                    "running model. Run a move on each axis, let it settle "
                    "fully, and set .RDBD above the residual error. Motor "
                    "names marked _pv_basis 'declared by the model' came from "
                    "epics:motorPV on the joint prim and agree with the asset "
                    "by construction; any marked 'generated' are invented and "
                    "the asset does not know them."
                    % descriptor.get("prim", "?")),
        "prefix": prefix,
        "bind": {"http_host": "127.0.0.1", "http_port": 8227,
                 "ca_interface": "127.0.0.1"},
        "stale_timeout": 1.0,
        "update_rate": 20.0,
        "axes": axes,
    }
    with open(path, "w") as handle:
        json.dump(config, handle, indent=2)
        handle.write("\n")
    return config


def find_articulations(usd_stage):
    """List prims that carry PhysicsArticulationRootAPI, for the error path."""
    found = []
    try:
        from pxr import UsdPhysics
        for prim in usd_stage.Traverse():
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                found.append(prim.GetPath().pathString)
    except Exception:  # noqa: BLE001 - diagnostics only
        pass
    return found


def stage_meters_per_unit(usd_stage):
    """Metres per stage unit. 1.0 unless the asset was authored otherwise."""
    try:
        from pxr import UsdGeom
        return float(UsdGeom.GetStageMetersPerUnit(usd_stage))
    except Exception:  # noqa: BLE001
        return 1.0


def main():
    description = args.description or os.path.join(
        args.mechanism_root, "mechanism", "beamline_stage.json")

    if args.usd:
        # Bring-your-own asset: open the stage and drive a named articulation.
        from isaacsim.core.utils.stage import open_stage
        open_stage(args.usd)
        usd_stage = stage_utils.get_current_stage()
        prim_path = args.prim
        if not prim_path:
            print("ERROR: --usd requires --prim naming the articulation root.",
                  file=sys.stderr)
            print("       A stage may hold several articulations; choosing one "
                  "silently would look like a physics bug.", file=sys.stderr)
            for candidate in find_articulations(usd_stage):
                print("       candidate: %s" % candidate, file=sys.stderr)
            simulation_app.close()
            return 2
        log({"event": "stage_opened", "usd": args.usd, "prim": prim_path})
    else:
        sys.path.insert(0, os.path.abspath(args.mechanism_root))
        try:
            from mechanism.build_mechanism import build
        except ImportError as exc:
            print(f"ERROR: no mechanism package under {args.mechanism_root!r}: {exc}",
                  file=sys.stderr)
            print("       Either mount the digital-twin checkout and pass "
                  "--mechanism-root,", file=sys.stderr)
            print("       or bring your own stage with --usd and --prim.",
                  file=sys.stderr)
            simulation_app.close()
            return 2
        stage_utils.create_new_stage(template="sunlight")
        usd_stage = stage_utils.get_current_stage()
        GroundPlane("/World/GroundPlane")
        DomeLight("/World/DomeLight").set_intensities(np.array([1200.0]))
        report = build(usd_stage, description, root=args.root)
        log({"event": "mechanism_built",
             **{k: v for k, v in report.items() if k != "body_paths"}})
        prim_path = report["articulation_roots"][0]
    simulation_app.update()

    SimulationManager.setup_simulation(dt=args.physics_dt, device=args.device)
    robot = Articulation(prim_path)
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    simulation_app.update()

    dof_names = list(robot.dof_names)
    lower, upper = robot.get_dof_limits()
    lower, upper = lower.numpy()[0], upper.numpy()[0]
    index = {name: i for i, name in enumerate(dof_names)}

    meters_per_unit = stage_meters_per_unit(usd_stage)
    bindings = discover_pv_bindings(usd_stage)
    if bindings:
        log({"event": "pv_bindings", "declared": bindings})
    descriptor = describe_articulation(robot, prim_path, meters_per_unit, bindings)
    log({"event": "articulation", "dofs": dof_names, "num_dofs": robot.num_dofs,
         "prim": str(prim_path), "meters_per_unit": meters_per_unit})

    if args.emit_config:
        emit_config(args.emit_config, descriptor, args.prefix)
        log({"event": "config_written", "path": args.emit_config,
             "axes": len(descriptor["joints"])})
        simulation_app.close()
        return 0

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
                articulation=descriptor if frames <= 2 else None,
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
