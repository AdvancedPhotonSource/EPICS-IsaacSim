# EPICS-IsaacSim

An EPICS adapter for NVIDIA Isaac Sim digital twins, built for APS beamlines.
It connects an Isaac Sim articulation to Channel Access in either direction:

- **The twin stands in for hardware.** The articulation is served as ordinary
  **motor records**, so existing scan code, ophyd device definitions and
  Phoebus screens drive it unmodified.
- **The twin displays hardware.** [Mirror mode](docs/mirror.md) drives the
  articulation from a real beamline's readbacks, read-only — a 3D model that
  moves because the instrument is moving.

Driving real hardware *from* the twin is **not implemented**: a
controls-safety question, not a software one.

> **This serves simulated motors.** Channel Access binds to loopback and the
> PV prefix defaults to `SIM:TWIN:`. Read [Safety](#safety) before changing
> either.

## Install

```bash
pip install -e .              # the adapter: caproto only, Python >= 3.9
pip install -e '.[bluesky]'   # + ophyd, bluesky, the interactive console
```

Python ≥3.9 so it runs on the APS hosts' stock interpreter. Nothing is
installed into Isaac Sim: the simulator side is one stdlib-only file,
[`isaacsim_client/epics_tick.py`](isaacsim_client/epics_tick.py), copied next
to your standalone script.

## Quick start — no EPICS, no GPU, no Isaac Sim

```bash
isaac-epics-ioc                 # terminal 1: HTTP tick + Channel Access server
python examples/fake_sim.py     # terminal 2: stand-in for Isaac Sim, fake physics

# terminal 3
export EPICS_CA_ADDR_LIST=127.0.0.1 EPICS_CA_AUTO_ADDR_LIST=NO
caput SIM:TWIN:m1 10
camonitor SIM:TWIN:m1.RBV
python examples/bluesky_scan.py   # or drive it with a real Bluesky scan
```

`fake_sim.py --pause-after 5` and `--die-after 10` reproduce the pause and
staleness paths. `curl -s http://127.0.0.1:8227/state` is a usable debugger.

## Running against Isaac Sim

```bash
isaac-epics-ioc -c my_stage.json                        # host

/isaac-sim/python.sh examples/isaacsim_standalone.py \  # container
    --usd my_stage.usd --prim /World/MyStage/base --url http://127.0.0.1:8227
```

For your own model: probe the stage (`examples/isaacsim_probe.py`) for the
real joint names, generate a config from the live articulation
(`--emit-config`), then **measure the deadbands** — the one step that cannot
be automated, and the one that wastes an afternoon when skipped.
[`docs/byo-model.md`](docs/byo-model.md) covers all of it, including vendor
CAD conversion and letting a joint prim declare the PV that drives it.

On the first frame the adapter checks the config against the articulation —
joint names matching nothing, soft limits outside the physical joint, a stage
at the wrong scale — logs every finding and publishes the verdict on `CFGOK` /
`CFGMSG`. Findings are not fatal: bringing up a new model is when you most
need the PVs to diagnose it.

## Watching real hardware instead

```bash
python examples/mirror_demo.py      # the whole chain on loopback, ~30 s, no beamline
isaac-epics-mirror -c mirror.json   # against a real beamline, read-only
```

Mirror mode never writes to a PV, and a test enforces that statically. None
of the configuration below applies to it — see [`docs/mirror.md`](docs/mirror.md).

## Configuration

One JSON file per instrument, one entry per axis — normally produced by
`--emit-config` rather than hand-written. It names the PV prefix, the HTTP and
Channel Access binds, and per axis the joint, engineering units, soft limits,
velocity and retry deadband. The wire is always in simulation units (metres,
radians); the axis `units` block is the only place conversion happens. Run
without `-c` for the built-in 4-DOF x/y/z/theta stage.

Full reference, with every field and how to tune it:
[`docs/deployment.md`](docs/deployment.md).

## PVs served

| PV | Meaning |
|---|---|
| `<prefix><motor>` | Full motor record — 153 fields. [Which are live](docs/motor-record.md). |
| `<prefix><motor>:SIM_LINK` / `:SIM_TARGET` / `:SIM_JOINT` | Per-axis simulation state. No real motor has a `:SIM_` PV. |
| `<prefix>LINK` | `disconnected` / `paused` / `running` |
| `<prefix>CONNECTED` `PLAYING` `AGE` `TSIM` `FRAMES` `PROTO` | Link and simulation diagnostics |
| `<prefix>CFGOK` / `CFGMSG` | Verdict of the configuration check |

`isaac-epics-ioc --list-pvs` prints every one without starting a server.

## Interactive control

```bash
isaac-epics-console -c examples/beamline.json     # %mov x 10 theta 45, %wa, RE(bp.scan(...))
```

Bluesky already ships the SPEC vocabulary and the adapter serves ordinary
motor records, so there is no new command language. Phoebus works for the same
reason — the stock synApps motor screens bind to `SIM:TWIN:m1` unmodified.
[`docs/console.md`](docs/console.md) covers both, and a WebRTC viewer alongside.

## Status

Verified end to end on APS hardware (Sentosa, Isaac Sim 6.2.0, stock Python
3.9): a stock `bp.scan` drives a stock ophyd `EpicsMotor` over Channel Access
into the adapter's motor record, over the HTTP tick, into PhysX, and the
measured joint position comes back the same way. No twin-aware code appears
anywhere in the scan; worst error on a 5-point scan was 17 µm. All 19 fields
ophyd 1.11.2 binds are served. Mirror mode works against live PVs but has not
yet been run against a real beamline.

This raises the fidelity of the *control path*, not of the *model*. A twin
with wrong geometry behind a perfect IOC is still wrong — just more
convincingly.

## Safety

- **Staleness is loud.** When frames stop, readbacks go `INVALID`, `.MSTA`
  raises `PROBLEM` and `COMM_ERR`, and `.DMOV` is **not** asserted — a scan
  waiting on it blocks with an alarm. A visible hang beats a fabricated
  completion.
- **A move can never complete while the simulation is not stepping.**
  Simulation time drives the trajectory, so pausing freezes the ramp rather
  than letting it run on and snap; wall time governs staleness only.
- **Refused moves are not clamped.** Outside the soft limits, `.LVIO` is set
  and `.VAL` left alone, rather than recording a position never reached.
- **Channel Access binds to loopback by default**, so starting the adapter
  cannot serve simulated motors onto the controls subnet. Changing that is a
  deliberate act and is logged as one, and any prefix that could be confused
  with a real beamline IOC is warned about at startup.
- **Mirror mode never writes to a PV**, and a test enforces it statically.

## Documentation

- [`docs/byo-model.md`](docs/byo-model.md) — using your own CAD or USD model
- [`docs/mirror.md`](docs/mirror.md) — driving the model from live PVs
- [`docs/console.md`](docs/console.md) — interactive control and WebRTC streaming
- [`docs/deployment.md`](docs/deployment.md) — where each piece runs, and running it at APS
- [`docs/protocol.md`](docs/protocol.md) — the wire protocol
- [`docs/motor-record.md`](docs/motor-record.md) — which fields are live, inert, or absent
- [`docs/validation.md`](docs/validation.md) — checking the record against the real one
- [`EPICS-INTEGRATION-CONTEXT.md`](EPICS-INTEGRATION-CONTEXT.md) — the design record and its rationale

## Development

```bash
uv venv --python 3.11 .venv && uv pip install -e '.[dev]'
.venv/bin/python -m pytest -q
.venv/bin/ruff check .
```
