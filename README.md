# EPICS-IsaacSim

An EPICS adapter layer for NVIDIA Isaac Sim digital twins, built for APS beamlines.

It serves an Isaac Sim articulation to the control system as ordinary **motor
records**, so existing scan code, ophyd device definitions and Phoebus screens
drive the twin unmodified. You run the actual scan file against the actual
device definitions; the twin stands in for the hardware, not for the experiment.

> **This serves simulated motors.** Channel Access binds to loopback and the PV
> prefix defaults to `SIM:TWIN:`. See [Safety](#safety) before changing either.

## Status

Verified end to end on APS hardware (Sentosa, Isaac Sim 6.2.0, stock Python
3.9): a stock `bp.scan` drives a stock ophyd `EpicsMotor` over Channel Access
into the adapter's motor record, over the HTTP tick, into Isaac Sim's PhysX
articulation, and the measured joint position comes back the same way. No
twin-aware code appears anywhere in the scan. 179 tests pass.

| Piece | State |
|---|---|
| Motor record state machine, Channel Access server, HTTP tick | working |
| ophyd `EpicsMotor` | all 19 bound fields served (ophyd 1.11.2) |
| Bluesky scan → real articulation | 5-point scan on Sentosa, worst error 17 µm |
| Mirror mode — real beamline → twin, read-only | working against live PVs; not yet run against a real beamline |
| Twin commanding real hardware | **not implemented** — a controls-safety question, not a software one |

This raises the fidelity of the *control path*, not of the *model*. A twin with
wrong geometry behind a perfect IOC is still wrong — just more convincingly,
because now it is wrong in response to real commands.

## Install

```bash
pip install -e .              # the adapter: caproto only, Python >= 3.9
pip install -e '.[bluesky]'   # + ophyd, bluesky and the interactive console
```

Python ≥3.9 means the adapter runs in a venv on the APS hosts' stock
interpreter — no new Python. Nothing is installed into Isaac Sim: the simulator
side is one stdlib-only file,
[`isaacsim_client/epics_tick.py`](isaacsim_client/epics_tick.py), copied next to
your standalone script.

## Quick start — no EPICS, no GPU, no Isaac Sim

```bash
# terminal 1: the adapter (HTTP endpoint + Channel Access server, one process)
isaac-epics-ioc

# terminal 2: a stand-in for Isaac Sim (same client and protocol, fake physics)
python examples/fake_sim.py

# terminal 3: drive it
export EPICS_CA_ADDR_LIST=127.0.0.1 EPICS_CA_AUTO_ADDR_LIST=NO
caput SIM:TWIN:m1 10        # or: python -m caproto.commandline.put SIM:TWIN:m1 10
camonitor SIM:TWIN:m1.RBV

# or run a real Bluesky scan against it
python examples/bluesky_scan.py
```

`fake_sim.py --pause-after 5` and `--die-after 10` reproduce the pause and
staleness paths. `curl` works as a debugger:

```bash
curl -s http://127.0.0.1:8227/state | python -m json.tool
curl -s -o /dev/null -w '%{http_code}\n' http://127.0.0.1:8227/healthz   # 503 = no simulator
```

## Running against Isaac Sim

```bash
# host
isaac-epics-ioc -c my_stage.json

# inside the container
/isaac-sim/python.sh examples/isaacsim_standalone.py \
    --usd my_stage.usd --prim /World/MyStage/base \
    --url http://127.0.0.1:8227
```

For your own model: probe the stage (`examples/isaacsim_probe.py`) for the real
joint names, generate a configuration from the live articulation
(`--emit-config`), then **measure the deadbands** — the one step that cannot be
automated, and the one that wastes an afternoon when skipped.
[`docs/byo-model.md`](docs/byo-model.md) walks through it;
[`docs/deployment.md`](docs/deployment.md) covers podman and running at APS.

On the first frame the adapter checks its configuration against the articulation
the simulation describes — joint names that match nothing, soft limits outside
the physical joint limits, a stage authored at the wrong scale — and logs every
finding, also publishing the verdict on `CFGOK` / `CFGMSG`. Findings are not
fatal: bringing up a new model is when you most need the PVs to diagnose it.

## Configuration

One JSON file per instrument, one entry per axis:

```jsonc
{
  "prefix": "SIM:TWIN:",
  "bind": { "http_host": "127.0.0.1", "http_port": 8227, "ca_interface": "127.0.0.1" },
  "stale_timeout": 1.0,          // seconds without a frame before readbacks go INVALID
  "update_rate": 20.0,           // Hz, motor state machine
  "axes": [
    {
      "motor": "m1",             // PV is prefix + motor
      "joint": "x_axis",         // as Isaac Sim reports it
      "alias": "x",              // console name: %mov x 10
      "units": { "sim": "m", "egu": "mm" },
      "low_limit": -24.0, "high_limit": 24.0,
      "velocity": 5.0, "max_velocity": 20.0, "accel_time": 0.2,
      "retry_deadband": 0.02,    // must exceed the drive's measured residual error
      "settle_timeout": 5.0, "stall_time": 0.3, "precision": 4
    }
  ]
}
```

The wire is always in simulation units (metres, radians); `units` is the only
place conversion happens. Soft limits must sit inside the articulation's
physical joint limits, exactly as on a real beamline. Run without `-c` and you
get the built-in 4-DOF x/y/z/theta stage.

## PVs served

| PV | Meaning |
|---|---|
| `<prefix><motor>` | Full motor record — 153 fields. [Which are live](docs/motor-record.md). |
| `<prefix><motor>:SIM_LINK` / `:SIM_TARGET` / `:SIM_JOINT` | Per-axis simulation state. No real motor has a `:SIM_` PV. |
| `<prefix>LINK` | `disconnected` / `paused` / `running` |
| `<prefix>CONNECTED` `PLAYING` `AGE` `TSIM` `FRAMES` `PROTO` | Link and simulation diagnostics |
| `<prefix>CFGOK` / `CFGMSG` | Verdict of the configuration check |

`isaac-epics-ioc --list-pvs` prints every one without starting a server.

caproto supplies all 153 motor field *definitions* and no behaviour at all — no
`.DMOV` transitions, no backlash, no retries, no dial conversion, no limit
enforcement. [`src/isaac_epics/motor.py`](src/isaac_epics/motor.py) is that
missing half, kept pure (no caproto, no I/O) so motor semantics can be tested
without a Channel Access stack.

## Interactive control

Bluesky already ships the SPEC vocabulary, and the adapter serves ordinary motor
records, so it works unmodified — there is no new command language to learn.

```bash
isaac-epics-console -c examples/beamline.json
```
```
In [1]: %mov x 10          # mv
In [2]: %mov x 5 theta 45  # several axes at once
In [3]: %movr x -2         # mvr
In [4]: %wa                # where-all
In [5]: RE(bp.scan([], x, -10, 10, 11))
```

Phoebus works too — the stock synApps motor screens bind to `SIM:TWIN:m1`
because the record is faithful. See [docs/console.md](docs/console.md), which
also covers running a WebRTC viewer alongside the console.

## Architecture

```
        ┌──────────────────────────┐          ┌─────────────────────────────────┐
        │  Isaac Sim (container)   │          │  Adapter (host, one process)    │
        │                          │  HTTP    │                                 │
        │  per-frame tick:         │ ───────► │  TickServer  →  SharedPlant     │
        │    positions, t_sim,     │   JSON   │                     ▲  │        │
        │    playing               │ ◄─────── │                     │  ▼        │
        │  applies joint targets   │          │              MotorAxis × N      │
        │                          │          │                     │           │
        │  stdlib only — no        │          │              caproto IOC        │
        │  caproto, numpy, ZMQ     │          │                     │           │
        └──────────────────────────┘          └─────────────────────┼───────────┘
                                                                    │ Channel Access
                                                    ophyd / Bluesky / Phoebus / caput
```

- The adapter runs **out of process** from Isaac Sim, which owns its own main
  thread — and this keeps caproto, ophyd and their numpy out of Kit's embedded
  Python.
- **Isaac Sim is the client**: one outbound connection to one endpoint, never inbound.
- **One process holds both the IOC and the socket**, because the motor record
  needs command input and position feedback in the same place, synchronously.
- Motor logic talks to a [`Plant`](src/isaac_epics/plant.py), not to a
  transport, so the backend can change without touching motor code.

Reasoning for each is recorded in §7 of
[`EPICS-INTEGRATION-CONTEXT.md`](EPICS-INTEGRATION-CONTEXT.md).

## Safety

- **Staleness is loud.** When frames stop, readbacks go `INVALID`, `.MSTA`
  raises `PROBLEM` and `COMM_ERR`, and **`.DMOV` is not asserted** — a scan
  waiting on it blocks with an alarm raised. A visible hang beats a fabricated
  completion.
- **A move can never complete while the simulation is not stepping.** Simulation
  time drives the trajectory, so pausing freezes the ramp instead of letting it
  run on and snap; wall time governs staleness only. A frozen readback sitting
  inside the deadband is not evidence a move finished, and two independent
  mechanisms prevent it counting as one.
- **The client refuses to heartbeat stale data.** If Isaac Sim's render loop
  stops calling `exchange()`, the client goes quiet rather than reposting the
  last snapshot, which would make a frozen simulator look healthy
  (`max_frame_age`).
- **Refused moves are not clamped.** Outside the soft limits, `.LVIO` is set and
  `.VAL` left alone, rather than recording a position that was never reached.
- **Channel Access binds to loopback by default**, so starting the adapter
  cannot serve simulated motors onto the controls subnet. Changing that is a
  deliberate act and is logged as one.
- **Naming discipline.** The adapter warns at startup about any prefix that
  could be confused with a real beamline IOC.
- **Mirror mode never writes to a PV**, and a test enforces it statically.

## Documentation

- [`docs/byo-model.md`](docs/byo-model.md) — using your own CAD or USD model
- [`docs/console.md`](docs/console.md) — interactive control and WebRTC streaming
- [`docs/protocol.md`](docs/protocol.md) — the wire protocol
- [`docs/motor-record.md`](docs/motor-record.md) — which fields are live, inert, or absent
- [`docs/deployment.md`](docs/deployment.md) — running it at APS
- [`EPICS-INTEGRATION-CONTEXT.md`](EPICS-INTEGRATION-CONTEXT.md) — the original design record

## Development

```bash
uv venv --python 3.11 .venv && uv pip install -e '.[dev]'
.venv/bin/python -m pytest -q
.venv/bin/ruff check .
```
