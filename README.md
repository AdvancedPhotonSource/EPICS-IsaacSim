# EPICS-IsaacSim

An EPICS adapter layer for NVIDIA Isaac Sim digital twins, built for APS beamlines.

Moving an instrument in an Isaac Sim scene currently means writing Python against
the simulator's API. This exposes the simulation to the control system as ordinary
**motor records** instead, so existing scan code, ophyd device definitions and
Phoebus screens drive the twin unmodified.

The point is not convenience. With an ordinary CAD twin, simulating an experiment
means *rewriting the experiment* as an Isaac Sim script, and what you then test is
your reimplementation. Here you run the actual scan file against the actual device
definitions; the twin stands in for the hardware, not for the experiment.

> **This serves simulated motors.** It is not a beamline. See
> [Safety](#safety-and-honesty) before pointing anything real at it.

## Status

| Piece | State |
|---|---|
| Motor record state machine | implemented, 135 tests passing |
| Channel Access server (caproto) | working — verified with `caget`/`caput`/`camonitor` |
| HTTP tick protocol | working — verified end to end |
| Stdlib-only Isaac Sim client | working — verified, zero third-party imports |
| ophyd `EpicsMotor` | **verified** against ophyd 1.11.2 — all 19 bound fields served |
| Bluesky `scan` | **verified** — 6-point scan steps correctly, all points inside deadband |
| Mirror mode (direction A) | implemented, tested without a live beamline |
| Adapter on APS hardware | **verified on Sentosa** — full suite + live move/unit checks on stock Python 3.9 |
| Isaac Sim integration script | **verified on Sentosa** against Isaac Sim 6.2.0 — drives the real 4-DOF articulation |
| Bluesky scan → real articulation | **verified on Sentosa** — 5-point scan, worst error 17 µm |

Every row has been executed. The full chain runs on Sentosa: a stock `bp.scan`
drives a stock ophyd `EpicsMotor` over Channel Access, into the adapter's motor
record, over the HTTP tick, into Isaac Sim's PhysX articulation, and the measured
joint position comes back the same way. No twin-aware code appears anywhere in
the scan.

Measured on the live articulation (Isaac Sim 6.2.0, physics on CPU at 1/240 s):

| Axis | Settled error | `.RDBD` | Margin over the floor |
|---|---|---|---|
| `x_axis` | 0.003 mm | 0.02 mm | 6.7× |
| `y_axis` | 0.004 mm | 0.02 mm | 5× |
| `z_axis` | 0.144 mm | 0.25 mm | 1.7× |
| `theta_axis` | 0.003° | 0.05° | ~19× |

`z_axis` is the loose one for a physical reason: it is the only axis carrying a
gravity load into a finite-stiffness force drive (about 44 N against 300000 N/m),
so it rests ~0.15 mm below target however long it is given. The measurement
agrees with that prediction to within a few percent — which is the useful part,
because it means these drives behave the way the model says and the deadbands can
be derived rather than guessed.

## Quick start — no EPICS, no GPU, no Isaac Sim

```bash
pip install -e .

# terminal 1: the adapter (HTTP endpoint + Channel Access server, one process)
isaac-epics-ioc

# terminal 2: a stand-in for Isaac Sim
python examples/fake_sim.py

# terminal 3: drive it
export EPICS_CA_ADDR_LIST=127.0.0.1 EPICS_CA_AUTO_ADDR_LIST=NO
caput SIM:TWIN:m1 10        # or: python -m caproto.commandline.put SIM:TWIN:m1 10
camonitor SIM:TWIN:m1.RBV

# or run a real Bluesky scan against it
python examples/bluesky_scan.py
```

`curl` works as a debugger, which at motor readback rates matters more than throughput:

```bash
curl -s http://127.0.0.1:8227/state | python -m json.tool
curl -s -o /dev/null -w '%{http_code}\n' http://127.0.0.1:8227/healthz   # 503 = no simulator
```

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

Four decisions carry the design. They come from `EPICS-INTEGRATION-CONTEXT.md`
(§7), which is kept in the repository as the record of why:

1. **The adapter runs out of process from Isaac Sim.** Isaac Sim owns the main
   thread; Bluesky's RunEngine and caproto's server each want to own a loop. Two
   frameworks that both want to *be* the main loop cannot share a process
   politely. Cross-process, the socket is the queue. It also keeps caproto, ophyd
   and their numpy out of Kit's embedded Python.
2. **Isaac Sim is the client; the adapter is the server.** One outbound
   connection to one endpoint means the simulator structurally cannot multiplex,
   and never accepts inbound connections.
3. **One process holds both the IOC and the socket.** The motor record needs
   command input and position feedback in the same place, synchronously. Split
   it and something reads a stale `.DMOV` — a distributed-state bug landed on
   the exact signal a step scan's pace depends on.
4. **The plant seam exists on day one.** Motor logic talks to
   [`Plant`](src/isaac_epics/plant.py) — `set_targets()` / `get_state()` — not to
   a transport. If one simulation ever needs several IOCs, that interface is
   reimplemented against a bridge daemon and neither the motor code nor Isaac Sim
   changes.

## What caproto does and does not give you

The highest-leverage unknown in the design document was whether caproto's
generated record definitions cover the motor record. Measured against caproto
1.3.0:

- **Field coverage is complete.** `caproto.server.records.MotorFields` defines
  **153 fields** — every one of ophyd's 19, plus `.MSTA`, `.LVIO`, `.MRES`,
  `.BDST`, `.RTRY` and the rest. Correct names, types and enums, generated from
  the dbd.
- **Behaviour coverage is zero.** No putters, no state machine: no `.DMOV`
  transitions, no backlash, no retries, no dial/user conversion, no limit
  enforcement.

So the shortcut is real but partial — it saves the schema, not the semantics.
[`src/isaac_epics/motor.py`](src/isaac_epics/motor.py) is the missing half, and
it is deliberately pure: no caproto, no I/O, so motor semantics can be tested
exhaustively without a Channel Access stack.

caproto needs only Python ≥3.8, so the adapter runs in a venv on the APS hosts'
stock 3.9 — no new interpreter.

## Two clocks

An explicit decision, because getting it wrong is subtle and expensive:

- **Simulation time drives the trajectory.** The motion is motion *of the
  simulated articulation*, so pausing the simulation freezes the ramp instead of
  letting it run on and snap.
- **Wall time drives staleness only.** Whether the link is alive is a question
  about the network, not the simulation.

The corollary: **a move can never complete while the simulation is not stepping.**
A paused sim holds a frozen readback, and a frozen readback that happens to sit
inside the deadband is not evidence a move finished. That is blocked structurally,
by two independent mechanisms, both tested.

## Safety and honesty

These are requirements, not preferences. They follow from the originating
project's standing rules and from §8 of the design document.

- **Three states are distinguishable and surfaced**: `disconnected`, `paused`,
  `running` — on `SIM:TWIN:LINK`, per-axis `:SIM_LINK`, and in alarm severity.
- **Staleness is loud.** When frames stop, readbacks go `INVALID`, `.MSTA` raises
  `PROBLEM` and `COMM_ERR`, and **`.DMOV` is not asserted**. A scan waiting on
  `.DMOV` blocks with an alarm raised. A visible hang beats a fabricated
  completion.
- **The client refuses to heartbeat stale data.** If Isaac Sim's render loop
  stops calling `exchange()`, the client goes quiet rather than reposting the
  last snapshot — otherwise a frozen simulator would look perfectly healthy and
  defeat the adapter's own staleness detection. This was a real bug, caught in
  testing; see `max_frame_age`.
- **Refused moves are not clamped.** A move outside the soft limits sets `.LVIO`
  and leaves `.VAL` alone. Silently clamping would record a position that was
  asked for and never reached.
- **Naming discipline.** The default prefix is `SIM:TWIN:`; the adapter warns at
  startup about any prefix that could be confused with a real beamline IOC.
- **Channel Access binds to loopback by default**, so starting the adapter cannot
  accidentally serve simulated motors onto the controls subnet. Changing that is
  a deliberate act and is logged as one.
- **Mirror mode never writes to a PV**, and a test enforces it statically.
- **Direction C — the twin commanding real hardware — is not implemented.** It is
  a controls-safety and access-security question, not a software one.

What none of this changes is **the physics**. This raises the fidelity of the
*control path*, not of the *model*. A twin with wrong geometry behind a perfect
IOC is still wrong — just more convincingly, because now it is wrong in response
to real commands.

## Interactive control

No new command language -- Bluesky already ships the SPEC vocabulary, and the
adapter serves ordinary motor records, so it works unmodified:

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

Phoebus works too: the stock synApps motor screens bind to `SIM:TWIN:m1`
because the record is faithful. See [docs/console.md](docs/console.md), which
also covers running a WebRTC viewer alongside the console.

## Documentation

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
