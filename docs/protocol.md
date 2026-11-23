# Wire protocol

One request/response per simulation frame carries both directions. Isaac Sim is
the client; the adapter is the server.

The protocol is deliberately dumb. Isaac Sim knows nothing about EPICS — no
motors, no limits, no engineering units, no `.DMOV`. All motor-record semantics
live in the adapter. Keeping it this way is what allows the Isaac Sim side to be
stdlib-only, and what keeps an Isaac Sim version bump from being a controls
change.

## Transport

`POST /tick`, `Content-Type: application/json`, HTTP/1.1 with keep-alive.

HTTP rather than ZMQ because at motor readback rates (~10–60 Hz, single-digit
joint counts) throughput is not the constraint, and `curl` working as a debugger
is worth more than the margin. ZMQ remains a later upgrade if rate ever justifies
it; at these rates it will not.

## Units

**The wire is always in simulation units**: metres for prismatic joints, radians
for revolute — exactly what the Isaac Sim runtime reports. Conversion to
engineering units (mm, degrees) happens in the adapter, configured per axis by
`sim_units_per_egu`.

This matters more than it sounds. The existing twin reports rotary joints in
radians from the runtime while the USD authors joint limits in degrees. Putting
the single conversion in one named place, rather than at each call site, is the
difference between a bug you find in a unit test and one you find in a scan.

## Request — Isaac Sim to adapter

```json
{
  "protocol": 1,
  "positions": {"x_axis": 0.042, "theta_axis": 0.7853},
  "t_sim": 1234.5,
  "playing": true,
  "seq": 8271
}
```

| Field | Type | Required | Meaning |
|---|---|---|---|
| `protocol` | int | no (default 1) | Wire version. A mismatch is rejected, never guessed. |
| `positions` | object | **yes** | Joint name → measured position, simulation units. |
| `t_sim` | number | no (default 0) | Simulation clock, seconds. Drives trajectories. |
| `playing` | bool | **yes** | Whether the simulation is actually stepping. |
| `seq` | int ≥ 0 | no | Monotonic counter; a regression is read as a client restart. |
| `meta` | object | no | Free-form diagnostics. Never interpreted by motor logic. |
| `articulation` | object | no | Sent on the first frames only. Describes the mechanism so the adapter can check its configuration against what exists. |

### `articulation`

```json
{
  "prim": "/World/cradle",
  "meters_per_unit": 1.0,
  "joints": [
    {"name": "chi_axis", "type": "revolute", "lower": -1.745, "upper": 1.745,
     "motor_pv": "SIM:CRADLE:m1"}
  ]
}
```

`motor_pv` is optional and carries a binding the **model** declares — read
from an `epics:motorPV` attribute on the joint prim. When present, the
adapter checks it against the PV it is actually serving for that joint and
reports a disagreement as an error. When absent, nothing is reported and the
config's own joint names are used, which is every stage authored before this
field existed.

Isaac Sim still knows nothing about EPICS here. It does not interpret the
string, it copies one it found in the stage; all motor-record semantics stay
in the adapter.

`playing` is mandatory and has no default. A paused simulation reporting a frozen
readback as a settled move is the failure most likely to quietly corrupt a scan,
so the client is required to state it rather than let the adapter infer it.

## Response — adapter to Isaac Sim

```json
{
  "protocol": 1,
  "targets": {"x_axis": 0.0415, "theta_axis": 0.7850},
  "t_wall": 0.0,
  "axes": {"m1": {"joint": "x_axis", "phase": "moving", "setpoint": 41.5}}
}
```

`targets` are **instantaneous setpoints, not destinations.** The adapter runs the
trapezoidal velocity profile and streams the ramped position each frame.

Two reasons. First, `.VELO` and `.ACCL` are motor-record semantics, and if the
adapter sent only a destination they would be readable, writable and have no
effect — worse than not serving them. Second, one of the stated uses for this
project is reproducing a collision by replaying a scan; a joint teleported to its
destination passes through no intermediate configuration and collides with
nothing. The ramp *is* the fidelity.

`axes` is diagnostics for humans and `curl`. Clients must not depend on it.

## Validation

Both sides validate, and a frame that cannot be fully trusted is rejected
outright rather than partly applied — a half-applied frame is indistinguishable
from fabricated telemetry.

Rejected on sight: wrong protocol version; `positions` not an object; a joint
value that is not a finite number; `playing` missing or not a boolean; `seq`
negative; more than 256 joints; a body over 64 KiB.

NaN and infinity get particular attention in both directions. `nan <= deadband`
is `False`, so a NaN readback would make a move never complete; an infinite
target applied to an articulation is a physics explosion that presents as a
simulator bug and is painful to trace back to a control message. The client
validates the adapter's response for the same reason the adapter validates the
client's.

| Status | Meaning |
|---|---|
| 200 | Frame accepted; targets returned. |
| 400 | Malformed JSON, bad `Content-Length`, or truncated body. |
| 404 | Unknown endpoint. |
| 411 | No `Content-Length`. |
| 413 | Body over 64 KiB. |
| 422 | Well-formed JSON that failed schema validation. |

## Other endpoints

| Endpoint | Purpose |
|---|---|
| `GET /state` | Full link state, positions, targets, per-axis diagnostics. |
| `GET /healthz` | `200` when a simulator is connected, **`503` when not.** |

`/healthz` distinguishes "the adapter is running" from "the adapter is driving a
simulation". Those are different facts and an external check must not conflate
them.

## Staleness: a layered contract

Three independent mechanisms, because each covers a failure the others miss:

1. **Client-side** (`max_frame_age`, default 0.5 s). The worker stops posting if
   the render loop stops feeding it. Without this the worker would heartbeat the
   last snapshot forever and the adapter would see a perfectly healthy link while
   serving a frozen readback — defeating mechanism 2 entirely. Only the client
   can tell "the simulation is paused" (loop running, `playing=false`) from "the
   loop is gone".
2. **Adapter-side** (`stale_timeout`, default 1.0 s). No frame within the window
   → readbacks `INVALID`, `.MSTA` gains `PROBLEM` and `COMM_ERR`, `.DMOV` is not
   asserted, `/healthz` returns 503.
3. **Sim-clock-side.** Trajectories advance on `t_sim`, so a frozen or paused
   clock cannot advance a move regardless of what the link reports.

Keep `max_frame_age` comfortably below `stale_timeout`, and `stale_timeout`
comfortably above the client's exchange period. Isaac Sim's frame rate is not
guaranteed, so a timeout near the frame period will flap.

## Security

The adapter parses JSON into a fixed schema and can do exactly two things:
record joint positions and hand back joint targets. There is no path from a
request to an interpreter.

This is deliberate contrast with `isaac-sim-remote` (port 8226), which executes
arbitrary Python sent to a socket. That trust model is fine for a local developer
control plane and wrong for a control path a scan depends on. The architecture is
borrowed; the trust model is not.

Both endpoints bind loopback by default.
