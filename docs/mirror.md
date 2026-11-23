# Mirror mode: driving a model from live PVs

**Direction A.** Real hardware's readbacks drive the simulated articulation,
read-only, so a 3D model moves because the instrument is moving. This is the
path to use for visualisation and monitoring.

It is the opposite direction from the rest of this repository. Everywhere
else, the twin stands in for hardware so that scan code can be tested against
it ([direction B](../README.md#running-against-isaac-sim)). Here the hardware
is real and the twin is a display.

```
real beamline IOC ──CA monitors──► isaac-epics-mirror ──HTTP──► Isaac Sim
      .RBV                          (read-only)                 joint targets
```

## See it work first

```bash
pip install -e .
python examples/mirror_demo.py
```

No beamline, no GPU, no Isaac Sim, about thirty seconds. It stands up a
stand-in beamline IOC, runs real mirror mode against it over real Channel
Access, attaches a stand-in for Isaac Sim, moves the "real" motors, and shows
the joints following. Then it kills the source IOC to show the channels going
stale.

The middle piece is not mocked: `isaac-epics-mirror` in that demo is exactly
the process you would run against a beamline.

## What mirror mode does not need

This is the short version of why this path is much less work than direction B:

| | Direction B | Mirror |
|---|---|---|
| Config fields per joint | 14 | **3** |
| Channel Access server | full motor records | **none** — it is a CA *client* |
| Deadband measurement | required, cannot be automated | **not applicable** |
| Velocity, acceleration, soft limits | required | **none** |
| PV naming-safety question | yes | **none** — it serves no PVs |

There is no `.DMOV`, no completion criterion and no trajectory, because
nothing is being commanded. The joint follows a readback. That removes the
step [byo-model.md](byo-model.md) calls the one most likely to waste your
afternoon.

## Configuration

One entry per joint. `pv` and `joint` are required; always set `units` too:

```json
{
  "mirror": [
    {"pv": "XSD:m1.RBV", "joint": "x_axis",     "units": {"sim": "m",   "egu": "mm"}},
    {"pv": "XSD:m4.RBV", "joint": "theta_axis", "units": {"sim": "rad", "egu": "deg"}}
  ]
}
```

- **`pv`** — the real readback PV, normally `<motor>.RBV`.
- **`joint`** — the joint name as Isaac Sim reports it. Get these from
  `examples/isaacsim_probe.py`; a name that matches nothing drives nothing.
- **`units`** — `sim` is what the articulation uses (metres, radians), `egu`
  is what the real motor record serves (mm, degrees). The conversion happens
  here and nowhere else. Strictly this key is optional and defaults to
  mm → metres, but leave it off for a rotary axis and the joint is driven
  wrong by a factor of about 57 while every channel reports healthy. Write it
  out on every mapping.

Two optional keys, for mirroring onto a joint whose kinematics differ from the
real axis: `scale` (gain, applied in engineering units) and `offset` (added in
engineering units before conversion).

Each joint may be driven by exactly one mapping; two mappings onto one joint
is a configuration error rather than a race.

## Running it

```bash
# against a real beamline, read-only
isaac-epics-mirror -c mirror.json

# then point Isaac Sim at the tick endpoint it prints
/isaac-sim/python.sh examples/isaacsim_standalone.py \
    --usd my_stage.usd --prim /World/MyStage/base \
    --url http://127.0.0.1:8227
```

`--stale-after` sets how long a channel may go without an update before it is
reported stale (default 5 s). `--http-host` / `--http-port` move the tick
endpoint.

## Safety

**This process never writes to a PV.** It opens read-only subscriptions and
nothing else. There is no code path from it to a `caput`, and a test enforces
that statically by parsing the module for write calls. That property is the
whole reason direction A is safe to point at a live beamline: it structurally
cannot command one, so it works through a read-only CA Gateway and needs no
negotiation with controls.

**Staleness is reported, not hidden.** Mirror mode is driven by PV timestamps,
not a free-running clock. If the beamline's IOC dies, the joints hold their
last position and every affected channel is flagged `stale` on the `/state`
endpoint and in the log. A model frozen at the last known position with no
indication that it is frozen is the failure this is built to refuse — it looks
exactly like an instrument that is not moving.

Check it from outside the process:

```bash
curl -s http://127.0.0.1:8227/state | python -m json.tool
```

**Direction C — the twin commanding real hardware — is not implemented**, and
is a controls-safety and access-security question rather than a software one.

## Bringing up your own model

Mirror mode is geometry-agnostic in exactly the way the rest of the adapter
is: it never sees a mesh, a mass or a material. What it does require is the
same irreducible prerequisite as direction B — **a USD articulation with named
degrees of freedom.** CAD carries shape, never joints, so converted geometry
alone is not enough. [byo-model.md](byo-model.md) covers that conversion; skip
its deadband section, which does not apply here.

Order of work:

1. Convert or author the USD, and confirm it has an articulation root.
2. `examples/isaacsim_probe.py` for the real joint names.
3. Write the mapping file against those names and your real PVs.
4. `isaac-epics-mirror -c mirror.json`, then start Isaac Sim pointed at it.

## Watching it

Isaac Sim streams its own viewport over WebRTC; mirror mode has nothing to do
with that path and neither knows about the other. Streaming needs an RTX card
with NVENC, and the client runs on macOS or Windows.
See [console.md](console.md#watching-it-move-webrtc-streaming) for the ports
and constraints.

That independence is worth keeping in mind when something looks wrong: a
smoothly rendering viewport tells you nothing about whether the joints are
being driven. The `stale` flags do.
