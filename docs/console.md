# Interactive instrument control

## The short answer: Bluesky already has the command line

This project adds **no new command vocabulary**. Bluesky ships the SPEC-style
magics that beamline users already know, and the adapter serves ordinary motor
records, so they work against the twin unmodified:

| Command | SPEC | Meaning |
|---|---|---|
| `%mov x 10` | `mv` | move to an absolute position |
| `%mov x 5 theta 45` | `mv` | move several axes at once |
| `%movr x -2` | `mvr` | move relative |
| `%wa` | `wa` | where-all: every positioner and its position |
| `%ct` | `ct` | count the detectors |

Inventing a bespoke CLI would mean staff learning a second vocabulary that
works only against the simulation. That is the same mistake, one layer up,
that section 5 of the design document warns about for scans: what you end up
testing is the reimplementation. The argument that says the twin should speak
the motor record says the console should speak Bluesky.

`isaac-epics-console` therefore only *assembles* existing pieces — it builds
the ophyd device from the adapter config, binds each axis to its alias, points
the magics at one shared RunEngine, attaches live-table feedback, and reports
the link state before handing over the prompt.

## Using it

```bash
export EPICS_CA_ADDR_LIST=127.0.0.1 EPICS_CA_AUTO_ADDR_LIST=NO
isaac-epics-console -c examples/beamline.json
```

```
link: running
name     pv     joint               low       high    egu     readback
----------------------------------------------------------------------
x        m1     x_axis          -24.000     24.000     mm       0.0000
y        m2     y_axis          -24.000     24.000     mm       0.0000
z        m3     z_axis            1.000     49.000     mm       0.0000
theta    m4     theta_axis     -179.000    179.000    deg       0.0000

In [1]: %mov x 10
In [2]: %wa
In [3]: RE(bp.scan([], x, -10, 10, 11))
In [4]: status()
```

Axis names come from the `alias` field in the configuration, so `%mov x 10`
works rather than `%mov m1 10`. Everything else in the namespace is standard:
`stage` (the ophyd device), `RE`, `bp`, `bps`, and `status()`.

The console warns but still starts when the link is not `running` — you may
legitimately want to inspect PVs while the simulation is down. It will not
pretend the link is healthy.

## Other front ends, none of which need code from us

**Phoebus / CS-Studio.** The adapter serves a faithful motor record, so the
stock synApps motor screens (`motorx_all.bob`, `motor.bob`) bind to
`SIM:TWIN:m1` and work. This is the "train users on real screens" case from
section 5 — the screen is the one from the beamline, not a mock-up.

**Plain command line.** `caput SIM:TWIN:m1 10`, `camonitor SIM:TWIN:m1.RBV`.

**Typhos** auto-generates a GUI from the ophyd device in `isaac_epics.devices`
if you prefer a widget to a prompt.

**bluesky-queueserver** if you want queued, remotely submitted plans.

## Watching it move: WebRTC streaming

The natural workflow is a viewer open on your laptop while you drive from the
console. Isaac Sim streams its viewport over WebRTC; the adapter is unrelated
to that path and neither knows about the other.

Start the bridge with `--livestream`:

```bash
podman run -d --name epics-bridge --network=host \
  --device nvidia.com/gpu=<UUID of an idle RTX card> \
  -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y \
  -v $HOME/EPICS-IsaacSim:/workspace/epics:ro \
  -v $HOME/digital-twin/prototype:/workspace/twin:ro \
  -v /local/$USER/isaac-cache/kit:/isaac-sim/kit/cache:rw \
  -v /local/$USER/isaac-cache/glcache:/isaac-sim/.cache/nvidia/GLCache:rw \
  -v /local/$USER/isaac-cache/computecache:/isaac-sim/.nv/ComputeCache:rw \
  -v /local/$USER/isaac-cache/warp:/isaac-sim/.cache/warp:rw \
  --entrypoint /isaac-sim/python.sh \
  nvcr.io/nvidia/isaac-sim:6.2.0 \
  /workspace/epics/examples/isaacsim_standalone.py \
    --mechanism-root /workspace/twin --url http://127.0.0.1:8227 --livestream
```

Then connect the Isaac Sim WebRTC Streaming Client to the host's address.

Constraints, all established by the digital-twin project on this host and not
invented here:

- `--network=host` is mandatory; `-p` port mapping does not work for WebRTC.
- **TCP 49100** (signaling) and **UDP 47998** (media). TCP alone is not enough.
- **UDP 47998 does not bind until a client connects.** An absent UDP socket on
  an idle server is normal and not a fault to chase.
- **One client, and one streaming instance per host, at a time.** Stop any
  other streaming container first.
- Wait for `Isaac Sim Full Streaming App is loaded.` before connecting.
- Streaming needs **NVENC**, so it must run on an RTX card. The H200 NVL and
  H100 parts have no encoder at all.
- The endpoints have **no authentication and no encryption** — private or
  trusted networks only.
- Do not try to run the viewer on an APS RHEL box: NVIDIA ships the Linux
  client only as a `.deb`, and the glibc that blocks the simulator binary
  blocks it too. Use the macOS or Windows client.

### Why these are two independent paths

The viewer and the control path share nothing but the simulation. Streaming
can fail, stall or be disconnected without affecting a scan, and the scan can
run with no viewer attached at all. That separation is worth preserving: a
control path whose correctness depended on a video stream would be a strange
thing to put a beamline behind.

It also means the viewer tells you nothing trustworthy about control-path
health. `SIM:TWIN:LINK` does. A smoothly rendering viewport with a dead tick
endpoint is entirely possible — the simulation keeps running and rendering,
and the adapter correctly reports `disconnected`.
