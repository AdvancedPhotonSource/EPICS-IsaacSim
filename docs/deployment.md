# Deployment at APS

## Where each piece runs

The adapter runs **on the host**, outside the Isaac Sim container. Isaac Sim
runs in the container and connects out to it.

```
host (RHEL 9.8)                          container (Isaac Sim 6.2.0)
  isaac-epics-ioc                          standalone script
    :8227  HTTP  ◄──────────────────────── epics_tick.EpicsTickClient
    :5064  Channel Access
       ▲
       └── ophyd / Bluesky / Phoebus / caput
```

This is not an arbitrary split. EPICS base already lives on the host at
`/APSshare/epics/base-7.0.8`; putting the adapter in the container would mean
layering the image against a shared container store that must not be disturbed.
Out of process also keeps caproto, ophyd and their numpy out of Kit's embedded
Python, where a resolver pulling a different numpy breaks the simulator in ways
that present as simulator bugs.

A free win worth knowing: the container already runs `--network=host` on these
hosts, because WebRTC streaming requires it. That is also exactly what Channel
Access needs for UDP search and beacons, and it means `127.0.0.1:8227` reaches
the host from inside the container with no extra plumbing. The container
networking obstacle was solved for an unrelated reason.

## Installing

The adapter needs only caproto, which requires Python ≥3.8 — so the APS hosts'
stock Python 3.9 is sufficient and **no new interpreter is needed**.

```bash
python3 -m venv ~/venvs/isaac-epics
~/venvs/isaac-epics/bin/pip install -e /path/to/EPICS-IsaacSim
~/venvs/isaac-epics/bin/isaac-epics-ioc --list-pvs
```

Nothing is installed into Isaac Sim. `isaacsim_client/epics_tick.py` is a single
stdlib-only file — copy it next to your standalone script. There is no package,
no pip step and no dependency for the simulator side, by design.

## Hosts

Verified facts from the project record; re-check anything load-bearing.

| Host | Notes |
|---|---|
| **Sentosa** (`164.54.100.54`) | 2× RTX PRO 6000 Blackwell + 2× H200 NVL, rootless Podman 5.8.2, container storage on `/local`. Prototype host. **Shared container store — do not prune or `rmi -a`.** |
| **Alleppey** | 4× H100. Renders fine but has **zero NVENC engines**, so live streaming is blocked in hardware. Headless render-to-disk only. |
| Engineer workstations | Windows 11, native standalone install. The glibc constraint below does not apply. |

**The Linux standalone binary cannot run on APS RHEL hosts.** Site-wide RHEL 9.8
ships glibc 2.34 / GLIBCXX_3.4.29; Isaac Sim 6.2.0 needs 2.35 / 3.4.30. The
container is the only route on Linux. Verify glibc first on any new host:

```bash
ldd --version | head -1
strings /lib64/libstdc++.so.6 | grep GLIBCXX | tail -3
```

## Network and naming

Two independent safeguards, both on by default:

```bash
# Channel Access binds loopback unless told otherwise
isaac-epics-ioc --ca-interface 127.0.0.1     # default

# Serving to other hosts is a deliberate act, and is logged as one
isaac-epics-ioc --ca-interface 164.54.100.54
```

Keep the PV prefix distinct from any real beamline IOC. The default is
`SIM:TWIN:`, and the adapter warns at startup about any prefix that could be
confused with a real one. Once the simulation is indistinguishable to the
control system it becomes possible to lose track of which one you are connected
to — the `:SIM_LINK`, `:SIM_TARGET` and `:SIM_JOINT` PVs exist so that question
always has a fast answer.

**Never serve the simulated IOC onto the controls subnet.**

For clients:

```bash
export EPICS_CA_ADDR_LIST=127.0.0.1
export EPICS_CA_AUTO_ADDR_LIST=NO
```

## Configuration

```json
{
  "prefix": "SIM:TWIN:",
  "bind": {"http_host": "127.0.0.1", "http_port": 8227, "ca_interface": "127.0.0.1"},
  "stale_timeout": 1.0,
  "update_rate": 20.0,
  "axes": [
    {
      "motor": "m1", "joint": "x_axis", "desc": "Sample X",
      "units": {"sim": "m", "egu": "mm"},
      "low_limit": -50.0, "high_limit": 50.0,
      "velocity": 10.0, "max_velocity": 50.0, "accel_time": 0.2,
      "retry_deadband": 0.001, "precision": 4
    },
    {
      "motor": "m4", "joint": "theta_axis", "desc": "Sample Theta",
      "units": {"sim": "rad", "egu": "deg"},
      "low_limit": -180.0, "high_limit": 180.0,
      "velocity": 30.0, "max_velocity": 90.0, "accel_time": 0.3,
      "retry_deadband": 0.01, "precision": 3
    }
  ]
}
```

The `units` block computes the conversion ratio so configs carry no hand-typed
magic numbers. An unknown key is an error, not a silent no-op — a typo in
`velocty` must not leave a motor quietly running at its default speed.

### Tuning for a real articulation

| Setting | Guidance |
|---|---|
| `retry_deadband` | Must exceed the articulation's steady-state position error. Too tight and moves never complete; too loose and scans record positions they did not reach. Measure it against the real stage. |
| `settle_timeout` | Cap on waiting for a PhysX drive to converge after the ramp ends. Raise it for a heavy or soft-tuned joint. |
| `stall_time` | How long the readback must stop improving before the axis concludes it has arrived as close as it will get. |
| `stale_timeout` | Several times the client's exchange period. Isaac Sim's frame rate is not guaranteed; a value near the frame period will flap. |
| `update_rate` | 20 Hz is ample for motor readback. Raising it does not improve fidelity — the simulator's own rate bounds that. |

## Running

```bash
isaac-epics-ioc -c beamline.json
./python.sh examples/isaacsim_standalone.py --usd stage.usd --prim /World/Stage
```

Checks that are worth doing in order, because each rules out a different layer:

```bash
curl -s http://127.0.0.1:8227/healthz          # 503 until Isaac Sim connects
caget SIM:TWIN:LINK                            # disconnected / paused / running
caget SIM:TWIN:m1.RBV                          # INVALID until the link is up
caput SIM:TWIN:m1 5 && camonitor SIM:TWIN:m1.RBV
```

If `LINK` is `running` but a joint does not move, compare `:SIM_JOINT` against
the articulation's DOF names — a joint-name mismatch is the most common cause
and the quietest.

## Mirror mode (direction A)

Read-only: real beamline readbacks drive the twin. This process never writes to
a PV, and a test enforces that statically.

```json
{"mirror": [
  {"pv": "XSD:m1.RBV", "joint": "x_axis",     "units": {"sim": "m",   "egu": "mm"}},
  {"pv": "XSD:m4.RBV", "joint": "theta_axis", "units": {"sim": "rad", "egu": "deg"}}
]}
```

```bash
isaac-epics-mirror -c mirror.json
```

Cross-subnet IOC access at APS normally goes through CA Gateway; set
`EPICS_CA_ADDR_LIST` to the gateway. A readback that stops updating is reported
stale and its joint is left alone rather than held at a value the twin can no
longer justify.

## Open items before beamline use

These are from §9 of the design document. The first two are now answered; the
rest are not coding tasks and should be started early.

| Item | Status |
|---|---|
| caproto motor record coverage | **Answered.** 153 fields, schema only, no behaviour. |
| ophyd `EpicsMotor` field list | **Answered.** 19 fields, all served, verified against 1.11.2. |
| Isaac Sim container Python version | Unverified. Matters little — the client is stdlib-only. |
| Host reachability of beamline IOCs | Unverified. Which subnet, which gateway, what `EPICS_CA_ADDR_LIST`. |
| Access security for any write path | **Not started.** ASG/ACF configuration and sign-off with the APS controls group. The long pole for direction C. |
| Confirm the one-process topology with Ming | Not done. §7.3 is a recommendation reached in discussion, not an agreed decision. |
