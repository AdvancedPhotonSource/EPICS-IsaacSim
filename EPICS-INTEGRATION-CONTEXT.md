# EPICS integration for Isaac Sim — context handoff

**Written for an agent starting a fresh session in the correct project directory.**
This document was produced in `/home/beams0/HASKELS/digital-twin`, which is the
*wrong* repo for this work. Every path below prefixed `digital-twin/` refers to
that other repo and **will not exist** where you are reading this. Those paths are
recorded as provenance, not as things to open.

Nothing here has been implemented. This is a feasibility assessment and a set of
architecture decisions reached in discussion. No EPICS code exists yet.

---

## 1. The question

> Is it feasible to build an EPICS integration for Isaac Sim, to let APS beamlines
> use Isaac Sim digital twins for remote instrument control and for running real
> experiments against the twin?

**Verdict: yes.** The mechanics are more tractable than expected. The hard parts
are (a) faithful motor-record semantics and (b) APS network/controls policy —
not the simulator, and not the EPICS protocol.

---

## 2. Verified facts

Checked directly on an APS RHEL 9.8 login host (`beams0`), 2026-11-20.

| Fact | How established |
|---|---|
| EPICS base 7.0.8 installed site-wide at `/APSshare/epics/base-7.0.8/bin/rhel9-x86_64/` — `caget`, `caput`, `camonitor`, `pvget`, `softIoc` all on PATH | `which` |
| `EPICS_BASE=/APSshare/epics/base-7.0.8`, `EPICS_HOST_ARCH=rhel9-x86_64`, `EPICS_EXTENSIONS=/APSshare/epics/extensions`, `epicsSetupBASE=/APSshare` already in the shell environment | `env \| grep EPICS` |
| System Python is 3.9.25; `caproto`, `pyepics`, and `p4p` are **all absent** | import test |
| NVIDIA's `isaac-sim-remote` skill is a loopback IPC control plane on `127.0.0.1:8226` that executes arbitrary Python sent to the port — described in the skill as "the product, not a hole" | read `digital-twin/.cache/isaac-skills/isaac-sim-remote/SKILL.md` |
| The existing twin prototype is a 4-DOF articulation: `x_axis`, `y_axis`, `z_axis`, `theta_axis`; linear in metres, rotary reported in radians by the runtime while USD authors limits in degrees | read `digital-twin/prototype/beamline_stage_authored.py` |

A free win worth noting: **Isaac Sim's container already must run `--network=host`**
on these hosts (required for WebRTC streaming, per that project's host notes). That
is also exactly what Channel Access needs for UDP search and beacons. The container
networking obstacle is already solved, for an unrelated reason.

---

## 3. Facts carried from the other repo (not independently re-verified)

From `digital-twin/AGENTS.md` and `digital-twin/docs/hosts/*.md`. Treat as reliable
project record, but re-check anything load-bearing.

- **Site-wide RHEL 9.8 constraint:** glibc 2.34 / GLIBCXX_3.4.29, below Isaac Sim
  6.2.0's required 2.35 / 3.4.30. The standalone Linux binary **cannot run** on APS
  RHEL hosts. **The container is the only route on Linux.** Verify glibc first on any
  new host.
- **Sentosa** (`ssh sentosa`, `164.54.100.54`) — prototype host. 2x RTX PRO 6000
  Blackwell Server Edition + 2x H200 NVL; rootless Podman 5.8.2; container storage on
  `/local`. Shared container store — do not prune or `rmi -a`.
- **Alleppey** — 4x H100, rootless Podman, renders fine but has **zero NVENC
  engines**, so live streaming is blocked in hardware. Headless render-to-disk only.
- **Engineers** run Windows 11 with a native standalone install; the Linux glibc
  constraint does not apply to them.
- Isaac Sim pinned at **6.2.0**; NVIDIA's 44 official skills pinned to revision
  `74a9356b20250dd60337d90b4b4d913c9454d32b`. NVIDIA's skills are the implementation
  authority and must stay unmodified.
- That project's own skill currently states "**No EPICS.** Isaac Sim ships a ROS 2
  bridge; there is no EPICS integration," with an untested guess that caproto/pyepics
  would work and that `EPICS_CA_ADDR_LIST` plus IOC-subnet routing would be the
  obstacles. **That guess holds up.** If this work proceeds, that scope note needs
  updating.
- Claim that Isaac Sim standalone scripts are "plain Python 3.12" — **from that
  repo's skill, not verified by me.** Confirm the container's actual Python version
  before planning any install.

---

## 4. Three directions, not one project

These have very different risk profiles and should never be conflated.

### A. Mirror — real beamline → twin (read-only)
Subscribe to motor `.RBV`, drive articulation joint targets. No writes to hardware.
Works through a read-only CA Gateway. Lowest risk, nothing to negotiate, genuinely
prototype-scale. **Start here.**

### B. Twin as the IOC — control software → twin
Expose the simulation as a motor-record-compatible soft IOC so spec, ophyd/Bluesky,
and Phoebus screens drive the twin **unmodified**. This is the direction that
delivers "run real experiments from the digital twin." Biggest payoff, biggest
semantic work. **This is the prize and the main subject of this document.**

### C. Twin → real hardware (commanding)
`caput` is one line. This is not a software feasibility question — it is controls
safety and access security. Last, if ever, and never as an accidental side effect
of A or B.

---

## 5. Why B matters, stated plainly

With an ordinary CAD twin, to simulate an experiment you must **rewrite the
experiment** as an Isaac Sim script. What you then test is your reimplementation.
The bug you find is a bug in the rewrite; the bug you miss is the one that only
exists in the real scan code.

With B you **run the actual experiment**: same scan file, same GUI, same ophyd
device definitions. The twin stands in for the hardware, not for the experiment.
The scan does not know it is talking to a simulation and needs no modification.

That enables: rehearsing scan plans before beamtime, training users on real screens,
reproducing a collision by *replaying the scan* rather than guessing at the motion,
and commissioning beamline software before hardware exists.

**What B does not change:** the physics. Same articulation, same joint limits, same
invented masses. B raises the fidelity of the *control path*, not of the *model*. A
twin with wrong geometry behind a perfect IOC is still wrong — just more
convincingly, because now it is wrong in response to real commands.

---

## 6. The hard part of B is the motor record, not the simulator

A naive IOC exposing only `.VAL` and `.RBV` demos beautifully and then breaks scans
in ways that look like beamline faults.

ophyd's `EpicsMotor` binds roughly these fields — **this list is from model
knowledge, verify against the actual ophyd version in use**: `.RBV`, `.VAL`, `.OFF`,
`.DIR`, `.FOFF`, `.SET`, `.VELO`, `.ACCL`, `.EGU`, `.MOVN`, `.DMOV`, `.HLS`, `.LLS`,
`.HLM`, `.LLM`, `.TDIR`, `.STOP`, `.HOMF`, `.HOMR`. A real beamline additionally
cares about `.MSTA`, `.LVIO`, `.MRES`, and backlash (`.BDST`, `.RTRY`).

Two routes, neither of which is "write it by hand":

1. **caproto's server record definitions.** caproto reportedly ships generated record
   field definitions including the motor record (`caproto.server.records`). If that
   coverage is real and faithful, it is the single biggest shortcut available.
   **UNVERIFIED — check this first, it changes the estimate more than anything else
   in this document.**
2. **Real synApps motor IOC with a custom asyn port driver** whose plant is Isaac Sim
   instead of a simulated trajectory. Exact semantics by construction; costs C++/asyn
   work.

Related, out of scope for v1: serving rendered frames as an areaDetector /
NTNDArray source via `p4p`. Plausible and valuable. Do not scope it into the first
version.

---

## 7. Architecture decisions

### 7.1 The adapter runs **out of process** from Isaac Sim

Raised by **Ming** and adopted. Do not pip-install caproto/ophyd/bluesky into Isaac
Sim's embedded Python. Reasons, in rough order of severity:

1. **Event-loop conflict.** Isaac Sim owns the main thread via
   `simulation_app.update()`. Bluesky's RunEngine owns an asyncio loop and expects to
   drive. caproto's async server wants a loop too. Two frameworks that both want to
   *be* the main loop cannot share a process politely. Cross-process, the socket *is*
   the queue and the OS enforces the boundary instead of your discipline.
2. **Dependency collision.** Isaac Sim's embedded Python has pinned versions that
   Kit, USD and warp were built against — numpy especially. Bluesky pulls a large
   tree (ophyd, event-model, databroker, msgpack). A resolver pulling a different
   numpy breaks the simulator in ways that present as simulator bugs, in an
   environment NVIDIA's troubleshooting assumes is stock.
3. **It sidesteps the container entirely.** Isaac Sim runs in Podman; EPICS base
   already lives on the host. In-process would mean layering the container image,
   against a shared container store that must not be disturbed. Out-of-process puts
   the adapter on the host where CA works natively and controls staff can reach it.
4. **Blast radius.** An IOC crash should not destroy a simulation that took minutes
   to load the stage and warm shaders. A sim crash should produce a clean fault on
   the control system, not a vanished IOC.
5. **Independent lifecycles.** Controls code stays normal CPython maintained on its
   own schedule; an Isaac Sim 6.2 → 6.3 bump does not force re-validation of it.

### 7.2 Isaac Sim is the **client**; the adapter is the **server**

- Isaac Sim has a natural per-frame tick to poll on and never blocks.
- One outbound connection to one endpoint means Isaac Sim **structurally cannot**
  multiplex — the rule in 7.4 is enforced by design, not by discipline.
- It never accepts inbound connections. That matters for a component whose existing
  IPC surface (`isaac-sim-remote`, port 8226) is arbitrary code execution by design.
- Container-reaches-host is trivially allowed; host-reaches-into-container is not,
  and will not be on a Windows engineer's machine even though `--network=host` makes
  it moot on Sentosa.

### 7.3 **One process** for the IOC and the socket, not an IOC plus a separate bridge

- The motor record is a state machine with two inputs — a `.VAL` write starts a move,
  `.DMOV` drops, readback converges, backlash may fire, a limit may trip, `.DMOV`
  rises. It needs command input and position feedback in the same place,
  synchronously. Split it and you must decide which side owns `.DMOV`; the other side
  reads a stale copy. That is the classic distributed-state bug, landed on the exact
  signal a step-scan's pace depends on.
- A split needs a second protocol, second serialization, second failure mode, second
  startup-ordering constraint — bought before there is a problem it solves.
- One process is the only topology compatible with **both** motor-record routes in
  §6; the asyn-driver route puts the plant inside the IOC process by construction.

### 7.4 Multiplexing never happens inside Isaac Sim

The condition that would justify splitting into bridge + IOCs is **count**, not
complexity: a real beamline is a sample-stage IOC, a slit IOC, a detector IOC,
plausibly owned by different people and restarted independently. When there is more
than one IOC against one simulation, something must arbitrate joint ownership. A
single bridge holding the one connection to Isaac Sim and fanning out to N IOCs is
the right answer *then*.

Whether that arbitration lives in one adapter process or a separate bridge is
deferrable. **Whether Isaac Sim grows connection management and joint-ownership
arbitration is architectural — decide no now.** That code would live in precisely
the environment this design keeps thin.

### 7.5 Build the seam on day one

Inside the adapter, put motor logic behind a small plant interface — roughly
`set_targets(dict)` and `get_state() -> positions, t_sim, playing`. Today it is
implemented by the HTTP handler. If the N-IOC condition arrives, the same interface
is reimplemented as a client to a bridge daemon; the motor-record code does not
change and Isaac Sim does not change, because it still talks to exactly one endpoint
(now the bridge). Cost: one class. It is what makes "start simple" honest rather
than a thing said before a rewrite.

### 7.6 Transport: stdlib on the Isaac Sim side

Ming suggested ZMQ or HTTP. Push the principle to its limit: **the Isaac Sim side
should need no third-party package at all.** pyzmq is a small, stable compiled wheel
— but still an install into the environment being kept clean. `http.client` or plain
`socket` gets to zero.

Recommendation for v1: **HTTP or newline-delimited JSON over TCP**, with ZMQ as a
later upgrade if rate ever justifies it (at ~10 Hz motor readback rates it will not).
HTTP also makes `curl` a debugger, which matters more here than throughput.

One request/response per frame carries both directions:

```
Isaac Sim  ──►  {"positions": {...}, "t_sim": 1234.5, "playing": true}
           ◄──  {"targets": {"x_axis": 0.042, ...}}
```

Keep the protocol dumb. Isaac Sim's side is a stdlib call in the per-frame tick and
knows **nothing** about EPICS. All motor-record semantics — deadband and `.DMOV`,
soft limits, `.EGU`, offsets, PV prefix, connection health — live in the adapter.
"Done moving" is decided by the adapter comparing commanded to measured within a
deadband, never by the simulation.

---

## 8. Honesty and safety requirements specific to this work

These follow from the originating project's standing rules (keep simulator state
authoritative; never report fabricated telemetry, validation results, or connection
status) and are not optional.

- **Three states must be distinguishable and surfaced: disconnected, connected but
  paused, moving.** That is why `playing` is in the protocol. A paused sim reporting
  a frozen readback as a settled move is the failure most likely to quietly corrupt
  a scan.
- **Staleness must be loud.** If the socket goes quiet the adapter raises alarm
  severity and stops asserting `.DMOV` — it does not hold the last value. A frozen
  readback presented as live is exactly the fabricated-telemetry failure the project
  prohibits.
- **Naming discipline.** Once the simulation is indistinguishable to the control
  system, it becomes possible to lose track of which one you are connected to. Use a
  distinct PV prefix for the simulated IOC, never the real one, and never serve it
  onto the controls subnet. Decide this before the first line of code.
- **Isaac Sim is not hard real-time.** Physics at 240 Hz and render at 60 Hz, with no
  scheduling guarantee, on GPUs shared with other work. Fine for mirroring at motor
  readback rates. Not fine inside any feedback loop, and not a basis for claiming the
  twin's *timing* reflects the beamline's. Decide explicitly whether sim clock or
  wall clock is authoritative; mirror mode should be driven by PV timestamps, not
  free-running. Sim time is in the protocol for this reason.
- Do not copy `isaac-sim-remote`'s trust model. Borrow the out-of-process
  architecture; its arbitrary-Python-over-a-socket protocol is fine for a local
  developer control plane and wrong for a control path a scan depends on.

---

## 9. Open questions — resolve these first

1. **Does caproto's `records` module provide faithful motor record coverage?**
   Highest-leverage unknown in this document. Changes the estimate for direction B
   more than anything else.
2. **What Python version does the Isaac Sim 6.2.0 container ship, and is pip into it
   practical?** Matters much less under this architecture (stdlib-only client), but
   confirm the claim of 3.12.
3. **Can the intended host reach any beamline IOC at all?** Which subnet, which
   gateway, what `EPICS_CA_ADDR_LIST`. Sentosa is `164.54.100.54`. Cross-subnet IOC
   access at APS normally goes through CA Gateway.
4. **Access security for any write path.** ASG/ACF configuration and sign-off. This is
   the long pole for directions B-on-real-prefixes and C, and it is a conversation
   with the APS controls group, not a coding task. Start it early.
5. **Confirm the ophyd `EpicsMotor` field list** in §6 against the ophyd version
   actually deployed.
6. **Ask Ming** whether the one-process recommendation in §7.3 matches their intent —
   the question of IOC-plus-bridge versus a single adapter was raised and this is the
   answer reached, not an agreed decision.

---

## 10. Recommended first milestone

Prove the whole chain with zero contact with real hardware and zero network policy:

1. Run `softIoc` on the host with a handful of fake motor PVs (EPICS base 7.0.8 is
   already installed; nothing to build).
2. Add a stdlib HTTP client call to the per-frame tick of a standalone Isaac Sim
   script, posting joint state and applying returned targets to a simple articulation
   — the 4-DOF `x/y/z/theta` stage from the other repo's prototype is a ready target.
3. Run the adapter on the host: HTTP server plus caproto IOC in one process, behind
   the plant interface from §7.5.
4. Drive it with `caput`/`camonitor` first, then with an ophyd `EpicsMotor`, then with
   a trivial Bluesky `scan`.

If a Bluesky scan steps correctly against the simulation, direction A has no
architectural unknowns left and direction B reduces to motor-record completeness.

---

## 11. Provenance

- Produced 2026-11-20 in `/home/beams0/HASKELS/digital-twin` (wrong repo for this
  work; that project explicitly excludes EPICS from its scope).
- §2 is directly verified on an APS RHEL 9.8 host. §3 is that repo's project record.
- §6's ophyd field list and the caproto motor-record claim are **model knowledge,
  unverified** — flagged in place.
- §7.1 and the ZMQ/HTTP suggestion originate with **Ming**. §7.3–7.6 are
  recommendations reached in discussion and have not been reviewed by them.
- No code was written and nothing was run against EPICS.
