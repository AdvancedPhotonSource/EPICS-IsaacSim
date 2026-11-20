# Motor record coverage

Three tiers. Knowing which tier a field is in is the difference between a twin
that behaves like a beamline and one that demos well and then breaks a scan in a
way that looks like a beamline fault.

- **Live** — backed by the state machine in `src/isaac_epics/motor.py`. Reads
  reflect the simulation; writes change behaviour.
- **Inert** — served by caproto's generated `MotorFields` so clients bind and
  connect cleanly, but not wired to anything. Reads return a default. **Writes
  have no effect.**
- **Absent** — not applicable to a simulated axis.

caproto 1.3.0 supplies all 153 field *definitions*. It supplies no behaviour, so
everything in the Live tier below is implemented here.

## Live fields

### Motion
| Field | Notes |
|---|---|
| `.VAL` | Write starts a move. Outside soft limits → `.LVIO`, and `.VAL` is left alone (never clamped). |
| `.RBV` | From the simulation, converted to engineering units. `INVALID` when the link is stale. |
| `.DVAL` / `.DRBV` | Dial coordinates. |
| `.DMOV` | 0 from the instant `.VAL` is written. Rises only when the readback is inside `.RDBD` **and** the simulation is stepping. |
| `.MOVN` | 1 while the ramp is running. |
| `.TDIR` | Direction of the current or last move. |
| `.STOP` | Halts at the simulation's *measured* position, not the commanded one — stopping at the commanded value would make the axis creep forward. |
| `.HOMF` / `.HOMR` | Modelled as a move to dial zero. See caveat below. |
| `.TWV` / `.TWF` / `.TWR` | Relative tweaks. |

### Velocity profile
| Field | Notes |
|---|---|
| `.VELO` | Cruise velocity, EGU/s. Clamped to `.VMAX` on write, as the real record does. |
| `.VBAS` | Floor on cruise velocity. |
| `.VMAX` | Ceiling. |
| `.ACCL` | Seconds from rest to `.VELO`. Zero gives a step profile. |

### Limits
| Field | Notes |
|---|---|
| `.HLM` / `.LLM` | User soft limits. A write that would invert them is refused and logged. |
| `.DHLM` / `.DLLM` | Dial limits, derived. They swap when `.DIR` is `Neg`. |
| `.HLS` / `.LLS` | Limit switches, derived from dial position. |
| `.LVIO` | Set by a refused move; cleared by an accepted one. |

### Calibration
| Field | Notes |
|---|---|
| `.OFF` | `user = dial × dir_sign + offset`. |
| `.DIR` | `Pos` / `Neg`. Inverts the joint and swaps the dial limits. |
| `.FOFF` | `Variable` → `.SET` moves the offset; `Frozen` → `.SET` redefines the dial. |
| `.SET` | Set mode: redefines the present position without commanding motion. |

### Completion and faults
| Field | Notes |
|---|---|
| `.RDBD` | Retry deadband. Must exceed the simulation's position noise or moves never complete. |
| `.RTRY` / `.RCNT` | Retry limit and counter. |
| `.MISS` | Motion stopped without reaching the target. `.DMOV` still rises — the record reports, it does not hang. |
| `.BDST` / `.BVEL` / `.BACC` | Backlash take-up: a move whose direction opposes `.BDST` undershoots, then approaches at `.BVEL`. |
| `.DLY` | Settle time before `.DMOV`. |
| `.MSTA` | Status word: `DONE`, `MOVING`, `PLUS_LS`, `MINUS_LS`, `DIRECTION`, `HOME`, `HOMED`, `PRESENT`, `GAIN_SUPPORT`, `PROBLEM`, `COMM_ERR`. |

### Descriptive
`.EGU`, `.DESC`, `.PREC`, `.MRES` — from configuration.

## Simulation-only PVs

Namespaced with `:SIM_` so they can never be mistaken for motor record fields.
A real motor has no `:SIM_` PVs; seeing one tells you immediately which system
you are connected to.

| PV | Meaning |
|---|---|
| `<motor>:SIM_LINK` | `disconnected` / `paused` / `running` |
| `<motor>:SIM_TARGET` | Commanded joint position, simulation units |
| `<motor>:SIM_JOINT` | Isaac Sim joint name |

Adapter-wide: `LINK`, `CONNECTED`, `PLAYING`, `AGE`, `TSIM`, `FRAMES`, `PROTO`.

## ophyd `EpicsMotor`

Verified against **ophyd 1.11.2**: it binds 19 signals, and all 19 are Live.

```
.ACCL .DIR .DMOV .EGU .FOFF .HLM .HLS .HOMF .HOMR .LLM
.LLS .MOVN .OFF .RBV .SET .STOP .TDIR .VAL .VELO
```

This confirms the field list in §6 of the design document, which was flagged
there as unverified model knowledge. It was correct.

## Caveats

**Homing is not a homing sequence.** The simulation has no home switch. `.HOMF`
and `.HOMR` move to dial zero and set the `HOMED` bit. If your procedure depends
on finding a physical switch, this will not reproduce it.

**No encoder model.** `.UEIP`, `.ERES` and encoder/motor disagreement are inert.
`.RRBV` and `.REP` do not model steps.

**`.MRES` is cosmetic.** Positions are continuous doubles; there is no step
quantisation, so a scan cannot discover that a requested position is unreachable
between steps.

**No slip or stall detection.** `SLIP_STALL` in `.MSTA` is never set. A joint
that cannot reach its target reports `.MISS` after retries, not a stall.

**Timing is not beamline timing.** Isaac Sim runs physics at 240 Hz and render at
60 Hz with no scheduling guarantee, on GPUs shared with other work. Fine at motor
readback rates. Not a basis for claiming the twin's *timing* reflects the
beamline's, and not for use inside any feedback loop.
