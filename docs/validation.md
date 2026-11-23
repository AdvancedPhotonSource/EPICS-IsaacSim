# Validating the motor record against the real thing

Everything in `tests/` checks our motor record against our own expectations.
That catches regressions; it cannot catch a misunderstanding. If we believed
something wrong about how a motor record behaves, the tests encode the same
wrong belief and pass.

So there is a second kind of test: drive **the genuine EPICS `motorRecord`**
and this adapter with identical commands, and compare.

The oracle is [`motorSim`](https://github.com/epics-motor/motorMotorSim) — the
real `motorRecord` from `epics-modules/motor` on the real simulated asyn
controller, in a container. Real trapezoidal acceleration, real backlash, real
retries, real limit handling.

## Running it

```bash
# 1. the oracle (once)
mkdir -p ~/motorsim-oracle/config
cp tools/motorsim/ioc.yaml ~/motorsim-oracle/config/ioc.yaml

podman run -dit --name motorsim-oracle \
  --network host --security-opt label=disable \
  -e IOC_NAME=motorsim-oracle \
  -e EPICS_CA_SERVER_PORT=5074 \
  -e EPICS_CA_REPEATER_PORT=5075 \
  -e EPICS_PVA_SERVER_PORT=5076 \
  -v ~/motorsim-oracle/config:/epics/ioc/config \
  ghcr.io/epics-containers/ioc-motorsim-runtime:R1-3ec6

# 2. the comparison (starts our adapter and fake_sim itself)
python tools/motor_oracle.py --oracle-port 5074
```

`-dit` is not optional: without a held-open stdin the IOC shell reads EOF and
the container exits cleanly the moment it finishes starting, which looks like
a crash and is not. Pin the image tag — there is no `latest`.

Choose a CA port nothing else owns. The adapter defaults to 5064, so the
oracle is put on 5074 and the comparison's own subject adapter on 5084.

## What is compared, and what is deliberately not

**Not trajectories.** The oracle's plant is a kinematic model; ours is a PhysX
articulation with gravity and finite drive stiffness, or a first-order lag
under `fake_sim`. Two correct implementations will take different paths to the
same place, and a test that demanded otherwise would be noise.

What must agree is the behaviour a scan depends on:

| Claim | Why it matters |
|---|---|
| `.DMOV` drops on command, rises exactly once | a step scan paces on this edge |
| readback ends inside `.RDBD` | otherwise moves never complete |
| `.MOVN` is the complement of `.DMOV` | clients use either one |
| out-of-limit move sets `.LVIO` and does not move | a silent clamp corrupts a dataset |
| `.VAL` is never clamped | the recorded position must be the asked-for one |
| `.STOP` ends the move and raises `.DMOV` | an abort that hangs is worse than a fault |
| backlash approaches from a consistent direction | it is the point of backlash |

A disagreement is a real defect in one of the two implementations. It is not
automatically ours — read the trace before assuming.

## What it has already caught

**`.LVIO` was never set on a refused move.** The README promised it; the code
could not deliver it. caproto's `ChannelNumeric.verify_value` raises
`CannotExceedLimits` whenever the control limits differ and the value falls
outside them, and it raises *before* the putter runs — so `request_move` was
never reached and the client got a Channel Access write error where real
hardware gives it a flag. The oracle showed exactly what should happen
instead: the write succeeds, `.VAL` reverts to the previous setpoint, `.LVIO`
reads 1, and the next legal move clears it. Fixed in `ioc._AdvisoryLimitDouble`,
which keeps the control limits advertised — ophyd's `EpicsMotor.limits` *is*
`user_setpoint.limits`, so clearing them would blind `check_limits` — while
leaving the decision to the state machine.

That bug was invisible to 180 passing tests, because the tests asserted the
behaviour the code had rather than the behaviour a motor record has.
