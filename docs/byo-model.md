# Using your own model

The adapter is geometry-agnostic. It never sees a mesh, a mass or a material
— the bridge touches exactly four things on the simulation side:

```python
robot.dof_names                    # joint names
robot.get_dof_positions()          # measured
robot.set_dof_position_targets()   # commanded
timeline.get_current_time/is_playing()
```

Everything below the articulation is yours. The boxes in the starter stage
are a `"shape": "box"` entry in a mechanism table; CAD is the same table with
`"shape": "usd"`. Nothing in the EPICS path changes between them.

What the adapter *does* require is a USD articulation with named degrees of
freedom. Geometry alone is not enough: CAD carries shape, never joints.

## Two routes in

**A. Describe the kinematics in a mechanism table** (what the digital-twin
prototype does). Bodies reference either primitives or converted CAD:

```jsonc
{"name": "carriage", "shape": "usd", "usd": "cad_usd/mgn12c_m.usd", "mass_kg": 0.08}
```

Joints, limits, masses and drive gains are declared alongside. Reviewable by
the engineer who knows the hardware, and the same builder handles both cases.

**B. Bring a finished USD** with the articulation already authored — in the
Isaac Sim GUI, by a URDF import, or from any pipeline that produces one:

```bash
./python.sh examples/isaacsim_standalone.py \
    --usd /workspace/assets/my_stage.usd \
    --prim /World/MyStage/base \
    --url http://127.0.0.1:8227
```

`--prim` names the articulation root — the prim carrying
`PhysicsArticulationRootAPI`. It is required rather than guessed, because a
stage can hold several articulations and silently driving the wrong one looks
like a physics bug. Omit it and the bridge lists the candidates it found.

## The workflow

### 1. See what the articulation actually has

```bash
./python.sh examples/isaacsim_probe.py --usd my_stage.usd --prim /World/MyStage/base
```

Prints the real `dof_names` and limits. This is the ground truth; everything
else is written to match it.

### 2. Generate a configuration from it

```bash
./python.sh examples/isaacsim_standalone.py \
    --usd my_stage.usd --prim /World/MyStage/base \
    --emit-config /workspace/epics/my_stage.json --prefix SIM:MYSTAGE:
```

Joint names, types and limits come from the mechanism, so they cannot be
mistyped. Prismatic joints are mapped m → mm, revolute rad → deg, and soft
limits are placed 2% inside the physical ones.

**What it cannot know is marked in the file.** Velocities are a guess scaled
to the travel. The deadband is a placeholder. Both need your attention.

### 3. Fix the deadbands by measurement

This is the one step that cannot be automated away, and the one most likely
to waste your afternoon if skipped.

A position drive with finite stiffness rests a fixed distance short of its
target: roughly *load ÷ stiffness*. Set `retry_deadband` below that floor and
**every move on that axis reports `.MISS` forever**, while the joint sits
exactly where the physics says it belongs. Set it far above and you throw
away position certainty for nothing, because `.DMOV` rises on *entering* the
deadband.

Measure it:

```bash
caput SIM:MYSTAGE:m1 <somewhere mid-travel>
sleep 10                       # well past settling
caget SIM:MYSTAGE:m1.RBV       # the residual is the floor
```

Then set `retry_deadband` to roughly 2–5× that residual. On the prototype
stage the measured floors were 0.003 mm on x and y, 0.144 mm on the
gravity-loaded z, and 0.003° on theta — a fifty-fold spread across one
mechanism, which is why a single global value does not work.

### 4. Start the adapter and check what it says

```bash
isaac-epics-ioc -c my_stage.json
```

On the first frame the adapter compares its configuration against the
articulation the simulation describes, and reports what disagrees:

```
CONFIGURATION CHECK: 2 error(s), 1 warning(s)
  ERROR: m1: joint 'x_axis' does not exist in the articulation, so this motor
         will never move. Available joints: slide_x, slide_y, rot_theta
  ERROR: m3: soft high limit 49 mm is outside the z_axis joint limit
         (0.03 in simulation units, = 30 mm). Moves beyond the joint limit
         will stall and report .MISS.
  WARNING: the articulation has joints no motor drives: rot_theta
```

The verdict is also on `SIM:MYSTAGE:CFGOK` (1 ok, 0 errors) and `CFGMSG`.

Errors are **not** fatal. Bringing up a new model is exactly when you need
the PVs to diagnose it, and refusing to serve would remove the instrument
panel at the moment it is most wanted.

### 5. Drive it

```bash
isaac-epics-console -c my_stage.json
```

Nothing after this point is model-specific. `%mov`, `%wa`, Bluesky plans,
Phoebus screens and ophyd devices all work the same way.

## What is automated, and what is not

| Per-model task | Automated? |
|---|---|
| Joint names match the mechanism | **Yes** — generated from it, and checked every run |
| Soft limits inside joint limits | **Yes** — generated inside, errors if you widen them |
| Prismatic/revolute → mm/deg | **Yes** — from the joint type |
| Stage authored at the wrong scale | **Detected** — `metersPerUnit ≠ 1` is an error |
| Joints nothing drives | **Detected** — warning |
| Velocity and acceleration | **No** — a guess; they are your engineering choice |
| `retry_deadband` | **No** — must be measured on the running model |

The last two are not oversights. Velocity is a decision about how the
instrument should behave, and the deadband is a property of mass and drive
gains that only exists once the thing is running. Guessing either and
presenting the guess as configured truth would be the wrong kind of help.

## Things that bite

**Scale.** STEP files are usually authored in millimetres. If the converted
USD carries `metersPerUnit = 0.001`, every position is out by a thousand.
The adapter reports this as an error rather than letting you discover it from
a stage that moves wildly.

**Joint names from CAD importers.** They are often `joint_1`, `joint_2`, or
assembly names with characters that are awkward as Python identifiers. The
generated config sets an `alias` so the console gets usable names.

**Drive gains.** A heavier payload on the same drive sags further. Changing a
mass means re-measuring the deadband for that axis.

**One articulation per bridge.** The bridge drives a single articulation. Two
instruments means two bridges, two adapters and two PV prefixes — and that is
the point at which the multi-IOC question in §7.4 of the design document
becomes real, rather than deferred.
