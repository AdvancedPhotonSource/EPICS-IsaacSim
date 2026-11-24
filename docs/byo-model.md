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

## Starting from real vendor CAD

`tools/cad/` automates the route from a vendor STEP file to a riggable USD,
and gates it at both ends. Worked end to end on a Huber 512_1 Eulerian cradle:

```bash
# 1. fetch and check the STEP is an assembly, not a fused solid
python3 -I tools/cad/fetch_huber.py 512_1 --out /local/$USER/cad

# 2. convert (inside the Isaac Sim container)
tools/cad/convert_step.sh in.stp out.usd

# 3. check the conversion survived
/isaac-sim/python.sh tools/cad/usd_structure.py out.usd BASE Z CHI_PHI PHI_PLATE
```

Result on the cradle: `metersPerUnit 1.0`, 25 prims by `Traverse()` and 25 by
`TraverseAll()` (so nothing is instanced), 16 meshes, 0 physics prims, and all
four bodies addressable — `BASE`, `Z`, `CHI_PHI`, `PHI_PLATE`, which is a real
3-DOF goniometer.

Four things this pipeline exists to stop:

- **Fused solids.** A single merged body has nothing to put a joint between.
  Huber's own `5101_05` is fused although its family was reported as
  `BASE/SLIDE` assemblies, so family names predict nothing — parse the file.
- **Scale.** STEP is usually millimetres. The converter's default keeps the
  model's units, so the USD arrives at `metersPerUnit 0.001` and everything is
  1000× out in a metres scene. `convert_step.sh` forces `1.0`.
- **Instancing.** The converter's default hides meshes under `/Prototypes`,
  where `CollisionAPI` cannot be applied and per-prim attributes cannot be
  authored — which would also block PV bindings. Forced off.
- **Licensing.** Geometry is fetched, never committed. Huber and Kohzu publish
  no terms at all, which is default copyright rather than permission; SmarAct
  and GrabCAD forbid redistribution outright.

Two things it does *not* fix. The converter mangles prim names into forms like
`tn__512_145166_202401_BASE_mW06`, because the CAD names start with digits and
contain characters USD does not allow in identifiers; they are addressable but
ugly, and a mechanism table has to reference them exactly. And **CAD carries no
kinematics at all** — `physics prims: 0` is the expected result, not a failure.
Joints, masses and drive gains are authored by someone who knows the hardware.

## Letting the model declare its own PVs

A joint prim can name the PV that drives it:

```usda
def PhysicsRevoluteJoint "chi_axis"
{
    custom uniform string epics:motorPV = "SIM:CRADLE:m1"
    uniform token physics:axis = "Z"
}
```

`tools/cad/rig_articulation.py` writes these from the `pv` field of a
kinematics table. The bridge discovers them, ships them in the articulation
descriptor it already sends, and the adapter checks them against what it is
actually serving on the first frame:

```
ERROR: joint 'chi_axis' is declared in the model as 'SIM:CRADLE:m1' but is
served here as 'SIM:CRADLE:m7'. One of the two is wrong, and a scan written
against the declared name would drive nothing while every PV looked healthy.
```

Without this, the binding exists only as a hand-typed `"joint": "chi_axis"`
in a config file the asset knows nothing about — two files that have to agree
with nothing re-reading either. The failure is silent: the axis never moves
and every PV looks fine.

Declaring nothing is still fine. A stage with no `epics:` attributes is not
nagged about it, and the config's joint names are used as before.

Four things worth knowing if you author these by hand:

- **Use a namespaced custom attribute, not `customData`.** A colon in a
  `customData` key is a dictionary path separator, so what `SetCustomData`
  writes flat, `GetCustomDataByKey` cannot read back.
- **Put them in a layer that references the CAD**, never in the converted
  file. A re-import regenerates its output and does not merge, so anything
  authored into it is destroyed on the next conversion.
- **Instancing must be off.** An instanced prim cannot carry per-instance
  attributes at all, and it is invisible to a plain `Traverse()` — so a
  binding on one is both unauthorable and undiscoverable. `convert_step.sh`
  disables instancing for this reason as well as for collision.
- **The prim name is the join.** Discovery matches the joint prim's name
  against the articulation's `dof_names`; paths differ, names are what both
  sides agree on.

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

## The twin does not detect collisions

A model rigged by this pipeline has **no colliders**. `rig_articulation.py`
applies `CollisionAPI` only under `--collision`, and that flag is off by
default. So a move that would crash the real instrument runs cleanly here:
the joint passes straight through the other body, `.DMOV` is asserted, no
alarm is raised, and the scan records a normal completion. **A move that
succeeds against the twin is not evidence that it is safe against the
hardware.**

The default is deliberate, and flipping the flag does not fix it. PhysX
approximates a collider as a **convex hull**, and the convex hull of a ring
is a solid disc. On the Huber cradle the chi ring's hull swallows the phi box
and the Z slide whole, so every link starts deeply interpenetrating and the
solver locks the articulation rigid — which presents as dead drives rather
than as a collision problem. Beamline mechanisms are rings, yokes, C-arms and
frames; almost none of them are convex, so this is the normal case and not a
quirk of this one asset.

Doing it properly means authoring **convex decomposition or SDF colliders**
per body and then verifying them, because a bad decomposition fails the same
silent way a hull does. That is mechanism work, not control-path work, and it
is outside what this project does: the adapter serves *motion*, and motion is
what a clean run here demonstrates.

If you need collision checking, treat it as a separate piece of work on the
asset, and until it is done do not read a successful simulated move as
clearance.

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
| Collision detection | **No** — no colliders are authored; see above |

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
