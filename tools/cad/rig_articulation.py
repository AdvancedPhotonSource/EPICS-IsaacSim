"""Author a physics articulation onto converted CAD, in a layer of its own.

CAD gives geometry and nothing else -- no joints, no masses, no limits. This
adds them, driven by a kinematics table so the assumptions live in data that
can be read and argued with rather than buried in code.

The output is a NEW layer that *references* the converted CAD and adds physics
as overs. The CAD file is never modified. That matters because a CAD
re-import regenerates its output and does not merge: anything hand-authored
into the generated file is destroyed on the next conversion. Keeping the
physics in a separate layer is what makes re-converting the geometry a safe
operation.

The same layer is where PV bindings belong, for the same reason.

Usage, inside the Isaac Sim container::

    /isaac-sim/python.sh tools/cad/rig_articulation.py \\
        --cad /workspace/cad/512_1/usd/cradle.usd \\
        --kinematics /workspace/epics/tools/cad/kinematics/huber_512_1.json \\
        --out /workspace/cad/512_1/usd/cradle_rigged.usda
"""

import argparse
import glob
import json
import os
import sys


def _bootstrap_pxr():
    """Isaac Sim ships USD as an extension; see usd_structure.py."""
    try:
        import pxr  # noqa: F401
        return
    except ImportError:
        pass
    roots = sorted(glob.glob("/isaac-sim/extscache/omni.usd.libs-*"))
    root = next((r for r in roots if os.path.isdir(os.path.join(r, "pxr"))), None)
    if root is None:
        print("ERROR: run this inside the Isaac Sim container.", file=sys.stderr)
        raise SystemExit(2)
    if os.environ.get("_USD_BOOTSTRAPPED") != "1":
        env = dict(os.environ)
        env["_USD_BOOTSTRAPPED"] = "1"
        env["PYTHONPATH"] = root + os.pathsep + env.get("PYTHONPATH", "")
        env["LD_LIBRARY_PATH"] = (os.path.join(root, "bin") + os.pathsep
                                  + env.get("LD_LIBRARY_PATH", ""))
        os.execve(sys.executable, [sys.executable] + sys.argv, env)
    sys.path.insert(0, root)


def find_body(stage, token):
    """Resolve a body by substring, requiring exactly one match.

    The CAD converter mangles names -- `512_1--45166_202401_BASE` becomes
    `tn__512_145166_202401_BASE_mW06`, because the original starts with a
    digit and contains characters USD forbids in identifiers. Matching on a
    substring survives that; requiring uniqueness stops it silently picking
    the wrong body when two names share a fragment.
    """
    from pxr import UsdGeom

    hits = [p for p in stage.TraverseAll()
            if p.IsA(UsdGeom.Xform) and token.lower() in p.GetName().lower()]
    # A token like "Z" matches almost everything; prefer an exact tail match.
    exact = [p for p in hits if p.GetName().lower().split("_")[-2:-1] == [token.lower()]
             or p.GetName().lower().endswith(token.lower())]
    chosen = exact or hits
    if not chosen:
        raise SystemExit(f"ERROR: no body matching {token!r}")
    if len(chosen) > 1:
        names = ", ".join(p.GetName() for p in chosen)
        raise SystemExit(f"ERROR: {token!r} is ambiguous, matches: {names}")
    return chosen[0]


def world_translation(prim):
    from pxr import Usd, UsdGeom

    xf = UsdGeom.Xformable(prim)
    matrix = xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    return matrix.ExtractTranslation()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--cad", required=True, help="Converted CAD USD (read only)")
    parser.add_argument("--kinematics", required=True, help="Kinematics JSON")
    parser.add_argument("--out", required=True, help="Rigged layer to write")
    parser.add_argument(
        "--collision", action="store_true",
        help="Also apply CollisionAPI to each body. OFF by default, and the "
             "default is the considered one: PhysX approximates a collider as "
             "a CONVEX HULL, and the convex hull of a ring is a solid disc. On "
             "this cradle the chi ring's hull swallows the phi box and the Z "
             "slide whole, so every link starts deeply interpenetrating and the "
             "solver locks the articulation rigid -- it looks like dead drives "
             "rather than a collision problem. A joint-driven twin does not "
             "need link-to-link collision at all. Turn this on only with a "
             "proper convex decomposition, and only if you need collision "
             "checking rather than motion.")
    args = parser.parse_args(argv)

    _bootstrap_pxr()
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    with open(args.kinematics) as handle:
        spec = json.load(handle)

    stage = Usd.Stage.CreateNew(args.out) if not os.path.exists(args.out) \
        else Usd.Stage.Open(args.out)
    if stage is None:
        print(f"ERROR: could not create {args.out}", file=sys.stderr)
        return 1
    stage.RemovePrim("/World")

    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    # Give the stage a time range. A stage authored from scratch has none,
    # which leaves omni.timeline with a degenerate play range: the simulation
    # steps, the articulation settles under gravity, and then
    # timeline.get_current_time() sits at zero forever.
    #
    # That is fatal for this adapter specifically. Simulation time drives the
    # trajectory -- deliberately, so pausing the simulation freezes a ramp
    # rather than letting it run on -- so a frozen clock means no move can
    # ever complete, and .DMOV would never rise. Isaac Sim's own
    # create_new_stage sets these, which is why a built stage works and a
    # hand-authored one silently does not.
    stage.SetTimeCodesPerSecond(60.0)
    stage.SetStartTimeCode(0.0)
    stage.SetEndTimeCode(1.0e7)

    root = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(root.GetPrim())
    asset = UsdGeom.Xform.Define(stage, "/World/cradle")
    # Reference, not copy: the CAD stays the single source of geometry and
    # can be regenerated without touching anything authored here.
    asset.GetPrim().GetReferences().AddReference(os.path.abspath(args.cad))

    # The articulation root goes on the asset, not the stage root, so the
    # whole thing can be referenced into a larger scene later.
    UsdPhysics.ArticulationRootAPI.Apply(asset.GetPrim())

    bodies, masses = {}, {}
    for body in spec["bodies"]:
        prim = find_body(stage, body["match"])
        bodies[body["match"]] = prim
        UsdPhysics.RigidBodyAPI.Apply(prim)
        if args.collision:
            UsdPhysics.CollisionAPI.Apply(prim)
        mass_api = UsdPhysics.MassAPI.Apply(prim)
        mass_api.CreateMassAttr(float(body["mass"]))
        masses[body["match"]] = float(body["mass"])
        print(f"  body  {body['match']:<10} mass {body['mass']} kg"
              f"{'  +collision' if args.collision else ''}")

    # Pin the base to the world, otherwise the articulation is free-floating
    # and the whole cradle falls under gravity the moment physics starts.
    base = bodies[spec["root_body"]]
    fixed = UsdPhysics.FixedJoint.Define(stage, "/World/cradle/base_to_world")
    fixed.CreateBody1Rel().SetTargets([base.GetPath()])
    print(f"  fixed base_to_world -> {base.GetPath()}")

    axis_token = {"X": "X", "Y": "Y", "Z": "Z"}
    for spec_joint in spec["joints"]:
        name = spec_joint["name"]
        parent = bodies[spec_joint["parent"]]
        child = bodies[spec_joint["child"]]
        pivot = Gf.Vec3f(*[float(v) for v in spec_joint["pivot"]])
        path = f"/World/cradle/{name}"

        if spec_joint["type"] == "revolute":
            joint = UsdPhysics.RevoluteJoint.Define(stage, path)
            drive_token = "angular"
        else:
            joint = UsdPhysics.PrismaticJoint.Define(stage, path)
            drive_token = "linear"

        joint.CreateBody0Rel().SetTargets([parent.GetPath()])
        joint.CreateBody1Rel().SetTargets([child.GetPath()])
        joint.CreateAxisAttr(axis_token[spec_joint["axis"]])

        # The pivot is one point in world space, expressed in each body's
        # own frame. Converted CAD bodies carry translation but no rotation,
        # so subtracting the body origin is sufficient here; a rotated body
        # would need the full inverse transform.
        p_world = world_translation(parent)
        c_world = world_translation(child)
        joint.CreateLocalPos0Attr(Gf.Vec3f(pivot[0] - p_world[0],
                                           pivot[1] - p_world[1],
                                           pivot[2] - p_world[2]))
        joint.CreateLocalPos1Attr(Gf.Vec3f(pivot[0] - c_world[0],
                                           pivot[1] - c_world[1],
                                           pivot[2] - c_world[2]))
        joint.CreateLowerLimitAttr(float(spec_joint["lower"]))
        joint.CreateUpperLimitAttr(float(spec_joint["upper"]))

        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), drive_token)
        gains = spec_joint["drive"]
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(float(gains["stiffness"]))
        drive.CreateDampingAttr(float(gains["damping"]))
        drive.CreateMaxForceAttr(float(gains["max_force"]))
        drive.CreateTargetPositionAttr(0.0)

        unit = "deg" if spec_joint["type"] == "revolute" else "m"
        print(f"  joint {name:<10} {spec_joint['type']:<9} axis {spec_joint['axis']}  "
              f"limits {spec_joint['lower']}..{spec_joint['upper']} {unit}")

    stage.GetRootLayer().Save()
    print(f"\nwrote {args.out}")
    print(f"articulation root: {asset.GetPath()}")
    print("\nDrive gains and the mass split are ESTIMATES. The deadbands that "
          "depend on them\nmust be measured on the running model -- see "
          "docs/byo-model.md.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
