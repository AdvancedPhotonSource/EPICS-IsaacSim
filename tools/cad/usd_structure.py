"""Report whether a converted USD is actually riggable, and correctly scaled.

The companion to step_structure.py, run on the other side of the conversion.
STEP structure surviving the converter is not a given, and two failures here
are silent:

* **Scale.** ``metersPerUnit`` must be 1.0. STEP is usually millimetres, and
  a converter left on its default emits ``0.001``. Composed into a metres
  scene every dimension is 1000x out, which looks like a physics explosion
  rather than a units bug. The adapter treats ``metersPerUnit != 1`` as a
  configuration error for the same reason.

* **Instancing.** If the converter instanced the geometry, the meshes live
  under ``/Prototypes`` and a plain ``Traverse()`` cannot see them. You then
  cannot apply ``CollisionAPI``, and -- relevant to PV bindings -- you cannot
  author per-instance attributes on them either, so ``epics:setpointPV`` has
  nowhere to go. The report compares ``Traverse()`` against ``TraverseAll()``
  so a gap is visible rather than inferred.

Expected and normal: no physics prims at all. CAD carries shape and nothing
else -- no joints, no masses, no limits. Those are authored, by someone who
knows the hardware.

Usage, inside the Isaac Sim container (needs pxr)::

    /isaac-sim/python.sh tools/cad/usd_structure.py <stage.usd> [body ...]

Naming any expected bodies checks they survived the conversion and reports
their prim paths, which is what a mechanism table has to reference.
"""

import glob
import os
import sys


def _bootstrap_pxr():
    """Make ``pxr`` importable under a bare ``python.sh`` run.

    Isaac Sim ships USD as an extension rather than on the default path, so
    a standalone script gets ModuleNotFoundError for pxr, and then -- once
    PYTHONPATH is set -- ImportError for libusd_tf.so, because the native
    libraries sit in a bin/ directory beside the Python package. Both are
    discoverable, so discover them here instead of making every caller pass
    two environment variables they have to look up first.

    Re-execs once with LD_LIBRARY_PATH set, because the dynamic loader reads
    it at process start and setting it from inside Python is too late.
    """
    try:
        import pxr  # noqa: F401
        return
    except ImportError:
        pass

    roots = sorted(glob.glob("/isaac-sim/extscache/omni.usd.libs-*"))
    root = next((r for r in roots if os.path.isdir(os.path.join(r, "pxr"))), None)
    if root is None:
        print("ERROR: could not find Isaac Sim's USD libraries. Run this "
              "inside the Isaac Sim container.", file=sys.stderr)
        raise SystemExit(2)

    libdir = os.path.join(root, "bin")
    if os.environ.get("_USD_BOOTSTRAPPED") != "1":
        env = dict(os.environ)
        env["_USD_BOOTSTRAPPED"] = "1"
        env["PYTHONPATH"] = root + os.pathsep + env.get("PYTHONPATH", "")
        env["LD_LIBRARY_PATH"] = libdir + os.pathsep + env.get("LD_LIBRARY_PATH", "")
        os.execve(sys.executable, [sys.executable] + sys.argv, env)
    sys.path.insert(0, root)


def main(argv):
    if len(argv) < 2:
        print("usage: usd_structure.py <stage.usd> [expected body ...]",
              file=sys.stderr)
        return 2
    path = argv[1]
    expected = [a.lower() for a in argv[2:]]

    _bootstrap_pxr()
    from pxr import Usd, UsdGeom, UsdPhysics

    stage = Usd.Stage.Open(path)
    if stage is None:
        print(f"ERROR: could not open {path}", file=sys.stderr)
        return 1

    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    up = UsdGeom.GetStageUpAxis(stage)

    default_prims = list(stage.Traverse())
    all_prims = list(stage.TraverseAll())
    meshes = [p for p in all_prims if p.GetTypeName() == "Mesh"]
    visible_meshes = [p for p in default_prims if p.GetTypeName() == "Mesh"]
    xforms = [p for p in all_prims if p.GetTypeName() == "Xform"]

    physics = [p for p in all_prims
               if p.HasAPI(UsdPhysics.CollisionAPI)
               or p.HasAPI(UsdPhysics.RigidBodyAPI)
               or p.IsA(UsdPhysics.Joint)]

    print(f"\n{path}")
    print(f"  metersPerUnit    {mpu}", end="")
    print("   OK" if abs(mpu - 1.0) < 1e-9 else
          "   *** WRONG: geometry will be out by a factor of "
          f"{1.0 / mpu:g} in a metres scene ***")
    print(f"  up axis          {up}")
    print(f"  prims            {len(default_prims)} via Traverse(), "
          f"{len(all_prims)} via TraverseAll()")
    print(f"  meshes           {len(visible_meshes)} visible, {len(meshes)} total", end="")
    if len(visible_meshes) < len(meshes):
        print("   *** INSTANCED: meshes hidden under prototypes; collisions "
              "and per-prim attributes will not work ***")
    else:
        print("   OK")
    print(f"  physics prims    {len(physics)}  (0 is expected: CAD has no kinematics)")

    print(f"  Xform prims ({len(xforms)})")
    for prim in xforms[:25]:
        print(f"      {prim.GetPath()}")
    if len(xforms) > 25:
        print(f"      ... and {len(xforms) - 25} more")

    missing = []
    if expected:
        print("  expected bodies")
        names = {str(p.GetPath()).lower(): str(p.GetPath()) for p in all_prims}
        for want in expected:
            hit = next((full for low, full in names.items() if want in low), None)
            if hit:
                print(f"      {want:<12} -> {hit}")
            else:
                print(f"      {want:<12} -> *** NOT FOUND ***")
                missing.append(want)

    ok = abs(mpu - 1.0) < 1e-9 and len(visible_meshes) == len(meshes) and not missing
    print(f"  => {'RIGGABLE' if ok else 'NOT READY - see the marked lines above'}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
