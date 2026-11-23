"""Report whether a STEP file is a jointable assembly or a fused solid.

Run this BEFORE building anything on a vendor CAD model. A single merged
solid cannot be rigged: there are no separable bodies to put a joint between,
and discovering that after writing a mechanism table wastes the afternoon.

Vendor-level generalisations do not hold. Among Huber's own exports, 512_1 is
a clean four-body assembly while 511_1 is fused; among Kohzu's, SA05A-R2S01 is
well structured while MVRA10A-W01 is a lump. The only reliable answer is to
parse the file in front of you.

What it reports
---------------
* **schema** - AP203 / AP214 / AP242.
* **units** - the STEP length unit. This is the one that silently ruins a
  simulation: STEP is usually millimetres and USD stages are usually metres,
  so a 12 mm rail can arrive as a 12 m rail. The adapter catches the USD side
  of this (``metersPerUnit != 1`` is a configuration error), but knowing it
  here is cheaper than debugging a stage that moves wildly.
* **products** - the named parts. Kinematically meaningful names
  (``BASE``, ``CHI_PHI``, ``THETA_SHAFT``) are what let you map joints onto
  bodies without opening a CAD tool.
* **assembly links** - ``NEXT_ASSEMBLY_USAGE_OCCURRENCE`` entries, i.e. the
  parent/child structure. Zero means fused.

Usage::

    python3 -I tools/cad/step_structure.py <file.step> [more.step ...]

Reads only; never writes. Pass downloaded CAD as an argument rather than
running anything from inside its directory.
"""

import re
import sys

#: Exporters differ on whitespace: Creo and SolidWorks write `PRODUCT ( '`
#: while others write `PRODUCT('`. A parser that assumes one spelling reports
#: real assemblies as empty, which is worse than failing.
_PRODUCT = re.compile(r"PRODUCT\s*\(\s*'([^']*)'", re.I)
_ASSEMBLY = re.compile(r"NEXT_ASSEMBLY_USAGE_OCCURRENCE", re.I)
_SCHEMA = re.compile(r"FILE_SCHEMA\s*\(\s*\(\s*'([^']*)'", re.I)
_SI_UNIT = re.compile(r"SI_UNIT\s*\(\s*\.?([A-Z]*)\.?\s*,\s*\.([A-Z]+)\.", re.I)
_CONV_UNIT = re.compile(r"CONVERSION_BASED_UNIT\s*\(\s*'([^']*)'", re.I)
#: A solid body. Counting these separates "one part, many lumps" from
#: "one part, one lump" -- Thorlabs ships the former and it is still not
#: jointable, because the lumps are anonymous.
_SOLID = re.compile(r"(MANIFOLD_SOLID_BREP|BREP_WITH_VOIDS|SHELL_BASED_SURFACE_MODEL)", re.I)


def inspect(path):
    with open(path, errors="replace") as handle:
        text = handle.read()

    schema = _SCHEMA.search(text)
    products = []
    for name in _PRODUCT.findall(text):
        clean = name.strip()
        if clean and clean not in products:
            products.append(clean)

    units = sorted({m.group(1).lower() for m in _SI_UNIT.finditer(text) if m.group(1)})
    conv = sorted({m.group(1).strip().lower() for m in _CONV_UNIT.finditer(text)})

    return {
        "schema": schema.group(1) if schema else "unknown",
        "products": products,
        "assembly_links": len(_ASSEMBLY.findall(text)),
        "solids": len(_SOLID.findall(text)),
        "units": units or conv or ["unstated"],
    }


def verdict(info):
    if info["assembly_links"] == 0 or len(info["products"]) <= 1:
        return "FUSED - not jointable, do not build on this"
    if len(info["products"]) < 3:
        return "MINIMAL - two bodies, one joint at most"
    return "ASSEMBLY - jointable"


def main(argv):
    if len(argv) < 2:
        print(__doc__.strip().splitlines()[0], file=sys.stderr)
        print("usage: python3 -I tools/cad/step_structure.py <file.step> ...",
              file=sys.stderr)
        return 2
    worst = 0
    for path in argv[1:]:
        try:
            info = inspect(path)
        except OSError as exc:
            print(f"{path}: cannot read: {exc}", file=sys.stderr)
            worst = 2
            continue
        call = verdict(info)
        print(f"\n{path}")
        print(f"  schema         {info['schema']}")
        print(f"  units          {', '.join(info['units'])}")
        print(f"  assembly links {info['assembly_links']}")
        print(f"  solids         {info['solids']}")
        print(f"  products ({len(info['products'])})")
        for name in info["products"][:20]:
            print(f"      {name}")
        if len(info["products"]) > 20:
            print(f"      ... and {len(info['products']) - 20} more")
        print(f"  => {call}")
        if call.startswith("FUSED"):
            worst = max(worst, 1)
    return worst


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
