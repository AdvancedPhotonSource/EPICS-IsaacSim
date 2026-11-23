"""Fetch Huber goniometer CAD, then check it is actually jointable.

Why fetch instead of commit
---------------------------
Huber publishes no terms of use at all -- no licence, no grant, no
prohibition. That is default copyright, not permission, so the geometry stays
out of this repository and is pulled on demand instead. The same applies to
Kohzu. SmarAct explicitly forbids redistribution, and GrabCAD's terms bar it
outright, so neither is an option even as a fallback.

If geometry in-repo ever matters, Huber and Kohzu are the ones to ask: having
published no prohibition, they have nothing to reverse.

Why the structure check is not optional
---------------------------------------
Vendor-level claims break per model. Huber's own 512_1 is a clean four-body
assembly while 511_1 is a single fused solid, and the two were exported four
years apart. A fused solid has no separable bodies to put a joint between, so
it cannot be rigged at all -- and that is invisible until something parses it.
Every fetch is therefore gated through step_structure.py.

Usage::

    python3 -I tools/cad/fetch_huber.py --list
    python3 -I tools/cad/fetch_huber.py 512_1 --out ~/cad/huber

Downloads are untrusted data: each model lands in its own directory and is
only ever passed to the parser as a path.
"""

import argparse
import os
import subprocess
import sys
import urllib.error
import urllib.request
import zipfile

HERE = os.path.dirname(os.path.abspath(__file__))
BASE_URL = "https://www.xhuber.com/fileadmin/user_upload/3d/{model}_Export_Files.zip"

#: Every entry here was produced by running step_structure.py on the actual
#: download, not read off a catalogue page. The list is short because the
#: verified list IS short -- see the note below.
KNOWN_GOOD = {
    "512_1": ("Eulerian cradle", "BASE / Z / CHI_PHI / PHI_PLATE",
              "3 DOF: Z prismatic, chi and phi revolute", "AP242, mm"),
    "414": ("2-circle goniometer",
            "TABLE / GONIOMETER / THETA_SHAFT / 2THETA_PLATE",
            "2 DOF: theta and 2theta revolute", "AP203, mm"),
}

#: Verified fused by parsing: they download fine and cannot be rigged.
#:
#: 5101_05 is the cautionary one. A survey reported Huber's 5101_* linear
#: stages as BASE/SLIDE assemblies, and this specific model is a single
#: product with zero assembly links. The family generalisation was right in
#: aggregate and wrong for the model actually fetched, which is the entire
#: reason this tool refuses to take a vendor's or a survey's word for it.
KNOWN_FUSED = {"5101_05"}

#: Model names that 404. Huber's naming is not guessable from the family:
#: "5102_02" and "5103_C1" both look reasonable and neither exists. Find the
#: real name on the product page before fetching.
KNOWN_MISSING = {"5102_02", "5103_C1"}


def fetch(model, out_root):
    if model in KNOWN_FUSED:
        print(f"WARNING: {model} was verified as a fused solid with no "
              f"separable bodies. It cannot be rigged. Fetching anyway.",
              file=sys.stderr)
    if model in KNOWN_MISSING:
        print(f"WARNING: {model} returned 404 when last tried; the name is "
              f"probably wrong.", file=sys.stderr)

    target = os.path.join(os.path.abspath(out_root), model)
    os.makedirs(target, exist_ok=True)
    archive = os.path.join(target, "download.zip")
    url = BASE_URL.format(model=model)

    print(f"fetching {url}")
    request = urllib.request.Request(url, headers={"User-Agent": "cad-fetch"})
    try:
        with urllib.request.urlopen(request, timeout=120) as response:
            if response.status != 200:
                print(f"ERROR: HTTP {response.status}", file=sys.stderr)
                return None
            payload = response.read()
    except (urllib.error.URLError, OSError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return None

    # An unknown model name can return the site's own 200-with-HTML page
    # rather than a 404, so check what actually arrived.
    if not payload[:2] == b"PK":
        print(f"ERROR: {model} did not return a zip ({len(payload)} bytes). "
              f"Check the model name.", file=sys.stderr)
        return None

    with open(archive, "wb") as handle:
        handle.write(payload)
    extracted = os.path.join(target, "extracted")
    with zipfile.ZipFile(archive) as zf:
        # Refuse absolute or traversing members rather than trusting the file.
        for member in zf.namelist():
            if os.path.isabs(member) or ".." in member.split("/"):
                print(f"ERROR: refusing unsafe archive member {member!r}",
                      file=sys.stderr)
                return None
        zf.extractall(extracted)
    print(f"extracted to {extracted}")
    return extracted


def steps_in(root):
    found = []
    for dirpath, _, filenames in os.walk(root):
        for name in filenames:
            if name.lower().endswith((".stp", ".step")):
                found.append(os.path.join(dirpath, name))
    # The main assembly is the largest; gearbox sub-models are extras.
    return sorted(found, key=lambda p: -os.path.getsize(p))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("model", nargs="?", help="Huber model, e.g. 512_1")
    parser.add_argument("--out", default="./cad", help="Directory to download into")
    parser.add_argument("--list", action="store_true", help="Show verified models")
    args = parser.parse_args(argv)

    if args.list or not args.model:
        print("Verified jointable Huber models:\n")
        for name, (desc, bodies, dof, fmt) in KNOWN_GOOD.items():
            print(f"  {name:<10} {desc}  [{fmt}]")
            print(f"  {'':<10} bodies: {bodies}")
            print(f"  {'':<10} {dof}\n")
        print("Verified fused (cannot be rigged): " + ", ".join(sorted(KNOWN_FUSED)))
        print("Names that 404: " + ", ".join(sorted(KNOWN_MISSING)))
        print("\nAnything not listed: fetch it and read the verdict. Family\n"
              "names do not predict structure -- 5101_05 is fused although\n"
              "its family was reported as jointable.")
        return 0

    extracted = fetch(args.model, args.out)
    if extracted is None:
        return 1

    steps = steps_in(extracted)
    if not steps:
        print("ERROR: no STEP file in the archive", file=sys.stderr)
        return 1

    print(f"\nchecking structure of the main assembly ({len(steps)} STEP files found)")
    return subprocess.call(
        [sys.executable, "-I", os.path.join(HERE, "step_structure.py"), steps[0]])


if __name__ == "__main__":
    raise SystemExit(main())
