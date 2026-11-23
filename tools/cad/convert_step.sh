#!/usr/bin/env bash
# STEP -> USD, headless, with the options physics actually needs.
#
# Run inside the Isaac Sim container. Conversion is a BUILD step, not a
# runtime dependency: it spawns its own Kit process, takes about half a
# second per part, and the simulation never needs the converter again.
#
# Two things this encodes, both of which cost someone a debugging session
# before they were written down (digital-twin docs/cad-import-findings.md):
#
#   1. omni.kit.asset_converter -- the API every tutorial shows -- REJECTS
#      .step outright. CAD goes through omni.kit.converter.hoops_core.
#      omni.kit.converter.cad is a licence/config bundle with no code;
#      enabling it reports success and does nothing.
#
#   2. The converter's Python entry point opens GUI dialogs, so it cannot be
#      called in-process headlessly. The only scripted route is spawning a
#      separate Kit process, which is what NVIDIA's own launcher does.
#
# And the two options in opts.json are not optional:
#
#   sceneOpMetersPerUnit 1.0  -- default 0.0 keeps the model's units. STEP is
#       usually millimetres, so the USD arrives at metersPerUnit 0.001 and
#       everything is 1000x out in a metres scene.
#
#   instancingStyle 0         -- default instances the geometry under
#       /Prototypes, where a plain Traverse() cannot see the meshes and
#       CollisionAPI cannot be applied to the instance proxies. It also
#       blocks per-instance USD attributes, which is how PV bindings are
#       meant to be attached, so instancing has to be off for both reasons.
#       The value is a combo INDEX: 0 None, 1 Reference, 2 Instanceable.
#       Passing the UI label "None" raises a ValueError from int().
#
# Usage (inside the container):
#   convert_step.sh <input.step> <output.usd> [log.txt]

set -euo pipefail

INPUT="${1:?usage: convert_step.sh <input.step> <output.usd> [log]}"
OUTPUT="${2:?usage: convert_step.sh <input.step> <output.usd> [log]}"
LOG="${3:-/tmp/hoops_convert.log}"

ISAAC=/isaac-sim
HOOPS_EXT=$(ls -d "$ISAAC"/extscache/omni.kit.converter.hoops-* 2>/dev/null | head -1)
if [ -z "$HOOPS_EXT" ]; then
    echo "ERROR: omni.kit.converter.hoops not found under $ISAAC/extscache" >&2
    exit 1
fi
LAUNCHER="$HOOPS_EXT/omni/kit/converter/hoops/process/launch_hoops_app.py"

OPTS=$(mktemp /tmp/hoops_opts_XXXX.json)
cat > "$OPTS" <<JSON
{"sceneOpMetersPerUnit": 1.0, "instancingStyle": 0}
JSON

mkdir -p "$(dirname "$OUTPUT")"

# Capture the WHOLE log then filter. Three runs were once wasted on a
# one-character fix because the log was sent to /dev/null, and then because a
# grep for "warn" matched hundreds of unrelated reconnect warnings.
"$ISAAC/kit/kit" \
    --ext-folder "$ISAAC/extscache" \
    --ext-folder "$ISAAC/exts" \
    --ext-folder "$ISAAC/apps" \
    --enable omni.kit.converter.hoops_core \
    --no-window \
    --exec "$LAUNCHER --config-path $OPTS --input-path $INPUT --output-path $OUTPUT" \
    > "$LOG" 2>&1 || true

rm -f "$OPTS"

if [ -s "$OUTPUT" ]; then
    echo "OK  $(basename "$OUTPUT")  $(stat -c%s "$OUTPUT") bytes"
    exit 0
fi
echo "FAILED - no output written. Log tail:" >&2
tail -25 "$LOG" >&2
exit 1
