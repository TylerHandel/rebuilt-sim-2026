#!/usr/bin/env bash
# Export an Onshape assembly as GLB (binary glTF).
#   ONSHAPE_ACCESS_KEY=... ONSHAPE_SECRET_KEY=... tools/onshape-export.sh <out.glb> [element-id]
# Defaults to the official field, "FE-2026: REBUILT Playing Field". Other elements in that document:
# Block CAD 5e2b2310531e01f25fd97afd, carpet part studio 0dd2e12e8cfc4635c3f8fb96.
# Then slim it for the browser: node tools/slim-glb.mjs <out.glb> cad/field/field.glb
set -euo pipefail
OUT=${1:?usage: onshape-export.sh <out.glb> [element-id]}
DID=8a691e28680da30504859fce
WID=c6aa636fb23edb3f1e272fb1
EID=${2:-f4e47c668796f504844c94a0}
: "${ONSHAPE_ACCESS_KEY:?set ONSHAPE_ACCESS_KEY}" "${ONSHAPE_SECRET_KEY:?set ONSHAPE_SECRET_KEY}"
curl -sSfL --max-time 900 -u "$ONSHAPE_ACCESS_KEY:$ONSHAPE_SECRET_KEY" -H 'Accept: model/gltf-binary' \
  -o "$OUT" "https://cad.onshape.com/api/v6/assemblies/d/$DID/w/$WID/e/$EID/gltf"
echo "wrote $OUT ($(du -h "$OUT" | cut -f1))"
