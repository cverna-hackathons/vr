#!/bin/sh
# Clone a model: tools/new_model.sh NAME [FROM]   (FROM defaults to crate)
# Copies data/objects/FROM.blend to NAME.blend, points its glTF exporter at NAME.glb, rebuilds the helpers and exports NAME.glb.
# Then edit NAME.blend in Blender and use File > Export All Collections. BLENDER=/path/to/blender if it is not on PATH.
set -e
cd "$(dirname "$0")/.."
name=$1; from=${2:-crate}
[ -n "$name" ] || { echo "usage: $0 NAME [FROM]"; exit 2; }
src=data/objects/$from.blend; dst=data/objects/$name.blend
[ -f "$src" ] || { echo "missing $src"; exit 1; }
[ ! -e "$dst" ] || { echo "$dst already exists"; exit 1; }
blender=${BLENDER:-$(command -v blender || ls -d "$HOME"/Downloads/blender-*/blender 2>/dev/null | tail -1)}
[ -x "$blender" ] || { echo "blender not found; set BLENDER=/path/to/blender"; exit 1; }
cp "$src" "$dst"
"$blender" --background "$dst" --python tools/blender_helpers.py 2>&1 | grep -E "helpers added|Error|Traceback" || true
echo "Now edit $dst in Blender (File > Export All Collections writes data/objects/$name.glb). Map glossary line:"
echo "  Q  model  file=data/objects/$name.glb | $name"
