#!/usr/bin/env bash
# Convert a LIO-SAM GlobalMap.pcd into map.pgm + map.yaml.
# Usage: bash scripts/pcd_to_map.sh <maps/relative-directory>

set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MAPS_DIR="$PROJECT_DIR/maps"
CONVERTER="$PROJECT_DIR/third_party/pcd2pgm/build/pcd2gridmap"

if [[ ! -x "$CONVERTER" ]]; then
  echo "PCD converter is not built. Build it first:" >&2
  echo "  cmake -S third_party/pcd2pgm -B third_party/pcd2pgm/build" >&2
  echo "  cmake --build third_party/pcd2pgm/build" >&2
  exit 1
fi

if [[ $# -ne 1 ]]; then
  echo "Usage: bash scripts/pcd_to_map.sh <maps/relative-directory>" >&2
  echo "Example: bash scripts/pcd_to_map.sh maps/mini" >&2
  exit 1
fi

if [[ "$1" = /* ]]; then
  echo "Map directory must be relative to the project, for example: maps/mini" >&2
  exit 1
fi

MAP_DIR="$(realpath -m "$PROJECT_DIR/$1")"
if [[ "$MAP_DIR" != "$MAPS_DIR"/* || ! -f "$MAP_DIR/GlobalMap.pcd" ]]; then
  echo "No GlobalMap.pcd found. Save a LIO-SAM map first." >&2
  exit 1
fi

echo "Converting: $MAP_DIR/GlobalMap.pcd"
exec "$CONVERTER" "$MAP_DIR/GlobalMap.pcd" -o "$MAP_DIR/map"
