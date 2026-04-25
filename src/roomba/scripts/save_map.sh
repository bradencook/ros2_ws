#!/usr/bin/env bash
# Save a SLAM Toolbox map in both formats:
#   - PGM + YAML  (what Nav2 loads)
#   - .data + .posegraph  (what slam_toolbox deserialize_map reads)
#
# Usage:  ./save_map.sh [map_name]     (default: home)
# Run while a mapping session is active (slam_toolbox node must be up).

set -e

MAP_NAME="${1:-home}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAPS_DIR="$(realpath "$SCRIPT_DIR/../maps")"
MAP_PATH="$MAPS_DIR/$MAP_NAME"

echo "Saving map as: $MAP_PATH"

echo ""
echo "--- PGM + YAML (for Nav2) ---"
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: '$MAP_PATH'}}"

echo ""
echo "--- .data + .posegraph (for deserialize / resume) ---"
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '$MAP_PATH'}"

echo ""
echo "Done. Files in $MAPS_DIR:"
ls -la "$MAPS_DIR/"
