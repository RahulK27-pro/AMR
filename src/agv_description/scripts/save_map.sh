#!/usr/bin/env bash
# =============================================================================
# save_map.sh
# -----------
# Saves the current SLAM Toolbox map to ~/agv_ws/maps/
#
# The map is saved as two files:
#   warehouse_map.pgm  — greyscale image (white=free, black=wall, grey=unknown)
#   warehouse_map.yaml — metadata (resolution, origin, thresholds)
#
# Usage:
#   bash ~/agv_ws/src/agv_description/scripts/save_map.sh
#
# Run this while the SLAM launch is still active and the map looks complete.
# =============================================================================

set -e

MAP_DIR="$HOME/agv_ws/maps"
MAP_NAME="warehouse_map"

echo "──────────────────────────────────────────"
echo "  AGV Map Saver"
echo "──────────────────────────────────────────"
echo "  Save directory : $MAP_DIR"
echo "  Map name       : $MAP_NAME"
echo "──────────────────────────────────────────"

# Create maps directory if it doesn't exist
mkdir -p "$MAP_DIR"

echo "[INFO] Saving map — this may take a few seconds..."

ros2 run nav2_map_server map_saver_cli \
    --ros-args -p use_sim_time:=true \
    -- -f "$MAP_DIR/$MAP_NAME" \
    --ros-args -p save_map_timeout:=10.0

echo ""
echo "✅ Map saved successfully!"
echo "   PGM image : $MAP_DIR/${MAP_NAME}.pgm"
echo "   YAML meta : $MAP_DIR/${MAP_NAME}.yaml"
echo ""
echo "You can view the map image with:"
echo "   eog $MAP_DIR/${MAP_NAME}.pgm"
echo "   (or open with any image viewer — it's a standard greyscale image)"
echo ""
echo "Next step:"
echo "   Shut down the SLAM session, then run:"
echo "   ros2 launch agv_description navigation_launch.py"
