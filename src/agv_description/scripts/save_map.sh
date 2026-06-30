#!/usr/bin/env bash
# =============================================================================
# save_map.sh
# -----------
# Saves the current SLAM Toolbox occupancy grid map to ~/maps/
#
# Output files:
#   warehouse_map.pgm  — greyscale image (254=free, 0=occupied, 205=unknown)
#   warehouse_map.yaml — metadata: resolution, origin, thresholds
#
# Usage (run while mapping_session.launch.py is still running):
#   bash ~/AMR/AMR-main/src/agv_description/scripts/save_map.sh
#
# Requirements:
#   - SLAM Toolbox must be active and publishing /map
#   - Run AFTER explore_lite has finished (no more frontier markers in RViz)
# =============================================================================

set -e

MAP_DIR="$HOME/maps"
MAP_NAME="warehouse_map"

echo "══════════════════════════════════════════"
echo "  AGV Map Saver"
echo "══════════════════════════════════════════"
echo "  Save directory : $MAP_DIR"
echo "  Output name    : $MAP_NAME"
echo "══════════════════════════════════════════"

# Verify ROS 2 environment is sourced
if ! command -v ros2 &> /dev/null; then
    echo "[ERROR] ros2 not found. Source your workspace first:"
    echo "   source ~/AMR/AMR-main/install/setup.bash"
    exit 1
fi

# Verify /map topic is being published
echo "[INFO] Checking /map topic is active..."
if ! ros2 topic info /map --no-daemon 2>/dev/null | grep -q "Publisher count: [1-9]"; then
    echo "[ERROR] /map is not being published."
    echo "        Is mapping_session.launch.py still running?"
    exit 1
fi
echo "[OK]   /map topic is active."

# Create output directory
mkdir -p "$MAP_DIR"

echo "[INFO] Saving map — this may take a few seconds..."
echo ""

# Save via nav2_map_server — all ros-args must be in ONE --ros-args group
ros2 run nav2_map_server map_saver_cli \
    --ros-args \
    -p use_sim_time:=true \
    -p save_map_timeout:=10.0 \
    -- -f "$MAP_DIR/$MAP_NAME"

echo ""
echo "══════════════════════════════════════════"
echo "  ✅  Map saved successfully!"
echo "══════════════════════════════════════════"
echo "  PGM image : $MAP_DIR/${MAP_NAME}.pgm"
echo "  YAML meta : $MAP_DIR/${MAP_NAME}.yaml"
echo ""
echo "  View the map:"
echo "    eog $MAP_DIR/${MAP_NAME}.pgm"
echo ""
echo "  Next steps:"
echo "  1. Verify the map looks complete in the image viewer"
echo "  2. Press Ctrl+C in the launch terminal to stop the mapping session"
echo "  3. Run the graph extraction pipeline (Phase 2)"
echo "══════════════════════════════════════════"
