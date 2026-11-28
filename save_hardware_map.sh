#!/usr/bin/env bash
# ==============================================================================
# save_hardware_map.sh
# ==============================================================================
# Saves the live SLAM Toolbox occupancy grid map to disk (.yaml + .pgm).
#
# Usage:
#   ./save_hardware_map.sh [MAP_NAME] [OUTPUT_DIR]
#
# Examples:
#   ./save_hardware_map.sh                           # Saves to maps/hardware_map.yaml & .pgm
#   ./save_hardware_map.sh room_test                 # Saves to maps/room_test.yaml & .pgm
#   ./save_hardware_map.sh warehouse_lab ~/my_maps   # Saves to ~/my_maps/warehouse_lab.*
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAP_NAME="${1:-hardware_map}"
OUTPUT_DIR="${2:-$SCRIPT_DIR/maps}"

mkdir -p "$OUTPUT_DIR"
FULL_PATH="$OUTPUT_DIR/$MAP_NAME"

echo "=================================================================="
echo "  AMR Real-Time Hardware Map Saver"
echo "  Target Output : $FULL_PATH.yaml & .pgm"
echo "=================================================================="

# 1. Source ROS 2 environment (pass empty or valid IP, not MAP_NAME)
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
fi

if [ -f "$SCRIPT_DIR/env_hardware.sh" ]; then
    source "$SCRIPT_DIR/env_hardware.sh" "10.244.248.208"
else
    export ROS_DOMAIN_ID=7
    if dpkg -s ros-jazzy-rmw-cyclonedds-cpp >/dev/null 2>&1; then
        export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
        if [ -f "$SCRIPT_DIR/cyclonedds.xml" ]; then
            export CYCLONEDDS_URI="file://$SCRIPT_DIR/cyclonedds.xml"
        fi
    fi
fi

# 2. Check if /map topic is alive
echo "[INFO] Verifying /map topic is available..."
MAP_PUB=$(ros2 topic info /map --no-daemon 2>/dev/null | grep "Publisher count:" | awk '{print $3}' || echo "0")

if [ "$MAP_PUB" = "0" ] || [ -z "$MAP_PUB" ]; then
    echo "❌ ERROR: No publishers found on topic /map!"
    echo "   Ensure SLAM Toolbox is running: ./start_hardware_mapping.sh"
    exit 1
fi

echo "✅ Topic /map is active ($MAP_PUB publisher(s)). Saving map..."

# 3. Save map via nav2_map_server map_saver_cli
# NOTE: Arguments (-f <name>) must precede --ros-args in ROS 2 Jazzy!
ros2 run nav2_map_server map_saver_cli \
    -f "$FULL_PATH" \
    --occ 0.65 \
    --free 0.25 \
    --fmt pgm \
    --ros-args \
    -p use_sim_time:=false \
    -p save_map_timeout:=10.0

if [ -f "${FULL_PATH}.yaml" ] && [ -f "${FULL_PATH}.pgm" ]; then
    # Auto-generate viewable PNG version and apply cluster noise filter
    python3 -c "
import cv2, numpy as np
from PIL import Image

# 1. Generate standard PNG
Image.open('${FULL_PATH}.pgm').save('${FULL_PATH}.png')

# 2. Filter isolated single-cell noise specks (< 4 cells = < 10cm x 10cm) without turning unknown space into false free pockets
img = cv2.imread('${FULL_PATH}.pgm', cv2.IMREAD_UNCHANGED)
if img is not None:
    occ = (img == 0).astype(np.uint8)
    free_mask = (img >= 250).astype(np.uint8)
    free_dilated = cv2.dilate(free_mask, np.ones((3, 3), dtype=np.uint8), iterations=1)
    n, labels, stats, _ = cv2.connectedComponentsWithStats(occ, connectivity=8)
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] < 4:
            comp_mask = (labels == i)
            if np.any(free_dilated[comp_mask] > 0):
                img[comp_mask] = 254
            else:
                img[comp_mask] = 205
    cv2.imwrite('${FULL_PATH}.pgm', img)
    cv2.imwrite('${FULL_PATH}.png', img)
" 2>/dev/null || true

    echo ""
    echo "=================================================================="
    echo "  ✅ MAP SAVED SUCCESSFULLY (Filtered & Cleaned)!"
    echo "=================================================================="
    echo "  YAML Metadata : ${FULL_PATH}.yaml"
    echo "  PGM Raw Map   : ${FULL_PATH}.pgm"
    if [ -f "${FULL_PATH}.png" ]; then
        echo "  PNG Viewer    : ${FULL_PATH}.png"
    fi
    echo ""
    echo "  Quick verification:"
    cat "${FULL_PATH}.yaml"
    echo "=================================================================="
else
    echo "❌ Map saving failed or files were not generated."
    exit 1
fi
