#!/usr/bin/env bash
# ==============================================================================
# save_hardware_map.sh
# ==============================================================================
# Saves the live SLAM Toolbox occupancy grid map to disk (.yaml + .pgm).
#
# Usage:
#   ./scripts/hardware/save_hardware_map.sh [MAP_NAME] [OUTPUT_DIR]
#
# Examples:
#   ./scripts/hardware/save_hardware_map.sh                           # Saves to maps/{yaml,pgm,png}/hardware_map.*
#   ./scripts/hardware/save_hardware_map.sh room_test                 # Saves to maps/{yaml,pgm,png}/room_test.*
#   ./scripts/hardware/save_hardware_map.sh warehouse_lab ~/my_maps   # Saves to ~/my_maps/warehouse_lab.*
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
MAP_NAME="${1:-hardware_map}"
OUTPUT_DIR="${2:-$WS_DIR/maps}"

# Use structured {yaml,pgm,png}/ subdirectories for workspace maps/ (or any dir that already uses yaml/ & pgm/)
if [ "$OUTPUT_DIR" = "$WS_DIR/maps" ] || { [ -d "$OUTPUT_DIR/yaml" ] && [ -d "$OUTPUT_DIR/pgm" ]; }; then
    YAML_DIR="$OUTPUT_DIR/yaml"
    PGM_DIR="$OUTPUT_DIR/pgm"
    PNG_DIR="$OUTPUT_DIR/png"
    mkdir -p "$YAML_DIR" "$PGM_DIR" "$PNG_DIR"
    SAVE_BASE="$YAML_DIR/$MAP_NAME"
    YAML_FILE="$YAML_DIR/${MAP_NAME}.yaml"
    PGM_FILE="$PGM_DIR/${MAP_NAME}.pgm"
    PNG_FILE="$PNG_DIR/${MAP_NAME}.png"
    STRUCTURED_LAYOUT=1
else
    mkdir -p "$OUTPUT_DIR"
    SAVE_BASE="$OUTPUT_DIR/$MAP_NAME"
    YAML_FILE="$OUTPUT_DIR/${MAP_NAME}.yaml"
    PGM_FILE="$OUTPUT_DIR/${MAP_NAME}.pgm"
    PNG_FILE="$OUTPUT_DIR/${MAP_NAME}.png"
    STRUCTURED_LAYOUT=0
fi

echo "=================================================================="
echo "  AMR Real-Time Hardware Map Saver"
echo "  Target YAML : $YAML_FILE"
echo "  Target PGM  : $PGM_FILE"
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
        if [ -f "$WS_DIR/config/cyclonedds.xml" ]; then
            export CYCLONEDDS_URI="file://$WS_DIR/config/cyclonedds.xml"
        fi
    fi
fi

# 2. Check if /map topic is alive
echo "[INFO] Verifying /map topic is available..."
MAP_PUB=$(ros2 topic info /map --no-daemon 2>/dev/null | grep "Publisher count:" | awk '{print $3}' || echo "0")

if [ "$MAP_PUB" = "0" ] || [ -z "$MAP_PUB" ]; then
    echo "❌ ERROR: No publishers found on topic /map!"
    echo "   Ensure SLAM Toolbox is running: ./scripts/hardware/start_hardware_mapping.sh"
    exit 1
fi

echo "✅ Topic /map is active ($MAP_PUB publisher(s)). Saving map..."

# 3. Save map via nav2_map_server map_saver_cli
# NOTE: Arguments (-f <name>) must precede --ros-args in ROS 2 Jazzy!
ros2 run nav2_map_server map_saver_cli \
    -f "$SAVE_BASE" \
    --occ 0.65 \
    --free 0.25 \
    --fmt pgm \
    --ros-args \
    -p use_sim_time:=false \
    -p save_map_timeout:=10.0

if [ "$STRUCTURED_LAYOUT" = "1" ] && [ -f "${SAVE_BASE}.pgm" ]; then
    mv "${SAVE_BASE}.pgm" "$PGM_FILE"
    sed -i "s|^image:.*|image: ../pgm/${MAP_NAME}.pgm|" "$YAML_FILE"
fi

if [ -f "$YAML_FILE" ] && [ -f "$PGM_FILE" ]; then
    # Auto-generate viewable PNG version and apply cluster noise filter
    python3 -c "
import cv2, numpy as np
from PIL import Image

# 1. Generate standard PNG
Image.open('$PGM_FILE').save('$PNG_FILE')

# 2. Filter isolated single-cell noise specks (< 4 cells = < 10cm x 10cm) without turning unknown space into false free pockets
img = cv2.imread('$PGM_FILE', cv2.IMREAD_UNCHANGED)
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
    cv2.imwrite('$PGM_FILE', img)
    cv2.imwrite('$PNG_FILE', img)
" 2>/dev/null || true

    echo ""
    echo "=================================================================="
    echo "  ✅ MAP SAVED SUCCESSFULLY (Filtered & Cleaned)!"
    echo "=================================================================="
    echo "  YAML Metadata : $YAML_FILE"
    echo "  PGM Raw Map   : $PGM_FILE"
    if [ -f "$PNG_FILE" ]; then
        echo "  PNG Viewer    : $PNG_FILE"
    fi
    echo ""
    echo "  Quick verification:"
    cat "$YAML_FILE"
    echo "=================================================================="
else
    echo "❌ Map saving failed or files were not generated."
    exit 1
fi
