#!/bin/bash
# view_rviz.sh — Launch RViz2 visualizer for the running AMR system
# =================================================================
# Usage:
#   ./view_rviz.sh          (loads navigation config agv_nav.rviz)
#   ./view_rviz.sh explore  (loads exploration config agv_explore.rviz)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash
if [ -f "$SCRIPT_DIR/install/setup.bash" ]; then
    source "$SCRIPT_DIR/install/setup.bash"
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-7}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

# WSL2 stable rendering
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
export MESA_GL_VERSION_OVERRIDE=4.5

MODE="$1"
if [ -z "$MODE" ]; then
    # Auto-detect whether mapping (SLAM) or navigation is currently running
    if pgrep -f "async_slam_toolbox_node" >/dev/null 2>&1 || pgrep -f "slam_toolbox" >/dev/null 2>&1 || pgrep -f "mapping_session" >/dev/null 2>&1; then
        MODE="explore"
    else
        MODE="nav"
    fi
fi

if [ "$MODE" = "explore" ] || [ "$MODE" = "mapping" ]; then
    RVIZ_CFG="$SCRIPT_DIR/src/agv_description/config/agv_explore.rviz"
    echo "=============================================="
    echo "  Launching RViz2 — SLAM / Exploration Mode"
    echo "  Config: $RVIZ_CFG"
    echo "=============================================="
else
    RVIZ_CFG="$SCRIPT_DIR/src/agv_description/config/agv_nav.rviz"
    echo "=============================================="
    echo "  Launching RViz2 — Navigation Mode"
    echo "  Config: $RVIZ_CFG"
    echo "=============================================="
fi

rviz2 -d "$RVIZ_CFG" --ros-args -p use_sim_time:=true

