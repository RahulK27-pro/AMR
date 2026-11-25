#!/bin/bash
# view_rviz.sh — Launch RViz2 visualizer for the running AMR system
# =================================================================
# Usage:
#   ./view_rviz.sh              (auto-detects sim nav/explore; or hardware if AMR_MODE=hardware)
#   ./view_rviz.sh explore      (simulation SLAM / exploration)
#   ./view_rviz.sh hw_nav       (hardware navigation with CycloneDDS Domain 7 & use_sim_time=false)
#   ./view_rviz.sh hw_explore   (hardware SLAM mapping with CycloneDDS Domain 7 & use_sim_time=false)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash
if [ -f "$SCRIPT_DIR/install/setup.bash" ]; then
    source "$SCRIPT_DIR/install/setup.bash"
fi

export DISPLAY="${DISPLAY:-:0}"
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"

# WSL2 stable rendering
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
export MESA_GL_VERSION_OVERRIDE=4.5

MODE="${1:-}"
AMR_MODE_ENV="${AMR_MODE:-sim}"

if [ -z "$MODE" ]; then
    if [ "$AMR_MODE_ENV" = "hardware" ]; then
        if pgrep -f "hardware_mapping" >/dev/null 2>&1 || pgrep -f "mapper_params_hardware" >/dev/null 2>&1; then
            MODE="hw_explore"
        else
            MODE="hw_nav"
        fi
    elif pgrep -f "async_slam_toolbox_node" >/dev/null 2>&1 || pgrep -f "slam_toolbox" >/dev/null 2>&1 || pgrep -f "mapping_session" >/dev/null 2>&1; then
        MODE="explore"
    else
        MODE="nav"
    fi
fi

if [ "$MODE" = "hw_nav" ] || [ "$MODE" = "hardware" ] || [ "$MODE" = "hw_explore" ] || [ "$MODE" = "hw_mapping" ] || [ "$AMR_MODE_ENV" = "hardware" ]; then
    unset ROS_LOCALHOST_ONLY
    if [ -z "$CYCLONEDDS_URI" ] && [ -f "$SCRIPT_DIR/env_hardware.sh" ]; then
        source "$SCRIPT_DIR/env_hardware.sh" "${AMR_PI_IP:-10.244.248.208}"
    else
        export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-7}"
        export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
        if [ -f "$SCRIPT_DIR/cyclonedds.xml" ] && [ -z "$CYCLONEDDS_URI" ]; then
            export CYCLONEDDS_URI="file://$SCRIPT_DIR/cyclonedds.xml"
        fi
    fi

    if [ "$MODE" = "hw_explore" ] || [ "$MODE" = "hw_mapping" ] || [ "$MODE" = "explore" ] || [ "$MODE" = "mapping" ]; then
        RVIZ_CFG="$SCRIPT_DIR/src/agv_description/config/agv_hardware_mapping.rviz"
        echo "=============================================="
        echo "  Launching RViz2 — Hardware Mapping Mode"
        echo "  Config: $RVIZ_CFG"
        echo "=============================================="
    else
        RVIZ_CFG="$SCRIPT_DIR/src/agv_description/config/agv_hardware_nav.rviz"
        echo "=============================================="
        echo "  Launching RViz2 — Hardware Navigation Mode"
        echo "  Config: $RVIZ_CFG"
        echo "=============================================="
    fi
    rviz2 -d "$RVIZ_CFG" --ros-args -p use_sim_time:=false
else
    unset CYCLONEDDS_URI
    export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-7}"
    export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
    export ROS_LOCALHOST_ONLY=1

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
fi

