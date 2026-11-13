#!/usr/bin/env bash
# ==============================================================================
# AMR Master Physical Hardware Startup Script (Raspberry Pi 5)
# ==============================================================================
# Sourcing environment:
#   - ROS 2 Jazzy (/opt/ros/jazzy)
#   - YDLIDAR ROS 2 Driver (~/ydlidar_ws)
#   - AMR Main Stack (~/AMR/AMR-main)
#
# Verification:
#   In a second terminal, run:
#     ros2 topic hz /scan /imu/data /odom /odometry/filtered
# ==============================================================================

set -e

echo "=================================================================="
echo "  Starting AMR Hardware Stack (Pi 5 + RMCS-2305 + MPU-6500 + X2)  "
echo "=================================================================="

# 1. Source ROS 2 Core
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
    echo "[OK] Sourced ROS 2 Jazzy"
else
    echo "[ERROR] /opt/ros/jazzy/setup.bash not found!"
    exit 1
fi

# 2. Source YDLIDAR Workspace if present
if [ -f "$HOME/ydlidar_ws/install/setup.bash" ]; then
    source "$HOME/ydlidar_ws/install/setup.bash"
    echo "[OK] Sourced YDLIDAR driver workspace"
fi

# 3. Source AMR Workspace
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Auto-build if hardware_bringup.launch.py is not yet installed in share directory
if [ ! -f "$SCRIPT_DIR/install/agv_description/share/agv_description/launch/hardware_bringup.launch.py" ]; then
    echo "[INFO] New launch files detected. Building agv_description package..."
    (cd "$SCRIPT_DIR" && colcon build --packages-select agv_description --symlink-install)
fi

if [ -f "$SCRIPT_DIR/install/setup.bash" ]; then
    source "$SCRIPT_DIR/install/setup.bash"
    echo "[OK] Sourced AMR workspace"
else
    echo "[ERROR] $SCRIPT_DIR/install/setup.bash not found! Run colcon build first."
    exit 1
fi

# Set ROS Domain ID (stack uses domain 7)
export ROS_DOMAIN_ID=7
echo "[OK] ROS_DOMAIN_ID=7"

# 4. Launch Master Hardware Stack
echo "Launching hardware_bringup.launch.py..."
ros2 launch agv_description hardware_bringup.launch.py "$@"

