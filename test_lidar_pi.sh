#!/usr/bin/env bash
# ==============================================================================
# AMR Isolated LiDAR Test Runner for Raspberry Pi
# ==============================================================================
# Usage:
#   ./test_lidar_pi.sh                 # Run interactive test (Ctrl+C to stop)
#   ./test_lidar_pi.sh --duration 15   # Run 15-second test and auto-save report
#   ./test_lidar_pi.sh --monitor-only  # Test against already-running Gazebo
# ==============================================================================

set -e

# Setup ROS 2 Jazzy and AMR workspace
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "$SCRIPT_DIR/install/setup.bash" ]; then
    source "$SCRIPT_DIR/install/setup.bash"
elif [ -f "$HOME/AMR/install/setup.bash" ]; then
    source "$HOME/AMR/install/setup.bash"
fi

# Multi-robot DDS & Mesa software OpenGL overrides for Raspberry Pi
export ROS_DOMAIN_ID=7
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3

# Clean up any leftover processes first
pkill -9 -f "ros_gz_bridge" 2>/dev/null || true
pkill -9 -f "gz sim" 2>/dev/null || true

# Run test recorder
python3 "$SCRIPT_DIR/test_lidar_pi.py" "$@"
