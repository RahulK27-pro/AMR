#!/bin/bash
# view_gazebo.sh — Connect Gazebo GUI client to running headless simulation
# =========================================================================
# Usage:
#   ./view_gazebo.sh
#
# Note: You can open and close the GUI window at any time.
# Closing the GUI does NOT stop the simulation server or navigation stack.

echo "========================================================="
echo "  Connecting Gazebo Harmonic GUI client..."
echo "  (Close the window or press Ctrl+C anytime to exit GUI)"
echo "========================================================="

# Source ROS 2 environment to ensure gz tools are in PATH
source /opt/ros/jazzy/setup.bash

# WSL2 / Linux environment variables for stable software OpenGL rendering and loopback networking
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
export MESA_GL_VERSION_OVERRIDE=4.5
export GZ_IP=127.0.0.1
unset ROS_LOCALHOST_ONLY
unset ROS_AUTOMATIC_DISCOVERY_RANGE
export DISPLAY="${DISPLAY:-:0}"
export WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/1000}"

# Connect GUI client to running simulation server
gz sim -g
