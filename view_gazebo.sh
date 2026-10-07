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

# WSL2 environment variables for stable software OpenGL rendering
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
export MESA_GL_VERSION_OVERRIDE=4.5

# Connect GUI client to running simulation server
gz sim -g
