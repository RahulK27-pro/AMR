#!/bin/bash
# view_hospital.sh — Launch and view Hospital Ground Floor Plan simulation in Gazebo
# ===================================================================================

echo "================================================================="
echo "  Starting Hospital Simulation (hospital.world) in Gazebo Sim..."
echo "  Dimensions: 16.0m x 12.0m | 2.0m Main Corridor | 10 Rooms"
echo "  Press Ctrl+C to close Gazebo"
echo "================================================================="

# Source ROS 2 and Workspace environments
source /opt/ros/jazzy/setup.bash
if [ -f "$HOME/AMR/AMR-main/install/setup.bash" ]; then
  source "$HOME/AMR/AMR-main/install/setup.bash"
fi

# Mesa / software rendering variables for WSL2 & Linux
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
export MESA_GL_VERSION_OVERRIDE=4.5

# Network interface setting to avoid multicast router issues
export GZ_IP=127.0.0.1
export ROS_LOCALHOST_ONLY=1

# World path
WORLD_PATH="$HOME/AMR/AMR-main/src/agv_description/worlds/hospital.world"

# Launch Gazebo Sim with hospital.world directly
gz sim -r "$WORLD_PATH"
