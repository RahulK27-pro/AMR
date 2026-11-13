#!/usr/bin/env bash
# ==============================================================================
# env_hardware.sh
# ==============================================================================
# Source this file to configure your current terminal for the AMR robot:
#   source env_hardware.sh
# ==============================================================================

if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
fi

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
fi

export ROS_DOMAIN_ID=7
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$WS_DIR/cyclonedds.xml"

echo "=================================================================="
echo "  [OK] Terminal configured for AMR Physical Robot Stack"
echo "  ROS_DOMAIN_ID       : 7"
echo "  RMW_IMPLEMENTATION  : rmw_cyclonedds_cpp"
echo "  CYCLONEDDS_URI      : $CYCLONEDDS_URI (Unicast -> 10.73.81.208)"
echo "=================================================================="

