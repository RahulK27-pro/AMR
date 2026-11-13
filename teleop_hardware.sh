#!/usr/bin/env bash
# ==============================================================================
# teleop_hardware.sh
# ==============================================================================
# ROS 2 Keyboard Teleoperation for AMR Physical Hardware
# Publishes geometry_msgs/Twist on /cmd_vel with Domain 7.
# Can be run directly on the Raspberry Pi OR remotely from your Laptop.
# ==============================================================================

if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
fi

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Source local hardware environment if present
if [ -f "$WS_DIR/env_hardware.sh" ]; then
    source "$WS_DIR/env_hardware.sh"
else
    export ROS_DOMAIN_ID=7
    if dpkg -s ros-jazzy-rmw-cyclonedds-cpp >/dev/null 2>&1; then
        export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
        if [ -f "/tmp/cyclonedds_pi.xml" ]; then
            export CYCLONEDDS_URI="file:///tmp/cyclonedds_pi.xml"
        fi
    fi
fi

echo "=================================================================="
echo "  AMR Teleoperation Keyboard Controller (Domain 7 -> /cmd_vel)"
echo "=================================================================="
echo "  Key Controls:"
echo "    [i] : Move Forward"
echo "    [,] : Move Reverse"
echo "    [j] : Turn Left"
echo "    [l] : Turn Right"
echo "    [k] : Stop"
echo "    [q] / [z] : Increase / Decrease linear speed (+10%)"
echo "    [w] / [x] : Increase / Decrease angular speed (+10%)"
echo "=================================================================="

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/cmd_vel

