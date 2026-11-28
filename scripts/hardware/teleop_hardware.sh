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

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Prefer active Pi bringup DDS config when running on Raspberry Pi; otherwise configure via env_hardware.sh
if [ -f "/tmp/cyclonedds_pi.xml" ] && [ ! -f "/etc/wsl.conf" ]; then
    if [ -f "$WS_DIR/install/setup.bash" ]; then
        source "$WS_DIR/install/setup.bash"
    fi
    export ROS_DOMAIN_ID=7
    if dpkg -s ros-jazzy-rmw-cyclonedds-cpp >/dev/null 2>&1; then
        export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
        export CYCLONEDDS_URI="file:///tmp/cyclonedds_pi.xml"
    fi
elif [ -f "$SCRIPT_DIR/env_hardware.sh" ]; then
    source "$SCRIPT_DIR/env_hardware.sh"
else
    export ROS_DOMAIN_ID=7
    if dpkg -s ros-jazzy-rmw-cyclonedds-cpp >/dev/null 2>&1; then
        export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
        if [ -f "/tmp/cyclonedds_pi.xml" ]; then
            export CYCLONEDDS_URI="file:///tmp/cyclonedds_pi.xml"
        fi
    fi
fi

# Launch the dedicated WASD Hold-to-Move ROS 2 Teleop Controller
exec python3 "$SCRIPT_DIR/teleop_wasd.py" "$@"

