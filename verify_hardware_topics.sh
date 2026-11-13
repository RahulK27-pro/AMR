#!/usr/bin/env bash
# ==============================================================================
# Quick Hardware Topics Verification Tool
# ==============================================================================
# Checks that /scan, /imu/data, /odom, and /odometry/filtered are publishing live.
# ==============================================================================

if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "$SCRIPT_DIR/install/setup.bash" ]; then
    source "$SCRIPT_DIR/install/setup.bash"
fi

export ROS_DOMAIN_ID=7

if [ -f "/tmp/cyclonedds_pi.xml" ] && dpkg -s ros-jazzy-rmw-cyclonedds-cpp >/dev/null 2>&1; then
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI="file:///tmp/cyclonedds_pi.xml"
fi

echo "=================================================================="
echo "          AMR Hardware ROS 2 Topic Verification Monitor           "
echo "=================================================================="

echo ""
echo "[Step 1] Checking Active ROS 2 Topics:"
ros2 topic list

echo ""
echo "[Step 2] Monitoring Topic Frequencies (Sampling for 6 seconds)..."
echo "------------------------------------------------------------------"

ros2 topic hz /odom &
PID1=$!

ros2 topic hz /imu/data &
PID2=$!

ros2 topic hz /odometry/filtered &
PID3=$!

ros2 topic hz /scan &
PID4=$!

trap "kill $PID1 $PID2 $PID3 $PID4 2>/dev/null || true; exit 0" SIGINT SIGTERM

sleep 6

kill $PID1 $PID2 $PID3 $PID4 2>/dev/null || true
echo ""
echo "=================================================================="
echo " Verification Complete!"
echo " Expected Frequencies:"
echo "   /odom              : ~50 Hz"
echo "   /imu/data          : ~50 Hz"
echo "   /odometry/filtered : ~50 Hz"
echo "   /scan              : ~10 Hz"
echo "=================================================================="

