#!/usr/bin/env bash
# ==============================================================================
# start_hardware_nav.sh
# ==============================================================================
# Master Physical Hardware Navigation Launch Script
# Runs AMCL Localization, Topological Roadmap Routing, 10 Hz MPPI Control,
# and RViz2 on Laptop connected to Raspberry Pi 5.
#
# Usage:
# Usage:
#   ./start_hardware_nav.sh [PI_IP] [MAP_NAME_OR_YAML] [MAX_SPEED]
#
# Examples:
#   ./start_hardware_nav.sh 10.244.248.208
#   ./start_hardware_nav.sh 10.244.248.208 prakash_room
#   ./start_hardware_nav.sh 10.244.248.208 prakash_room 0.10
#   ./start_hardware_nav.sh 10.244.248.208 final_map
#   ./start_hardware_nav.sh 10.244.248.208 maps/prakash_room.yaml
# ==============================================================================

set -e

PI_IP="${1:-10.244.248.208}"
MAP_INPUT="${2:-prakash_room}"
MAX_SPEED="${3:-0.18}"

# Detect laptop IP on the route to the Pi
LAPTOP_IP=$(ip route get "$PI_IP" 2>/dev/null | awk '{for(i=1;i<=NF;i++) if($i=="src") print $(i+1)}' || hostname -I | awk '{print $1}')

echo "=================================================================="
echo "  AMR Physical Hardware Autonomous Navigation Session"
echo "  Target Pi IP      : $PI_IP"
echo "  Laptop Host IP    : $LAPTOP_IP"
echo "  Selected Map      : $MAP_INPUT"
echo "  Linear Speed Cap  : $MAX_SPEED m/s (Minimum Safe Crawl)"
echo "=================================================================="

# 1. Source ROS 2 Core
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
    echo "[OK] Sourced ROS 2 Jazzy"
else
    echo "[ERROR] /opt/ros/jazzy/setup.bash not found!"
    exit 1
fi

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 2. Build packages if modified
if [ ! -f "$WS_DIR/install/agv_description/share/agv_description/launch/hardware_navigation.launch.py" ]; then
    echo "[INFO] Updating package installation..."
    (cd "$WS_DIR" && colcon build --packages-select agv_description agv_navigation --symlink-install)
fi

if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
    echo "[OK] Sourced AMR workspace"
fi

# 3. Configure CycloneDDS unicast peering (Domain 7)
CFG_FILE="/tmp/cyclonedds_nav_$$.xml"
cat <<EOF > "$CFG_FILE"
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain Id="7">
    <General>
      <Interfaces>
        <NetworkInterface address="$LAPTOP_IP"/>
      </Interfaces>
    </General>
    <Discovery>
      <Peers>
        <Peer Address="127.0.0.1"/>
        <Peer Address="$PI_IP"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF

export ROS_DOMAIN_ID=7
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$CFG_FILE"

# Clean up temporary DDS config on exit
trap "rm -f $CFG_FILE 2>/dev/null || true" EXIT

# 4. Kill any lingering SLAM or static transform publishers
if pgrep -f "slam_toolbox|static_transform_publisher.*map.*odom" >/dev/null 2>&1; then
    echo "[WARN] Found lingering SLAM / static TF processes. Terminating them so AMCL owns 'map -> odom'..."
    pkill -f "slam_toolbox|static_transform_publisher.*map.*odom" || true
    sleep 0.5
fi

# 5. Check topic discovery to the Raspberry Pi
echo "[INFO] Verifying ROS 2 connection to Raspberry Pi ($PI_IP)..."
sleep 1.0

TOPICS=$(ros2 topic list --no-daemon 2>/dev/null || true)

if echo "$TOPICS" | grep -q "/scan"; then
    echo "  ✅ LiDAR /scan topic detected from Raspberry Pi!"
else
    echo "  ⚠️  LiDAR /scan topic not yet detected."
    echo "     Ensure the hardware stack is running on the Pi:"
    echo "       ssh amr@$PI_IP"
    echo "       cd ~/AMR && ./start_hardware.sh $LAPTOP_IP"
fi

if echo "$TOPICS" | grep -q "/odom\|/odometry/filtered"; then
    echo "  ✅ Odometry topic detected!"
else
    echo "  ⚠️  Odometry topic not yet detected."
fi

echo ""
echo "=================================================================="
echo "  Launching AMCL Localization + Topological Route Runner (MPPI) + RViz2"
echo "  - Fixed Frame : map"
echo "  - Map Topic   : /map"
echo "  - Laser Topic : /scan"
echo "  - Odom Topic  : /odometry/filtered"
echo "  - Goal Topic  : /goal_pose (Click '2D Goal Pose' in RViz)"
echo ""
echo "  👉 STEP 1: Once RViz opens, inspect the cyan LiDAR scan."
echo "             Does it align with the room walls?"
echo "  👉 STEP 2: If tilted or shifted, click '2D Pose Estimate' on the"
echo "             top toolbar and click-drag the robot's real pose & heading."
echo "  👉 STEP 3: Verify the cyan scan aligns ON TOP of the black walls."
echo "  👉 STEP 4: Click '2D Goal Pose' on a golden roadmap node to navigate!"
echo "=================================================================="
echo ""

# 6. Launch Hardware Navigation Stack
exec ros2 launch agv_description hardware_navigation.launch.py \
    map:="$MAP_INPUT" \
    max_linear_speed:="$MAX_SPEED" \
    use_rviz:=true
