#!/usr/bin/env bash
# ==============================================================================
# connect_laptop_rviz.sh
# ==============================================================================
# Launch RViz2 on laptop connected directly to the Raspberry Pi's ROS 2 DDS domain (Domain 7).
# Works over Wi-Fi even when the router blocks UDP multicast by configuring CycloneDDS unicast peers.
#
# Usage:
#   ./connect_laptop_rviz.sh [PI_IP]
#
# Example:
#   ./connect_laptop_rviz.sh 10.73.81.208
# ==============================================================================

set -e

PI_IP="${1:-10.73.81.208}"

# Automatically detect laptop IP on the route to the Pi
LAPTOP_IP=$(ip route get "$PI_IP" 2>/dev/null | awk '{for(i=1;i<=NF;i++) if($i=="src") print $(i+1)}' || hostname -I | awk '{print $1}')

echo "=================================================================="
echo "  AMR Remote Laptop Visualizer (RViz2) -> Raspberry Pi"
echo "  Pi IP     : $PI_IP"
echo "  Laptop IP : $LAPTOP_IP"
echo "=================================================================="

# 1. Source ROS 2
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
else
    echo "[ERROR] /opt/ros/jazzy/setup.bash not found!"
    exit 1
fi

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
fi

# 2. Configure CycloneDDS unicast peer discovery (localhost + Pi)
CFG_FILE="/tmp/cyclonedds_laptop_$$.xml"
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

echo "[OK] Configured CycloneDDS (Domain 7) with Unicast Peer -> $PI_IP"
echo "[OK] Testing topic connection to Raspberry Pi (waiting 2s for DDS handshake)..."
sleep 2

TOPICS=$(ros2 topic list --no-daemon 2>/dev/null || true)

if echo "$TOPICS" | grep -q "/odom\|/scan\|/imu/data\|/cmd_vel"; then
    echo "✅ Successfully connected to AMR robot on Raspberry Pi!"
    echo "   Active topics found:"
    echo "$TOPICS" | grep "/odom\|/scan\|/imu\|/robot_description\|/tf" | sed 's/^/     - /'
else
    echo "⚠️  Waiting for AMR topics from Raspberry Pi..."
    echo "   If RViz shows no data, ensure CycloneDDS is running on the Pi:"
    echo "     1. On Pi: sudo apt update && sudo apt install -y ros-jazzy-rmw-cyclonedds-cpp"
    echo "     2. On Pi: ./start_hardware.sh $LAPTOP_IP"
fi

# 3. Publish fallback map -> odom TF so RViz renders cleanly even if Fixed Frame is left as 'map'
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --yaw 0 --pitch 0 --roll 0 --frame-id map --child-frame-id odom >/dev/null 2>&1 &
TF_PID=$!
trap "kill $TF_PID 2>/dev/null || true" EXIT

# 4. Resolve RViz config (prefer dedicated hardware bringup config with Fixed Frame: odom)
RVIZ_CFG="$WS_DIR/src/agv_description/config/agv_hardware.rviz"
if [ ! -f "$RVIZ_CFG" ]; then
    RVIZ_CFG="$WS_DIR/install/agv_description/share/agv_description/config/agv_hardware.rviz"
fi
if [ ! -f "$RVIZ_CFG" ]; then
    RVIZ_CFG="$WS_DIR/src/agv_description/config/agv_nav.rviz"
fi

echo "[OK] Launching RViz2 with config: $(basename "$RVIZ_CFG") (Fixed Frame: odom)..."
if [ -f "$RVIZ_CFG" ]; then
    rviz2 -d "$RVIZ_CFG"
else
    rviz2
fi
