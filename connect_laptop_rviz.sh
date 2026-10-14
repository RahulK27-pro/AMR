#!/usr/bin/env bash
# connect_laptop_rviz.sh
# ======================
# Launch RViz2 on laptop connected directly to the Raspberry Pi's ROS 2 DDS domain (Domain 7).
# Works over Wi-Fi even when the router blocks UDP multicast by configuring CycloneDDS unicast peers.
#
# Usage:
#   ./connect_laptop_rviz.sh [PI_IP]
#
# Example:
#   ./connect_laptop_rviz.sh 10.87.204.118

set -e

PI_IP="${1:-10.87.204.118}"

echo "=================================================================="
echo "  AMR Remote Laptop Visualizer (RViz2) -> Raspberry Pi @ $PI_IP"
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

# 2. Configure CycloneDDS unicast peer discovery
CFG_FILE="/tmp/cyclonedds_laptop_$$.xml"
cat <<EOF > "$CFG_FILE"
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain Id="7">
    <Discovery>
      <Peers>
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
echo "[OK] Testing topic connection to Raspberry Pi..."
TOPICS=$(ros2 topic list 2>/dev/null || true)

if echo "$TOPICS" | grep -q "/cmd_vel\|/clock\|/odom"; then
    echo "✅ Successfully connected to AMR robot on Raspberry Pi!"
else
    echo "⚠️  Connected to network, waiting for AMR topics..."
fi

# 3. Resolve RViz config
RVIZ_CFG="$WS_DIR/src/agv_description/config/agv_nav.rviz"
if [ ! -f "$RVIZ_CFG" ]; then
    RVIZ_CFG="$WS_DIR/install/agv_description/share/agv_description/config/agv_nav.rviz"
fi

echo "[OK] Starting RViz2 with live robot telemetry..."
if [ -f "$RVIZ_CFG" ]; then
    exec rviz2 -d "$RVIZ_CFG"
else
    exec rviz2
fi
