#!/usr/bin/env bash
# ==============================================================================
# start_hardware_mapping.sh
# ==============================================================================
# Master SLAM Mapping Launch Script for AMR Physical Hardware
# Runs SLAM Toolbox and RViz2 on Laptop connected to Raspberry Pi 5.
#
# Usage:
#   ./start_hardware_mapping.sh [PI_IP]
#
# Example:
#   ./start_hardware_mapping.sh 10.73.81.208
# ==============================================================================

set -e

PI_IP="${1:-10.244.248.208}"

# Automatically detect laptop IP on the route to the Pi
LAPTOP_IP=$(ip route get "$PI_IP" 2>/dev/null | awk '{for(i=1;i<=NF;i++) if($i=="src") print $(i+1)}' || true)
if [ -z "$LAPTOP_IP" ]; then
    LAPTOP_IP=$(hostname -I 2>/dev/null | awk '{print $1}' || true)
fi

echo "=================================================================="
echo "  AMR Physical Hardware SLAM Mapping Session (SLAM Toolbox + RViz)"
echo "  Pi IP     : $PI_IP"
echo "  Laptop IP : ${LAPTOP_IP:-auto}"
echo "=================================================================="

# 1. Source ROS 2 Jazzy Core
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
    echo "[OK] Sourced ROS 2 Jazzy"
else
    echo "[ERROR] /opt/ros/jazzy/setup.bash not found!"
    exit 1
fi

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 2. Build agv_description if config, launch, or rviz changed
if [ ! -f "$WS_DIR/install/agv_description/share/agv_description/launch/hardware_mapping.launch.py" ] || \
   [ "$WS_DIR/src/agv_description/launch/hardware_mapping.launch.py" -nt "$WS_DIR/install/agv_description/share/agv_description/launch/hardware_mapping.launch.py" ] || \
   [ "$WS_DIR/src/agv_description/config/agv_hardware_mapping.rviz" -nt "$WS_DIR/install/agv_description/share/agv_description/config/agv_hardware_mapping.rviz" ] || \
   [ "$WS_DIR/src/agv_description/config/mapper_params_hardware.yaml" -nt "$WS_DIR/install/agv_description/share/agv_description/config/mapper_params_hardware.yaml" ]; then
    echo "[INFO] Updating agv_description installation..."
    (cd "$WS_DIR" && colcon build --packages-select agv_description --symlink-install)
fi

if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
    echo "[OK] Sourced AMR workspace"
fi

# 3. Configure CycloneDDS unicast peering (Domain 7)
CFG_FILE="/tmp/cyclonedds_mapping_$$.xml"
if [ -n "$LAPTOP_IP" ]; then
    NET_IFACE_XML="<NetworkInterface address=\"$LAPTOP_IP\"/>"
else
    NET_IFACE_XML="<NetworkInterface autodetermine=\"true\"/>"
fi
cat <<EOF > "$CFG_FILE"
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain Id="7">
    <General>
      <Interfaces>
        $NET_IFACE_XML
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

# 4. CRITICAL: Kill any lingering static transform publishers (map -> odom)
# In pure hardware bringup, a static publisher is used; in SLAM, SLAM Toolbox MUST own map -> odom!
if pgrep -f "static_transform_publisher.*map.*odom" >/dev/null 2>&1; then
    echo "[WARN] Found existing static 'map -> odom' transform publisher. Terminating it to avoid TF collisions with SLAM Toolbox..."
    pkill -f "static_transform_publisher.*map.*odom" || true
    sleep 0.5
fi

# 5. Check topic discovery to the Raspberry Pi
echo "[INFO] Verifying ROS 2 connection to Raspberry Pi ($PI_IP)..."
sleep 1.5

TOPICS=$(ros2 topic list --no-daemon 2>/dev/null || true)

if echo "$TOPICS" | grep -q "/scan"; then
    echo "✅ LiDAR /scan topic detected from Raspberry Pi!"
else
    echo "⚠️  LiDAR /scan topic not yet detected."
    echo "   Ensure the hardware stack is running on the Pi:"
    echo "     ssh amr@$PI_IP"
    echo "     cd ~/AMR && ./start_hardware.sh $LAPTOP_IP"
fi

if echo "$TOPICS" | grep -q "/odom\|/odometry/filtered"; then
    echo "✅ Odometry topic detected!"
else
    echo "⚠️  Odometry topic not yet detected. (Will attach once Pi starts)"
fi

echo ""
echo "=================================================================="
echo "  Launching SLAM Toolbox (Real Hardware Parameters) + RViz2"
echo "  - Fixed Frame : map"
echo "  - Map Topic   : /map"
echo "  - Laser Topic : /scan"
echo "  - Odom Frame  : odom"
echo "=================================================================="
echo ""

# 6. Launch Hardware SLAM Mapping
exec ros2 launch agv_description hardware_mapping.launch.py \
    use_sim_time:=false \
    slam_params_file:="$WS_DIR/src/agv_description/config/mapper_params_hardware.yaml" \
    rviz_config:="$WS_DIR/src/agv_description/config/agv_hardware_mapping.rviz" \
    use_rviz:=true
