#!/usr/bin/env bash
# ==============================================================================
# start_hardware_nav.sh
# ==============================================================================
# Master Physical Hardware Navigation Launch Script
# Runs AMCL Localization, Topological Roadmap Routing, 10 Hz MPPI Control,
# and RViz2 on Laptop connected to Raspberry Pi 5.
#
# Usage:
#   ./scripts/hardware/start_hardware_nav.sh [PI_IP] [MAP_NAME_OR_YAML] [MAX_SPEED]
#
# Examples:
#   ./scripts/hardware/start_hardware_nav.sh 10.244.248.208
#   ./scripts/hardware/start_hardware_nav.sh 10.244.248.208 prakash_room
#   ./scripts/hardware/start_hardware_nav.sh 10.244.248.208 prakash_room 0.10
#   ./scripts/hardware/start_hardware_nav.sh 10.244.248.208 final_map
#   ./scripts/hardware/start_hardware_nav.sh 10.244.248.208 maps/yaml/prakash_room.yaml
# ==============================================================================

set -e

PI_IP="${1:-10.244.248.208}"
MAP_INPUT="${2:-prakash_room}"
MAX_SPEED="${3:-0.13}"

# Detect laptop IP on the route to the Pi
LAPTOP_IP=$(ip route get "$PI_IP" 2>/dev/null | awk '{for(i=1;i<=NF;i++) if($i=="src") print $(i+1)}' || true)
if [ -z "$LAPTOP_IP" ]; then
    LAPTOP_IP=$(hostname -I 2>/dev/null | awk '{print $1}' || true)
fi

echo "=================================================================="
echo "  AMR Physical Hardware Autonomous Navigation Session"
echo "  Target Pi IP      : $PI_IP"
echo "  Laptop Host IP    : ${LAPTOP_IP:-auto}"
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

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

# 2. Build packages if missing or source files updated
NEEDS_BUILD=0
if [ ! -f "$WS_DIR/install/agv_description/share/agv_description/launch/hardware_navigation.launch.py" ] || \
   [ "$WS_DIR/src/agv_description/launch/hardware_navigation.launch.py" -nt "$WS_DIR/install/agv_description/share/agv_description/launch/hardware_navigation.launch.py" ] || \
   [ "$WS_DIR/src/agv_description/config/nav2_params_hardware.yaml" -nt "$WS_DIR/install/agv_description/share/agv_description/config/nav2_params_hardware.yaml" ] || \
   [ "$WS_DIR/src/agv_description/config/agv_hardware_nav.rviz" -nt "$WS_DIR/install/agv_description/share/agv_description/config/agv_hardware_nav.rviz" ]; then
    NEEDS_BUILD=1
fi
if [ ! -f "$WS_DIR/install/agv_navigation/lib/agv_navigation/route_runner" ]; then
    NEEDS_BUILD=1
else
    for pyf in "$WS_DIR"/src/agv_navigation/agv_navigation/*.py; do
        if [ "$pyf" -nt "$WS_DIR/install/agv_navigation/lib/agv_navigation/route_runner" ]; then
            NEEDS_BUILD=1
            break
        fi
    done
fi
if [ "$NEEDS_BUILD" = "1" ]; then
    echo "[INFO] Updating package installation..."
    (cd "$WS_DIR" && colcon build --packages-select agv_description agv_navigation --symlink-install)
fi

if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
    echo "[OK] Sourced AMR workspace"
fi

# 3. Configure CycloneDDS unicast peering (Domain 7)
CFG_FILE="/tmp/cyclonedds_nav_$$.xml"
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

# 0. Inter-machine system clock synchronization
echo "[INFO] Synchronizing system clock with Raspberry Pi ($PI_IP)..."
if ssh -o ConnectTimeout=3 -o BatchMode=yes -o StrictHostKeyChecking=no "amr@$PI_IP" "sudo date -s '@$(date +%s.%N)'" 2>/dev/null; then
    echo "  ✅ Clock synchronized successfully with Raspberry Pi!"
else
    echo "  ⚠️  Could not auto-sync clock over SSH without password."
    echo "     If scans are dropped in AMCL, run on Pi: sudo date -s \"$(date -u +'%Y-%m-%d %H:%M:%S')\""
fi

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
    echo "       cd ~/AMR && ./scripts/hardware/start_hardware.sh $LAPTOP_IP"
fi

if echo "$TOPICS" | grep -q "/odom\|/odometry/filtered"; then
    echo "  ✅ Odometry topic detected!"
else
    echo "  ⚠️  Odometry topic not yet detected."
fi

# Prepare run logging directory
LOG_DIR="$WS_DIR/logs"
mkdir -p "$LOG_DIR"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
LAUNCH_LOG="$LOG_DIR/hardware_nav_${TIMESTAMP}.log"
LATEST_LAUNCH_LOG="$LOG_DIR/latest_launch.log"
ln -sf "$LAUNCH_LOG" "$LATEST_LAUNCH_LOG"

echo ""
echo "=================================================================="
echo "  Launching AMR Navigation Stack with Automatic Global Localization"
echo "  - Fixed Frame : map"
echo "  - Map Topic   : /map"
echo "  - Laser Topic : /scan"
echo "  - Auto-Pose   : ZERO-PRIOR SCAN MATCHER (Auto-detects robot position)"
echo "  - Goal Topic  : /goal_pose (Click '2D Goal Pose' in RViz)"
echo "  - Run Logs    : $LAUNCH_LOG (and latest_launch.log)"
echo "  - Telemetry   : $LOG_DIR/navigation_run_${TIMESTAMP}.csv"
echo ""
echo "  ✅ The robot will auto-identify its starting location anywhere in the room."
echo "  ✅ To re-localize after picking up / moving the robot at any time:"
echo "       ros2 service call /trigger_auto_relocalization std_srvs/srv/Trigger"
echo "  ✅ Or click '2D Pose Estimate' in RViz to set position manually."
echo "  ✅ Once cyan scan snaps onto the black walls, click '2D Goal Pose' to navigate!"
echo "=================================================================="
echo ""

# 6. Launch Hardware Navigation Stack and tee all output to log file
ros2 launch agv_description hardware_navigation.launch.py \
    map:="$MAP_INPUT" \
    max_linear_speed:="$MAX_SPEED" \
    cruise_linear_speed:="0.10" \
    max_angular_speed:="0.40" \
    lookahead_dist:="0.25" \
    use_rviz:=true 2>&1 | tee "$LAUNCH_LOG"
