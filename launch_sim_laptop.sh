#!/usr/bin/env bash
# launch_sim_laptop.sh
# =====================
# Launches Gazebo Harmonic simulation on the Laptop with hardware GPU acceleration.
# Streams /clock, /odom, /scan, and /imu/data over CycloneDDS (Domain 7) to the Raspberry Pi.
# Receives /cmd_vel from the Raspberry Pi's MPPI route runner to move the AGV in Gazebo.
#
# Usage:
#   ./launch_sim_laptop.sh [PI_IP] [WORLD] [HEADLESS]
#
# Examples:
#   ./launch_sim_laptop.sh                          # Default: Pi @ 10.87.204.118, test1.world, GUI visible
#   ./launch_sim_laptop.sh 10.87.204.118 warehouse  # Launch warehouse.world
#   ./launch_sim_laptop.sh 10.87.204.118 test1 true # Headless mode on laptop

set -e

PI_IP="${1:-10.87.204.118}"
WORLD_INPUT="${2:-test1.world}"
HEADLESS="${3:-false}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=================================================================="
echo "  AMR Distributed Digital Twin Simulator (Laptop Side)"
echo "  Target Raspberry Pi IP : $PI_IP"
echo "  World                  : $WORLD_INPUT"
echo "  Headless Mode          : $HEADLESS"
echo "=================================================================="

# 1. Source ROS 2
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
else
    echo "[ERROR] /opt/ros/jazzy/setup.bash not found!"
    exit 1
fi

if [ -f "$SCRIPT_DIR/install/setup.bash" ]; then
    source "$SCRIPT_DIR/install/setup.bash"
else
    echo "[WARN] Workspace overlay not found. Building agv_description..."
    colcon build --packages-select agv_description
    source "$SCRIPT_DIR/install/setup.bash"
fi

# 2. Configure CycloneDDS unicast peer discovery targeting the Pi
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

# Clean up stale processes and temp files on exit
cleanup() {
    echo ""
    echo "[SIM] Stopping Gazebo simulation and cleaning up..."
    pkill -9 -f "parameter_bridge" 2>/dev/null || true
    pkill -9 -f "gz sim" 2>/dev/null || true
    rm -f "$CFG_FILE"
    echo "[SIM] Done."
}
trap cleanup EXIT SIGINT SIGTERM

# Kill any existing stale simulation processes before starting
pkill -9 -f "parameter_bridge" 2>/dev/null || true
pkill -9 -f "gz sim" 2>/dev/null || true
sleep 0.5

# Resolve world argument
WORLD="$WORLD_INPUT"
if [[ "$WORLD" != *".world"* ]] && [[ "$WORLD" != *".sdf"* ]]; then
    WORLD="${WORLD}.world"
fi

echo "[OK] Configured CycloneDDS (Domain 7) with Unicast Peer -> $PI_IP"
echo "[OK] Starting Gazebo Harmonic with GPU LiDAR raycasting..."
echo "     Topics published to network: /clock, /odom, /scan, /imu/data"
echo "     Listening for motor commands : /cmd_vel"
echo "------------------------------------------------------------------"

ros2 launch agv_description gazebo.launch.py world:="$WORLD" headless:="$HEADLESS"
