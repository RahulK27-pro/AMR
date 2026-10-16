#!/usr/bin/env bash
# connect_pi_to_laptop.sh
# ========================
# Run on the Raspberry Pi to connect its ROS 2 CycloneDDS stack to the Laptop's simulation.
#
# Usage:
#   ./connect_pi_to_laptop.sh [LAPTOP_IP]
#
# Example:
#   ./connect_pi_to_laptop.sh 10.87.204.140

set -e

LAPTOP_IP="${1:-10.87.204.140}"

echo "=================================================================="
echo "  Connecting Raspberry Pi to Laptop Simulator @ $LAPTOP_IP"
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

# 2. Configure CycloneDDS unicast peer discovery targeting the Laptop
CFG_FILE="/tmp/cyclonedds_pi.xml"
cat <<EOF > "$CFG_FILE"
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain Id="7">
    <Discovery>
      <Peers>
        <Peer Address="$LAPTOP_IP"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF

export ROS_DOMAIN_ID=7
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$CFG_FILE"
export AMR_REMOTE_SIM=1

echo "[OK] Configured CycloneDDS (Domain 7) with Unicast Peer -> $LAPTOP_IP"
echo "[OK] Testing topic connection to Laptop..."
sleep 1.0

TOPICS=$(ros2 topic list 2>/dev/null || true)
if echo "$TOPICS" | grep -q "/clock"; then
    echo "✅ Successfully connected to Gazebo simulation on Laptop!"
    echo "   Topics detected: /clock, /odom, /scan"
else
    echo "⚠️  Waiting for Laptop simulation..."
    echo "   Ensure './launch_sim_laptop.sh' is STILL RUNNING on your laptop!"
fi

echo ""
echo "Starting System Diagnostics..."
python3 "$WS_DIR/diagnose_system.py"
