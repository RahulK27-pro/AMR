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
echo "[OK] Testing live message stream from Laptop @ $LAPTOP_IP..."

# Actually probe for live incoming /clock message from laptop
CLOCK_RECEIVED=$(python3 -c "
import rclpy, sys, time
from rosgraph_msgs.msg import Clock

rclpy.init()
node = rclpy.create_node('clock_probe')
got_msg = [False]
def cb(msg): got_msg[0] = True
sub = node.create_subscription(Clock, '/clock', cb, 10)

t0 = time.time()
while time.time() - t0 < 3.0 and not got_msg[0]:
    rclpy.spin_once(node, timeout_sec=0.1)

node.destroy_node()
rclpy.shutdown()
sys.exit(0 if got_msg[0] else 1)
" 2>/dev/null && echo "YES" || echo "NO")

if [ "$CLOCK_RECEIVED" = "YES" ]; then
    echo "✅ Successfully connected to Gazebo simulation on Laptop!"
    echo "   /clock is actively streaming from $LAPTOP_IP"
else
    echo "⚠️  Gazebo /clock is NOT streaming from $LAPTOP_IP yet."
    echo "   Check: Did you start the simulation on your laptop?"
    echo "          Run on Laptop: ./start_sim_background.sh $(hostname -I | awk '{print \$1}')"
fi

echo ""
echo "Starting System Diagnostics..."
python3 "$WS_DIR/diagnose_system.py"

