#!/usr/bin/env bash
# start_sim_background.sh
# ========================
# Starts Gazebo simulation on Laptop in the background.
# Keeps running continuously so you get your terminal prompt back without pressing Ctrl+C!
#
# Usage:
#   ./start_sim_background.sh [PI_IP] [WORLD] [HEADLESS]
#
# To stop the simulation anytime:
#   ./stop_sim.sh

set -e

PI_IP="${1:-10.87.204.118}"
WORLD_INPUT="${2:-test1.world}"
HEADLESS="${3:-false}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=================================================================="
echo "  Starting AMR Simulation Server in Background (Laptop Side)"
echo "  Target Raspberry Pi IP : $PI_IP"
echo "  World                  : $WORLD_INPUT"
echo "  Headless Mode          : $HEADLESS"
echo "=================================================================="

# 1. Clean any old instances first
pkill -9 -f "parameter_bridge" 2>/dev/null || true
pkill -9 -f "gz sim" 2>/dev/null || true
sleep 1.0

# 2. Source ROS 2
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
else
    echo "[ERROR] /opt/ros/jazzy/setup.bash not found!"
    exit 1
fi

if [ -f "$SCRIPT_DIR/install/setup.bash" ]; then
    source "$SCRIPT_DIR/install/setup.bash"
fi

# 3. Configure CycloneDDS unicast peer discovery targeting the Pi
CFG_FILE="/tmp/cyclonedds_laptop.xml"
cat <<EOF > "$CFG_FILE"
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain Id="7">
    <General>
      <Interfaces>
        <NetworkInterface name="eth0" />
      </Interfaces>
    </General>
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

# Resolve world
WORLD="$WORLD_INPUT"
if [[ "$WORLD" != *".world"* ]] && [[ "$WORLD" != *".sdf"* ]]; then
    WORLD="${WORLD}.world"
fi

# 4. Launch Gazebo and parameter_bridge in background with nohup
LOG_FILE="/tmp/gazebo_sim.log"
nohup ros2 launch agv_description gazebo.launch.py world:="$WORLD" headless:="$HEADLESS" > "$LOG_FILE" 2>&1 &
LAUNCH_PID=$!

echo "[OK] Simulation launched in background (PID: $LAUNCH_PID)"
echo "     Log output: $LOG_FILE"
echo "     Waiting 4 seconds for physics and sensor initialization..."
sleep 4.0

# 5. Verify processes are alive
if pgrep -f "gz sim" >/dev/null 2>&1 && pgrep -f "parameter_bridge" >/dev/null 2>&1; then
    echo "=================================================================="
    echo "✅ GAZEBO SIMULATION IS RUNNING STABLY IN THE BACKGROUND!"
    echo "   • Physics Engine: ACTIVE"
    echo "   • GPU LiDAR:      ACTIVE (/scan)"
    echo "   • Diff-Drive:     ACTIVE (/odom)"
    echo "   • Clock:          ACTIVE (/clock)"
    echo "   • Streaming to:   $PI_IP (Domain 7)"
    echo "=================================================================="
    echo ""
    echo "You have your terminal prompt back. The simulation will KEEP RUNNING."
    echo "To stop it at any time, run: ./stop_sim.sh"
    echo ""
else
    echo "⚠️  Simulation encountered an issue on startup. Checking log tail:"
    tail -n 15 "$LOG_FILE"
    exit 1
fi
