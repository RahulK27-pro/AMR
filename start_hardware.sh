#!/usr/bin/env bash
# ==============================================================================
# AMR Master Physical Hardware Startup Script (Raspberry Pi 5)
# ==============================================================================
# Usage:
#   ./start_hardware.sh [LAPTOP_IP]
#
# Examples:
#   ./start_hardware.sh                # Local domain 7 multicast
#   ./start_hardware.sh 10.73.81.140   # Connects unicast to remote Laptop RViz
# ==============================================================================

set -e

LAPTOP_IP=""
if [[ "$1" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    LAPTOP_IP="$1"
    shift
fi

echo "=================================================================="
echo "  Starting AMR Hardware Stack (Pi 5 + RMCS-2305 + MPU-6500 + X2)  "
if [ -n "$LAPTOP_IP" ]; then
echo "  Target Remote Laptop RViz : $LAPTOP_IP"
fi
echo "=================================================================="

# 1. Source ROS 2 Core
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
    echo "[OK] Sourced ROS 2 Jazzy"
else
    echo "[ERROR] /opt/ros/jazzy/setup.bash not found!"
    exit 1
fi

# 2. Source YDLIDAR Workspace if present
if [ -f "$HOME/ydlidar_ws/install/setup.bash" ]; then
    source "$HOME/ydlidar_ws/install/setup.bash"
    echo "[OK] Sourced YDLIDAR driver workspace"
fi

# 3. Source AMR Workspace
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Auto-build if hardware_bringup.launch.py is not yet installed in share directory
if [ ! -f "$SCRIPT_DIR/install/agv_description/share/agv_description/launch/hardware_bringup.launch.py" ]; then
    echo "[INFO] New launch files detected. Building agv_description package..."
    (cd "$SCRIPT_DIR" && colcon build --packages-select agv_description --symlink-install)
fi

if [ -f "$SCRIPT_DIR/install/setup.bash" ]; then
    source "$SCRIPT_DIR/install/setup.bash"
    echo "[OK] Sourced AMR workspace"
else
    echo "[ERROR] $SCRIPT_DIR/install/setup.bash not found! Run colcon build first."
    exit 1
fi

# Set ROS Domain ID (stack uses domain 7)
export ROS_DOMAIN_ID=7
echo "[OK] ROS_DOMAIN_ID=7"

# 4. Configure CycloneDDS for reliable Wi-Fi streaming to Laptop
if dpkg -s ros-jazzy-rmw-cyclonedds-cpp >/dev/null 2>&1; then
    CFG_FILE="/tmp/cyclonedds_pi.xml"
    if [ -n "$LAPTOP_IP" ]; then
        PI_MY_IP=$(ip route get "$LAPTOP_IP" 2>/dev/null | awk '{for(i=1;i<=NF;i++) if($i=="src") print $(i+1)}' || hostname -I | awk '{print $1}')
        cat <<EOF > "$CFG_FILE"
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain Id="7">
    <General>
      <Interfaces>
        <NetworkInterface address="$PI_MY_IP"/>
      </Interfaces>
    </General>
    <Discovery>
      <Peers>
        <Peer Address="127.0.0.1"/>
        <Peer Address="$LAPTOP_IP"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF
    else
        cat <<EOF > "$CFG_FILE"
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain Id="7">
    <Discovery>
      <Peers>
        <Peer Address="127.0.0.1"/>
        <Peer Address="255.255.255.255"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF
    fi
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI="file://$CFG_FILE"
    echo "[OK] Configured CycloneDDS (rmw_cyclonedds_cpp) bound to $PI_MY_IP -> Peer $LAPTOP_IP"
else
    echo "[WARN] ros-jazzy-rmw-cyclonedds-cpp is not installed on Pi."
    echo "       If laptop RViz cannot see topics over Wi-Fi, install it on Pi via:"
    echo "         sudo apt update && sudo apt install -y ros-jazzy-rmw-cyclonedds-cpp"
fi

# 5. Launch Master Hardware Stack
echo "Launching hardware_bringup.launch.py..."
ros2 launch agv_description hardware_bringup.launch.py "$@"
