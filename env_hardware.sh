#!/usr/bin/env bash
# ==============================================================================
# env_hardware.sh
# ==============================================================================
# Source this file to configure your current terminal for the AMR robot:
#   source env_hardware.sh [PI_IP]
# ==============================================================================

unalias ros2 2>/dev/null || true

if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
fi

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
fi

PI_IP="${1:-10.244.248.208}"

# Dynamically ensure cyclonedds.xml binds to eth0 and targets the active Pi IP
cat <<EOF > "$WS_DIR/cyclonedds.xml"
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain Id="7">
    <General>
      <Interfaces>
        <NetworkInterface name="eth0"/>
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
export CYCLONEDDS_URI="file://$WS_DIR/cyclonedds.xml"

# Smart wrapper to bypass WSL2 loopback daemon socket timeouts
ros2() {
    if [ "$1" = "node" ] && [ "$2" = "list" ]; then
        shift 2
        command ros2 node list --no-daemon "$@"
    elif [ "$1" = "topic" ] && [ "$2" = "list" ]; then
        shift 2
        command ros2 topic list --no-daemon "$@"
    elif [ "$1" = "topic" ] && [ "$2" = "hz" ]; then
        shift 2
        command ros2 topic hz --no-daemon "$@"
    elif [ "$1" = "topic" ] && [ "$2" = "echo" ]; then
        shift 2
        command ros2 topic echo --no-daemon "$@"
    else
        command ros2 "$@"
    fi
}

echo "=================================================================="
echo "  [OK] Terminal configured for AMR Physical Robot Stack"
echo "  ROS_DOMAIN_ID       : 7"
echo "  RMW_IMPLEMENTATION  : rmw_cyclonedds_cpp"
echo "  Interface           : eth0"
echo "  CYCLONEDDS_URI      : $CYCLONEDDS_URI (Unicast -> $PI_IP)"
echo "=================================================================="
