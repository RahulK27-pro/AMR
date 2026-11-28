#!/usr/bin/env bash
# ==============================================================================
# env_hardware.sh
# ==============================================================================
# Source this file to configure your current terminal for the AMR robot:
#   source scripts/hardware/env_hardware.sh [PI_IP]
# ==============================================================================

unalias ros2 2>/dev/null || true

if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
fi

DEFAULT_PI_IP="10.244.248.208"
# Only treat $1 as PI_IP if it matches an IPv4 format
if [[ "$1" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    PI_IP="$1"
else
    PI_IP="$DEFAULT_PI_IP"
fi

# Auto-detect local interface IP on route to PI_IP (fallback to autodetermine)
LOCAL_IP=$(ip route get "$PI_IP" 2>/dev/null | awk '{for(i=1;i<=NF;i++) if($i=="src") print $(i+1)}' || true)
if [ -z "$LOCAL_IP" ]; then
    LOCAL_IP=$(hostname -I 2>/dev/null | awk '{print $1}' || true)
fi
if [ -n "$LOCAL_IP" ]; then
    NET_IFACE_XML="<NetworkInterface address=\"$LOCAL_IP\"/>"
else
    NET_IFACE_XML="<NetworkInterface autodetermine=\"true\"/>"
fi

# Dynamically ensure config/cyclonedds.xml binds to the active routed NIC and targets the active Pi IP
mkdir -p "$WS_DIR/config"
cat <<EOF > "$WS_DIR/config/cyclonedds.xml"
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
export CYCLONEDDS_URI="file://$WS_DIR/config/cyclonedds.xml"

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
echo "  Interface           : ${LOCAL_IP:-auto}"
echo "  CYCLONEDDS_URI      : $CYCLONEDDS_URI (Unicast -> $PI_IP)"
echo "=================================================================="
