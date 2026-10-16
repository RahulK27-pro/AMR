#!/bin/bash
# run_bridge.sh — Start the AMR control bridge server
# =====================================================
# Run this AFTER sourcing your ROS 2 workspace.
# The bridge connects ROS 2 topics to the Flutter app and web dashboard.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

echo "=============================================="
echo "  AMR Control Bridge Server v2.0"
echo "  Workspace: $WS_ROOT"
echo "=============================================="

# Source ROS 2 if not already sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "[bridge] Sourcing ROS 2 Jazzy..."
    source /opt/ros/jazzy/setup.bash
fi

# Source workspace overlay
if [ -f "$WS_ROOT/install/setup.bash" ]; then
    echo "[bridge] Sourcing workspace overlay: $WS_ROOT/install/setup.bash"
    source "$WS_ROOT/install/setup.bash"
else
    echo "[WARN] Workspace overlay not found. Run 'colcon build' first."
fi

# Export workspace root so bridge_server can find map PNG
export AMR_WS="$WS_ROOT"

# Install Python deps if missing (includes websockets for uvicorn WebSocket support)
echo "[bridge] Checking Python dependencies..."
if ! python3 -c "import fastapi, uvicorn, pydantic, PIL, numpy, websockets" 2>/dev/null; then
    pip install --break-system-packages -q -r "$SCRIPT_DIR/requirements.txt" 2>/dev/null || \
    pip install -q -r "$SCRIPT_DIR/requirements.txt"
fi

# Terminate any existing bridge server instances on port 8000
OLD_PID=$(lsof -Pi :8000 -sTCP:LISTEN -t 2>/dev/null || true)
if [ -n "$OLD_PID" ]; then
    echo "[bridge] Port 8000 is already in use by PID $OLD_PID. Stopping previous instance..."
    kill -9 $OLD_PID 2>/dev/null || true
    sleep 1
fi

echo ""
echo "[bridge] Starting FastAPI bridge on http://0.0.0.0:8000"
echo "[bridge] WebSocket telemetry: ws://0.0.0.0:8000/ws/telemetry"
echo "[bridge] Map endpoint:        http://0.0.0.0:8000/api/map"
echo ""
echo "Open web dashboard at:  http://localhost:5173"
echo ""

# Parse optional Laptop IP argument
LAPTOP_IP="10.87.204.140"
for arg in "$@"; do
    if [[ "$arg" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
        LAPTOP_IP="$arg"
    fi
done

if [[ "$*" == *"--remote-sim"* ]] || [ "$AMR_REMOTE_SIM" = "1" ]; then
    echo "=========================================================="
    echo "  [MODE] Distributed Simulation Architecture (Option C)"
    echo "  Gazebo Simulator runs on Laptop ($LAPTOP_IP)"
    echo "  Autonomy runs on Pi (Domain 7, CycloneDDS Unicast)"
    echo "=========================================================="
    export AMR_REMOTE_SIM=1

    # 1. Generate CycloneDDS peer configuration
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
    echo "[bridge] CycloneDDS Unicast Peer configured -> $LAPTOP_IP"
else
    export LIBGL_ALWAYS_SOFTWARE=1
    export MESA_GL_VERSION_OVERRIDE=3.3
fi

python3 "$SCRIPT_DIR/bridge_server.py"

