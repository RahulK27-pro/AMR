#!/bin/bash
# run_bridge.sh — Start the AMR control bridge server
# =====================================================
# Supports both Simulation (default) and Real Hardware modes:
#   ./run_bridge.sh                    # Simulation mode (ROS_LOCALHOST_ONLY=1, use_sim_time=true)
#   ./run_bridge.sh sim                # Simulation mode
#   ./run_bridge.sh hardware [PI_IP]   # Real Hardware mode (CycloneDDS Domain 7, use_sim_time=false)
#   ./run_bridge.sh 10.244.248.208     # Shorthand for hardware mode targeting PI_IP

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

AMR_MODE="${AMR_MODE:-sim}"
AMR_PI_IP="${AMR_PI_IP:-10.244.248.208}"

if [[ "${1:-}" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    AMR_MODE="hardware"
    AMR_PI_IP="$1"
elif [ "${1:-}" = "hardware" ] || [ "${1:-}" = "hw" ]; then
    AMR_MODE="hardware"
    if [[ "${2:-}" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
        AMR_PI_IP="$2"
    fi
elif [ "${1:-}" = "sim" ] || [ "${1:-}" = "simulation" ]; then
    AMR_MODE="sim"
    if [[ "${2:-}" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
        AMR_PI_IP="$2"
    fi
fi

echo "=============================================="
echo "  AMR Control Bridge Server v2.1"
echo "  Workspace : $WS_ROOT"
echo "  Mode      : $AMR_MODE"
if [ "$AMR_MODE" = "hardware" ]; then
echo "  Target Pi : $AMR_PI_IP"
fi
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

# Export workspace root & mode config
export AMR_WS="$WS_ROOT"
export AMR_MODE="$AMR_MODE"
export AMR_PI_IP="$AMR_PI_IP"

if [ "$AMR_MODE" = "hardware" ]; then
    unset ROS_LOCALHOST_ONLY
    if [ -f "$WS_ROOT/env_hardware.sh" ]; then
        source "$WS_ROOT/env_hardware.sh" "$AMR_PI_IP"
    else
        export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-7}"
        export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
        if [ -f "$WS_ROOT/cyclonedds.xml" ]; then
            export CYCLONEDDS_URI="file://$WS_ROOT/cyclonedds.xml"
        fi
    fi
else
    # Ensure loopback isolation in simulation so external Wi-Fi/Ethernet changes never drop ROS 2 DDS
    export ROS_LOCALHOST_ONLY=1
    unset CYCLONEDDS_URI
    export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-7}"
    export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
fi

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
echo "[bridge] Starting FastAPI bridge on http://0.0.0.0:8000 (mode=$AMR_MODE)"
echo "[bridge] WebSocket telemetry: ws://0.0.0.0:8000/ws/telemetry"
echo "[bridge] Map endpoint:        http://0.0.0.0:8000/api/map"
echo ""
echo "Open web dashboard at:  http://localhost:5173"
echo ""

export RCUTILS_LOGGING_SEVERITY_THRESHOLD=ERROR
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3

exec python3 "$SCRIPT_DIR/bridge_server.py" --mode "$AMR_MODE" --pi-ip "$AMR_PI_IP"
