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

# Install Python deps if missing
echo "[bridge] Checking Python dependencies..."
if ! python3 -c "import fastapi, uvicorn, pydantic, PIL, numpy" 2>/dev/null; then
    pip install --break-system-packages -q -r "$SCRIPT_DIR/requirements.txt" 2>/dev/null || \
    pip install -q -r "$SCRIPT_DIR/requirements.txt"
fi

echo ""
echo "[bridge] Starting FastAPI bridge on http://0.0.0.0:8000"
echo "[bridge] WebSocket telemetry: ws://0.0.0.0:8000/ws/telemetry"
echo "[bridge] Map endpoint:        http://0.0.0.0:8000/api/map"
echo ""
echo "Connect Flutter app to: http://<YOUR_IP>:8000"
echo "Open web dashboard at:  http://localhost:5173"
echo ""

python3 "$SCRIPT_DIR/bridge_server.py"
