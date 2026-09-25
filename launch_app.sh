#!/usr/bin/env bash
# =============================================================================
#  launch_app.sh  —  AMR APP LAUNCHER
#  Starts both the FastAPI bridge server and the React web dashboard
#  in a single tmux session:
#    Pane 0 (left)  : FastAPI Bridge Server  (:8000)
#    Pane 1 (right) : React Web Dashboard    (:5173)
#
#  Usage:
#    ./launch_app.sh             # launch bridge + dashboard
#    ./launch_app.sh attach      # re-attach to running session
#    ./launch_app.sh stop        # kill the session
#
#  Prerequisites:
#    • ROS system must be running first  (./launch_system.sh)
# =============================================================================

SESSION="amr_app"
WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BRIDGE_DIR="$WS_ROOT/app/bridge"
DASH_DIR="$WS_ROOT/app/web_dashboard"

# ── Colour helpers ────────────────────────────────────────────────────────────
RED='\033[0;31m'; GRN='\033[0;32m'; YLW='\033[1;33m'
CYN='\033[0;36m'; BLD='\033[1m'; RST='\033[0m'

# ── Handle subcommands ────────────────────────────────────────────────────────
if [[ "$1" == "attach" ]]; then
    tmux attach-session -t "$SESSION" 2>/dev/null || { echo -e "${RED}No session '$SESSION' found.${RST}"; exit 1; }
    exit 0
fi

if [[ "$1" == "stop" ]]; then
    tmux kill-session -t "$SESSION" 2>/dev/null && echo -e "${GRN}✓ App session stopped.${RST}" || echo "No session to stop."
    exit 0
fi

# ── Kill existing session ─────────────────────────────────────────────────────
tmux kill-session -t "$SESSION" 2>/dev/null

echo ""
echo -e "${BLD}${CYN}══════════════════════════════════════════${RST}"
echo -e "${BLD}${CYN}   AMR App Launcher${RST}"
echo -e "${BLD}${CYN}   Bridge    →  http://localhost:8000${RST}"
echo -e "${BLD}${CYN}   Dashboard →  http://localhost:5173${RST}"
echo -e "${BLD}${CYN}══════════════════════════════════════════${RST}"
echo ""

# ── Sanity checks ─────────────────────────────────────────────────────────────
if [ ! -f "$BRIDGE_DIR/bridge_server.py" ]; then
    echo -e "${RED}[ERROR] bridge_server.py not found at $BRIDGE_DIR${RST}"; exit 1
fi
if [ ! -f "$DASH_DIR/package.json" ]; then
    echo -e "${RED}[ERROR] package.json not found at $DASH_DIR${RST}"; exit 1
fi
if [ ! -f "$WS_ROOT/install/setup.bash" ]; then
    echo -e "${RED}[ERROR] Workspace not built. Run 'colcon build' first.${RST}"; exit 1
fi

# ── Verify run_dashboard.sh exists (has correct Node.js path logic) ───────────
if [ ! -f "$DASH_DIR/run_dashboard.sh" ]; then
    echo -e "${RED}[ERROR] run_dashboard.sh not found at $DASH_DIR${RST}"; exit 1
fi
chmod +x "$DASH_DIR/run_dashboard.sh"

# ── Verify Node.js is available (prioritise home-installed v20) ───────────────
if [ -d "$HOME/node-v20.18.0-linux-x64/bin" ]; then
    NODE_OK="$HOME/node-v20.18.0-linux-x64/bin/node"
elif command -v node &>/dev/null && [ -x "$(dirname "$(which node)")/npm" ]; then
    NODE_OK="$(which node)"
else
    echo -e "${RED}[ERROR] Node.js with npm not found.${RST}"
    echo -e "Install: wget https://nodejs.org/dist/v20.18.0/node-v20.18.0-linux-x64.tar.xz"
    echo -e "         tar -xf node-v20.18.0-linux-x64.tar.xz -C \$HOME"
    exit 1
fi
echo -e "${GRN}✓ Node.js: $NODE_OK ($($NODE_OK --version 2>/dev/null))${RST}"

# ── Source ROS command ────────────────────────────────────────────────────────
SOURCE_CMD="source /opt/ros/jazzy/setup.bash && source $WS_ROOT/install/setup.bash"

# =============================================================================
# Create tmux session — side by side panes
# =============================================================================
#   ┌─────────────────────┬──────────────────────┐
#   │  Pane 0             │  Pane 1              │
#   │  BRIDGE SERVER      │  WEB DASHBOARD       │
#   │  (FastAPI :8000)    │  (Vite React :5173)  │
#   └─────────────────────┴──────────────────────┘

tmux new-session -d -s "$SESSION" -x 220 -y 50
tmux rename-window -t "$SESSION:0" "AMR App"

# ── Pane 0: Bridge Server ─────────────────────────────────────────────────────
tmux send-keys -t "$SESSION:0.0" "
printf '\033[1;36m╔══════════════════════════════════════════════╗\n'
printf '║  PANE 0 — BRIDGE SERVER  (port 8000)         ║\n'
printf '╚══════════════════════════════════════════════╝\033[0m\n'
$SOURCE_CMD
cd $BRIDGE_DIR

# Auto-install Python deps if missing
if ! python3 -c 'import fastapi, uvicorn, pydantic, PIL, numpy, websockets' 2>/dev/null; then
    echo -e '\033[1;33m[Bridge] Installing Python dependencies...\033[0m'
    pip install --break-system-packages -q -r requirements.txt 2>/dev/null || pip install -q -r requirements.txt
fi

# Check if port 8000 is already in use
if ss -tlnp 2>/dev/null | grep -q ':8000'; then
    echo -e '\033[1;33m[Bridge] Port 8000 already in use — bridge already running.\033[0m'
    echo -e '\033[0;32m[Bridge] Skipping bridge start. Dashboard will use existing bridge.\033[0m'
    echo ''
    echo -e '\033[0;36m[Bridge] API:       http://localhost:8000/api/status\033[0m'
    echo -e '\033[0;36m[Bridge] WebSocket: ws://localhost:8000/ws/telemetry\033[0m'
    echo ''
    # Keep this pane alive showing bridge logs via curl polling
    echo -e '\033[0;37m[Bridge] Monitoring existing bridge...\033[0m'
    while true; do
        STATUS=\$(curl -sf http://localhost:8000/api/status 2>/dev/null | python3 -c \"import sys,json; d=json.load(sys.stdin); print(f'State={d[\\\"nav_state\\\"]} Pose=({d[\\\"pose\\\"][\\\"x\\\"]:.2f},{d[\\\"pose\\\"][\\\"y\\\"]:.2f})')\" 2>/dev/null || echo 'bridge not responding')
        echo -e \"\$(date '+%H:%M:%S') \$STATUS\"
        sleep 3
    done
else
    echo -e '\033[1;36m[Bridge] Starting FastAPI bridge on http://0.0.0.0:8000\033[0m'
    echo -e '\033[1;36m[Bridge] WebSocket: ws://0.0.0.0:8000/ws/telemetry\033[0m'
    echo ''
    export AMR_WS=$WS_ROOT
    python3 bridge_server.py
fi
" Enter

# ── Split vertically for dashboard ───────────────────────────────────────────
tmux split-window -t "$SESSION:0.0" -h -p 52

# ── Pane 1: Web Dashboard ─────────────────────────────────────────────────────
tmux send-keys -t "$SESSION:0.1" "
printf '\033[1;32m╔══════════════════════════════════════════════╗\n'
printf '║  PANE 1 — WEB DASHBOARD  (port 5173)         ║\n'
printf '╚══════════════════════════════════════════════╝\033[0m\n'
echo ''

# Wait until bridge is accepting connections (max 30 s)
echo -e '\033[1;33m[Dashboard] Waiting for bridge on port 8000...\033[0m'
TRIES=0
until curl -sf http://localhost:8000/api/map/metadata >/dev/null 2>&1; do
    TRIES=\$((TRIES+1))
    if [ \$TRIES -ge 15 ]; then
        echo -e '\033[0;31m[Dashboard] Bridge did not respond after 30 s — check bridge pane.\033[0m'
        break
    fi
    echo -e '\033[0;33m[Dashboard] Retrying... (\${TRIES}/15)\033[0m'
    sleep 2
done

curl -sf http://localhost:8000/api/map/metadata >/dev/null 2>&1 && \\
    echo -e '\033[1;32m[Dashboard] ✓ Bridge is ready!\033[0m'

echo ''
echo -e '\033[1;32m[Dashboard] Starting React dashboard...\033[0m'
echo -e '\033[1;32m[Dashboard] Open in browser:  http://localhost:5173\033[0m'
echo ''
cd $DASH_DIR
bash run_dashboard.sh
" Enter

# Focus the dashboard pane so user sees the URL
tmux select-pane -t "$SESSION:0.1"

echo -e "${GRN}✓ App session '${SESSION}' created.${RST}"
echo ""
echo -e "  ${BLD}Pane layout:${RST}"
echo -e "  ${CYN}[LEFT]   Pane 0 — Bridge Server   → http://localhost:8000${RST}"
echo -e "  ${GRN}[RIGHT]  Pane 1 — Web Dashboard    → http://localhost:5173${RST}"
echo ""
echo -e "  ${BLD}The right pane (dashboard) is focused by default.${RST}"
echo -e "  ${BLD}Use ${YLW}Ctrl+B then ← / →${RST}${BLD} to switch panes.${RST}"
echo ""
echo -e "  ${BLD}Commands:${RST}"
echo -e "  ${YLW}./launch_app.sh attach${RST}   — re-attach to session"
echo -e "  ${YLW}./launch_app.sh stop${RST}     — kill bridge + dashboard"
echo -e "  ${YLW}Ctrl+B then D${RST}            — detach (keep running)"
echo ""

# Attach — focused on dashboard pane (right)
tmux attach-session -t "$SESSION"
