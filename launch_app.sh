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
#    • websockets + fastapi installed    (auto-installed if missing)
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
echo -e "${BLD}${CYN}   Bridge  →  http://localhost:8000${RST}"
echo -e "${BLD}${CYN}   Dashboard → http://localhost:5173${RST}"
echo -e "${BLD}${CYN}══════════════════════════════════════════${RST}"
echo ""

# ── Sanity checks ─────────────────────────────────────────────────────────────
if [ ! -f "$BRIDGE_DIR/bridge_server.py" ]; then
    echo -e "${RED}[ERROR] bridge_server.py not found at $BRIDGE_DIR${RST}"; exit 1
fi
if [ ! -f "$DASH_DIR/package.json" ]; then
    echo -e "${RED}[ERROR] package.json not found at $DASH_DIR${RST}"; exit 1
fi

# ── Find native Node.js (prioritise home-installed version) ──────────────────
NODE_BIN=""
if [ -d "$HOME/node-v20.18.0-linux-x64/bin" ]; then
    NODE_BIN="$HOME/node-v20.18.0-linux-x64/bin"
elif command -v node &>/dev/null; then
    NODE_BIN="$(dirname "$(which node)")"
fi

if [ -z "$NODE_BIN" ]; then
    echo -e "${RED}[ERROR] Node.js not found. Install from https://nodejs.org/ and re-run.${RST}"
    exit 1
fi

echo -e "${GRN}✓ Node.js found: ${NODE_BIN}/node ($("${NODE_BIN}/node" --version 2>/dev/null))${RST}"

# ── Source ROS command (bridge needs rclpy) ───────────────────────────────────
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
printf '\033[1;36m╔══════════════════════════════════════════════╗\033[0m\n'
printf '\033[1;36m║  PANE 0 — BRIDGE SERVER  (port 8000)         ║\033[0m\n'
printf '\033[1;36m╚══════════════════════════════════════════════╝\033[0m\n'
$SOURCE_CMD
cd $BRIDGE_DIR

# Auto-install Python deps if missing
if ! python3 -c 'import fastapi, uvicorn, pydantic, PIL, numpy, websockets' 2>/dev/null; then
    echo -e '\033[1;33m[Bridge] Installing Python dependencies...\033[0m'
    pip install --break-system-packages -q -r requirements.txt 2>/dev/null || pip install -q -r requirements.txt
fi

echo -e '\033[1;36m[Bridge] Starting FastAPI bridge on http://0.0.0.0:8000\033[0m'
echo -e '\033[1;36m[Bridge] WebSocket: ws://0.0.0.0:8000/ws/telemetry\033[0m'
echo ''
export AMR_WS=$WS_ROOT
python3 bridge_server.py
" Enter

# ── Split vertically for dashboard ───────────────────────────────────────────
tmux split-window -t "$SESSION:0.0" -h -p 50

# ── Pane 1: Web Dashboard ─────────────────────────────────────────────────────
tmux send-keys -t "$SESSION:0.1" "
printf '\033[1;32m╔══════════════════════════════════════════════╗\033[0m\n'
printf '\033[1;32m║  PANE 1 — WEB DASHBOARD  (port 5173)         ║\033[0m\n'
printf '\033[1;32m╚══════════════════════════════════════════════╝\033[0m\n'
export PATH=$NODE_BIN:\$PATH
cd $DASH_DIR

echo -e '\033[1;33m[Dashboard] Waiting 4 s for bridge to start...\033[0m'
sleep 4

# Wait until bridge is accepting connections
for i in \$(seq 1 20); do
    curl -sf http://localhost:8000/api/map/metadata >/dev/null 2>&1 && break
    echo -e '\033[0;33m[Dashboard] Bridge not ready yet, retrying (\$i/20)...\033[0m'
    sleep 2
done

echo -e '\033[1;32m[Dashboard] Bridge is up. Starting React dashboard...\033[0m'
echo -e '\033[1;32m[Dashboard] Open:  http://localhost:5173\033[0m'
echo ''
npm run dev -- --host 0.0.0.0
" Enter

# Focus bridge pane
tmux select-pane -t "$SESSION:0.0"

echo -e "${GRN}✓ App session '${SESSION}' created in tmux.${RST}"
echo ""
echo -e "  ${BLD}Pane layout:${RST}"
echo -e "  ${CYN}[LEFT]   Pane 0 — Bridge Server   → http://localhost:8000${RST}"
echo -e "  ${GRN}[RIGHT]  Pane 1 — Web Dashboard    → http://localhost:5173${RST}"
echo ""
echo -e "  ${BLD}Dashboard opens automatically after bridge is ready (~8 s)${RST}"
echo ""
echo -e "  ${BLD}Commands:${RST}"
echo -e "  ${YLW}./launch_app.sh attach${RST}   — re-attach to session"
echo -e "  ${YLW}./launch_app.sh stop${RST}     — kill bridge + dashboard"
echo -e "  ${YLW}Ctrl+B then D${RST}            — detach (keep running)"
echo -e "  ${YLW}Ctrl+B then arrow keys${RST}   — switch panes"
echo ""

# Attach
tmux attach-session -t "$SESSION"
