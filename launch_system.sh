#!/usr/bin/env bash
# =============================================================================
#  launch_system.sh  —  AMR SYSTEM LAUNCHER
#  Starts the complete ROS 2 navigation stack in a tmux session:
#    Pane 0 : Gazebo + AMCL localization  (navigation_launch.py)
#    Pane 1 : Route Runner  (Dijkstra + MPPI + state machine)
#    Pane 2 : Behavior Tree Manager  (mission orchestration + recovery)
#
#  Usage:
#    ./launch_system.sh           # launch everything
#    ./launch_system.sh attach    # re-attach to running session
#    ./launch_system.sh stop      # kill the session
# =============================================================================

SESSION="amr_system"
WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ── Colour helpers ────────────────────────────────────────────────────────────
RED='\033[0;31m'; GRN='\033[0;32m'; YLW='\033[1;33m'
CYN='\033[0;36m'; BLD='\033[1m'; RST='\033[0m'

# ── Handle subcommands ────────────────────────────────────────────────────────
if [[ "$1" == "attach" ]]; then
    tmux attach-session -t "$SESSION" 2>/dev/null || { echo -e "${RED}No session '$SESSION' found. Run ./launch_system.sh first.${RST}"; exit 1; }
    exit 0
fi

if [[ "$1" == "stop" ]]; then
    tmux kill-session -t "$SESSION" 2>/dev/null && echo -e "${GRN}✓ AMR system session stopped.${RST}" || echo "No session to stop."
    exit 0
fi

# ── Kill existing session if running ─────────────────────────────────────────
tmux kill-session -t "$SESSION" 2>/dev/null

echo ""
echo -e "${BLD}${CYN}══════════════════════════════════════════${RST}"
echo -e "${BLD}${CYN}   AMR System Launcher${RST}"
echo -e "${BLD}${CYN}   Workspace: ${WS_ROOT}${RST}"
echo -e "${BLD}${CYN}══════════════════════════════════════════${RST}"
echo ""

# ── Verify workspace is built ─────────────────────────────────────────────────
if [ ! -f "$WS_ROOT/install/setup.bash" ]; then
    echo -e "${RED}[ERROR] Workspace not built. Run 'colcon build' first from $WS_ROOT${RST}"
    exit 1
fi

# ── Source command used in every pane ─────────────────────────────────────────
SOURCE_CMD="source /opt/ros/jazzy/setup.bash && source $WS_ROOT/install/setup.bash"

# =============================================================================
# Create tmux session with 3 panes
# =============================================================================
# Layout:
#   ┌──────────────────────────────────────────┐
#   │  Pane 0: GAZEBO + AMCL  (top, 60%)       │
#   ├──────────────────┬───────────────────────┤
#   │  Pane 1:         │  Pane 2:              │
#   │  ROUTE RUNNER    │  BT MANAGER           │
#   │  (bottom left)   │  (bottom right)       │
#   └──────────────────┴───────────────────────┘

# Create session with first window, first pane
tmux new-session  -d -s "$SESSION" -x 220 -y 50

# ── Pane 0: Gazebo + AMCL ─────────────────────────────────────────────────────
tmux rename-window -t "$SESSION:0" "AMR System"
tmux send-keys -t "$SESSION:0.0" "
printf '\033[1;36m╔══════════════════════════════════════════════╗\033[0m\n'
printf '\033[1;36m║  PANE 0 — GAZEBO + AMCL LOCALIZATION         ║\033[0m\n'
printf '\033[1;36m╚══════════════════════════════════════════════╝\033[0m\n'
$SOURCE_CMD
cd $WS_ROOT
echo -e '\033[1;33m[System] Launching Gazebo + AMCL... (takes ~10-15 s)\033[0m'
ros2 launch agv_description navigation_launch.py
" Enter

# ── Split horizontally for bottom row ─────────────────────────────────────────
tmux split-window -t "$SESSION:0.0" -v -p 35

# ── Pane 1: Route Runner ──────────────────────────────────────────────────────
tmux send-keys -t "$SESSION:0.1" "
printf '\033[1;32m╔══════════════════════════════════════════════╗\033[0m\n'
printf '\033[1;32m║  PANE 1 — ROUTE RUNNER (Dijkstra + MPPI)     ║\033[0m\n'
printf '\033[1;32m╚══════════════════════════════════════════════╝\033[0m\n'
$SOURCE_CMD
cd $WS_ROOT
echo -e '\033[1;33m[System] Waiting 12 s for Gazebo + AMCL to initialise...\033[0m'
sleep 12
echo -e '\033[1;32m[System] Starting route_runner...\033[0m'
ros2 run agv_navigation route_runner
" Enter

# ── Split pane 1 vertically for BT Manager ────────────────────────────────────
tmux split-window -t "$SESSION:0.1" -h -p 50

# ── Pane 2: BT Manager ────────────────────────────────────────────────────────
tmux send-keys -t "$SESSION:0.2" "
printf '\033[1;35m╔══════════════════════════════════════════════╗\033[0m\n'
printf '\033[1;35m║  PANE 2 — BEHAVIOR TREE MANAGER              ║\033[0m\n'
printf '\033[1;35m╚══════════════════════════════════════════════╝\033[0m\n'
$SOURCE_CMD
cd $WS_ROOT
echo -e '\033[1;33m[System] Waiting 15 s for route_runner to initialise...\033[0m'
sleep 15
echo -e '\033[1;35m[System] Starting bt_manager...\033[0m'
ros2 run agv_navigation bt_manager
" Enter

# Focus the top pane (Gazebo) so the user sees the main launch log
tmux select-pane -t "$SESSION:0.0"

echo -e "${GRN}✓ AMR System session '${SESSION}' created in tmux.${RST}"
echo ""
echo -e "  ${BLD}Pane layout:${RST}"
echo -e "  ${CYN}[TOP]          Pane 0 — Gazebo + AMCL${RST}"
echo -e "  ${GRN}[BOTTOM LEFT]  Pane 1 — Route Runner${RST}"
echo -e "  ${YLW}[BOTTOM RIGHT] Pane 2 — BT Manager${RST}"
echo ""
echo -e "  ${BLD}Timing:${RST}"
echo -e "  • Route Runner starts automatically after 12 s"
echo -e "  • BT Manager starts automatically after 15 s"
echo ""
echo -e "  ${BLD}Commands:${RST}"
echo -e "  ${YLW}./launch_system.sh attach${RST}  — re-attach to session"
echo -e "  ${YLW}./launch_system.sh stop${RST}    — kill everything"
echo -e "  ${YLW}Ctrl+B then D${RST}              — detach (keep running in background)"
echo -e "  ${YLW}Ctrl+B then arrow keys${RST}     — switch between panes"
echo ""

# Attach to the session
tmux attach-session -t "$SESSION"
