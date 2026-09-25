#!/usr/bin/env bash
# =============================================================================
#  launch_obstacle.sh  —  DYNAMIC OBSTACLE LAUNCHER
#  Spawns an autonomous dynamic obstacle in Gazebo for testing MPPI evasion,
#  corridor yielding, and Dijkstra rerouting.
#
#  Usage:
#    ./launch_obstacle.sh                          # default: aisle_crossing
#    ./launch_obstacle.sh aisle_crossing           # worker crossing aisles
#    ./launch_obstacle.sh doorway_blocker          # blocks a doorway (triggers reroute)
#    ./launch_obstacle.sh corridor_walker          # head-on traffic in narrow aisle
#    ./launch_obstacle.sh manual                   # spawn + keyboard teleop control
#    ./launch_obstacle.sh stop                     # kill the obstacle session
#
#  Pattern descriptions:
#    aisle_crossing   — moves back-and-forth across aisles at 0.35 m/s
#    doorway_blocker  — parks at a doorway; triggers reroute after 4.5 s
#    corridor_walker  — walks toward the robot in narrow corridors
#    manual           — spawns obstacle, then you drive it with WASD keyboard
#
#  Spawn position can be customised (default: x=1.8, y=0.0):
#    ./launch_obstacle.sh aisle_crossing x=2.5 y=1.0 speed=0.4
# =============================================================================

SESSION="amr_obstacle"
WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ── Colour helpers ────────────────────────────────────────────────────────────
RED='\033[0;31m'; GRN='\033[0;32m'; YLW='\033[1;33m'
CYN='\033[0;36m'; BLD='\033[1m'; RST='\033[0m'

# ── Parse arguments ───────────────────────────────────────────────────────────
PATTERN="${1:-aisle_crossing}"
X="1.8"; Y="0.0"; SPEED="0.35"

# Parse optional key=value overrides (e.g. x=2.5 y=1.0 speed=0.4)
for arg in "$@"; do
    case "$arg" in
        x=*)     X="${arg#x=}" ;;
        y=*)     Y="${arg#y=}" ;;
        speed=*) SPEED="${arg#speed=}" ;;
    esac
done

# ── Handle stop subcommand ────────────────────────────────────────────────────
if [[ "$PATTERN" == "stop" ]]; then
    tmux kill-session -t "$SESSION" 2>/dev/null && echo -e "${GRN}✓ Obstacle session stopped.${RST}" || echo "No obstacle session running."
    exit 0
fi

# ── Kill existing obstacle session ───────────────────────────────────────────
tmux kill-session -t "$SESSION" 2>/dev/null

# ── Validate pattern ─────────────────────────────────────────────────────────
VALID_PATTERNS="aisle_crossing doorway_blocker corridor_walker manual"
if [[ ! " $VALID_PATTERNS " =~ " $PATTERN " ]]; then
    echo -e "${RED}[ERROR] Unknown pattern '$PATTERN'. Choose from: $VALID_PATTERNS${RST}"
    exit 1
fi

# ── Verify workspace ─────────────────────────────────────────────────────────
if [ ! -f "$WS_ROOT/install/setup.bash" ]; then
    echo -e "${RED}[ERROR] Workspace not built. Run 'colcon build' first.${RST}"; exit 1
fi

SOURCE_CMD="source /opt/ros/jazzy/setup.bash && source $WS_ROOT/install/setup.bash"

echo ""
echo -e "${BLD}${YLW}══════════════════════════════════════════${RST}"
echo -e "${BLD}${YLW}   Dynamic Obstacle Launcher${RST}"
echo -e "${BLD}${YLW}   Pattern : ${PATTERN}${RST}"
echo -e "${BLD}${YLW}   Position: x=${X}  y=${Y}  speed=${SPEED}${RST}"
echo -e "${BLD}${YLW}══════════════════════════════════════════${RST}"
echo ""

tmux new-session -d -s "$SESSION" -x 180 -y 40
tmux rename-window -t "$SESSION:0" "AMR Obstacle"

if [[ "$PATTERN" == "manual" ]]; then
    # ── Manual mode: obstacle spawn + keyboard teleop side-by-side ────────────
    tmux send-keys -t "$SESSION:0.0" "
printf '\033[1;33m╔══════════════════════════════════════════════╗\033[0m\n'
printf '\033[1;33m║  OBSTACLE SPAWNER  (manual, no controller)   ║\033[0m\n'
printf '\033[1;33m╚══════════════════════════════════════════════╝\033[0m\n'
$SOURCE_CMD
cd $WS_ROOT
ros2 launch agv_description dynamic_obstacle.launch.py run_controller:=false x:=$X y:=$Y
" Enter

    tmux split-window -t "$SESSION:0.0" -h -p 50
    tmux send-keys -t "$SESSION:0.1" "
printf '\033[1;33m╔══════════════════════════════════════════════╗\033[0m\n'
printf '\033[1;33m║  OBSTACLE TELEOP  (keyboard WASD control)    ║\033[0m\n'
printf '\033[1;33m╚══════════════════════════════════════════════╝\033[0m\n'
echo -e '\033[1;33mControls: W/S=forward/back  A/D=turn  Q/E=diagonal  Space=stop  +/-=speed\033[0m'
echo ''
$SOURCE_CMD
cd $WS_ROOT
sleep 3
ros2 run agv_navigation obstacle_teleop
" Enter
    tmux select-pane -t "$SESSION:0.1"

else
    # ── Autonomous mode: single pane ──────────────────────────────────────────
    # Map pattern to specific launch args
    case "$PATTERN" in
        doorway_blocker)
            EXTRA_ARGS="x:=3.0 y:=2.8"
            DESC="Parks at doorway — robot reroutes after 4.5 s"
            ;;
        corridor_walker)
            EXTRA_ARGS="x:=-2.0 y:=1.5"
            DESC="Head-on walker in narrow corridor — triggers YIELDING"
            ;;
        aisle_crossing|*)
            EXTRA_ARGS="x:=$X y:=$Y"
            DESC="Crosses aisles — triggers MPPI evasion swerves"
            ;;
    esac

    tmux send-keys -t "$SESSION:0.0" "
printf '\033[1;33m╔══════════════════════════════════════════════╗\033[0m\n'
printf '\033[1;33m║  DYNAMIC OBSTACLE  —  ${PATTERN}  \033[0m\n'
printf '\033[1;33m╚══════════════════════════════════════════════╝\033[0m\n'
echo -e '\033[0;33mBehaviour: ${DESC}\033[0m'
echo ''
$SOURCE_CMD
cd $WS_ROOT
ros2 launch agv_description dynamic_obstacle.launch.py pattern:=$PATTERN speed:=$SPEED $EXTRA_ARGS
" Enter
fi

echo -e "${GRN}✓ Obstacle session '${SESSION}' created in tmux.${RST}"
echo ""
echo -e "  ${BLD}Pattern: ${YLW}${PATTERN}${RST}  at  x=${X}, y=${Y}, speed=${SPEED}"
echo ""
echo -e "  ${BLD}Watch the AMR dashboard for:${RST}"
echo -e "  • ${CYN}MPPI evasion swerves${RST}  (obstacle within 0.85 m)"
echo -e "  • ${YLW}YIELDING state${RST}        (narrow corridor blocked)"
echo -e "  • ${RED}Dijkstra reroute${RST}      (blocked > 4.5 s)"
echo ""
echo -e "  ${BLD}Commands:${RST}"
echo -e "  ${YLW}./launch_obstacle.sh stop${RST}   — remove obstacle"
echo -e "  ${YLW}Ctrl+B then D${RST}               — detach session"
echo ""

tmux attach-session -t "$SESSION"
