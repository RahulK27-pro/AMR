#!/usr/bin/env bash
# ==============================================================================
# sync_to_pi.sh  —  Bidirectional sync between Laptop and Raspberry Pi
# ==============================================================================
#
# Usage:
#   ./sync_to_pi.sh                       # Push src + scripts + maps to Pi (default)
#   ./sync_to_pi.sh push [PI_IP]          # Same as default
#   ./sync_to_pi.sh pull [PI_IP]          # Pull maps saved on Pi back to laptop
#   ./sync_to_pi.sh both [PI_IP]          # Push first, then pull
#   ./sync_to_pi.sh [PI_IP]               # IP as first arg → push
#
# Examples:
#   ./sync_to_pi.sh                       # Push to 10.244.248.208
#   ./sync_to_pi.sh 192.168.1.50          # Push to custom IP
#   ./sync_to_pi.sh pull 192.168.1.50     # Pull maps from custom IP
#   ./sync_to_pi.sh both                  # Full bidirectional sync
#
# Pi-side workspace:   amr@<PI_IP>:~/AMR/
# Laptop workspace:    this directory (auto-detected)
# ==============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------
DIRECTION="push"
PI_IP="10.244.248.208"
PI_USER="amr"

for arg in "$@"; do
    case "$arg" in
        push|pull|both) DIRECTION="$arg" ;;
        [0-9]*.[0-9]*.[0-9]*.[0-9]*) PI_IP="$arg" ;;
    esac
done

PI="$PI_USER@$PI_IP"
PI_AMR="$PI:~/AMR"

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; CYAN='\033[0;36m'; NC='\033[0m'
info()    { echo -e "${CYAN}[INFO]${NC} $*"; }
ok()      { echo -e "${GREEN}[OK]${NC}   $*"; }
warn()    { echo -e "${YELLOW}[WARN]${NC} $*"; }
fail()    { echo -e "${RED}[FAIL]${NC} $*"; }

echo "======================================================================"
echo "  AMR File Sync  |  Mode: $DIRECTION  |  Pi: $PI"
echo "======================================================================"

# ---------------------------------------------------------------------------
# 0. Reachability check
# ---------------------------------------------------------------------------
info "Checking Pi reachability..."
if ! ping -c1 -W2 "$PI_IP" &>/dev/null; then
    fail "Pi ($PI_IP) is not reachable. Check Wi-Fi / cable."
    exit 1
fi
ok "Pi is reachable."

# ---------------------------------------------------------------------------
# 1. Clock sync (reduce SLAM TF message-filter drops)
# ---------------------------------------------------------------------------
info "Syncing system clock..."
if ssh -o ConnectTimeout=4 -o BatchMode=yes -o StrictHostKeyChecking=no \
       "$PI" "sudo date -s '@$(date +%s)'" &>/dev/null; then
    ok "Clock synchronized."
else
    warn "Could not sync clock (passwordless sudo may not be configured)."
    warn "  Run manually on Pi: sudo date -s \"$(date -u +"%Y-%m-%d %H:%M:%S")\""
fi

# ---------------------------------------------------------------------------
# PUSH — laptop → Pi
# ---------------------------------------------------------------------------
do_push() {
    echo ""
    echo "──────────────────────────────────────────────"
    info "PUSH  →  Laptop → Pi ($PI_IP)"
    echo "──────────────────────────────────────────────"

    # 1. ROS2 source packages (Python nodes, launch files, URDF, configs)
    info "[1/4] Syncing src/ (ROS2 packages)..."
    rsync -avz --delete \
        --exclude='build' --exclude='install' --exclude='log' \
        --exclude='.git' --exclude='*.pyc' --exclude='__pycache__' \
        "$SCRIPT_DIR/src/" "$PI_AMR/src/"
    ok "src/ synced."

    # 2. Hardware shell scripts (Pi runs these directly)
    info "[2/4] Syncing hardware shell scripts..."
    rsync -avz \
        "$SCRIPT_DIR/start_hardware.sh" \
        "$SCRIPT_DIR/env_hardware.sh" \
        "$SCRIPT_DIR/teleop_hardware.sh" \
        "$SCRIPT_DIR/verify_hardware_topics.sh" \
        "$SCRIPT_DIR/save_hardware_map.sh" \
        "$SCRIPT_DIR/start_hardware_mapping.sh" \
        "$SCRIPT_DIR/cyclonedds.xml" \
        "$PI_AMR/"
    ok "Shell scripts synced."

    # 3. Maps (yaml + pgm/png + graph json) — Pi needs these for Nav2 localization
    info "[3/4] Syncing maps/ to Pi..."
    rsync -avz \
        --include='*.yaml' --include='*.pgm' --include='*.png' \
        --include='*.json' --exclude='*' \
        "$SCRIPT_DIR/maps/" "$PI_AMR/maps/"
    ok "maps/ synced."

    # 4. Navigation parameters / config files
    info "[4/4] Syncing config files (nav params, bt, rviz)..."
    rsync -avz \
        --include='*.yaml' --include='*.xml' --include='*.rviz' --exclude='*' \
        "$SCRIPT_DIR/src/agv_navigation/config/" \
        "$PI_AMR/src/agv_navigation/config/" 2>/dev/null || true
    ok "Config files synced."

    echo ""
    ok "PUSH complete."
    echo ""
    warn "Now run on Pi to rebuild:"
    echo "    ssh $PI"
    echo "    cd ~/AMR && colcon build --symlink-install"
    echo "    # or for quick iteration (Python-only changes):"
    echo "    # colcon build --packages-select agv_description agv_navigation --symlink-install"
}

# ---------------------------------------------------------------------------
# PULL — Pi → laptop  (maps saved during hardware sessions)
# ---------------------------------------------------------------------------
do_pull() {
    echo ""
    echo "──────────────────────────────────────────────"
    info "PULL  ←  Pi ($PI_IP) → Laptop"
    echo "──────────────────────────────────────────────"

    # 1. Maps created on Pi (new SLAM sessions)
    info "[1/2] Pulling maps/ from Pi..."
    mkdir -p "$SCRIPT_DIR/maps"
    rsync -avz \
        --include='*.yaml' --include='*.pgm' --include='*.png' \
        --include='*.json' --exclude='*' \
        "$PI_AMR/maps/" "$SCRIPT_DIR/maps/"
    ok "maps/ pulled."

    # 2. Any new logs or diagnostics (run_logs/)
    info "[2/2] Pulling run_logs/ from Pi (if exists)..."
    rsync -avz \
        "$PI_AMR/run_logs/" "$SCRIPT_DIR/run_logs/pi/" 2>/dev/null \
    && ok "run_logs/ pulled." \
    || warn "No run_logs/ on Pi — skipped."

    echo ""
    ok "PULL complete. New maps are in: $SCRIPT_DIR/maps/"
}

# ---------------------------------------------------------------------------
# Dispatch
# ---------------------------------------------------------------------------
case "$DIRECTION" in
    push) do_push ;;
    pull) do_pull ;;
    both) do_push; do_pull ;;
    *)
        fail "Unknown direction: $DIRECTION (use push | pull | both)"
        exit 1
        ;;
esac

echo "======================================================================"
ok "Sync finished — direction=$DIRECTION  Pi=$PI_IP"
echo "======================================================================"
