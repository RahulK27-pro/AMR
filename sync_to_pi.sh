#!/usr/bin/env bash
# ==============================================================================
# sync_to_pi.sh
# ==============================================================================
# Quickly syncs modified source files to the Raspberry Pi over Wi-Fi
#
# Usage:
#   ./sync_to_pi.sh [PI_IP]
# ==============================================================================

set -e

PI_IP="${1:-10.244.248.208}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=================================================================="
echo "  Syncing updated AMR source files -> Raspberry Pi ($PI_IP)"
echo "=================================================================="

# 1. Attempt inter-machine clock sync to prevent SLAM TF message filter drops
echo "[1/2] Syncing system clock with Raspberry Pi..."
if ssh -o ConnectTimeout=3 -o BatchMode=yes "amr@$PI_IP" "sudo date -s '@$(date +%s)'" 2>/dev/null; then
    echo "  ✅ Clock synchronized successfully with Raspberry Pi!"
else
    echo "  ⚠️  Could not passwordlessly sync clock over SSH. Run this manually on Pi if needed:"
    echo "      sudo date -s \"$(date -u +'%Y-%m-%d %H:%M:%S')\""
fi

echo "[2/2] Syncing workspace files..."

rsync -avz \
    --exclude='build' \
    --exclude='install' \
    --exclude='log' \
    --exclude='.git' \
    --exclude='*.pyc' \
    --exclude='__pycache__' \
    "$SCRIPT_DIR/src/" "amr@$PI_IP:~/AMR/src/"

rsync -avz \
    "$SCRIPT_DIR/start_hardware.sh" \
    "$SCRIPT_DIR/env_hardware.sh" \
    "$SCRIPT_DIR/teleop_hardware.sh" \
    "$SCRIPT_DIR/verify_hardware_topics.sh" \
    "amr@$PI_IP:~/AMR/"

echo ""
echo "=================================================================="
echo "  ✅ Source files synced successfully!"
echo "  Now run this on your Raspberry Pi:"
echo "    cd ~/AMR"
echo "    colcon build --packages-select agv_description --symlink-install"
echo "=================================================================="
