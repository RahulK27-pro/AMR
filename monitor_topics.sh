#!/usr/bin/env bash
# monitor_topics.sh
# =================
# Launch continuous live multi-topic terminal monitor for the AMR system.
# Sets up Domain 7, CycloneDDS, sources the ROS 2 workspace, and starts the monitor.
#
# Usage:
#   ./monitor_topics.sh            # Live full dashboard (clean in-place refresh)
#   ./monitor_topics.sh --stream   # Continuous scrolling log mode

set -e

# Source ROS 2 Jazzy
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
else
    echo "[ERROR] /opt/ros/jazzy/setup.bash not found!"
    exit 1
fi

# Source workspace install overlay
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
fi

# Ensure Domain 7 and CycloneDDS matching AMR stack
export ROS_DOMAIN_ID=7
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export RCUTILS_LOGGING_SEVERITY_THRESHOLD=ERROR

exec python3 "$WS_DIR/monitor_topics.py" "$@"
