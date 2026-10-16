#!/usr/bin/env bash
# stop_sim.sh
# ============
# Stops any running Gazebo simulation and ROS 2 bridges on the laptop.

echo "=================================================="
echo "  Stopping Gazebo Simulation & ROS 2 Bridges..."
echo "=================================================="

pkill -9 -f "parameter_bridge" 2>/dev/null || true
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f "ros_gz_sim" 2>/dev/null || true
pkill -9 -f "agv_description/launch/gazebo.launch.py" 2>/dev/null || true

sleep 1.0

if pgrep -f "gz sim" >/dev/null 2>&1; then
    echo "⚠️  Some Gazebo processes are still lingering. Forcing termination..."
    killall -9 -q gz-sim-server gz-sim-gui ruby 2>/dev/null || true
fi

echo "✅ Gazebo simulation and bridges stopped successfully."
