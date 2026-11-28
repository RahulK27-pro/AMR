#!/usr/bin/env bash
# ==============================================================================
# run_automated_ml_pipeline.sh
# ==============================================================================
# Automated Parallel Multi-Agent Simulation, Obstacle Spawning, 80/20 Data
# Collection & Best-Model Training Pipeline (visible live in Gazebo GUI + RViz2).
#
# Agents:
#   - Gazebo + RViz2 + AMCL + Map Server (navigation_launch.py)
#   - route_runner (ML Perception + Kalman-NN + MPPI) + graph_visualizer
#   - Agent 1: sim_mission_agent       (autonomous patrol & evasion verification)
#   - Agent 2: obstacle_director_agent (spawns/controls pedestrians, carts, clutter)
#   - Agent 3: dataset_trainer_agent   (live capture + 80/20 mix + trains all models)
#
# Usage:
#   ./run_automated_ml_pipeline.sh [--headless] [--duration 95] [--keep-open]
# ==============================================================================

set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HEADLESS="false"
USE_RVIZ="true"
DURATION="95"
TARGET_LAPS="5"
SCENARIO_SEC="16"
KEEP_OPEN="0"
MAP_NAME="hospital_map"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --headless) HEADLESS="true"; USE_RVIZ="false"; shift ;;
        --duration) DURATION="$2"; shift 2 ;;
        --laps) TARGET_LAPS="$2"; shift 2 ;;
        --scenario-sec) SCENARIO_SEC="$2"; shift 2 ;;
        --map) MAP_NAME="$2"; shift 2 ;;
        --keep-open) KEEP_OPEN="1"; shift ;;
        *) echo "Unknown arg: $1"; exit 1 ;;
    esac
done

set +u
source /opt/ros/jazzy/setup.bash
cd "$WS_DIR"
colcon build --packages-select agv_description agv_navigation --symlink-install
source "$WS_DIR/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-7}"
export ROS_LOCALHOST_ONLY=1
export DISPLAY="${DISPLAY:-:0}"
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
export MESA_GL_VERSION_OVERRIDE="${MESA_GL_VERSION_OVERRIDE:-4.5}"
export GZ_IP=127.0.0.1

LOG_DIR="$WS_DIR/run_logs/ml_pipeline_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"

# Prevent duplicate /clock or /scan publishers from orphaned previous runs
kill_stale_sim() {
    pkill -TERM -f "[g]z sim|[p]arameter_bridge|[r]oute_runner|[g]raph_visualizer|[o]bstacle_director_agent|[s]im_mission_agent|[d]ataset_trainer_agent|[r]viz2|[n]av2_map_server|[n]av2_amcl|[n]av2_lifecycle_manager|[e]kf_node|[r]obot_state_publisher" 2>/dev/null || true
    sleep 1.0
    pkill -KILL -f "[g]z sim|[p]arameter_bridge|[r]oute_runner|[o]bstacle_director_agent|[s]im_mission_agent|[d]ataset_trainer_agent" 2>/dev/null || true
}
kill_stale_sim

PIDS=()
cleanup() {
    if [[ "$KEEP_OPEN" == "1" ]]; then
        echo "[PIPELINE] --keep-open active: leaving Gazebo/RViz2/route_runner open."
        return
    fi
    echo "[PIPELINE] Stopping background simulation & agent processes..."
    for pid in "${PIDS[@]:-}"; do
        kill -TERM "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
    done
    sleep 1.2
    for pid in "${PIDS[@]:-}"; do
        kill -KILL "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
    done
    kill_stale_sim
}
trap cleanup EXIT INT TERM

echo "========================================================================="
echo "  AMR Automated Multi-Agent ML Pipeline"
echo "  Map=$MAP_NAME | Headless=$HEADLESS | RViz=$USE_RVIZ | Collect=${DURATION}s"
echo "  Logs: $LOG_DIR"
echo "========================================================================="

# 1. Gazebo + Map Server + AMCL + RViz2
setsid ros2 launch agv_description navigation_launch.py \
    map:="$MAP_NAME" headless:="$HEADLESS" use_rviz:="$USE_RVIZ" \
    >"$LOG_DIR/01_navigation_launch.log" 2>&1 &
PIDS+=($!)

echo "[1/5] Waiting for Gazebo world..."
for _ in $(seq 1 30); do
    if gz service -s /gazebo/worlds --reqtype gz.msgs.Empty --reptype gz.msgs.StringMsg_V --timeout 800 --req "" 2>/dev/null | grep -q "data:"; then
        break
    fi
    sleep 1
done

# 2. Route Runner & Graph Visualizer
setsid ros2 run agv_navigation route_runner --ros-args -p map_name:="$MAP_NAME" -p use_sim_time:=true \
    >"$LOG_DIR/02_route_runner.log" 2>&1 &
PIDS+=($!)

setsid ros2 run agv_navigation graph_visualizer --ros-args -p map_name:="$MAP_NAME" -p use_sim_time:=true \
    >"$LOG_DIR/02b_graph_visualizer.log" 2>&1 &
PIDS+=($!)

sleep 2.5

# 3. Agent 2: Obstacle Director (Spawns & drives pedestrians, carts, clutter)
setsid ros2 run agv_navigation obstacle_director_agent --ros-args \
    -p use_sim_time:=true -p scenario_duration_sec:="$SCENARIO_SEC" \
    >"$LOG_DIR/03_obstacle_director.log" 2>&1 &
PIDS+=($!)

# 4. Agent 1: Sim Mission Agent (Autonomous patrol goals & evasion monitor)
setsid ros2 run agv_navigation sim_mission_agent --ros-args \
    -p use_sim_time:=true -p map_name:="$MAP_NAME" -p target_laps:="$TARGET_LAPS" -p max_duration_sec:="$DURATION" \
    >"$LOG_DIR/04_sim_mission.log" 2>&1 &
PIDS+=($!)

# 5. Agent 3: Dataset Collector & 80/20 Multi-Model Trainer (foreground wait)
echo "[2/5] Running live parallel collection, 80/20 assembly & multi-model training..."
setsid bash -c 'ros2 run agv_navigation dataset_trainer_agent --ros-args \
    -p use_sim_time:=true -p ws_root:="$1" -p map_name:="$2" \
    -p collect_duration_sec:="$3" -p auto_train:=true \
    2>&1 | tee "$4/05_dataset_trainer.log"' _ "$WS_DIR" "$MAP_NAME" "$DURATION" "$LOG_DIR" &
TRAINER_PID=$!
PIDS+=($TRAINER_PID)

# Monitor until training completes inside dataset_trainer_agent (up to 720s)
TRAIN_OK=0
for _ in $(seq 1 360); do
    if grep -q "Best models selected, exported, and hot-reload triggered!" "$LOG_DIR/05_dataset_trainer.log" 2>/dev/null; then
        echo "[PIPELINE] Training & best-model hot-reload completed!"
        TRAIN_OK=1
        sleep 0.5
        break
    fi
    if ! kill -0 "$TRAINER_PID" 2>/dev/null; then
        wait "$TRAINER_PID" 2>/dev/null || true
        break
    fi
    sleep 2
done

echo "========================================================================="
echo "  Pipeline Summary:"
grep -E "DATA_80_20|CLF_EVAL|SELECTED_CLF|PRED_EVAL|SELECTED_PRED|METADATA" "$LOG_DIR/05_dataset_trainer.log" || true
echo "-------------------------------------------------------------------------"
echo "  Closed-Loop Navigation & Evasion Summary:"
grep -E "ML_PERCEPTION|BRAIN: SWERVE|FINAL DESTINATION REACHED|STUCK" "$LOG_DIR/02_route_runner.log" | tail -n 25 || true
echo "========================================================================="
if [[ "$TRAIN_OK" != "1" ]]; then
    echo "[PIPELINE] WARNING: Trainer did not reach hot-reload completion banner!" >&2
    exit 1
fi
