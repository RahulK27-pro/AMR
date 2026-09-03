# AMR Project — Running Guide

This guide covers the complete startup, navigation, mission dispatch, dynamic obstacle testing, and troubleshooting for the Autonomous Mobile Robot (AMR) warehouse navigation system.

**Stack summary:**
- **Gazebo Harmonic** simulation with Gazebo–ROS bridge
- **AMCL** localization on a pre-saved occupancy-grid map
- **EKF** sensor fusion (`/odom` + `/imu/data` → `map → base_link` TF)
- **Dijkstra** global planner over a 756-node topological graph (KD-Tree accelerated, O(log N))
- **MPPI** local controller (80 samples × 15-step horizon, 10 Hz, LiDAR-only)
- **py_trees Behavior Tree** mission orchestrator (`bt_manager`)
- **Multi-goal waypoint queue** via `/goal_sequence` topic

---

## Prerequisites

Open every terminal in the workspace root and source before running any command:

```bash
cd ~/AMR/AMR-main
colcon build
source install/setup.bash
```

**Dependencies check:**
```bash
python3 -c "from scipy.spatial import KDTree; import py_trees; print('All dependencies OK')"
```

---

## Launch Order

You need **3–4 terminals** for a standard run. Open them all in `~/AMR/AMR-main` and source each one.

---

### Terminal 1 — Gazebo + AMCL Localization

```bash
ros2 launch agv_description navigation_launch.py
```

This launches:
| Component | Role |
|---|---|
| Gazebo Harmonic (`gz sim`) | Physics simulation + LiDAR / IMU / encoders |
| `ros_gz_bridge` | Gazebo ↔ ROS 2 topic bridge |
| `robot_state_publisher` | Publishes URDF transforms |
| `ekf_node` | Fuses `/odom` + `/imu/data` → `odom` frame |
| `map_server` | Serves `warehouse_map.yaml` on `/map` |
| `amcl` | Particle-filter localization (`map → base_link` TF) |
| RViz2 | Visualizer (auto-launched after 7 s) |

> **Wait** until you see `Localization Active! Received map -> base_link TF transform.` in the `route_runner` terminal before sending goals.

To use a different map:
```bash
ros2 launch agv_description navigation_launch.py \
    map:=/full/path/to/your_map.yaml
```

---

### Terminal 2 — Route Runner (Core Navigation)

```bash
source install/setup.bash
ros2 run agv_navigation route_runner
```

Expected startup log:
```
[route_runner]: Route Runner Active with KD-Tree (756 nodes) & Dynamic Obstacle Avoidance.
[route_runner]: Localization Active! Received map -> base_link TF transform.
```

The route runner owns **all `/cmd_vel` publishing**. Nav2's DWB controller is intentionally NOT launched to avoid conflicting velocity commands.

---

### Terminal 3 — Graph Visualizer (Optional but Recommended)

```bash
source install/setup.bash
ros2 run agv_navigation graph_visualizer
```

In RViz, add a **MarkerArray** display subscribed to `/agv_graph_markers`. You will see:

| Color | Meaning |
|---|---|
| 🟡 Yellow spheres | Topological graph nodes |
| ⬜ White text | Node ID labels (N0 … N755) |
| 🩵 Cyan lines | Graph edges (bidirectional) |
| 🟣 Magenta line | Current active dense path |
| 🟠 Orange dots | Waypoints along active path |

---

### Terminal 4 — Behavior Tree Manager (Optional)

```bash
source install/setup.bash
ros2 run agv_navigation bt_manager
```

The BT manager adds:
- **Localization check** — verifies TF before dispatching goals
- **Automatic mission dispatch** — dequeues goals from the mission queue
- **Recovery behaviors** — `BackUp` (0.3 m reverse) + `Spin` (60° rotation) when all paths are blocked

> If you use `bt_manager`, send goal sequences to `/goal_sequence` rather than clicking RViz manually. Both modes can coexist — RViz clicks will still work through `route_runner` directly.

---

## Sending Goals

### Option A — Single Goal via RViz

1. Open RViz (launched automatically after 7 s)
2. Click **"2D Goal Pose"** in the top toolbar
3. Click and drag anywhere on the map near a valid node
4. Watch `route_runner` terminal:
   ```
   Received Goal: (x, y)
   Snapping to nodes: Start=N62, Target=N309
   Dijkstra Path: ['N62', 'N52', 'N350', 'N309'] (13 waypoints)
   ```

### Option B — Single Goal via Topic

```bash
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: 3.1, y: 3.0}}}'
```

### Option C — Multi-Goal Mission Sequence

Send an ordered list of **graph node IDs** as a JSON array. The robot navigates each goal in sequence automatically:

```bash
ros2 topic pub --once /goal_sequence std_msgs/String \
  'data: "[\"N5\", \"N12\", \"N40\"]"'
```

To find valid node IDs:
```bash
python3 -c "
import json
d = json.load(open('src/agv_description/maps/warehouse_graph.json'))
print([n['id'] for n in d['nodes'][:20]])
"
```

Monitor mission progress in real time:
```bash
ros2 topic echo /mission_progress
```

Example progress output:
```json
{"current": 2, "total": 3, "goal_node": "N12", "state": "NAVIGATING"}
{"current": 3, "total": 3, "goal_node": "N40", "state": "MISSION_COMPLETE"}
```

---

## Testing Dynamic Obstacles

### Spawn an Autonomous Obstacle

```bash
source install/setup.bash

# Aisle-crossing worker (back-and-forth across aisles)
ros2 launch agv_description dynamic_obstacle.launch.py \
    pattern:=aisle_crossing speed:=0.35 x:=1.8 y:=0.0

# Doorway blocker (triggers automatic re-routing after 4.5 s)
ros2 launch agv_description dynamic_obstacle.launch.py \
    pattern:=doorway_blocker x:=3.0 y:=2.8

# Corridor walker (head-on traffic in narrow aisles)
ros2 launch agv_description dynamic_obstacle.launch.py \
    pattern:=corridor_walker x:=-2.0 y:=1.5 speed:=0.45
```

### Manual Teleoperation of Obstacle

```bash
# Launch without autonomous controller
ros2 launch agv_description dynamic_obstacle.launch.py run_controller:=false x:=1.5 y:=0.0

source /opt/ros/jazzy/setup.bash
source ~/AMR/AMR-main/install/setup.bash
ros2 launch agv_description dynamic_obstacle.launch.py run_controller:=false x:=1.5 y:=0.0


# Drive it with WASD keyboard
ros2 run agv_navigation obstacle_teleop

source /opt/ros/jazzy/setup.bash
source ~/AMR/AMR-main/install/setup.bash
ros2 run agv_navigation obstacle_teleop

```


**Controls:** `W/S` forward/back, `A/D` turn, `Q/E` forward-diag, `Space` stop, `+/-` speed.

### What the AMR Does When Blocked

| Scenario | Response |
|---|---|
| Moving obstacle near path | MPPI swerves around proactively (repulsion field within 0.85 m) |
| Obstacle crossing narrow aisle | Robot yields (`YIELDING` state, v=0) and waits for it to pass |
| Obstacle stationary for > 4.5 s | Triggers Dijkstra **re-route** through an alternate corridor |
| All paths blocked | BT manager triggers `BackUp + Spin` recovery, then re-plans |

---

## Key Topics Reference

| Topic | Type | Direction | Purpose |
|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | Subscribed | LiDAR obstacle data |
| `/goal_pose` | `geometry_msgs/PoseStamped` | Subscribed | Single navigation goal |
| `/goal_sequence` | `std_msgs/String` (JSON) | Subscribed | Ordered multi-goal mission |
| `/mission_progress` | `std_msgs/String` (JSON) | Published | Real-time mission status |
| `/cmd_vel` | `geometry_msgs/Twist` | Published | Robot velocity commands |
| `/agv_dense_path` | `nav_msgs/Path` | Published | Current dense MPPI path |
| `/agv_graph_markers` | `visualization_msgs/MarkerArray` | Published | RViz graph visualization |
| `/dynamic_obstacle/cmd_vel` | `geometry_msgs/Twist` | Published | Obstacle movement commands |

---

## Troubleshooting

| Problem | Cause | Fix |
|---|---|---|
| `CRITICAL FAILURE: SERVER amcl IS DOWN` | Lifecycle bond timeout triggered too early | Already fixed — `bond_timeout: 0.0` in `navigation_launch.py` |
| `Waiting for localization/odometry` | AMCL not yet initialized | Wait for particle cloud to converge in RViz (set **2D Pose Estimate** first) |
| Robot navigates but ignores goal | Map → base_link TF missing | Set **2D Pose Estimate** in RViz to initialize AMCL particle filter |
| `NO VALID PATH FOUND` | Goal node is disconnected from start | Choose a goal closer to a visible node, or check graph connectivity |
| Robot oscillates at waypoint | Arrival threshold too tight | Reduce `dist_to_final < 0.30` threshold in `route_runner.py` if needed |
| `goal_sequence: unknown node IDs` | Node IDs not in the graph | Check valid IDs with the python3 command in Option C above |
| Graph markers not visible in RViz | Wrong topic subscribed | Subscribe to `/agv_graph_markers` (not `/graph_markers`) |

---

## MPPI & Obstacle Avoidance Tuning Reference

The key parameters in [`route_runner.py`](file:///home/rahul/AMR/AMR-main/src/agv_navigation/agv_navigation/route_runner.py):

| Parameter | Default | Effect |
|---|---|---|
| `v_max` | 0.8 m/s | Top linear speed |
| `w_max` | 1.8 rad/s | Max angular velocity |
| `horizon` | 15 steps (1.5 s) | MPPI lookahead depth |
| `num_samples` | 80 | Trajectory samples per tick (10 Hz) |
| `w_collision` | 5000.0 | Hard collision cost weight |
| `collision_radius` | 0.18 m | Hard safety footprint clearance (0.11m body + 0.07m margin) |
| `dynamic_repulsive_dist` | 0.85 m | Proactive repulsion bubble around moving obstacles |
| `dynamic_w_repulsive` | 45.0 | Dynamic obstacle repulsion weight |
| `static_repulsive_dist` | 0.45 m | Wall/shelf repulsion activation distance |
| `static_w_repulsive` | 80.0 | Quadratic wall repulsion weight |
| `w_cross_track_nominal` | 6.0 | Centerline tracking tightness in open corridors |
| `w_cross_track_evasion` | 0.5 | Relaxed centerline tracking during evasion |
| `yield_timeout` | 4.5 s | Seconds to wait before Dijkstra re-route |

---

## Recording & Diagnostic Logs

To diagnose dynamic obstacle avoidance, path tracking, room transitions, and decision-making, use the following logging commands.

### Method 1: Live Route Runner Logging to Console and File

Run `route_runner` in its terminal using `tee` so that all real-time brain decisions and telemetry are displayed on screen and simultaneously saved to a log file:

```bash
source install/setup.bash
mkdir -p logs
ros2 run agv_navigation route_runner | tee "logs/route_runner_$(date +%Y%m%d_%H%M%S).log"
```

---

### Method 2: Record Full ROS 2 Bag (All Topics, Sensors, TF & Commands)

Open a separate terminal and run this command before starting your navigation run:

```bash
source install/setup.bash
mkdir -p logs
ros2 bag record -a --exclude-topics /particle_cloud -o "logs/bag_$(date +%Y%m%d_%H%M%S)"
```

> **Note:** `--exclude-topics /particle_cloud` prevents the ROS 2 bag transport type conflict between AMCL (`nav2_msgs/ParticleCloud`) and RViz (`geometry_msgs/PoseArray`).

**What this records:**
- **Inputs:** `/scan` (LiDAR point ranges), `/goal_pose`, `/goal_sequence`, `/initialpose`, `/tf`, `/tf_static`
- **Outputs:** `/cmd_vel` (motor velocities), `/agv_dense_path` (MPPI path), `/mission_progress`, `/odometry/filtered`
- **Dynamic Obstacle:** `/dynamic_obstacle/cmd_vel`, `/dynamic_obstacle/odom`

Press `Ctrl+C` in that terminal when your test completes to save the bag.

---

### Method 3: Snapshot All Node Console Logs After a Run

After completing a simulation run, capture all individual ROS 2 node log files generated under `~/.ros/log/` into a timestamped directory:

```bash
LOG_DIR="logs/run_$(date +%Y%m%d_%H%M%S)" && mkdir -p "$LOG_DIR" && cp -r ~/.ros/log/latest/* "$LOG_DIR"/ && echo "All node logs saved to $LOG_DIR"
```

---

## Analyzing Decision Traceability in Logs

The navigation system uses a hierarchical brain architecture with explicit log tags to trace **who made each decision**:

| Log Tag | Subsystem ("Brain") | Role & Decision Captured |
|---|---|---|
| `[BRAIN: MPPI_CONTROLLER]` | Tactical Controller | Softened centerline tracking (`w_cross_track = 0.5`) to swerve around a dynamic or static obstacle. |
| `[BRAIN: STATE_MACHINE]` | Traffic Executive | Commanded `YIELD` (`v = 0.0`) when all trajectories are blocked in a narrow aisle; holds yield timer; resumes navigation when clear. |
| `[BRAIN: DIJKSTRA_ROUTER]` | Global Planner | Commanded `ROUTE CHANGE`: penalized blocked edge (`+999.0`) and recalculated alternate topological path after 4.5s yield timeout. |
| `[BRAIN: GOAL_ARRIVAL]` | Destination Executive | Detected arrival within destination threshold (`dist < 0.30m`), logged arrival error, and commanded full stop. |
| `[BRAIN: MISSION_EXEC]` | Mission Orchestrator | Progressed to next waypoint in the multi-goal mission queue. |
| `[LIDAR_SCAN: DYNAMIC_OBSTACLE]` | Perception / Tracking | Tracked moving obstacle with position `(x, y)`, velocity `(vx, vy)`, speed `(m/s)`, and distance to robot. |
| `[TELEMETRY]` | Health / Status Monitor | Periodic (1.5s) snapshot of robot map pose, heading, closest node, distance to goal, motor command `(v, w)`, and state. |
| `[DEVIATION ALERT]` | Graph Tracker | Alert when robot position drifts outside planned Dijkstra node sequence. |

### Quick Analysis Filter Commands

Filter and inspect specific behaviors directly from your saved run log:

```bash
# 1. Trace all high-level autonomous decisions:
grep "\[BRAIN:" logs/route_runner_*.log

# 2. Trace dynamic obstacle detection and velocity estimation:
grep "\[LIDAR_SCAN:" logs/route_runner_*.log

# 3. Trace tactical evasive swerves:
grep "\[BRAIN: MPPI_CONTROLLER\]" logs/route_runner_*.log

# 4. Trace corridor yields and rerouting detours:
grep -E "\[BRAIN: STATE_MACHINE\]|\[BRAIN: DIJKSTRA_ROUTER\]" logs/route_runner_*.log

# 5. Trace goal arrival and mission waypoint progression:
grep -E "\[BRAIN: GOAL_ARRIVAL\]|\[BRAIN: MISSION_EXEC\]" logs/route_runner_*.log

# 6. Trace any path deviations:
grep "\[DEVIATION ALERT\]" logs/route_runner_*.log
```

