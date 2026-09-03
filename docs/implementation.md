# Implementation Plan: Hybrid Topological Navigation AMR
## Edge-AI Autonomous Mobile Robot — ROS 2 Jazzy + Gazebo Harmonic

---

> **Engineering Assessment:** This plan was written from the perspective of a senior robotics engineer reviewing the gap between the current codebase and the target architecture. Every item is tied to a specific file change.

---

## Part 1 — Architecture Clarity Before Code

Before writing a single line, three architectural decisions must be locked down, because they cascade into every file that follows.

### Decision 1: TEB vs. MPPI Local Planner

The report specifies **TEB (Timed Elastic Band)**. TEB is the right choice conceptually — it deforms paths around obstacles like a rubber band and is well-suited to diff-drive kinematics. However:

- `teb_local_planner` for ROS 2 Jazzy is **not in the standard apt repos** — it must be built from source from the `rst-tu-dortmund/teb_local_planner` repository
- **MPPI (Model Predictive Path Integral)** is natively available in Nav2 for Jazzy, performs trajectory rollouts with GPU-optional computation, and handles dynamic obstacles smoothly

> **Recommendation:** Implement with **MPPI first** (zero build complexity, same behavior), then swap in TEB from source if MPPI trajectory quality is insufficient. The nav2_params.yaml plugin name is the only change required to swap between them.

### Decision 2: Graph Source of Truth

Current: `map_builder.py` (driven manually → nodes dropped every 1m → `test_map.json`)
New: `graph_extractor.py` (runs once on a saved `.pgm` → skeletonize → auto-detect junctions → store centerlines in JSON)

These are **mutually exclusive**. Once the auto-extractor is built, `map_builder.py` is retired.

### Decision 3: Who Drives the Robot

Current: `route_runner.py` publishes directly to `/cmd_vel`
New: `route_runner.py` publishes a `nav_msgs/Path` to `/global_reference_path` → TEB/MPPI consumes it → TEB publishes to `/cmd_vel`

This is the core architectural shift. The topological navigator becomes a **path publisher**, not a motor controller. The local planner handles all actual driving.

---

## Part 2 — Current State vs. Target State

| Component | Current Code | Target Architecture | Status |
|---|---|---|---|
| Robot model | `warehouse_agv.urdf` (cylinder, 12kg, LiDAR+IMU+Cam) | Same — keep | ✅ Complete |
| Simulation world | `warehouse.world` (16×12m warehouse) | Same — keep | ✅ Complete |
| Topic bridge | `bridge.yaml` (9 topics) | Same — keep | ✅ Complete |
| EKF sensor fusion | `ekf.yaml` (odom+IMU → filtered) | Same — keep | ✅ Complete |
| SLAM mapping | `sim_only.launch.py` → SLAM Toolbox | Same workflow — keep | ✅ Fixed |
| Map saving | `scripts/save_map.sh` | Same — keep | ✅ Complete |
| AMCL localization | `nav2_params.yaml` (basic) | Needs tuning + dedicated launch | ⚠️ Partial |
| Graph building | `map_builder.py` (manual drive) | `graph_extractor.py` (auto from .pgm) | ❌ Replace |
| Path storage | JSON with nodes+edges+distance | Add `centerline` pixel arrays in edges | ❌ Missing |
| Global planner | Dijkstra in `route_runner.py` → P-controller | Dijkstra → concatenate centerlines → `nav_msgs/Path` | ❌ Rewrite |
| Local planner | None (P-controller directly drives) | TEB/MPPI via Nav2 | ❌ Missing |
| Behavior Tree | None (if-else in route_runner) | `py_trees` BT orchestrator | ❌ Missing |
| Recovery behaviors | None | Turn-Out / Drive-Past / Merge as BT nodes | ❌ Missing |
| Mobile app bridge | None | FastAPI + WebSocket ROS 2 interface | ❌ Missing |
| TF frame | `/odom` (raw, drifts) | `map → base_link` via AMCL | ❌ Not wired |

---

## Part 3 — The Six Implementation Phases

---

### PHASE 1 — Solidify the Foundation (Weeks 1–2)
**Goal:** Clean mapping workflow, verified AMCL localization, stable TF tree.

#### 1.1 Create `mapping_session.launch.py` [NEW FILE]

A dedicated, minimal launch file for the "Map Once" phase. Cleaner than `sim_only.launch.py` because it also launches RViz for visual feedback while mapping.

**File:** `src/agv_description/launch/mapping_session.launch.py`

```
Starts: Gazebo → Bridge (t=0) → RSP (t=3s) → EKF (t=0) → SLAM (t=8s) → RViz (t=10s)
User: Teleoperate robot through warehouse until map is complete
Then: ros2 run nav2_map_server map_saver_cli -f ~/maps/warehouse_map
```

#### 1.2 Create `localization_params.yaml` [NEW FILE]

The existing `nav2_params.yaml` was written for exploration (no AMCL). A clean file is needed for the "Localize Only" runtime.

**File:** `src/agv_description/config/localization_params.yaml`

Key AMCL parameters to set:
```yaml
amcl:
  ros__parameters:
    use_sim_time: true
    alpha1: 0.2   # rotation noise from rotation
    alpha2: 0.2   # rotation noise from translation
    alpha3: 0.2   # translation noise from translation
    alpha4: 0.2   # translation noise from rotation
    base_frame_id: base_link
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: map
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 12.0
    laser_min_range: -1.0
    laser_model_type: likelihood_field
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: odom
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: nav2_amcl::DifferentialMotionModel
    save_pose_rate: 0.5
    scan_topic: /scan
    set_initial_pose: true
    initial_pose:
      x: 0.0
      y: 0.0
      yaw: 0.0
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05
```

#### 1.3 Create `localization_launch.py` [NEW FILE]

**File:** `src/agv_description/launch/localization_launch.py`

```
Starts: Gazebo → Bridge → RSP → EKF → map_server (loads .pgm) → AMCL → RViz
Does NOT start: SLAM Toolbox (map is frozen), Nav2 controller/planner
The topological navigator (Phase 3) runs on top of this.
```

```python
# Startup sequence with delays:
# t=0:  Gazebo + Bridge + EKF
# t=3:  RSP (after /clock)
# t=5:  map_server (loads warehouse_map.yaml)
# t=7:  AMCL (needs map_server to be up first)
# t=9:  RViz
```

#### 1.4 Verification Checklist for Phase 1

```bash
# After localization_launch.py:
ros2 run tf2_ros tf2_echo map base_link          # AMCL must publish this
ros2 topic echo /amcl_pose --once                # Pose with covariance
ros2 topic echo /map --once                      # map_server must publish
ros2 topic hz /scan                              # LiDAR must be ~10 Hz
```

---

### PHASE 2 — Automatic Graph Extraction Pipeline (Weeks 3–5)
**Goal:** Replace manual `map_builder.py` with a one-shot offline script that processes the saved `.pgm` and outputs a rich JSON graph with centerline arrays.

#### 2.1 New Package Dependency

Add to `src/agv_navigation/package.xml`:
```xml
<depend>python3-scikit-image</depend>
<depend>python3-scipy</depend>
<depend>python3-numpy</depend>
```

Install in WSL:
```bash
pip3 install scikit-image scipy numpy
```

#### 2.2 Create `graph_extractor.py` [NEW FILE — Core Algorithm]

**File:** `src/agv_navigation/agv_navigation/graph_extractor.py`

This is a **standalone offline script** (not a ROS node — it runs once). It takes a `.pgm` map as input and outputs `warehouse_graph.json`.

**Algorithm Pipeline:**

```
Step 1: Load .pgm → binary free-space image
         pgm_img = imread(map.pgm)
         free_space = (pgm_img > 200)  # 254=free, ignore unknown (205) and occupied (0)

Step 2: Morphological skeletonization (Zhang-Suen algorithm)
         from skimage.morphology import skeletonize, thin
         skeleton = skeletonize(free_space)
         # skeleton is a binary image of 1-pixel-wide centerlines

Step 3: Convert skeleton to graph
         Build adjacency from skeleton pixels:
         - A pixel is a JUNCTION if it has >2 neighbors in skeleton
         - A pixel is an ENDPOINT if it has exactly 1 neighbor
         - Trace paths between junctions/endpoints → edges

Step 4: Prune short edges (< min_corridor_length pixels)
         Remove dead-end stubs shorter than 20 pixels

Step 5: Convert pixel coordinates → world coordinates
         world_x = origin_x + (pixel_col * resolution)
         world_y = origin_y + ((height - pixel_row) * resolution)
         (y-axis flip: image row 0 = top = positive y in ROS)

Step 6: Store centerline as world-frame coordinate array in each edge
         edge = {
           "from": "N1", "to": "N2",
           "distance_m": 2.45,
           "centerline": [[1.0, 0.0], [1.1, 0.0], [1.2, 0.0], ...]
         }

Step 7: Save warehouse_graph.json
```

**Output JSON schema:**
```json
{
  "metadata": {
    "resolution": 0.05,
    "origin": [-7.0, -6.0, 0.0],
    "map_file": "warehouse_map.pgm",
    "node_count": 14,
    "edge_count": 18
  },
  "graph": {
    "N1": {
      "x": 0.0, "y": 0.0,
      "label": "origin",
      "edges": [
        {
          "to_node": "N2",
          "distance_m": 2.1,
          "centerline": [[0.0,0.0],[0.1,0.0],[0.2,0.0],"..."]
        }
      ]
    }
  }
}
```

#### 2.3 Retire `map_builder.py`

`map_builder.py` is now obsolete. Do NOT delete it (it demonstrates the old approach and documents the project history) but remove it from `setup.py` entry points so it isn't run accidentally.

#### 2.4 Update `setup.py` [MODIFY]

```python
entry_points={
    'console_scripts': [
        # 'map_builder = agv_navigation.map_builder:main',  # RETIRED
        'graph_extractor = agv_navigation.graph_extractor:main',
        'path_planner = agv_navigation.path_planner:main',
        'bt_manager = agv_navigation.bt_manager:main',
        'graph_visualizer = agv_navigation.graph_visualizer:main',
        'path_tracer = agv_navigation.path_tracer:main',
    ],
},
```

#### 2.5 Workflow After Phase 2

```bash
# 1. Run mapping session, save map
ros2 launch agv_description mapping_session.launch.py
# ... teleoperate ... 
ros2 run nav2_map_server map_saver_cli -f ~/maps/warehouse_map

# 2. Run graph extraction (offline, no ROS needed)
python3 graph_extractor.py \
    --map ~/maps/warehouse_map.pgm \
    --yaml ~/maps/warehouse_map.yaml \
    --output ~/AMR/AMR-main/src/agv_navigation/maps/warehouse_graph.json

# 3. Visualize in RViz
ros2 run agv_navigation graph_visualizer
# Should show junction nodes (yellow spheres) + corridor centerlines (cyan lines)
```

---

### PHASE 3 — Topological Path Planner with Centerline Concatenation (Weeks 5–7)
**Goal:** Replace `route_runner.py`'s P-controller with a `nav_msgs/Path` publisher that feeds the local planner. This is the core routing node.

#### 3.1 Create `path_planner.py` [NEW FILE — replaces route_runner.py logic]

**File:** `src/agv_navigation/agv_navigation/path_planner.py`

**ROS interface:**
```
Subscribes:
  /goal_node       (std_msgs/String)   — receives "N5" from BT or mobile app
  /tf              (via tf2_ros)       — reads map→base_link for current pose

Publishes:
  /global_reference_path  (nav_msgs/Path)    — dense centerline path for TEB/MPPI
  /current_node           (std_msgs/String)  — current closest node (for BT monitoring)
  /path_status            (std_msgs/String)  — "PLANNING", "EXECUTING", "ARRIVED", "FAILED"
```

**Internal logic:**
```python
class PathPlanner(Node):
    def __init__(self):
        # tf2 listener for map → base_link
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Load graph
        self.graph = load_graph('warehouse_graph.json')
        
        # Path publisher (dense PoseStamped array)
        self.path_pub = self.create_publisher(Path, '/global_reference_path', 1)
        
        # Goal subscriber
        self.goal_sub = self.create_subscription(String, '/goal_node', 
                                                   self.goal_callback, 10)
        
        # 10 Hz timer: re-publish path (allows path tracking)
        self.create_timer(0.1, self.publish_path)
    
    def get_current_pose_from_tf(self):
        # Use AMCL-corrected map→base_link, NOT raw /odom
        t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        return t.transform.translation.x, t.transform.translation.y
    
    def goal_callback(self, msg):
        goal_node = msg.data
        current_x, current_y = self.get_current_pose_from_tf()
        start_node = self.find_closest_node(current_x, current_y)
        
        # Dijkstra over graph
        node_sequence = self.dijkstra(start_node, goal_node)
        
        # Concatenate stored centerlines into one dense path
        self.current_path = self.build_dense_path(node_sequence)
        
    def build_dense_path(self, node_sequence):
        """Concatenates centerline arrays from graph edges → nav_msgs/Path"""
        path = Path()
        path.header.frame_id = 'map'
        
        for i in range(len(node_sequence) - 1):
            from_node = node_sequence[i]
            to_node = node_sequence[i + 1]
            # Find the edge between them
            edge = self.get_edge(from_node, to_node)
            # Each centerline point becomes a PoseStamped
            for wp in edge['centerline']:
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = wp[0]
                pose.pose.position.y = wp[1]
                # Compute heading from consecutive points
                path.poses.append(pose)
        
        return path
    
    def publish_path(self):
        if self.current_path:
            self.current_path.header.stamp = self.get_clock().now().to_msg()
            self.path_pub.publish(self.current_path)
```

#### 3.2 Prune Old `route_runner.py`

`route_runner.py` currently does both planning AND motor control. Strip it to just Dijkstra (keep for reference) and move all execution logic to `path_planner.py`. Eventually retire route_runner entirely.

---

### PHASE 4 — TEB/MPPI Local Planner Integration (Weeks 7–10)
**Goal:** Wire Nav2's local planner to follow `/global_reference_path`, replacing the P-controller entirely.

#### 4.1 Install TEB or MPPI

**Option A — MPPI (recommended first):**
```bash
sudo apt install ros-jazzy-nav2-mppi-controller
```
Zero additional build needed.

**Option B — TEB (from source, if MPPI quality is insufficient):**
```bash
cd ~/AMR/AMR-main/src
git clone -b ros2 https://github.com/rst-tu-dortmund/teb_local_planner.git
cd ~/AMR/AMR-main
colcon build --packages-select teb_local_planner
```

#### 4.2 Create `nav2_params_localization.yaml` [NEW FILE]

This replaces `nav2_params.yaml` for the navigation (post-mapping) phase. Key differences:

**File:** `src/agv_description/config/nav2_params_localization.yaml`

```yaml
# ── AMCL ──────────────────────────────────────────────────────────────
amcl:
  ros__parameters:
    # (as defined in Phase 1 localization_params.yaml)

# ── controller_server — MPPI local planner ────────────────────────────
controller_server:
  ros__parameters:
    use_sim_time: true
    controller_frequency: 20.0
    controller_plugins: ["FollowPath"]
    
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      time_steps: 56
      model_dt: 0.05
      batch_size: 2000
      vx_std: 0.2
      vy_std: 0.0        # diff-drive: no lateral velocity
      wz_std: 0.4
      vx_max: 0.5
      vx_min: -0.35
      vy_max: 0.0
      wz_max: 1.0
      ax_max: 3.0
      ax_min: -3.0
      az_max: 3.5
      iteration_count: 1
      prune_distance: 1.7
      transform_tolerance: 0.1
      temperature: 0.3
      gamma: 0.015
      motion_model: "DiffDrive"
      visualize: true
      critics:
        ["ConstraintCritic", "CurvatureCritic", "GoalCritic",
         "GoalAngleCritic", "PathAlignCritic", "PathFollowCritic",
         "PathAngleCritic", "PreferForwardCritic"]
      PathFollowCritic:
        enabled: true
        offset_from_furthest: 5
        threshold_to_consider: 1.4
      PathAlignCritic:
        enabled: true
        offset_from_furthest: 20
        threshold_to_consider: 0.5
        use_path_orientations: false

# ── planner_server — replaced by topological planner ──────────────────
# Note: Nav2's global GridBased planner is DISABLED.
# The topological path_planner.py publishes /global_reference_path directly.
# MPPI reads this path via its FollowPath critic.
planner_server:
  ros__parameters:
    use_sim_time: true
    planner_plugins: []    # No grid-based planner — topological planner handles this

# ── global_costmap — obstacle inflation only, no static_layer ─────────
global_costmap:
  global_costmap:
    ros__parameters:
      use_sim_time: true
      global_frame: map
      robot_base_frame: base_link
      robot_radius: 0.13
      resolution: 0.05
      rolling_window: false
      width: 100
      height: 100
      track_unknown_space: true
      plugins: ["obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: LaserScan
          raytrace_max_range: 12.0
          obstacle_max_range: 11.5
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.25

# ── local_costmap — for TEB/MPPI obstacle reactions ──────────────────
local_costmap:
  local_costmap:
    ros__parameters:
      use_sim_time: true
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 5
      height: 5
      resolution: 0.05
      robot_radius: 0.13
      plugins: ["obstacle_layer", "inflation_layer"]
      # (same obstacle/inflation layers as global)
```

#### 4.3 How Path Flows: Topological → MPPI

```
path_planner.py
  → publishes /global_reference_path (nav_msgs/Path, dense centerline)
  
Nav2 controller_server (MPPI)
  → reads /global_reference_path via follow_path action
  → publishes /cmd_vel

bt_navigator
  → orchestrates: sends FollowPath action to controller_server
  → monitors progress
```

**Critical wiring:** MPPI's `PathFollowCritic` and `PathAlignCritic` naturally consume `nav_msgs/Path`. The topological path becomes MPPI's reference trajectory — it deforms around obstacles while staying aligned with the centerline geometry.

#### 4.4 Update `localization_launch.py` to include Nav2 Controller

```python
# Add to localization_launch.py (Phase 1 file):
# Nav2 with MPPI — starts at t=10s after AMCL is settled
nav2_controller = TimerAction(period=10.0, actions=[
    Node(package='nav2_controller', executable='controller_server', ...),
    Node(package='nav2_behaviors', executable='behavior_server', ...),
    Node(package='nav2_bt_navigator', executable='bt_navigator', ...),
    Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
         parameters=[{'node_names': ['controller_server', 
                                     'behavior_server', 
                                     'bt_navigator']}]),
])
```

---

### PHASE 5 — Behavior Tree Orchestration (Weeks 10–13)
**Goal:** Replace if-else state management with a maintainable `py_trees` Behavior Tree.

#### 5.1 Install `py_trees`

```bash
pip3 install py-trees
sudo apt install ros-jazzy-py-trees-ros ros-jazzy-py-trees-ros-interfaces
```

#### 5.2 Create `bt_manager.py` [NEW FILE]

**File:** `src/agv_navigation/agv_navigation/bt_manager.py`

**Tree Structure:**
```
Root (Sequence)
├── Condition: Is AMCL localized?   (check /amcl_pose covariance < threshold)
├── Condition: Has goal been set?   (check /goal_node topic)
├── Action: Plan path               (call path_planner /goal_node)
├── Selector (navigation or recovery)
│   ├── Sequence (normal navigation)
│   │   ├── Action: FollowPath (MPPI via Nav2 controller)
│   │   └── Action: Check Arrival (distance to goal < 0.3m)
│   └── Sequence (recovery — TEB failed or path blocked)
│       ├── Action: TurnOut    (rotate ±60° from obstacle, P-ctrl on IMU yaw)
│       ├── Action: DrivePast  (drive 1.2m forward, check via odometry)
│       └── Action: BlindMerge (re-engage path_planner, ignore camera)
└── Action: Mission Complete        (publish status, stop robot)
```

**Key BT node implementations:**
```python
class IsAmclLocalized(py_trees.behaviour.Behaviour):
    """Returns SUCCESS when AMCL covariance diagonal < 0.1"""
    def update(self):
        cov = self.amcl_pose.pose.covariance
        if cov[0] < 0.1 and cov[7] < 0.1:  # xx and yy variance
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

class TurnOutRecovery(py_trees.behaviour.Behaviour):
    """Stage 1: Rotate ±60° using IMU yaw feedback (P-controller)"""
    def initialise(self):
        self.target_yaw = self.current_imu_yaw + (self.turn_dir * math.radians(60))
    def update(self):
        error = self.target_yaw - self.current_imu_yaw
        if abs(error) < 0.05:
            return py_trees.common.Status.SUCCESS
        self.publish_twist(0.0, 1.0 * error)
        return py_trees.common.Status.RUNNING

class DrivePastRecovery(py_trees.behaviour.Behaviour):
    """Stage 2: Drive 1.2m forward using odometry distance check"""
    def initialise(self):
        self.start_x = self.current_x
        self.start_y = self.current_y
    def update(self):
        dist = math.sqrt((self.current_x-self.start_x)**2 + 
                         (self.current_y-self.start_y)**2)
        if dist >= 1.2:
            return py_trees.common.Status.SUCCESS
        self.publish_twist(0.3, 0.0)
        return py_trees.common.Status.RUNNING
```

#### 5.3 BT Tick Rate and ROS Integration

```python
# In bt_manager.py main():
tree = py_trees_ros.trees.BehaviourTree(root=build_tree())
tree.setup(timeout=15)
tree.tick_tock(period_ms=100)  # 10 Hz BT tick rate
rclpy.spin(tree.node)
```

---

### PHASE 6 — FastAPI + WebSocket Mobile Bridge (Weeks 13–16)
**Goal:** Create a new `agv_api` ROS 2 package hosting a FastAPI server that bridges the Android app to the ROS 2 network.

#### 6.1 Create new package `agv_api` [NEW PACKAGE]

```
src/agv_api/
├── package.xml
├── setup.py
└── agv_api/
    ├── __init__.py
    └── api_server.py       ← FastAPI + WebSocket + ROS 2 node
```

```bash
pip3 install fastapi uvicorn websockets
```

#### 6.2 `api_server.py` Structure [NEW FILE]

```python
import asyncio
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from fastapi import FastAPI, WebSocket
import uvicorn
import threading
import json

app = FastAPI()

class ApiNode(Node):
    def __init__(self):
        super().__init__('agv_api_server')
        # Publish goal to path_planner
        self.goal_pub = self.create_publisher(String, '/goal_node', 10)
        # Subscribe to telemetry
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        self.path_status_sub = self.create_subscription(
            String, '/path_status', self.status_callback, 10)
        
        self.latest_pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self.latest_status = "IDLE"

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    while True:
        # Receive command from Android
        data = await websocket.receive_text()
        command = json.loads(data)
        
        if "goal" in command:
            # Forward goal to ROS 2
            msg = String()
            msg.data = command["goal"]  # e.g., "N5"
            api_node.goal_pub.publish(msg)
        
        # Send telemetry back every 500ms
        telemetry = {
            "x": api_node.latest_pose["x"],
            "y": api_node.latest_pose["y"],
            "status": api_node.latest_status
        }
        await websocket.send_text(json.dumps(telemetry))
        await asyncio.sleep(0.5)

@app.get("/health")
async def health():
    return {"status": "ok", "node": "agv_api_server"}
```

---

## Part 4 — Complete File Change Matrix

### Files to CREATE (in order of dependency)

| Priority | File | Phase | Purpose |
|---|---|---|---|
| 1 | `launch/mapping_session.launch.py` | 1 | Clean map-once workflow |
| 2 | `config/localization_params.yaml` | 1 | AMCL for post-mapping mode |
| 3 | `launch/localization_launch.py` | 1 | Runtime navigation launch |
| 4 | `config/nav2_params_localization.yaml` | 4 | MPPI + no grid planner |
| 5 | `agv_navigation/graph_extractor.py` | 2 | Auto graph from .pgm |
| 6 | `agv_navigation/path_planner.py` | 3 | Dijkstra + centerline publisher |
| 7 | `agv_navigation/bt_manager.py` | 5 | py_trees orchestrator |
| 8 | `src/agv_api/` (full package) | 6 | FastAPI/WebSocket bridge |

### Files to MODIFY

| File | Phase | Change |
|---|---|---|
| `launch/sim_only.launch.py` | 1 | ✅ Already fixed (SLAM lifecycle) |
| `agv_navigation/setup.py` | 2 | Add graph_extractor, path_planner, bt_manager |
| `agv_navigation/graph_visualizer.py` | 2 | Re-read JSON each tick, visualize centerlines as line strips |
| `agv_navigation/path_tracer.py` | 1 | Fix `/imu` → `/imu/data` topic bug |
| `launch/navigation_launch.py` | 4 | Wire to localization_launch + MPPI |

### Files to RETIRE (keep but disable from entry points)

| File | Reason |
|---|---|
| `agv_navigation/map_builder.py` | Replaced by `graph_extractor.py` |
| `agv_navigation/route_runner.py` | Replaced by `path_planner.py` + BT |

---

## Part 5 — Data Flow Architecture (Final State)

```
╔══════════════════════════════════════════════════════════════════════════╗
║                        GAZEBO HARMONIC                                    ║
║  warehouse.world → warehouse_agv.urdf spawned                            ║
║  LiDAR→/scan  IMU→/imu/data  Encoders→/odom  Camera→/camera/image_raw  ║
╚════════════════════════════╤═════════════════════════════════════════════╝
                             │ ros_gz_bridge (bridge.yaml)
╔════════════════════════════▼═════════════════════════════════════════════╗
║                         ROS 2 LAYER                                       ║
║                                                                           ║
║  [Sensor Fusion]                                                          ║
║  /odom + /imu/data → [ekf_node] → /odometry/filtered                    ║
║                                                                           ║
║  [Localization] — "Map Once, Localize Only"                              ║
║  /scan + /map → [amcl] → map→odom TF  (drift-free pose!)                ║
║  tf2: map → odom → base_link  ← path_planner reads this                 ║
║                                                                           ║
║  [Topological Global Planner]                                             ║
║  /goal_node → [path_planner.py] → Dijkstra → concatenate centerlines    ║
║             → /global_reference_path (nav_msgs/Path, ~500 dense points) ║
║                                                                           ║
║  [Local Planner — MPPI]                                                   ║
║  /global_reference_path + /scan + local_costmap                          ║
║  → [controller_server/MPPI] → /cmd_vel                                   ║
║                                                                           ║
║  [Behavior Tree Orchestrator]                                             ║
║  [bt_manager.py] ─ monitors: /amcl_pose, /path_status, /obstacle_alert  ║
║                 ─ controls:  /goal_node, recovery actions, /cmd_vel      ║
║                 ─ recovery:  TurnOut → DrivePast → BlindMerge           ║
║                                                                           ║
║  [Vision]                                                                 ║
║  /camera/image_raw → [obstacle_detector] → /obstacle_alert              ║
║                                          → /camera/vision_debug          ║
║                                                                           ║
║  [Mobile Bridge]                                                          ║
║  Android App ←→ WebSocket ←→ [agv_api FastAPI] ←→ /goal_node, /amcl_pose║
╚══════════════════════════════════════════════════════════════════════════╝
```

---

## Part 6 — Launch Workflow After All Phases Complete

```
SESSION TYPE 1: MAP THE WAREHOUSE (run once)
─────────────────────────────────────────────
Terminal 1:  ros2 launch agv_description mapping_session.launch.py
Terminal 2:  ros2 run teleop_twist_keyboard teleop_twist_keyboard
             [drive robot through all corridors]
Terminal 2:  ros2 run nav2_map_server map_saver_cli -f ~/maps/warehouse_map

SESSION TYPE 2: EXTRACT THE GRAPH (run once after mapping)
────────────────────────────────────────────────────────────
Terminal 1:  python3 graph_extractor.py \
               --map ~/maps/warehouse_map.pgm \
               --yaml ~/maps/warehouse_map.yaml \
               --output ~/AMR/AMR-main/src/agv_navigation/maps/warehouse_graph.json
             [inspect output in RViz via graph_visualizer]

SESSION TYPE 3: OPERATIONAL NAVIGATION (daily use)
────────────────────────────────────────────────────
Terminal 1:  ros2 launch agv_description localization_launch.py \
               map:=~/maps/warehouse_map.yaml
             [wait for "All nodes active"]
Terminal 2:  ros2 run agv_navigation path_planner
Terminal 3:  ros2 run agv_navigation bt_manager
Terminal 4:  ros2 run agv_api api_server       ← starts FastAPI
             [Android app connects on port 8000]
```

---

## Part 7 — Critical Technical Risks & Mitigations

| Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|
| TEB not available in Jazzy apt repos | High | Medium | Use MPPI first (native Nav2), TEB from source as upgrade |
| Skeletonization creates noisy micro-branches | Medium | High | Prune branches < 20 pixels; use `skimage.morphology.remove_small_objects` |
| AMCL kidnap (wrong initial pose) | Medium | High | Set `initial_pose` in params + publish `/initialpose` via RViz 2D Pose Estimate |
| MPPI doesn't track /global_reference_path natively | Low | High | MPPI PathFollowCritic accepts nav_msgs/Path — this is its primary use case |
| py_trees 2.x vs 3.x API breaking changes | Medium | Low | Pin `py-trees==2.2.3` in requirements.txt |
| FastAPI WebSocket blocks ROS spin | High | High | Run FastAPI in a separate thread/asyncio event loop; use `rclpy.spin_once` in async loop |
| Graph extraction fails on cluttered maps | Medium | Medium | Add map pre-processing: morphological closing to fill small gaps before skeletonize |

---

## Part 8 — Verification Plan Per Phase

### Phase 1
```bash
ros2 run tf2_ros tf2_echo map base_link        # AMCL must publish
ros2 topic hz /amcl_pose                       # Must be ~2 Hz
ros2 topic echo /map --once                    # Static map must load
```

### Phase 2
```bash
# Run graph_extractor, then:
ros2 run agv_navigation graph_visualizer       # See nodes in RViz
# Verify node world positions match physical warehouse layout
```

### Phase 3
```bash
ros2 topic echo /global_reference_path --once  # Must have >100 PoseStamped
ros2 run tf2_ros tf2_echo map base_link        # path_planner must use map frame
# In RViz: add Path display → /global_reference_path → should show corridor trace
```

### Phase 4
```bash
ros2 topic hz /cmd_vel                         # MPPI must publish ~20 Hz
ros2 topic echo /path_status                   # Must show EXECUTING
# In Gazebo: robot must drive smoothly along corridor without stop-and-rotate
```

### Phase 5
```bash
ros2 run py_trees_ros_viewer py_trees_ros_viewer  # Visual BT monitor
# Trigger a blocked path → verify recovery sequence executes
```

### Phase 6
```bash
curl http://localhost:8000/health              # FastAPI must respond
# Connect Android to ws://robot-ip:8000/ws
# Send {"goal": "N3"} → verify robot moves to N3
```

---

## Part 9 — 6-Month Timeline

```
Weeks 1–2  : Phase 1 — Mapping session, AMCL localization, verify TF tree
Weeks 3–5  : Phase 2 — graph_extractor.py, centerline storage, RViz validation
Weeks 5–7  : Phase 3 — path_planner.py, /global_reference_path, tf2 integration
Weeks 7–10 : Phase 4 — MPPI wiring, Nav2 params, smooth driving validation
Weeks 10–13: Phase 5 — py_trees BT, recovery behaviors, state machine
Weeks 13–16: Phase 6 — FastAPI bridge, WebSocket, Android integration
Weeks 17–20: Integration testing, edge-case handling, performance profiling
Weeks 21–24: Raspberry Pi 5 port, hardware deployment, final demo
```
