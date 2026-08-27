# Warehouse AGV — Autonomous Mobile Robot

A fully simulated **Autonomous Mobile Robot (AMR)** for warehouse navigation, built on **ROS 2 Jazzy Jalisco** and **Gazebo Harmonic**. The system performs two distinct phases: autonomous SLAM mapping of the warehouse, followed by production navigation using a custom **Dijkstra + MPPI** controller.

> **Platform:** Ubuntu 24.04 · ROS 2 Jazzy · Gazebo Harmonic

---

## ✨ Key Features

- **Custom MPPI Controller** — Model Predictive Path Integral local planner running at 10 Hz; owns `/cmd_vel` exclusively (no DWB conflict)
- **Dijkstra Global Planner** — Routes over a pre-extracted topological graph with dense 0.3 m waypoint interpolation
- **AMCL Localisation** — Monte Carlo particle filter for robust map-to-robot pose estimation
- **EKF Sensor Fusion** — Fuses wheel odometry (`/odom`) and IMU (`/imu/data`) into `/odometry/filtered` at 50 Hz
- **SLAM Mapping** — Autonomous frontier exploration via `explore_lite` + SLAM Toolbox; produces a full occupancy grid
- **Graph Extraction** — Converts the occupancy grid into a traversable JSON topological graph using distance-transform + line-of-sight checks
- **RViz Graph Overlay** — Live visualisation of graph nodes, edges, and the active planned path as RViz markers

---

## 🤖 Robot Specifications

| Property | Value |
|---|---|
| Base geometry | Cylinder, radius 0.15 m, height 0.12 m |
| Total mass | ~15 kg |
| Drive type | Differential drive (2 driven wheels + 2 caster wheels) |
| Wheel radius | 0.05 m |
| Wheel separation | 0.34 m |
| Sensors | 2D LiDAR (`/scan`), IMU (`/imu/data`), Camera (`/camera/image_raw`) |
| Max linear velocity | 0.8 m/s |
| Max angular velocity | 1.8 rad/s |

---

## 📦 Package Overview

```
src/
├── agv_description/     URDF, worlds, launch files, configs, maps
├── agv_navigation/      Route runner (Dijkstra + MPPI) and graph visualiser
├── agv_vision/          Camera-based obstacle detector
└── m-explore-ros2/      explore_lite — frontier exploration (mapping phase)
```

### `agv_description`
The simulation and configuration hub. Contains everything needed to spawn the robot and run the full navigation stack.

| Path | Contents |
|---|---|
| `urdf/warehouse_agv.urdf` | Full robot description: base, wheels, casters, LiDAR, IMU, camera |
| `worlds/warehouse.world` | Gazebo Harmonic warehouse world with aisles and shelving |
| `launch/navigation_launch.py` | **Production launch** — Gazebo + AMCL localisation + RViz |
| `launch/gazebo.launch.py` | Sim stack only: Gazebo + Bridge + RSP + EKF |
| `launch/mapping_session.launch.py` | Phase 1 mapping: Gazebo + SLAM + Nav2 + explore_lite |
| `config/nav2_params.yaml` | AMCL, map server parameters (navigation mode) |
| `config/nav2_params_explore.yaml` | Nav2 parameters for autonomous exploration mode |
| `config/ekf.yaml` | EKF fusing `/odom` (vx, vy, vyaw) + `/imu/data` (yaw, vyaw) |
| `config/bridge.yaml` | Gazebo ↔ ROS 2 topic bridge: `/scan`, `/odom`, `/cmd_vel`, `/imu/data`, `/tf`, `/clock` |
| `config/mapper_params.yaml` | SLAM Toolbox online-async parameters |
| `config/agv_nav.rviz` | RViz config for navigation session |
| `config/agv_explore.rviz` | RViz config for mapping session |
| `maps/warehouse_map.pgm` | Saved occupancy grid map (329×275 px @ 0.05 m/px) |
| `maps/warehouse_map.yaml` | Map metadata (resolution, origin) |
| `maps/warehouse_graph.json` | Topological graph (~300+ nodes, bidirectional edges) |
| `maps/graph_extractor.py` | Offline tool: generates `warehouse_graph.json` from the occupancy grid |
| `maps/graph_visualization.png` | Visual overlay of the extracted graph on the map |
| `scripts/save_map.sh` | Helper: saves the SLAM map to disk during/after mapping |

### `agv_navigation`

| File | Role |
|---|---|
| `route_runner.py` | Core navigation node. Dijkstra global planner + MPPI local controller. Subscribes to `/goal_pose` (RViz 2D Goal Pose), locates robot via `map → base_link` TF (AMCL), finds the Dijkstra path, densifies it to 0.3 m waypoints, and runs an 80-sample MPPI controller at 10 Hz. |
| `graph_visualizer.py` | RViz overlay node. Publishes the full graph (yellow spheres = nodes, cyan lines = edges) and the active planned path (magenta line + orange dots) to `/agv_graph_markers` at 2 Hz. |

### `agv_vision`

| File | Role |
|---|---|
| `obstacle_detector.py` | Camera-based colour detection node. Subscribes to `/camera/image_raw`, detects red obstacles via HSV thresholding, and publishes bounding-box offset to `/obstacle_alert`. |

---

## 🏗️ Architecture

```
┌──────────────────────────────────────────────────────────┐
│                    Gazebo Harmonic                        │
│  warehouse.world  →  warehouse_agv robot                  │
│  Sensors: /scan (LiDAR)  /odom  /imu/data  /tf           │
└──────────────────┬───────────────────────────────────────┘
                   │ ros_gz_bridge (bridge.yaml)
                   ▼
┌─────────────────────────────────────────┐
│  robot_state_publisher  (URDF → /tf)    │
│  ekf_node   /odom + /imu → /odometry/filtered            │
└──────────────────┬──────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────┐
│  map_server  (warehouse_map.yaml)        │
│  amcl        /scan + /map → map→odom TF │
│  lifecycle_manager_localization          │
└──────────────────┬──────────────────────┘
                   │  map → base_link TF
                   ▼
┌─────────────────────────────────────────┐
│  route_runner                            │
│  ├─ Dijkstra on warehouse_graph.json     │
│  ├─ Densify: 0.3 m waypoints            │
│  ├─ MPPI: 80 samples × 15-step horizon  │
│  │   costs: dist + heading + CTE + collision             │
│  └─ /cmd_vel → Gazebo → robot moves     │
└──────────────────┬──────────────────────┘
                   │  /agv_dense_path
                   ▼
┌─────────────────────────────────────────┐
│  graph_visualizer → /agv_graph_markers  │
│  (RViz: nodes, edges, active path)      │
└─────────────────────────────────────────┘
```

### Why no Nav2 controller server?
The `controller_server` (DWB) is intentionally **not launched** during navigation. Running DWB alongside `route_runner` caused interleaved `/cmd_vel` commands at 30 Hz — the primary cause of the circling bug. `route_runner`'s MPPI controller is the **sole owner** of `/cmd_vel`.

---

## 🛠️ Build & Installation

### Prerequisites

- Ubuntu 24.04
- [ROS 2 Jazzy Jalisco](https://docs.ros.org/en/jazzy/Installation.html)
- Gazebo Harmonic (`ros-jazzy-ros-gz`)
- Nav2 (`ros-jazzy-navigation2`)
- SLAM Toolbox (`ros-jazzy-slam-toolbox`)
- robot_localization (`ros-jazzy-robot-localization`)

```bash
sudo apt install \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-robot-localization \
  ros-jazzy-ros-gz \
  python3-opencv
```

### Clone & Build

```bash
git clone https://github.com/RahulK27-pro/AMR.git
cd AMR/AMR-main
colcon build
source install/setup.bash
```

> **Note:** Source `install/setup.bash` in **every** new terminal before running any ROS 2 commands.

---

## 🚀 Running the System

The project has two phases. The map is already saved — you only need Phase 2 for day-to-day use.

---

### Phase 2 — Navigation (day-to-day use)

Run these in separate terminals, all from the workspace root:

```bash
# Build & source first (once per terminal)
cd ~/AMR/AMR-main
source install/setup.bash
```

**Terminal 1 — Simulation + Localisation + RViz**
```bash
ros2 launch agv_description navigation_launch.py
```
Wait for the message: `Managed nodes are active`

**Terminal 2 — Set initial pose in RViz**
In the RViz window, click **"2D Pose Estimate"** and click on the robot's approximate starting location on the map.
Wait for: `AMCL: initialPoseReceived`

**Terminal 3 — Graph Visualiser** *(optional, recommended)*
```bash
ros2 run agv_navigation graph_visualizer
```
In RViz, add a **MarkerArray** display subscribed to `/agv_graph_markers` to see the full navigation graph.

**Terminal 4 — Route Runner**
```bash
ros2 run agv_navigation route_runner
```
Wait for: `Localization Active! Received map -> base_link TF transform.`

**Sending a Goal**
1. In RViz, click **"2D Goal Pose"**
2. Click anywhere on the map to set a destination
3. Watch `route_runner` snap to the nearest graph node and begin navigating

Expected output:
```
[route_runner]: Received RViz Goal: (-1.75, 3.96)
[route_runner]: Snapping to nodes: Start=N62, Target=N12
[route_runner]: DIJKSTRA PATH: ['N62', 'N44', 'N28', ...] → Densified to 13 waypoints
[route_runner]: FINAL DESTINATION REACHED! Mission Complete.
```

---

### Phase 1 — Autonomous Mapping *(only needed to re-map)*

```bash
ros2 launch agv_description mapping_session.launch.py
```

The launch sequence (fully timed, no manual steps):

| Time | Event |
|---|---|
| t=0s | Gazebo + Bridge + EKF start |
| t=3s | Robot State Publisher |
| t=12s | SLAM Toolbox (lifecycle-managed) |
| t=13s | RViz2 |
| t=20s | Nav2 (controller, planner, BT navigator) |
| t=45s | explore_lite frontier explorer — robot starts mapping |

When the warehouse is fully explored, **save the map** in a new terminal:
```bash
bash ~/AMR/AMR-main/src/agv_description/scripts/save_map.sh
```

**Regenerate the topological graph** after saving the map:
```bash
cd ~/AMR/AMR-main/src/agv_description/maps
python3 graph_extractor.py warehouse_map.yaml
```
This outputs `warehouse_graph.json` and `graph_visualization.png`.

---

### Manual Teleoperation

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Keys: `i`=forward · `,`=reverse · `j`/`l`=turn · `k`/space=stop

---

## ⚙️ Key Configuration

### MPPI Tuning (`route_runner.py`)

| Parameter | Value | Description |
|---|---|---|
| `v_max` | 0.8 m/s | Max linear velocity |
| `w_max` | 1.8 rad/s | Max angular velocity |
| `horizon` | 15 steps | Prediction horizon (1.5 s) |
| `num_samples` | 80 | Trajectory samples per iteration |
| `dt` | 0.1 s | Control period (10 Hz) |
| `w_dist` | 4.0 | Terminal distance cost weight |
| `w_heading` | 3.0 | Heading alignment cost weight |
| `w_cross_track` | 6.0 | Cross-track error cost weight |
| `w_collision` | 5000.0 | Collision penalty |
| `collision_radius` | 0.30 m | Safety clearance from obstacles |
| `lookahead_dist` | 1.2 m | Arc-length lookahead distance |

### EKF Sensor Fusion (`ekf.yaml`)

| Source | Fused signals |
|---|---|
| `/odom` | vx, vy, vyaw (velocity only) |
| `/imu/data` | yaw, vyaw (absolute heading + rate) |
| Output | `/odometry/filtered` at 50 Hz |

---

## 🗺️ ROS 2 Topic Map

| Topic | Type | Direction |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | Gazebo → ROS |
| `/odom` | `nav_msgs/Odometry` | Gazebo → ROS |
| `/imu/data` | `sensor_msgs/Imu` | Gazebo → ROS |
| `/odometry/filtered` | `nav_msgs/Odometry` | EKF output |
| `/cmd_vel` | `geometry_msgs/Twist` | route_runner → Gazebo |
| `/goal_pose` | `geometry_msgs/PoseStamped` | RViz → route_runner |
| `/agv_dense_path` | `nav_msgs/Path` | route_runner → graph_visualizer |
| `/agv_graph_markers` | `visualization_msgs/MarkerArray` | graph_visualizer → RViz |
| `/map` | `nav_msgs/OccupancyGrid` | map_server → AMCL |
| `/particle_cloud` | `nav2_msgs/ParticleCloud` | AMCL → RViz |

---

## 🔧 Troubleshooting

**AMCL keeps warning "cannot publish a pose"**
→ You haven't set an initial pose. Use **"2D Pose Estimate"** in RViz to click the robot's location on the map.

**`route_runner` says "Waiting for localization/odometry"**
→ AMCL TF (`map → base_link`) is not yet available. Ensure `navigation_launch.py` is fully up and initial pose is set.

**Robot spins or circles**
→ Tune `w_heading` and `w_cross_track` in `route_runner.py`. Increasing `w_cross_track` (currently 6.0) makes the robot track the path more strictly.

**EKF "Failed to meet update rate"**
→ System is under load. Try reducing `frequency` in `ekf.yaml` from 50 Hz to 30 Hz, or close other CPU-intensive processes.

**`graph_extractor.py` fails**
→ Ensure dependencies: `sudo apt install python3-opencv python3-yaml`

**`/particle_cloud` QoS incompatible warning in RViz**
→ Cosmetic warning only. RViz's default QoS for particle cloud doesn't match AMCL's reliable QoS. Navigation works correctly regardless.

---

## 📁 Repository Structure

```
AMR-main/
├── README.md
├── docs/                                  # Project documentation
│   ├── Running_Guide.md
│   ├── Phase1_Mapping.md
│   ├── Phase2_Node_Extraction.md
│   ├── Phase3_Navigation.md
│   ├── Evaluation_Results.md
│   ├── implementation.md
│   └── project_migration_and_troubleshooting.md
└── src/
    ├── agv_description/
    │   ├── config/
    │   │   ├── agv_explore.rviz
    │   │   ├── agv_nav.rviz
    │   │   ├── bridge.yaml
    │   │   ├── ekf.yaml
    │   │   ├── mapper_params.yaml
    │   │   ├── nav2_params.yaml
    │   │   └── nav2_params_explore.yaml
    │   ├── launch/
    │   │   ├── gazebo.launch.py
    │   │   ├── mapping_session.launch.py
    │   │   └── navigation_launch.py
    │   ├── maps/
    │   │   ├── graph_extractor.py
    │   │   ├── graph_visualization.png
    │   │   ├── warehouse_graph.json
    │   │   ├── warehouse_map.pgm
    │   │   └── warehouse_map.yaml
    │   ├── scripts/
    │   │   └── save_map.sh
    │   ├── urdf/
    │   │   └── warehouse_agv.urdf
    │   └── worlds/
    │       └── warehouse.world
    ├── agv_navigation/
    │   └── agv_navigation/
    │       ├── route_runner.py
    │       └── graph_visualizer.py
    ├── agv_vision/
    │   └── agv_vision/
    │       └── obstacle_detector.py
    └── m-explore-ros2/                    # explore_lite (mapping phase)
```
