# 02. Simulation Workflow (Build, Gazebo, SLAM, Graph & Navigation)

> **Navigation:** [Root README](../../README.md) · [01. Directory & Scripts](01-directory-and-scripts.md) · **02. Simulation Workflow** · [03. Simulation Errors & Fixes](03-simulation-errors-and-fixes.md) · [04. Hardware Setup & Pi](04-hardware-setup-and-pi-workflow.md) · [05. Hardware Mapping & Nav](05-hardware-mapping-and-navigation.md) · [06. Web Dashboard, Bridge & ML](06-web-dashboard-bridge-and-ml.md)

This guide covers building the ROS 2 Jazzy workspace, running Gazebo Harmonic worlds, executing manual or autonomous `explore_lite` SLAM mapping, extracting topological roadmaps, running Nav2 localization with `route_runner` (Dijkstra + 10 Hz MPPI) and `bt_manager`, spawning dynamic obstacles, and attaching visualization GUIs.

---

## 1. Prerequisites & Building the Workspace

The repository targets **Ubuntu 24.04 LTS**, **ROS 2 Jazzy**, and **Gazebo Harmonic**. [`app/COLCON_IGNORE`](../../app/COLCON_IGNORE) prevents `colcon` from scanning `app/`.

```bash
cd ~/AMR/AMR-main
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

For faster incremental rebuilds after editing launch/config/Python files:

```bash
colcon build --packages-select agv_description agv_navigation --symlink-install
source install/setup.bash
```

Quick dependency check for `route_runner` and `bt_manager`:

```bash
python3 -c "import numpy, scipy, yaml, PIL, py_trees; from scipy.spatial import KDTree; print('OK')"
```

> [!IMPORTANT]
> Always run `source /opt/ros/jazzy/setup.bash` and `source ~/AMR/AMR-main/install/setup.bash` in **every new terminal** before invoking ROS 2 commands.

---

## 2. Gazebo Worlds & Base Simulation (`gazebo.launch.py`)

Three SDF 1.8 worlds live in [`src/agv_description/worlds/`](../../src/agv_description/worlds/):

| World File | `<world name>` | Characteristics | Paired Default Map |
| :--- | :--- | :--- | :--- |
| [`test1.world`](../../src/agv_description/worlds/test1.world) | `test1` | Enclosed multi-room/corridor mapping testbed (`mapping_session.launch.py` default) | — |
| [`hospital.world`](../../src/agv_description/worlds/hospital.world) | `hospital` | ~17 m × 13 m multi-room hospital layout with central corridor (`navigation_launch.py` default) | `hospital_map` |
| [`warehouse.world`](../../src/agv_description/worlds/warehouse.world) | `warehouse` | Industrial warehouse aisles, racks, and walls (`gazebo.launch.py` default) | `warehouse_map` |

[`gazebo.launch.py`](../../src/agv_description/launch/gazebo.launch.py) launches:
1. `gz sim [-s --headless-rendering] -r -v 2 <world_file>` with `LIBGL_ALWAYS_SOFTWARE=1`, `MESA_GL_VERSION_OVERRIDE=3.3`, `GZ_IP=127.0.0.1`.
2. `ros_gz_bridge` (`parameter_bridge`) using [`src/agv_description/config/bridge.yaml`](../../src/agv_description/config/bridge.yaml) (`/clock`, `/odom`, `/cmd_vel`, `/scan`, `/joint_states`, `/imu/data`, `/dynamic_obstacle/cmd_vel`, `/dynamic_obstacle/odom`).
3. `robot_state_publisher` (`warehouse_agv.urdf`, `use_sim_time: True`).
4. `robot_localization` `ekf_node` using [`src/agv_description/config/ekf.yaml`](../../src/agv_description/config/ekf.yaml) (50 Hz fusion of `/odom` + `/imu/data` → `/odometry/filtered` + `odom -> base_link` TF).
5. Delayed (`2.0 s`) `ros_gz_sim create` spawning `warehouse_agv` at `z = 0.05`.

| Launch Argument | Default | Description |
| :--- | :--- | :--- |
| `world` | `warehouse.world` | World name (`warehouse`, `hospital.world`, `test1.world`) or absolute path |
| `headless` | `false` | `true` passes `-s --headless-rendering` to `gz sim` |

```bash
ros2 launch agv_description gazebo.launch.py world:=hospital.world headless:=false
```

---

## 3. Simulation SLAM Mapping & Frontier Exploration

[`src/agv_description/launch/mapping_session.launch.py`](../../src/agv_description/launch/mapping_session.launch.py) enforces `ROS_LOCALHOST_ONLY=1` and boots a timed SLAM + Nav2 + `explore_lite` session:

| Argument | Default | Description |
| :--- | :--- | :--- |
| `world` | `test1.world` | Target Gazebo world (`test1.world`, `hospital.world`, `warehouse.world`, or full path) |
| `run_sim` | `true` | Launch Gazebo (`true`) or attach to an already running Gazebo (`false`) |
| `run_explore` | `false` | `true` starts `explore_lite` autonomous frontier explorer; `false` leaves robot for manual teleop |
| `headless` | `false` | `true` disables Gazebo GUI (`-s --headless-rendering`) |
| `use_rviz` | `false` | `true` launches RViz2 with [`agv_explore.rviz`](../../src/agv_description/config/agv_explore.rviz) |

### Timed Boot Sequence inside `mapping_session.launch.py`
- **`t = 0s`**: `gz sim` (if `run_sim:=true`), `ros_gz_bridge`, `ekf_node`
- **`t = 1s`**: `robot_state_publisher`
- **`t = 3s`**: `ros_gz_sim create` spawns `warehouse_agv`
- **`t = 4s`**: `rviz2` (if `use_rviz:=true`)
- **`t = 5s`**: `slam_toolbox` (`online_async_launch.py` with [`mapper_params.yaml`](../../src/agv_description/config/mapper_params.yaml))
- **`t = 15s`**: Nav2 exploration nodes (`controller_server` MPPI `vx_max: 0.65`, `planner_server` Navfn `allow_unknown: true`, `behavior_server`, `bt_navigator`, `waypoint_follower`, `lifecycle_manager_navigation` with `bond_timeout: 40.0`) configured by [`nav2_params_explore.yaml`](../../src/agv_description/config/nav2_params_explore.yaml)
- **`t = 25s`**: `explore_lite` (`explore`) node (if `run_explore:=true`)

### Option A — Autonomous Frontier Exploration
```bash
ros2 launch agv_description mapping_session.launch.py \
  world:=test1.world run_explore:=true use_rviz:=true
```

### Option B — Manual Teleop Mapping
```bash
# Terminal 1: Launch SLAM without auto-explore
ros2 launch agv_description mapping_session.launch.py \
  world:=test1.world run_explore:=false use_rviz:=true

# Terminal 2: Drive manually
source ~/AMR/AMR-main/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

You can also pause or resume `explore_lite` mid-run via `/explore/resume`:
```bash
ros2 topic pub --once /explore/resume std_msgs/msg/Bool "data: false"  # pause
ros2 topic pub --once /explore/resume std_msgs/msg/Bool "data: true"   # resume
```

### Saving the Simulation Map
While `mapping_session.launch.py` is still running and `/map` is active, run [`src/agv_description/scripts/save_map.sh`](../../src/agv_description/scripts/save_map.sh):

```bash
# Syntax: bash src/agv_description/scripts/save_map.sh [MAP_NAME] [MAP_DIR]
# Defaults: MAP_NAME=warehouse_map, MAP_DIR=$HOME/maps

# Save directly into the simulation package structured maps/{yaml,pgm}/ directory:
bash ~/AMR/AMR-main/src/agv_description/scripts/save_map.sh \
  my_sim_map ~/AMR/AMR-main/src/agv_description/maps
```

When `<MAP_DIR>` contains `yaml/` and `pgm/` subdirectories, `save_map.sh` automatically saves `<MAP_DIR>/yaml/<name>.yaml`, moves `<name>.pgm` into `<MAP_DIR>/pgm/<name>.pgm`, and rewrites `image: ../pgm/<name>.pgm` inside the YAML.

---

## 4. Topological Graph Extraction

Generate the navigation roadmap JSON and PNG preview from any saved map YAML using [`src/agv_description/maps/graph_extractor.py`](../../src/agv_description/maps/graph_extractor.py):

```bash
python3 ~/AMR/AMR-main/src/agv_description/maps/graph_extractor.py \
  ~/AMR/AMR-main/src/agv_description/maps/yaml/my_sim_map.yaml
```

Optional tuning flags (verified defaults):
- `--robot-radius 0.11` (robot radius in m)
- `--safety-margin 0.09` (wall buffer in m → total `0.20 m` minimum clearance)
- `--search-radius 2.5` (maximum Bresenham line-of-sight edge length in m)
- `--step-corridor 0.40` / `--step-medium 0.55` / `--step-open 0.85` (adaptive sampling spacing)
- `--output <path/to/graph.json>`
- `--sync-default` (also copies output to `warehouse_graph.json`)

Outputs:
- `json/<name>_graph.json` (`nodes` with `id`, `x`, `y`, `px`, `py`, `clearance`; bidirectional `edges` with `from`, `to`, `cost`)
- `png/<name>_graph_vis.png`

---

## 5. Production Navigation (`navigation_launch.py` + `route_runner`)

[`src/agv_description/launch/navigation_launch.py`](../../src/agv_description/launch/navigation_launch.py) launches Gazebo + localization only (`map_server` + `amcl` + `lifecycle_manager_localization` with `bond_timeout: 0.0`) so that **`route_runner` maintains exclusive ownership of `/cmd_vel`** without Nav2 DWB controller contention.

| Argument | Default | Description |
| :--- | :--- | :--- |
| `map` | `hospital_map` | Map stem (e.g. `hospital_map`, `warehouse_map`) or path to `.yaml` |
| `world` | `auto` | `auto` pairs `hospital_map` → `hospital.world` and `warehouse_map` → `warehouse.world` |
| `headless` | `false` | Run Gazebo server without GUI |
| `use_rviz` | `true` | Launch RViz2 with [`agv_nav.rviz`](../../src/agv_description/config/agv_nav.rviz) |

Note that `navigation_launch.py` also copies `<map_stem>_graph.json` to `warehouse_graph.json` automatically, and [`nav2_params.yaml`](../../src/agv_description/config/nav2_params.yaml) sets `set_initial_pose: true` at `(0.0, 0.0, 0.0)` so AMCL initializes immediately on launch.

### Multi-Terminal Startup

**Terminal 1 — Simulation + Map Server + AMCL + RViz2:**
```bash
cd ~/AMR/AMR-main
source install/setup.bash
ros2 launch agv_description navigation_launch.py map:=hospital_map
```

**Terminal 2 — Topological Route Runner (Dijkstra + KD-Tree + 10 Hz MPPI + ML Perception):**
```bash
cd ~/AMR/AMR-main
source install/setup.bash
ros2 run agv_navigation route_runner --ros-args -p map_name:=hospital_map -p use_sim_time:=true
```
Wait for: `Localization Active! Received map -> base_link TF transform.`

**Terminal 3 (Optional) — Graph Visualizer:**
```bash
cd ~/AMR/AMR-main
source install/setup.bash
ros2 run agv_navigation graph_visualizer --ros-args -p map_name:=hospital_map -p use_sim_time:=true
```
Publishes `/agv_graph_markers` (`MarkerArray`: yellow node spheres, cyan edges, magenta dense path).

**Terminal 4 (Optional) — Behavior Tree Mission Orchestrator (`bt_manager`):**
```bash
cd ~/AMR/AMR-main
source install/setup.bash
ros2 run agv_navigation bt_manager --ros-args -p map_name:=hospital_map -p use_sim_time:=true
```
Adds `py_trees` condition guards (`CheckLocalization`, `CheckGoalQueue`) and `BackUpRecovery` (`0.3 m` reverse) + `SpinRecovery` (`60°` spin) when blocked.

### Dispatching Goals via CLI or RViz2
- **RViz2**: Click **2D Goal Pose** and drag on any corridor.
- **Single Goal Topic (`/goal_pose`)**:
  ```bash
  ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
    '{header: {frame_id: "map"}, pose: {position: {x: 3.1, y: 0.0}}}'
  ```
- **Multi-Goal Topological Sequence (`/goal_sequence`)**:
  ```bash
  ros2 topic pub --once /goal_sequence std_msgs/msg/String \
    'data: "[\"N5\", \"N12\", \"N40\"]"'
  ```
- **Inspect Live Topics / Diagnostics**:
  ```bash
  ./scripts/diagnostics/monitor_topics.sh
  python3 scripts/diagnostics/diagnose_system.py
  ```

---

## 6. Dynamic Obstacle Simulation & Stress Testing

[`src/agv_description/launch/dynamic_obstacle.launch.py`](../../src/agv_description/launch/dynamic_obstacle.launch.py) auto-detects the running Gazebo world via `gz service -s /gazebo/worlds` and spawns either a wheeled cart (`models/dynamic_obstacle`) or a pedestrian (`models/human_pedestrian`):

| Argument | Default | Options / Notes |
| :--- | :--- | :--- |
| `model_type` | `cart` | `cart` (`z=0.5`) or `pedestrian` / `human` / `person` (`z=0.01`) |
| `pattern` | `aisle_crossing` | `aisle_crossing`, `corridor_walker`, `doorway_blocker`, `circular_patrol` |
| `speed` | `0.35` | Obstacle speed in m/s |
| `x`, `y`, `z`, `yaw` | `1.8`, `0.0`, `auto`, `1.57` | Initial spawn pose |
| `name` | `dynamic_obstacle_1` | Model entity name in Gazebo |
| `run_controller` | `true` | `true` launches `dynamic_obstacle_manager`; `false` enables manual teleop |
| `world` | `auto` | Gazebo world name or `auto` |

```bash
# Autonomous pedestrian crossing an aisle:
ros2 launch agv_description dynamic_obstacle.launch.py \
  model_type:=pedestrian pattern:=aisle_crossing speed:=0.38 x:=1.6 y:=-0.58

# Doorway blocker (triggers Yield -> Tier-3 Dijkstra detour):
ros2 launch agv_description dynamic_obstacle.launch.py \
  pattern:=doorway_blocker x:=3.0 y:=2.8

# Manual obstacle driving:
ros2 launch agv_description dynamic_obstacle.launch.py run_controller:=false x:=1.5 y:=0.0
ros2 run agv_navigation obstacle_teleop
```

---

## 7. Visualization Helpers (`scripts/sim/`)

When running simulation headlessly (`headless:=true` or from the Web Dashboard), attach or detach GUIs anytime without interrupting Gazebo or navigation:

```bash
# Attach Gazebo Harmonic GUI client (`gz sim -g`) with Mesa llvmpipe env:
./scripts/sim/view_gazebo.sh

# Launch RViz2 (auto-detects nav vs explore, or pass mode explicitly):
./scripts/sim/view_rviz.sh          # auto-detects active SLAM vs navigation
./scripts/sim/view_rviz.sh explore  # agv_explore.rviz (use_sim_time:=true)
./scripts/sim/view_rviz.sh nav      # agv_nav.rviz (use_sim_time:=true)
```
