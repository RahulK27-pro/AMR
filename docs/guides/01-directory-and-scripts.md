# 01. Directory Structure, Map Contract, Scripts & ROS 2 Packages

> **Navigation:** [Root README](../../README.md) · **01. Directory & Scripts** · [02. Simulation Workflow](02-simulation-workflow.md) · [03. Simulation Errors & Fixes](03-simulation-errors-and-fixes.md) · [04. Hardware Setup & Pi](04-hardware-setup-and-pi-workflow.md) · [05. Hardware Mapping & Nav](05-hardware-mapping-and-navigation.md) · [06. Web Dashboard, Bridge & ML](06-web-dashboard-bridge-and-ml.md)

This reference documents every directory in the reorganized repository, the structured `maps/{yaml,pgm,png,json}/` contract, all CLI scripts in [`scripts/`](../../scripts/), and all ROS 2 packages under [`src/`](../../src/).

---

## 1. Top-Level Directory Breakdown

| Path | Tracked / Generated | Purpose |
| :--- | :--- | :--- |
| [`app/`](../../app/) | Tracked (`COLCON_IGNORE`) | Full-stack operator applications: FastAPI + `rclpy` bridge ([`app/bridge/`](../../app/bridge/)), React 18 + Vite dashboard ([`app/web_dashboard/`](../../app/web_dashboard/)), Flutter client ([`app/flutter_app/`](../../app/flutter_app/)), and 11 preserved legacy docs ([`app/docs/`](../../app/docs/)). Ignored by `colcon` via [`app/COLCON_IGNORE`](../../app/COLCON_IGNORE). |
| [`config/`](../../config/) | Tracked | Shared runtime middleware configuration — specifically [`config/cyclonedds.xml`](../../config/cyclonedds.xml) for CycloneDDS Domain 7 unicast peering between Laptop and Pi 5. |
| [`docs/`](../../docs/) | Tracked | Contains 25 preserved legacy/milestone docs directly inside `docs/*.md` plus the modular operational guide suite in [`docs/guides/`](.). |
| [`logs/`](../../logs/) | Gitignored (runtime) | Hardware mapping logs (`hardware_mapping_*.log`), hardware navigation launch logs (`hardware_nav_*.log`), 10 Hz CSV & milestone run logs from `run_logger` (`navigation_run_*.csv`, `navigation_run_*.log`), ML pipeline per-agent logs (`ml_pipeline_*/`), logs pulled from Pi (`logs/pi/`), and convenience symlinks (`latest_launch.log`, `latest_run.csv`, `latest_run.log`). |
| [`maps/`](../../maps/) | Tracked | Workspace-level hardware & user-saved occupancy grids and topological roadmaps organized into `json/`, `pgm/`, `png/`, and `yaml/`. |
| [`ml_datasets/`](../../ml_datasets/) | Gitignored (runtime) | Saved `.npz` training/capture datasets (`synthetic_lidar_dataset.npz`, `gazebo_live_dataset.npz`, `combined_80_20_dataset.npz`, and optional `hardware_*.npz`). |
| [`reports/`](../../reports/) | Tracked | Formal LaTeX source and compiled PDF reports: `AMR_Comprehensive_Technical_Report.{tex,pdf}` and `AMR_Mathematical_Formulations_Report.{tex,pdf}`. |
| [`scripts/`](../../scripts/) | Tracked | Operational bash & Python tooling grouped into `hardware/`, `sim/`, `ml/`, and `diagnostics/`. |
| [`src/`](../../src/) | Tracked | ROS 2 Jazzy workspace packages: [`agv_description`](../../src/agv_description/), [`agv_navigation`](../../src/agv_navigation/), and [`m-explore-ros2`](../../src/m-explore-ros2/). |
| `build/`, `install/`, `log/` | Gitignored (colcon) | Standard `colcon build --symlink-install` build/install/log artifacts. |
| `.venv_ml/` | Gitignored (local venv) | Python virtual environment containing PyTorch and scikit-learn used by `dataset_trainer_agent` / `train_all_models.py` for offline ML training. |

---

## 2. Structured Map Contract (`json/pgm/png/yaml`)

Both hardware maps ([`maps/`](../../maps/)) and simulation maps ([`src/agv_description/maps/`](../../src/agv_description/maps/)) follow the exact same 4-subdirectory layout:

```text
<maps_root>/
├── yaml/<name>.yaml             # Map metadata (always uses relative image: ../pgm/<name>.pgm)
├── pgm/<name>.pgm               # 8-bit trinary occupancy grid (254=free, 0=occupied, 205=unknown)
├── png/<name>.png               # Web-ready floorplan preview (dark or grayscale)
├── png/<name>_graph_vis.png     # Extracted topological roadmap visualization overlay
├── json/<name>_graph.json       # Topological roadmap nodes, bidirectional edges & clearance
├── json/<name>_places.json      # Semantic named stations saved via Web Dashboard
└── json/warehouse_graph.json    # Active synced graph copy read as fallback by route_runner/bt_manager
```

### Key Rules of the Contract
1. **YAML `image:` Path**: Inside `yaml/<name>.yaml`, the `image` field points across sibling folders:
   ```yaml
   image: ../pgm/hardware_map.pgm
   mode: trinary
   resolution: 0.0500
   origin: [-2.363863, -2.819677, 0]
   negate: 0
   occupied_thresh: 0.65
   free_thresh: 0.25
   ```
   Both [`scripts/hardware/save_hardware_map.sh`](../../scripts/hardware/save_hardware_map.sh), [`src/agv_description/scripts/save_map.sh`](../../src/agv_description/scripts/save_map.sh), and [`app/bridge/bridge_server.py`](../../app/bridge/bridge_server.py) rewrite `image: ../pgm/<name>.pgm` automatically when saving into structured directories.
2. **Resolution Helper ([`src/agv_navigation/agv_navigation/map_paths.py`](../../src/agv_navigation/agv_navigation/map_paths.py))**:
   - `get_workspace_root()`: Checks `AMR_WS`, then `~/AMR/AMR-main`, then `~/AMR`.
   - `resolve_map_yaml(map_name, fallbacks=('hospital_map', 'warehouse_map'))`: Searches `<ws>/maps/{yaml,}`, `<ws>/src/agv_description/maps/{yaml,}`, and installed `agv_description/maps/{yaml,}` (supporting both structured and legacy flat folders).
   - `resolve_graph_json(map_name, fallbacks=('warehouse_graph', 'warehouse_map_graph', 'warehouse_01_graph'))`: Resolves `<stem>_graph.json` or `<stem>.json`.
   - `resolve_places_json(map_name)`: Resolves `<stem>_places.json`.
   - `resolve_map_image_from_yaml(yaml_path, yaml_meta)`: Resolves relative `image:` (including `../pgm/<stem>.pgm` and sibling `pgm/` / `png/` fallbacks).
   - `iter_map_yamls()`: Enumerates all simulation and hardware YAML maps without duplicates.
3. **Topological Graph Extractor ([`src/agv_description/maps/graph_extractor.py`](../../src/agv_description/maps/graph_extractor.py))**:
   - Thresholds `>= 250` as free space (keeps ROS unknown `205` and occupied `0` non-traversable).
   - Computes $L_2$ Euclidean Distance Transform (`cv2.distanceTransform`), culling points closer than `robot_radius + safety_margin` (default `0.11 + 0.09 = 0.20 m`).
   - Multi-resolution adaptive grid sampling (`--step-corridor 0.40`, `--step-medium 0.55`, `--step-open 0.85`).
   - Injects verified doorway & corridor spine waypoints when a ~17 m × 13 m hospital map is detected.
   - Connects pairs within `--search-radius` (`2.5 m`) after Bresenham line-of-sight clearance checks, prunes isolated nodes, retains the largest connected component, and writes `<name>_graph.json` + `<name>_graph_vis.png`.
   - CLI options:
     ```bash
     python3 src/agv_description/maps/graph_extractor.py <map_yaml> \
       [--robot-radius 0.11] [--safety-margin 0.09] [--search-radius 2.5] \
       [--step-corridor 0.40] [--step-medium 0.55] [--step-open 0.85] \
       [--sync-default] [--output <json_path>]
     ```

---

## 3. Scripts Inventory ([`scripts/`](../../scripts/))

### 3.1 Hardware Scripts (`scripts/hardware/`)

| Script | Where Run | Usage & Arguments | Purpose |
| :--- | :--- | :--- | :--- |
| [`env_hardware.sh`](../../scripts/hardware/env_hardware.sh) | Laptop | `source scripts/hardware/env_hardware.sh [PI_IP]` *(default `10.244.248.208`)* | Sources Jazzy + workspace, auto-detects routed local IP, regenerates `config/cyclonedds.xml` (Domain 7), exports `CYCLONEDDS_URI`, and wraps `ros2 node list` / `topic {list,hz,echo}` with `--no-daemon`. |
| [`start_hardware.sh`](../../scripts/hardware/start_hardware.sh) | Pi 5 | `./scripts/hardware/start_hardware.sh [LAPTOP_IP] [launch_args...]` | Auto-rebuilds `agv_description` if modified, writes `/tmp/cyclonedds_pi.xml` (Domain 7 + loopback `127.0.0.1` + optional `LAPTOP_IP`), and runs `hardware_bringup.launch.py`. |
| [`start_hardware_mapping.sh`](../../scripts/hardware/start_hardware_mapping.sh) | Laptop | `./scripts/hardware/start_hardware_mapping.sh [PI_IP]` *(default `10.244.248.208`)* | Rebuilds `agv_description` if needed, configures Domain 7 CycloneDDS, syncs Pi clock over SSH (`sudo -n date`), kills conflicting `slam_toolbox`/AMCL/static TF nodes on Pi & laptop, checks `/scan` & `/odom`, and launches `hardware_mapping.launch.py` with RViz2. |
| [`start_hardware_nav.sh`](../../scripts/hardware/start_hardware_nav.sh) | Laptop | `./scripts/hardware/start_hardware_nav.sh [PI_IP] [MAP_NAME_OR_YAML] [MAX_SPEED]` *(defaults: `10.244.248.208`, `prakash_room`, `0.13`)* | Rebuilds if needed, configures CycloneDDS, syncs Pi clock, kills lingering SLAM/static TF, tees output to `logs/hardware_nav_<ts>.log` + `logs/latest_launch.log`, and launches `hardware_navigation.launch.py` (`cruise_linear_speed:=0.10`, `max_angular_speed:=0.40`, `lookahead_dist:=0.25`, `use_rviz:=true`). |
| [`save_hardware_map.sh`](../../scripts/hardware/save_hardware_map.sh) | Laptop | `./scripts/hardware/save_hardware_map.sh [MAP_NAME] [OUTPUT_DIR]` *(defaults: `hardware_map`, `$WS_DIR/maps`)* | Checks `/map` publisher, calls `map_saver_cli -f ... --occ 0.65 --free 0.25 --fmt pgm --ros-args -p use_sim_time:=false`, moves `.pgm` to `maps/pgm/`, updates YAML `image: ../pgm/<name>.pgm`, exports `.png`, and runs `< 4`-cell unknown-safe speckle filtering. |
| [`sync_to_pi.sh`](../../scripts/hardware/sync_to_pi.sh) | Laptop | `./scripts/hardware/sync_to_pi.sh [push\|pull\|both] [PI_IP]` *(defaults: `push`, `10.244.248.208`)* | Pings Pi, syncs Pi clock, and runs `rsync`: `push` syncs `src/`, `scripts/hardware/`, `scripts/diagnostics/`, `config/cyclonedds.xml`, `maps/`; `pull` pulls Pi `maps/` and Pi `logs/` into `logs/pi/`. |
| [`teleop_hardware.sh`](../../scripts/hardware/teleop_hardware.sh) | Laptop or Pi | `./scripts/hardware/teleop_hardware.sh` | Configures Domain 7 DDS (uses `/tmp/cyclonedds_pi.xml` on Pi or `env_hardware.sh` on laptop) and launches `teleop_wasd.py`. |
| [`teleop_wasd.py`](../../scripts/hardware/teleop_wasd.py) | Laptop or Pi | `python3 scripts/hardware/teleop_wasd.py` | Hold-to-move WASD/Arrow teleop (`KEY_TIMEOUT_SEC = 0.22s`) publishing `/cmd_vel` (`MIN_LINEAR=0.04`, `MAX_LINEAR=0.45`, `DEFAULT_LINEAR=0.18`, `DEFAULT_ANGULAR=0.80`, presets `1`/`2`/`3`, Space/X E-Stop). |
| [`verify_hardware_topics.sh`](../../scripts/hardware/verify_hardware_topics.sh) | Pi (or DDS host) | `./scripts/hardware/verify_hardware_topics.sh` | Lists Domain 7 topics and samples `ros2 topic hz` for 6 seconds on `/odom`, `/imu/data`, `/odometry/filtered`, and `/scan`. |
| [`connect_laptop_rviz.sh`](../../scripts/hardware/connect_laptop_rviz.sh) | Laptop | `./scripts/hardware/connect_laptop_rviz.sh [PI_IP]` *(default `10.244.248.208`)* | Configures CycloneDDS unicast peer to Pi, launches fallback `map -> odom` static TF, and opens RViz2 with `agv_hardware.rviz` (`Fixed Frame: odom`). |
| [`generate_hardware_graph.py`](../../scripts/hardware/generate_hardware_graph.py) | Laptop | `python3 scripts/hardware/generate_hardware_graph.py [MAP_NAME_OR_PATH]` *(default `prakash_room`)* | Resolves YAML under `maps/yaml/` or `src/agv_description/maps/yaml/` and runs `extract_graph()` with `robot_radius=0.11`, `safety_margin=0.09`, saving `<stem>_graph.json` and `<stem>_graph_vis.png`. |

### 3.2 Simulation & ML Scripts (`scripts/sim/`, `scripts/ml/`)

| Script | Usage & Arguments | Purpose |
| :--- | :--- | :--- |
| [`scripts/sim/view_gazebo.sh`](../../scripts/sim/view_gazebo.sh) | `./scripts/sim/view_gazebo.sh` | Sets Mesa software rendering env (`LIBGL_ALWAYS_SOFTWARE=1`, `GALLIUM_DRIVER=llvmpipe`, `MESA_GL_VERSION_OVERRIDE=4.5`, `GZ_IP=127.0.0.1`, `ROS_LOCALHOST_ONLY=1`) and opens Gazebo GUI client (`gz sim -g`) attached to a headless sim. |
| [`scripts/sim/view_rviz.sh`](../../scripts/sim/view_rviz.sh) | `./scripts/sim/view_rviz.sh [nav\|explore\|hw_nav\|hw_explore]` | Auto-detects sim vs `AMR_MODE=hardware` (or accepts explicit mode), sets Mesa/DDS env, and launches RViz2 with the matching `.rviz` config and `use_sim_time`. |
| [`scripts/ml/run_automated_ml_pipeline.sh`](../../scripts/ml/run_automated_ml_pipeline.sh) | `./scripts/ml/run_automated_ml_pipeline.sh [--headless] [--duration 95] [--laps 5] [--scenario-sec 16] [--map hospital_map] [--keep-open]` | Rebuilds packages, clears stale sim processes, starts `navigation_launch.py` + `route_runner` + `graph_visualizer` + Agents 1/2/3 (`sim_mission_agent`, `obstacle_director_agent`, `dataset_trainer_agent`), logs to `logs/ml_pipeline_<ts>/`, trains 80/20 models, and triggers hot-reload. |

### 3.3 Diagnostics & Bench Scripts (`scripts/diagnostics/`)

| Script | Usage & Arguments | Purpose |
| :--- | :--- | :--- |
| [`diagnose_mapping.py`](../../scripts/diagnostics/diagnose_mapping.py) | `python3 scripts/diagnostics/diagnose_mapping.py [--snapshot] [--duration 3.0] [--json <path>]` | Live HUD or snapshot analyzer for Pi-laptop clock skew, topic Hz (`/scan`, `/odom`, `/imu/data`, `/odometry/filtered`, `/map`), TF chain, IMU vs wheel odometry yaw sign/ratio, stationary gyro bias, and LiDAR self-hits (`< 0.12 m`). |
| [`diagnose_system.py`](../../scripts/diagnostics/diagnose_system.py) | `python3 scripts/diagnostics/diagnose_system.py` | End-to-end simulation health check: `ROS_DOMAIN_ID`, running processes, 2.0 s topic Hz (`/clock`..`/agv_state`), TF tree (`odom->base_link`, `map->odom`, `map->base_link`), and tail of `/tmp/amr_*.log`. |
| [`monitor_topics.sh`](../../scripts/diagnostics/monitor_topics.sh) & [`monitor_topics.py`](../../scripts/diagnostics/monitor_topics.py) | `./scripts/diagnostics/monitor_topics.sh [--stream]` | Multi-topic terminal telemetry dashboard (in-place HUD or scrolling `--stream`) displaying `/clock`, `/cmd_vel`, `/odometry/filtered`, `/odom`, `/scan` sectors, `/amcl_pose`, `/agv_state`, `/mission_progress`, `/agv_dense_path`, and `/dynamic_obstacle/odom`. |
| [`verify_sensors.py`](../../scripts/diagnostics/verify_sensors.py) | `python3 scripts/diagnostics/verify_sensors.py` | Interactive pre-SLAM validator: LiDAR quadrant check (Front/Left/Right/Back), wheel odom vs IMU yaw sign match, and EKF covariance stability. |
| [`test_3_encoders.py`](../../scripts/diagnostics/test_3_encoders.py) | `python3 scripts/diagnostics/test_3_encoders.py [--invert-left\|--no-invert-left] [--invert-right\|--no-invert-right] [--mock]` | Standalone `lgpio` quadrature counter (`GPIO 17/27`, `GPIO 22/23`) to measure exact 4× ticks and 1× pulses per wheel revolution. |
| [`test_full_hardware.py`](../../scripts/diagnostics/test_full_hardware.py) | `python3 scripts/diagnostics/test_full_hardware.py [--mock] [--skip-motors] [--skip-lidar] [--skip-imu] [--report] [--report-file hardware_test_report.json] [--motor-duty 30] [--motor-time 2.0] [--lidar-port /dev/ttyUSB0]` | 8-stage hardware verification suite: GPIO, I2C bus (`0x68`), MPU IMU, LiDAR serial port, LiDAR 360° scan, static encoders, motor+encoder spin, and synchronized multi-sensor test. |
| [`test_motor_teleop.py`](../../scripts/diagnostics/test_motor_teleop.py) | `python3 scripts/diagnostics/test_motor_teleop.py [--speed 40] [--freq 1000] [--invert-left\|--no-invert-left] [--invert-right\|--no-invert-right] [--watchdog 0.0] [--mock]` | Standalone raw GPIO (`gpiozero`/`lgpio`) hold-to-move bench tester for RMCS-2305 motors (cannot run while `amr_hardware_drive_node.py` holds GPIO). |
| [`test_udp_link.py`](../../scripts/diagnostics/test_udp_link.py) | Laptop: `python3 scripts/diagnostics/test_udp_link.py listen` <br>Pi: `python3 scripts/diagnostics/test_udp_link.py ping <LAPTOP_IP>` | Two-way UDP port `7410` connectivity ping/ACK tester to diagnose Windows Defender Firewall or router isolation blocks. |

---

## 4. ROS 2 Packages Overview ([`src/`](../../src/))

### 4.1 `agv_description` (`ament_cmake`)
Defined in [`src/agv_description/CMakeLists.txt`](../../src/agv_description/CMakeLists.txt) and [`src/agv_description/package.xml`](../../src/agv_description/package.xml):
- **URDF**: [`urdf/warehouse_agv.urdf`](../../src/agv_description/urdf/warehouse_agv.urdf) (`base_link`, `left_wheel`/`right_wheel` `r=0.035m, L=0.240m`, `front_caster`/`rear_caster` `r=0.015m`, `lidar_link` + `laser_frame`, `imu_link`, Gazebo `DiffDrive` with `publish_odom_tf=false`, `gpu_lidar`, `imu`, `JointStatePublisher`).
- **Worlds & Models**: `worlds/{hospital,test1,warehouse}.world`, `models/{dynamic_obstacle,human_pedestrian}/model.{config,sdf}`.
- **Launch Files (`launch/`)**: `gazebo.launch.py`, `mapping_session.launch.py`, `navigation_launch.py`, `dynamic_obstacle.launch.py`, `hardware_bringup.launch.py`, `hardware_mapping.launch.py`, `hardware_navigation.launch.py`.
- **Configs (`config/`)**:
  - `bridge.yaml` (Gazebo Harmonic ↔ ROS 2 bridge)
  - `ekf.yaml` & `ekf_hardware.yaml` (`robot_localization` 50 Hz EKF)
  - `mapper_params.yaml` & `mapper_params_hardware.yaml` (`slam_toolbox` online async)
  - `nav2_params.yaml` (sim AMCL + map_server), `nav2_params_explore.yaml` (sim exploration), `nav2_params_hardware.yaml` (hardware AMCL + map_server), `nav2_params_hardware_explore.yaml` (hardware speed-capped exploration + `velocity_smoother` + `collision_monitor`)
  - `ydlidar_x2.yaml` (reference YDLIDAR X2 driver parameters)
  - RViz profiles: `agv_nav.rviz`, `agv_explore.rviz`, `agv_hardware.rviz`, `agv_hardware_mapping.rviz`, `agv_hardware_nav.rviz`
- **Executable Hardware Drivers (`scripts/`)**: `amr_hardware_drive_node.py`, `amr_imu_node.py`, `amr_lidar_node.py`, `save_map.sh`.

### 4.2 `agv_navigation` (`ament_python`)
Defined in [`src/agv_navigation/setup.py`](../../src/agv_navigation/setup.py) with 11 `console_scripts`:
- **`route_runner`**: KD-Tree node snapper + Dijkstra global planner + 0.3 m path densifier + 10 Hz vectorized MPPI local controller + 3-tier dynamic obstacle evasion + ML perception runtime (`/agv_obstacles_ml`, `/agv_obstacle_markers`, `/obstacle_alert`, `/agv_state`, `/agv_dense_path`, `/mission_progress`).
- **`graph_visualizer`**: Publishes `/agv_graph_markers` (`MarkerArray`: yellow nodes, cyan edges, magenta active dense path).
- **`bt_manager`**: `py_trees` mission orchestrator (`CheckLocalization`, `CheckGoalQueue`, `PlanTopologicalPath`, `ExecuteMPPINavigation`, `BackUpRecovery`, `SpinRecovery`).
- **`auto_localizer`** (`amr_auto_localizer.py`): Zero-prior 2D correlative scan matcher with `/trigger_auto_relocalization` (`std_srvs/Trigger`) and `/localization_status` (`MAP_MISMATCH` / `VERIFIED` gate).
- **`run_logger`**: 10 Hz telemetry logger writing `logs/navigation_run_<ts>.csv` & `.log` + `latest_run.{csv,log}`.
- **`dynamic_obstacle_manager`** & **`obstacle_teleop`**: Automated patrol runner (`aisle_crossing`, `corridor_walker`, `doorway_blocker`, `circular_patrol`) and WASD teleop for Gazebo obstacles (`/dynamic_obstacle/cmd_vel`).
- **`dataset_recorder`**: Records synchronized `/scan`, `/tf`, and `/dynamic_obstacle/odom` into compressed `.npz` captures.
- **ML Pipeline Agents**: `sim_mission_agent` (Agent 1), `obstacle_director_agent` (Agent 2), `dataset_trainer_agent` (Agent 3).
- **Offline Training & Tests**: `ml_training/{synthetic_lidar_sim.py,train_classifier.py,train_predictor.py,train_all_models.py}`, exported pure-NumPy weights in `models/{obstacle_classifier.npz,trajectory_predictor.npz,model_metadata.json}`, and `test/test_ml_perception.py`.

### 4.3 `m-explore-ros2`
Contains `explore` (customized `explore_lite` with empty-frontier streak resilience, `< 0.8s` abort guard, `min_target_distance`, `local_radius`, `wall_bonus_scale`, `momentum_scale`, and hard `progress_timeout` stall recovery), `explore_lite_msgs` (`ExploreStatus.msg`), and `map_merge`.
