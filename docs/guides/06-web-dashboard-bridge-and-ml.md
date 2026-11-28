# 06. Web Dashboard, FastAPI Bridge, Flutter App & ML Pipeline

> **Navigation:** [Root README](../../README.md) · [01. Directory & Scripts](01-directory-and-scripts.md) · [02. Simulation Workflow](02-simulation-workflow.md) · [03. Simulation Errors & Fixes](03-simulation-errors-and-fixes.md) · [04. Hardware Setup & Pi](04-hardware-setup-and-pi-workflow.md) · [05. Hardware Mapping & Nav](05-hardware-mapping-and-navigation.md) · **06. Web Dashboard, Bridge & ML**

This guide covers starting and operating the FastAPI + `rclpy` bridge, using the React 18 + Vite Web Dashboard (including `sim` ↔ `hardware` mode switching and the map/graph/named-places workflow), Flutter mobile setup caveats, and running the 3-agent ML Dynamic Obstacle Perception pipeline.

---

## 1. FastAPI + `rclpy` Bridge ([`app/bridge/`](../../app/bridge/))

[`app/bridge/bridge_server.py`](../../app/bridge/bridge_server.py) runs **Uvicorn/FastAPI** on port `8000` in the main thread and a ROS 2 `AmrBridgeNode` (`MultiThreadedExecutor`) in a background daemon thread (`_ros_thread`), pushing a 5 Hz JSON snapshot over `/ws/telemetry`.

### 1.1 Starting the Bridge (`run_bridge.sh`)
[`app/bridge/run_bridge.sh`](../../app/bridge/run_bridge.sh) automatically sources ROS 2 Jazzy + workspace install, checks Python dependencies (`app/bridge/requirements.txt`), terminates any stale process listening on port `8000`, configures DDS, and launches `bridge_server.py`:

```bash
cd ~/AMR/AMR-main/app/bridge

# Simulation mode (default: ROS_LOCALHOST_ONLY=1, use_sim_time=true):
./run_bridge.sh
./run_bridge.sh sim

# Real Hardware mode (CycloneDDS Domain 7 via config/cyclonedds.xml, use_sim_time=false):
./run_bridge.sh hardware 10.244.248.208
./run_bridge.sh 10.244.248.208   # shorthand for hardware mode
```

### 1.2 Live Mode Switching (`sim` ↔ `hardware`)
Calling `POST /api/mode` (`{"mode": "hardware", "pi_ip": "10.244.248.208"}` or `{"mode": "sim"}`) stops any active `SystemLaunchManager` or `MappingManager` processes, gracefully exits Uvicorn (nulling `_tf_listener` & `_tf_buffer` before destroying the ROS node), reconfigures DDS environment variables (`ROS_LOCALHOST_ONLY=1` for `sim`; `config/cyclonedds.xml` + `ROS_DOMAIN_ID=7` for `hardware`), and calls `os.execv` to restart `bridge_server.py` cleanly in the target mode.

### 1.3 Verified Bridge REST & WebSocket Endpoints

| Category | Endpoint | Method | Payload / Notes |
| :--- | :--- | :--- | :--- |
| **Mode** | `/api/mode` | `GET` / `POST` | Inspect or switch runtime mode (`{"mode": "sim"\|"hardware", "pi_ip": "..."}`) |
| **System Lifecycle** | `/api/system/start` | `POST` | `{"map_name", "world", "headless", "use_rviz", "auto_start_pi", "stop_pi_on_stop"}` — 1-click launch (`navigation_launch` + `route_runner` + `bt_manager` + `graph_visualizer` in sim; `hardware_navigation` in hardware) |
| | `/api/system/stop` | `POST` | Optional `{"stop_pi": true\|false}` — stops navigation stack (and optionally Pi hardware) |
| | `/api/system/status` | `GET` | Current launch stage (`0..4`), uptime, active map, process PIDs |
| **Pi SSH Remote** | `/api/hardware/pi/status` | `GET` | Query `?force=true` — SSH reachability, remote PIDs, clock sync status |
| | `/api/hardware/pi/start` | `POST` | Starts `./scripts/hardware/start_hardware.sh <LAPTOP_IP>` on Pi via SSH |
| | `/api/hardware/pi/stop` | `POST` | Zeroes `/cmd_vel` and stops Pi bringup stack via SSH |
| | `/api/hardware/pi/logs` | `GET` | Query `?lines=60` — tails `~/AMR/logs/pi_hardware_latest.log` on Pi |
| **GUI Launchers** | `/api/gui/launch` | `POST` | `{"app": "rviz"\|"gazebo", "mode": "nav"\|"explore"\|"hw_nav"\|"hw_explore"}` — spawns `view_rviz.sh` or `view_gazebo.sh` |
| **Telemetry & Control** | `/ws/telemetry` | `WebSocket` | 5 Hz live telemetry (pose, velocity, IMU, scan, `nav_state`, `hardware_link`, `localization_status`, `mapping`, `system_status`) |
| | `/api/status` | `GET` | One-shot telemetry snapshot |
| | `/api/cmd_vel` | `POST` | `{"linear": 0.13, "angular": 0.0}` (paused auto-explore automatically on manual drive; blocked when E-Stop latched) |
| | `/api/goal` | `POST` | `{"x": 2.5, "y": 1.2, "yaw": 0.0}` → `/goal_pose` |
| | `/api/goal_sequence` | `POST` | `{"nodes": ["N12", "N45"]}` → `/goal_sequence` |
| | `/api/initial_pose` | `POST` | `{"x": 0.0, "y": 0.0, "theta": 0.0}` → `/initialpose` |
| | `/api/localization/auto` | `POST` | Calls `/trigger_auto_relocalization` (`std_srvs/Trigger`) |
| | `/api/stop` & `/api/estop/clear` | `POST` | Asserts or clears `/agv_estop` |
| **Maps & Graphs** | `/api/map` | `GET` | Query `?map_name=&clean=false` — Base64 JPEG map with optional live pose/path overlay |
| | `/api/map/raw`, `/api/map/clean` | `GET` | Query `?map_name=&theme=dark` — Raw/clean map image file |
| | `/api/map/metadata` | `GET` | Query `?map_name=` — `{map_name, resolution, origin_x, origin_y, width, height}` |
| | `/api/map/active` | `GET` / `POST` | Get or switch active map (`{"map_name": "..."}`) |
| | `/api/graph` | `GET` | Query `?map_name=` — Active graph `nodes` and `edges` JSON |
| **Mapping Studio** | `/api/mapping/start`, `/stop`, `/redo` | `POST` | Manage SLAM mapping session (`world`, `run_explore`, `headless`, `use_rviz`, `auto_start_pi`, `stop_pi_on_stop`) |
| | `/api/mapping/explore/{start,pause,resume}` | `POST` | Start/pause/resume `explore_lite` (`run_explore` attaches speed-capped Nav2 + `collision_monitor` in hardware mode) |
| | `/api/mapping/{status,live_map,verify,saved_maps}` | `GET` | Live SLAM grid, quality audit, and list of saved sim & hardware maps |
| | `/api/mapping/save` | `POST` | `{"map_name": "..."}` — saves `.yaml`, `.pgm`, `.png` (with `<4`-cell speckle filtering & Pi SCP sync in hardware mode) |
| | `/api/mapping/graph/extract` | `POST` | `{"map_name", "robot_radius", "safety_margin", "search_radius", "step_corridor", "step_medium", "step_open"}` |
| | `/api/mapping/graph/latest`, `/verify` | `GET` | Fetch latest extracted graph or run topological audit |
| | `/api/mapping/graph/plan_route` | `POST` | `{"start_node", "goal_node", "map_name"}` — Dijkstra probe before dispatch |
| | `/api/mapping/places`, `/places/save`, `/places/default_templates`, `/places/{id}` | `GET`/`POST`/`DELETE` | Manage semantic stations in `<map>_places.json` |
| | `/api/mapping/navigation/dispatch_place`, `/dispatch_sequence`, `/cancel`, `/status` | `POST`/`GET` | Dispatch single or multi-stop named-place missions |
| **Dynamic Obstacles** | `/api/obstacle/{cmd_vel,spawn,remove,status}` | `POST`/`GET` | Spawn, move, drive (`W/A/S/D`), or remove Gazebo obstacles in `sim` mode |

---

## 2. React 18 + Vite Web Dashboard ([`app/web_dashboard/`](../../app/web_dashboard/))

### 2.1 Installation & Startup
```bash
cd ~/AMR/AMR-main/app/web_dashboard
npm install                  # first time (or use native Linux Node v20)
./run_dashboard.sh           # prioritizes ~/node-v20.18.0-linux-x64/bin, runs Vite on :5173
```
Open **`http://localhost:5173`**. [`vite.config.js`](../../app/web_dashboard/vite.config.js) proxies `/api` to `http://localhost:8000` and `/ws` to `ws://localhost:8000`.

### 2.2 Dashboard Workflow: Mode, Maps, Graphs & Named Places
1. **Mode & Connection (`StatusBar.jsx`)**: Switch between **SIM** and **HARDWARE** (enter Pi IP, default `10.244.248.208`), monitor Pi SSH + sensor readiness, select active maps, or press **E-STOP**.
2. **1-Click Navigation (`ControlPanel.jsx` & `MapView.jsx`)**:
   - Click **Start System** to boot the full navigation stack for the selected map.
   - Left-click on `MapView` to send a goal, toggle graph nodes to build a multi-stop `/goal_sequence`, use the `nipplejs` joystick under `ControlInterlock`, or dispatch saved **Fleet Stations** (named places).
   - Inspect live 360° LiDAR proximity (`ScanRing.jsx`), pose/velocity/IMU (`TelemetryPanel.jsx`), and state transitions (`EventLog.jsx`).
3. **Mapping Studio (`MappingScreen.jsx`)**:
   - **Stage 1 (Map New Place)**: Choose `Manual Drive` or `Auto Explore` (plus optional `autoStartPi` in hardware mode) and monitor live SLAM occupancy.
   - **Stage 2 (Verify & Save)**: Audit coverage/area metrics and save into `maps/{yaml,pgm,png}/` (hardware) or `src/agv_description/maps/{yaml,pgm,png}/` (sim).
   - **Stage 3 & 4 (Graph Extract & Verify)**: Run `graph_extractor`, inspect node/edge connectivity, and probe point-to-point Dijkstra routes.
   - **Stage 5 (Name Places)**: Label graph nodes (e.g., docks, pickup/dropoff stations) and save `<map>_places.json`, then return to the main dashboard for named-place dispatch.

---

## 3. Flutter Mobile & Desktop Client ([`app/flutter_app/`](../../app/flutter_app/))

Located in [`app/flutter_app/`](../../app/flutter_app/) (`connect_screen.dart`, `home_screen.dart`, `map_screen.dart`, `control_tab.dart`, `monitor_tab.dart`, `amr_bridge_service.dart`):

```bash
cd ~/AMR/AMR-main/app/flutter_app
flutter pub get
hostname -I                 # find bridge host LAN IP
flutter run -d linux        # or Android device ID
```
In `connect_screen.dart`, connect to `http://<HOST_IP>:8000`.

> [!WARNING]
> **Hardcoded Map Metadata Caveat ([`map_screen.dart`](../../app/flutter_app/lib/screens/map_screen.dart)):**
> Lines 8–11 and 75 of `app/flutter_app/lib/screens/map_screen.dart` hardcode map calibration constants (`_mapResolution = 0.05`, `_mapOriginX = -8.3`, `_mapOriginY = -7.3`, `_mapHeightPx = 275`, and native width `329`). Unlike the React Web Dashboard—which dynamically queries `/api/map/metadata`—tap-to-goal coordinates in the Flutter app will be offset on maps with different origins/dimensions unless updated to match the active map or wired to `/api/map/metadata`.

---

## 4. Automated ML Dynamic Obstacle Perception Pipeline

The navigation stack embeds a pure-NumPy 10 Hz ML perception pipeline ([`src/agv_navigation/agv_navigation/ml_perception/`](../../src/agv_navigation/agv_navigation/ml_perception/)) that segments 2D `/scan` clusters, fuses bipedal legs, extracts 20 geometric + Kalman kinematic features, classifies clusters (`WALL`, `CLUTTER`, `PERSON`), and predicts 2.0 s (`20 steps × 0.10 s`) obstacle trajectories with uncertainty (`pred_std`) for `route_runner`'s MPPI planner.

### 4.1 3-Agent Automated Sim Collection & Training (`run_automated_ml_pipeline.sh`)
[`scripts/ml/run_automated_ml_pipeline.sh`](../../scripts/ml/run_automated_ml_pipeline.sh) runs the full closed-loop Gazebo pipeline:

```bash
cd ~/AMR/AMR-main
./scripts/ml/run_automated_ml_pipeline.sh [--headless] [--duration 95] [--laps 5] [--scenario-sec 16] [--map hospital_map] [--keep-open]
```

**Orchestrated Components & Logs (`logs/ml_pipeline_<timestamp>/`):**
1. **`navigation_launch.py` + `route_runner` + `graph_visualizer`**: Simulation, AMCL, MPPI navigation (`01_navigation_launch.log`, `02_route_runner.log`, `02b_graph_visualizer.log`).
2. **Agent 1 — `sim_mission_agent`**: Dispatches patrol waypoints (`HOSPITAL_PATROL_GOALS` / `WAREHOUSE_PATROL_GOALS`), monitors clearances & yields (`04_sim_mission.log`).
3. **Agent 2 — `obstacle_director_agent`**: Spawns and drives 6 Gazebo scenarios (`pedestrian_aisle_crossing`, `pedestrian_corridor_walker`, `cart_crossing_and_clutter`, `clutter_corridor_slalom`, `multi_pedestrian_and_clutter`, `cart_circular_corridor`) while publishing ground truth on `/ml_pipeline/active_scenario` (`03_obstacle_director.log`).
4. **Agent 3 — `dataset_trainer_agent`**: Captures `/scan` + `/tf` + ground truth into `ml_datasets/gazebo_live_dataset.npz`, invokes `.venv_ml/bin/python -u src/agv_navigation/ml_training/train_all_models.py`, and publishes `/ml_pipeline/reload_models` so `route_runner` hot-reloads the newly exported models (`05_dataset_trainer.log`).

### 4.2 80/20 Sim + Hardware Training & Empirical Beam Dropout (`train_all_models.py`)
[`src/agv_navigation/ml_training/train_all_models.py`](../../src/agv_navigation/ml_training/train_all_models.py) assembles a balanced **80% simulation / 20% hardware-domain** training dataset:
- **80% Simulation**: Live closed-loop captures from `ml_datasets/gazebo_live_dataset.npz` supplemented if needed by [`synthetic_lidar_sim.py`](../../src/agv_navigation/ml_training/synthetic_lidar_sim.py).
- **20% Hardware-Domain**: Uses any real recorded `ml_datasets/hardware_*.npz` captures plus raycast episodes over physical SLAM maps (`prakash_room`, `hardware_map`, `final_map`, `my_room_map`, `my_next_map`) simulated with hardware crawl speeds (`v <= 0.13 m/s`, `w <= 0.38 rad/s`) and **empirical YDLIDAR X2 beam dropout** measured directly from the `num_valid_beams` column in `logs/navigation_run_*.csv` (measured `0.3444` in [`model_metadata.json`](../../src/agv_navigation/models/model_metadata.json)).
- **Model Selection & Export**: Benchmarks 7 classifiers (`RandomForest_80`, `ExtraTrees_80`, `HistGradientBoosting`, `1D_CNN_Spatial`, `CompactMLP_64_32`, `CompactMLP_96_48`, `CompactMLP_128_64`) and 4 trajectory predictors (`ConstantVelocityKF`, `PureGRU_32`, `KalmanResidualGRU_32`, `KalmanResidualGRU_48`). Exports the top pure-NumPy weights to [`src/agv_navigation/models/obstacle_classifier.npz`](../../src/agv_navigation/models/obstacle_classifier.npz), [`src/agv_navigation/models/trajectory_predictor.npz`](../../src/agv_navigation/models/trajectory_predictor.npz), and [`src/agv_navigation/models/model_metadata.json`](../../src/agv_navigation/models/model_metadata.json) (recorded winners: `RandomForest_80` / `tree_ensemble` and `KalmanResidualGRU_32` with `25.17%` 2s FDE improvement over `ConstantVelocityKF`).

### 4.3 Recording Real Hardware Captures & Running ML Unit Tests
Record synchronized `/scan` + `/tf` episodes on physical hardware (naming `ml_datasets/hardware_*.npz` so `train_all_models.py` picks them up automatically):
```bash
ros2 run agv_navigation dataset_recorder --out ~/AMR/AMR-main/ml_datasets/hardware_run1.npz
```
Run the pure-NumPy ML perception unit & latency suite (run on an idle CPU outside Gazebo to avoid latency assertion jitter):
```bash
cd ~/AMR/AMR-main
.venv_ml/bin/python -m pytest src/agv_navigation/test/test_ml_perception.py -v
```
