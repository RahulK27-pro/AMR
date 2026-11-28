# 05. Hardware Mapping & Navigation Workflow

> **Navigation:** [Root README](../../README.md) · [01. Directory & Scripts](01-directory-and-scripts.md) · [02. Simulation Workflow](02-simulation-workflow.md) · [03. Simulation Errors & Fixes](03-simulation-errors-and-fixes.md) · [04. Hardware Setup & Pi](04-hardware-setup-and-pi-workflow.md) · **05. Hardware Mapping & Nav** · [06. Web Dashboard, Bridge & ML](06-web-dashboard-bridge-and-ml.md)

This guide covers end-to-end physical SLAM mapping (manual teleop and speed-capped autonomous exploration), the `collision_monitor` safety chain, saving and speckle-filtering maps, extracting topological roadmaps, launching physical autonomous navigation with zero-prior auto-localization, and resolving common hardware mapping/navigation errors.

> [!IMPORTANT]
> Before starting any hardware mapping or navigation session, ensure the Raspberry Pi bringup stack is running (`./scripts/hardware/start_hardware.sh <LAPTOP_IP>` on Pi, or 1-click Auto-Start Pi in the Web UI) and keep the robot stationary during startup IMU calibration.

---

## 1. Hardware SLAM Mapping (Manual & Auto-Explore)

### 1.1 Manual SLAM Mapping (`start_hardware_mapping.sh`)
On the **Laptop**, run [`scripts/hardware/start_hardware_mapping.sh`](../../scripts/hardware/start_hardware_mapping.sh):

```bash
cd ~/AMR/AMR-main
./scripts/hardware/start_hardware_mapping.sh 10.244.248.208
```

What `start_hardware_mapping.sh` executes:
1. Rebuilds `agv_description` if launch/config/RViz files changed.
2. Generates a temporary CycloneDDS Domain 7 unicast config pointing to `PI_IP`.
3. Synchronizes the Pi's system clock via SSH (`sudo -n date -s`) and clears any remote Pi `slam_toolbox` or static `map -> odom` TF publisher so laptop SLAM Toolbox exclusively owns `map -> odom`.
4. Terminates any local `amcl` or static `map -> odom` TF publisher.
5. Verifies `/scan` and `/odom` / `/odometry/filtered` are streaming.
6. Launches [`hardware_mapping.launch.py`](../../src/agv_description/launch/hardware_mapping.launch.py) with `use_sim_time:=false`, [`mapper_params_hardware.yaml`](../../src/agv_description/config/mapper_params_hardware.yaml), and [`agv_hardware_mapping.rviz`](../../src/agv_description/config/agv_hardware_mapping.rviz).

In a **second Laptop terminal**, drive the robot smoothly using [`scripts/hardware/teleop_hardware.sh`](../../scripts/hardware/teleop_hardware.sh):
```bash
./scripts/hardware/teleop_hardware.sh
```
- **Hold-to-move controls (`teleop_wasd.py`)**: `W/S/A/D` (or Arrow keys) move only while held (`KEY_TIMEOUT_SEC = 0.22 s`), `Q/E` curve, `+`/`-` adjust speed, `1` (`0.10 m/s`) / `2` (`0.20 m/s`) / `3` (`0.35 m/s`) presets, `Space` / `X` E-Stop.
- For clean SLAM maps, keep linear speed around `0.10–0.14 m/s` and avoid sharp fast turns (`<= 0.45 rad/s`). When mapping via the Web Dashboard, `bridge_server.py` enforces `HW_MAPPING_TELEOP_LIMITS` (`max_linear: 0.20 m/s`, `default_linear: 0.13 m/s`, `max_angular: 0.50 rad/s`, `default_angular: 0.35 rad/s`).

To inspect SLAM & sensor health during mapping:
```bash
source scripts/hardware/env_hardware.sh 10.244.248.208
python3 scripts/diagnostics/diagnose_mapping.py
```

### 1.2 Hardware Auto-Explore SLAM (`run_explore:=true`)
[`hardware_mapping.launch.py`](../../src/agv_description/launch/hardware_mapping.launch.py) supports all of the following arguments:

| Argument | Default | Description |
| :--- | :--- | :--- |
| `use_sim_time` | `false` | Must stay `false` on physical hardware |
| `use_rviz` | `true` | Launch RViz2 (`agv_hardware_mapping.rviz`) |
| `run_explore` | `false` | Launch speed-capped Nav2 + `collision_monitor` + `velocity_smoother` + `explore_lite` |
| `launch_slam` | `true` | Set `false` when dynamically attaching `run_explore:=true` to an existing SLAM session |
| `slam_params_file` | `mapper_params_hardware.yaml` | SLAM Toolbox config |
| `explore_params_file` | `nav2_params_hardware_explore.yaml` | Speed-capped hardware Nav2 & `explore_lite` config |

```bash
# Launch SLAM + speed-capped Auto-Explore from CLI:
source ~/AMR/AMR-main/scripts/hardware/env_hardware.sh 10.244.248.208
ros2 launch agv_description hardware_mapping.launch.py run_explore:=true
```

---

## 2. Speed Limits & Collision Monitor Safety Chain

Physical hardware exploration and navigation enforce strict speed limits and a multi-stage safety chain defined in [`nav2_params_hardware_explore.yaml`](../../src/agv_description/config/nav2_params_hardware_explore.yaml), [`hardware_mapping.launch.py`](../../src/agv_description/launch/hardware_mapping.launch.py), and [`hardware_navigation.launch.py`](../../src/agv_description/launch/hardware_navigation.launch.py):

### 2.1 Speed Caps by Mode
| Mode / Component | Max Linear (`vx`) | Min/Reverse (`vx_min`) | Max Angular (`wz`) | Config Source |
| :--- | :--- | :--- | :--- | :--- |
| **HW Auto-Explore MPPI & Smoother** | `0.14 m/s` | `-0.07 m/s` | `0.45 rad/s` | `nav2_params_hardware_explore.yaml` |
| **HW Navigation (`route_runner`)** | `0.13 m/s` (`0.10 m/s` cruise) | `0.04 m/s` min | `0.40 rad/s` | `hardware_navigation.launch.py` / `start_hardware_nav.sh` |
| **HW Mapping Teleop (Web UI)** | `0.20 m/s` (`0.13 m/s` default) | `-0.12 m/s` | `0.50 rad/s` | `bridge_server.py` (`HW_MAPPING_TELEOP_LIMITS`) |
| **HW Motor Driver Hard Ceiling** | `max_pwm_cap: 35.0%` duty | — | `cmd_vel_timeout: 1.5 s` | `hardware_bringup.launch.py` |

### 2.2 Hardware Exploration Velocity Safety Pipeline
In `hardware_mapping.launch.py` (`run_explore:=true`), velocity commands pass through `collision_monitor` and `velocity_smoother` before reaching `/cmd_vel`:

```text
controller_server & behavior_server (remapped /cmd_vel -> cmd_vel_nav)
  │
  ▼
nav2_collision_monitor (/scan observation source: min_height 0.15m)
  ├── VelocityPolygonStop (type: velocity_polygon, action_type: stop)
  │     • rotation:             [-0.15..0.15] x [-0.145..0.145]
  │     • translation_forward:  [-0.12..0.18] x [-0.155..0.155]
  │     • translation_backward: [-0.18..0.12] x [-0.155..0.155]
  │     • stopped:              [-0.15..0.15] x [-0.145..0.145]
  └── FootprintApproach (type: polygon, action_type: approach, time_before_collision: 0.8s)
  │ (cmd_vel_out_topic: cmd_vel_safe)
  ▼
nav2_velocity_smoother (20 Hz OPEN_LOOP, max_velocity: [0.14, 0.0, 0.45], max_accel: [0.25, 0.0, 0.70])
  │ (remapped cmd_vel_smoothed -> /cmd_vel)
  ▼
/cmd_vel ──► Pi amr_hardware_drive_node.py
```

---

## 3. Saving, Speckle-Filtering & Extracting Hardware Graphs

### 3.1 Save & Filter the Hardware Map (`save_hardware_map.sh`)
While `hardware_mapping.launch.py` is still running and `/map` is active, run [`scripts/hardware/save_hardware_map.sh`](../../scripts/hardware/save_hardware_map.sh):

```bash
cd ~/AMR/AMR-main
./scripts/hardware/save_hardware_map.sh my_room_map
```

What `save_hardware_map.sh` performs:
1. Checks that `/map` has an active publisher (`--no-daemon`).
2. Runs `ros2 run nav2_map_server map_saver_cli -f maps/yaml/my_room_map --occ 0.65 --free 0.25 --fmt pgm --ros-args -p use_sim_time:=false -p save_map_timeout:=10.0`.
3. Moves `.pgm` to `maps/pgm/my_room_map.pgm` and updates `maps/yaml/my_room_map.yaml` to `image: ../pgm/my_room_map.pgm`.
4. **Unknown-Safe Speckle Filtering**: Using OpenCV (`cv2.connectedComponentsWithStats` with 8-connectivity), removes isolated occupied clusters smaller than **4 cells** (`< 10 cm × 10 cm` LiDAR specks) without turning unknown space (`205`) into false free pockets — cells adjacent to dilated free space (`>= 250`) become free (`254`), while isolated specks surrounded by unknown stay unknown (`205`). Exports both `maps/pgm/<name>.pgm` and `maps/png/<name>.png`.
5. Stop the mapping session (`Ctrl+C` in the mapping terminal).

### 3.2 Generate the Topological Graph (`generate_hardware_graph.py`)
```bash
python3 scripts/hardware/generate_hardware_graph.py my_room_map
```
Outputs `maps/json/my_room_map_graph.json` and `maps/png/my_room_map_graph_vis.png`.
If maps were saved on the Pi or need to be mirrored to the Pi before running onboard tools, run:
```bash
./scripts/hardware/sync_to_pi.sh push 10.244.248.208   # or pull if saved on Pi
```

---

## 4. Hardware Autonomous Navigation (`start_hardware_nav.sh`)

Launch physical navigation from the **Laptop** using [`scripts/hardware/start_hardware_nav.sh`](../../scripts/hardware/start_hardware_nav.sh):

```bash
# Usage: ./scripts/hardware/start_hardware_nav.sh [PI_IP] [MAP_NAME_OR_YAML] [MAX_SPEED]
./scripts/hardware/start_hardware_nav.sh 10.244.248.208 prakash_room 0.13
```

[`hardware_navigation.launch.py`](../../src/agv_description/launch/hardware_navigation.launch.py) supports:
`map` (default `prakash_room`), `use_rviz` (`true`), `use_bt` (`false`), `initial_pose_x` (`0.0`), `initial_pose_y` (`0.0`), `initial_pose_yaw` (`0.0`), `max_linear_speed` (`0.13`), `max_angular_speed` (`0.40`), `cruise_linear_speed` (`0.10`), `lookahead_dist` (`0.25`).

### Nodes Started & Key Features
1. **`map_server` + `amcl` + `lifecycle_manager_localization`** ([`nav2_params_hardware.yaml`](../../src/agv_description/config/nav2_params_hardware.yaml), `bond_timeout: 0.0`, `use_sim_time: false`).
2. **`auto_localizer` (`amr_auto_localizer.py`)**: Zero-prior 2D correlative scan matcher (`72` yaw orientations at 5° steps) that locates the robot anywhere in the room on startup, checks the map-consistency gate (`min_match_score: 0.74`, `min_net_score: 0.70`, `min_hit_ratio: 0.55`, `max_pass_through_ratio: 0.14`, `max_oob_ratio: 0.15`), publishes `/initialpose` (latched `TRANSIENT_LOCAL`) + `/set_initial_pose` + `/request_nomotion_update`, and publishes `/localization_status` (`VERIFIED` or `MAP_MISMATCH`).
   - Re-localize anytime after moving the robot by hand:
     ```bash
     ros2 service call /trigger_auto_relocalization std_srvs/srv/Trigger
     ```
3. **`route_runner`**: Dijkstra + KD-Tree + 10 Hz MPPI + ML perception calibrated to hardware defaults (`v_max: 0.13`, `cruise_linear_speed: 0.10`, `w_max: 0.40`, `lookahead_dist: 0.25`, `goal_arrival_tolerance: 0.08`).
4. **`graph_visualizer`**: Publishes `/agv_graph_markers` for RViz.
5. **`bt_manager`** *(optional, enabled via `use_bt:=true`)*: `py_trees` mission orchestrator.
6. **`run_logger`**: Records 10 Hz telemetry (`ros_time_sec`, `state`, `map_x/y/yaw`, `amcl_*`, `ekf_*`, `cmd_v/w`, `min_scan_dist_m`, `num_valid_beams`, `ml_*`) to `logs/navigation_run_<ts>.csv/.log` and updates `logs/latest_run.{csv,log}`. Launch stdout is also teed to `logs/hardware_nav_<ts>.log` and `logs/latest_launch.log`.

---

## 5. Common Hardware Mapping & Navigation Errors

| Symptom | Root Cause | Verified Fix & Config Reference |
| :--- | :--- | :--- |
| **TF Future Extrapolation / AMCL & SLAM drop `/scan` messages** | Pi 5 clock and Laptop clock drifted out of sync (Pi 5 lacks a battery-backed RTC across reboots), causing `/scan` timestamps to appear in the past/future relative to Laptop TF lookups. | Configure `NOPASSWD: /usr/bin/date` on Pi so `start_hardware_mapping.sh`, `start_hardware_nav.sh`, `sync_to_pi.sh`, and `PiRemoteManager.sync_clock()` auto-sync Pi time. Hardware configs also widen `transform_timeout: 0.50` (`mapper_params_hardware.yaml`), `transform_tolerance: 0.20` (`nav2_params_hardware.yaml`), and `0.8` (`nav2_params_hardware_explore.yaml`). |
| **Pi Clock Drift After Power Cycle** | Raspberry Pi 5 boots with an old clock before NTP finishes or when offline on a private hotspot. | Run `ssh amr@10.244.248.208 "sudo -n date -s '@$(date +%s.%N)'"` or trigger any of the hardware launch/sync scripts. Check drift live via `python3 scripts/diagnostics/diagnose_mapping.py`. |
| **MPPI CPU Starvation / Slow Control Loop** | High-batch Nav2 MPPI (`batch_size: 800, time_steps: 32, 20 Hz`) starves CPU during hardware mapping/exploration alongside SLAM Toolbox & RViz2. | [`nav2_params_hardware_explore.yaml`](../../src/agv_description/config/nav2_params_hardware_explore.yaml) reduces MPPI load to `controller_frequency: 10.0`, `batch_size: 400`, `time_steps: 20`, `model_dt: 0.10`, `iteration_count: 1`. Hardware nav uses lightweight vectorized NumPy MPPI (`80 samples × 15 horizon` @ 10 Hz) in `route_runner`. |
| **Wall Clipping / Chassis Self-Hits / Smeared Walls** | LiDAR returns `< 0.12 m` reflect off chassis brackets; driving or pivoting faster than `10 Hz` YDLIDAR scan rate smears SLAM walls; tight inflation allows corners to clip doorways. | Hardware drivers enforce `range_min: 0.15` (`hardware_bringup.launch.py`) & `min_laser_range: 0.12` (`mapper_params_hardware.yaml`); `nav2_params_hardware_explore.yaml` uses `robot_radius: 0.15`, `inflation_radius: 0.30`, `VelocityPolygonStop` + `FootprintApproach` in `collision_monitor`, and `vx_max: 0.14`, `wz_max: 0.45`; `amr_hardware_drive_node` caps PWM at `max_pwm_cap: 35.0%`. |
| **Premature Frontier Stops (`explore_lite`)** | Transient empty costmap update or short `< 0.8s` Nav2 abort prematurely triggers `EXPLORATION_COMPLETE`. | `src/m-explore-ros2/explore/src/explore.cpp` enforces `kMaxEmptyFrontiersStreak = 5`, ignores `< 0.8s` instant aborts, retires targets `< min_target_distance` (`0.35 m`), and resets blacklist once when exhausted; `nav2_params_hardware_explore.yaml` sets `min_frontier_size: 0.25`, `progress_timeout: 30.0`, `local_radius: 3.0`. |
| **`MAP_MISMATCH` / Goal Dispatch Rejected** | Wrong map loaded for the physical room (`auto_localizer` scan-fit gate fails: `score < 0.74`, `hit_ratio < 0.55`, or `wall_pass_through > 0.14`). | Select the matching map in the UI/CLI (`prakash_room`, `hardware_map`, etc.), verify LiDAR is spinning, and re-run `/trigger_auto_relocalization` or set manual **2D Pose Estimate** in RViz2. |
