# 03. Simulation Errors & Verified Fixes Playbook

> **Navigation:** [Root README](../../README.md) · [01. Directory & Scripts](01-directory-and-scripts.md) · [02. Simulation Workflow](02-simulation-workflow.md) · **03. Simulation Errors & Fixes** · [04. Hardware Setup & Pi](04-hardware-setup-and-pi-workflow.md) · [05. Hardware Mapping & Nav](05-hardware-mapping-and-navigation.md) · [06. Web Dashboard, Bridge & ML](06-web-dashboard-bridge-and-ml.md)

This playbook documents verified simulation, build, Nav2 Jazzy, `explore_lite`, DDS, bridge, and test issues observed in this repository, along with their root causes and implemented fixes. For historical Gazebo Classic → Harmonic migration details, see [`docs/project_migration_and_troubleshooting.md`](../project_migration_and_troubleshooting.md).

---

## 1. Gazebo Harmonic Ogre2 / Mesa GPU Crash (`SIGSEGV` or Black Viewport)

- **Symptom:** Launching Gazebo Harmonic (`gz sim`) or RViz2 on WSL2, headless Linux, or GPUs without desktop OpenGL 3.3+ crashes with `SIGSEGV` (`exit code -11`), fails Ogre2 render engine initialization, or renders an empty black viewport.
- **Root Cause:** Gazebo Harmonic's `<render_engine>ogre2</render_engine>` (used by `gpu_lidar` in [`warehouse_agv.urdf`](../../src/agv_description/urdf/warehouse_agv.urdf) and all `.world` files) requires desktop OpenGL 3.3+ / 4.5, which WSL2 virtualized GPU drivers and headless/embedded GPUs frequently fail to expose directly to Qt/Ogre2.
- **Verified Fix:** Force Mesa software rasterization (`llvmpipe`) with OpenGL version override and XCB Qt backend. All simulation launch/helper scripts already inject these variables:
  - [`mapping_session.launch.py`](../../src/agv_description/launch/mapping_session.launch.py), [`view_gazebo.sh`](../../scripts/sim/view_gazebo.sh), [`view_rviz.sh`](../../scripts/sim/view_rviz.sh), and [`run_automated_ml_pipeline.sh`](../../scripts/ml/run_automated_ml_pipeline.sh):
    ```bash
    export QT_QPA_PLATFORM=xcb
    export LIBGL_ALWAYS_SOFTWARE=1
    export GALLIUM_DRIVER=llvmpipe
    export MESA_GL_VERSION_OVERRIDE=4.5
    ```
  - [`gazebo.launch.py`](../../src/agv_description/launch/gazebo.launch.py) and [`run_bridge.sh`](../../app/bridge/run_bridge.sh) similarly enforce `LIBGL_ALWAYS_SOFTWARE=1` and `MESA_GL_VERSION_OVERRIDE=3.3` (or `4.5` when spawning RViz/Gazebo GUIs).

---

## 2. WSL2 Picking Up Windows `npm` / `node`

- **Symptom:** Running `npm run dev` or `npm install` inside WSL2 Ubuntu invokes `/mnt/c/.../nodejs/npm` (Windows `npm.cmd`/`node.exe`), causing UNC path failures, broken Linux symlinks, or Vite file-watching errors.
- **Root Cause:** By default, WSL2 appends the Windows host `PATH` into the Linux shell, so if Linux Node.js is missing or placed after Windows entries in `PATH`, Windows Node/npm executes instead.
- **Verified Fix:** Install native Linux Node.js v20 in `~/node-v20.18.0-linux-x64` and prepend it to `PATH`. [`app/web_dashboard/run_dashboard.sh`](../../app/web_dashboard/run_dashboard.sh) automates this:
  ```bash
  if [ -d "$HOME/node-v20.18.0-linux-x64/bin" ]; then
    export PATH="$HOME/node-v20.18.0-linux-x64/bin:$PATH"
  fi
  ```
  Verify native Linux binaries with `which node && which npm` (both should resolve inside `$HOME/node-v20.18.0-linux-x64/bin` or `/usr/bin`, never `/mnt/c/`).

---

## 3. Nav2 Jazzy Lifecycle Manager & BT Plugin Crashes

### 3.1 `CRITICAL FAILURE: SERVER amcl IS DOWN` (Premature Lifecycle Bond Timeout)
- **Symptom:** `lifecycle_manager_localization` aborts startup while Gazebo is still booting or `map_server`/`amcl` take slightly longer to answer heartbeat bonds.
- **Root Cause:** The default Nav2 `bond_timeout` triggers while simulation time (`/clock`) is initializing or CPU load is high.
- **Verified Fix:**
  - In [`navigation_launch.py`](../../src/agv_description/launch/navigation_launch.py) (and [`hardware_navigation.launch.py`](../../src/agv_description/launch/hardware_navigation.launch.py)), `lifecycle_manager_localization` sets **`bond_timeout: 0.0`**.
  - In [`mapping_session.launch.py`](../../src/agv_description/launch/mapping_session.launch.py), `lifecycle_manager_navigation` sets **`bond_timeout: 40.0`** so heavy Nav2 exploration nodes have ample boot margin.

### 3.2 `ID [ComputePathToPose] already registered` in `bt_navigator`
- **Symptom:** `bt_navigator` crashes at startup with a BehaviorTree.CPP duplicate ID exception.
- **Root Cause:** In ROS 2 Jazzy, `nav2_bt_navigator` automatically registers standard Nav2 BT plugins. Explicitly specifying `plugin_lib_names:` in YAML attempts double registration.
- **Verified Fix:** `plugin_lib_names` is intentionally omitted in both [`nav2_params.yaml`](../../src/agv_description/config/nav2_params.yaml) and [`nav2_params_explore.yaml`](../../src/agv_description/config/nav2_params_explore.yaml).

### 3.3 `docking_server` Crash on Empty Plugin List
- **Symptom:** Lifecycle manager fails if `docking_server` is included with `dock_plugins: [""]`.
- **Verified Fix:** `docking_server` is omitted from [`nav2_params.yaml`](../../src/agv_description/config/nav2_params.yaml) and lifecycle `node_names`.

---

## 4. `map_saver_cli -f` Argument Ordering in ROS 2 Jazzy

- **Symptom:** Running `ros2 run nav2_map_server map_saver_cli --ros-args -p use_sim_time:=true -f my_map` fails to parse `-f` or treats `-f` as an unrecognized ROS argument.
- **Root Cause:** In ROS 2 Jazzy `rclcpp` / `map_saver_cli`, CLI flags (`-f`, `--occ`, `--free`, `--fmt`) cannot be mixed inside `--ros-args` without separating them.
- **Verified Fix:** Either place CLI arguments **before** `--ros-args`, or place `--` before `-f`:
  - Pattern in [`src/agv_description/scripts/save_map.sh`](../../src/agv_description/scripts/save_map.sh):
    ```bash
    ros2 run nav2_map_server map_saver_cli \
        --ros-args \
        -p use_sim_time:=true \
        -p save_map_timeout:=10.0 \
        -- -f "$YAML_OUT"
    ```
  - Pattern in [`scripts/hardware/save_hardware_map.sh`](../../scripts/hardware/save_hardware_map.sh):
    ```bash
    ros2 run nav2_map_server map_saver_cli \
        -f "$SAVE_BASE" \
        --occ 0.65 \
        --free 0.25 \
        --fmt pgm \
        --ros-args \
        -p use_sim_time:=false \
        -p save_map_timeout:=10.0
    ```

---

## 5. `explore_lite`: Costmap Dependency, `rolling_window: true` & Premature Stop Resilience

### 5.1 Costmap Circular Dependency (`Waiting for costmap to become available, topic: /global_costmap/costmap`)
- **Symptom:** `explore_lite` hangs indefinitely waiting for `/global_costmap/costmap`, or `global_costmap` stalls waiting for `/map` when Nav2 starts before `slam_toolbox` publishes `/map`.
- **Root Cause:** A `global_costmap` using `static_layer` blocks lifecycle activation until `/map` arrives; if `slam_toolbox` hasn't published `/map` yet, `lifecycle_manager` times out. Conversely, removing `static_layer` without setting a rolling window yields a `0 × 0` costmap.
- **Verified Fixes:**
  1. **Staggered Launch + `static_layer` Updates ([`mapping_session.launch.py`](../../src/agv_description/launch/mapping_session.launch.py) & [`nav2_params_explore.yaml`](../../src/agv_description/config/nav2_params_explore.yaml)):** `slam_toolbox` starts at `t=5s` (publishing `/map`), Nav2 nodes launch at `t=15s` with `static_layer` (`map_subscribe_transient_local: true`, `subscribe_to_updates: true`, `track_unknown_space: true`, `rolling_window: false`), and `explore_lite` launches at `t=25s`.
  2. **Standalone `rolling_window: true` Option ([`docs/project_migration_and_troubleshooting.md`](../project_migration_and_troubleshooting.md)):** If launching Nav2 exploration without waiting for `/map` initialization, drop `static_layer` from `global_costmap` and set `rolling_window: true`, `width: 30`, `height: 30` so Nav2 activates immediately over a 30 m × 30 m robot-centered window.

### 5.2 Premature Exploration Stop & Empty-Frontier Resilience
- **Symptom:** Stock `explore_lite` terminates exploration early when a single scan cycle returns zero frontiers, when an initial goal immediately aborts (`< 0.8 s`), or when frontiers lie inside the robot's immediate reach (`< min_target_distance`) triggering 0-motion Nav2 completion loops.
- **Root Cause:** Transient costmap update lag or aggressive blacklisting during startup/turns leaves zero valid frontier candidates.
- **Verified Fixes in [`src/m-explore-ros2/explore/src/explore.cpp`](../../src/m-explore-ros2/explore/src/explore.cpp):**
  - **Empty-frontier streak (`kMaxEmptyFrontiersStreak = 5`)**: Requires 5 consecutive empty-frontier cycles before declaring `EXPLORATION_COMPLETE` and stopping.
  - **Instant-abort guard (`< 0.8s`)**: If Nav2 returns `ABORTED` within `0.8 s` of dispatch, the frontier is **not** blacklisted (treating it as a transient planner/start glitch).
  - **Minimum target distance (`min_target_distance: 0.35`)**: Retires frontiers closer than `0.35 m` immediately rather than dispatching 0-motion Nav2 goals.
  - **Blacklist reset**: When all frontiers are blacklisted or within reach, clears `frontier_blacklist_` once to re-evaluate the expanded map.
  - **Hard stall constraint (`progress_timeout`: `8.0 s` sim / `30.0 s` hw)**: Cancels stalled goals if the robot advances `< 0.08 m` over the timeout, blacklists the target, and forces replanning.

---

## 6. DDS Leakage (`ROS_LOCALHOST_ONLY=1`, `GZ_IP=127.0.0.1` vs Domain 7) & Duplicate `/clock`

- **Symptom:** Simulation freezes at `0.000s`, `/scan` or `/clock` drops sporadically on Wi-Fi changes, or remote Raspberry Pi nodes on Domain 7 collide with local Gazebo simulation nodes. Or `gz sim` processes from a previous run continue publishing a duplicate `/clock`.
- **Root Cause:** Leaving `CYCLONEDDS_URI` pointing to `config/cyclonedds.xml` (physical Wi-Fi peer IP) or omitting `ROS_LOCALHOST_ONLY=1` causes simulation traffic to traverse unstable Wi-Fi adapters or mix with Pi hardware topics on `ROS_DOMAIN_ID=7`. Orphan `gz sim` / `parameter_bridge` processes also create duplicate `/clock` publishers.
- **Verified Fix:**
  - All simulation entrypoints (`mapping_session.launch.py`, `view_gazebo.sh`, `view_rviz.sh` in sim mode, `run_bridge.sh` in sim mode, and `run_automated_ml_pipeline.sh`) enforce:
    ```bash
    unset CYCLONEDDS_URI
    export ROS_LOCALHOST_ONLY=1
    export GZ_IP=127.0.0.1
    export ROS_DOMAIN_ID=7
    ```
  - Before starting a new simulation, clean up any lingering processes (`run_automated_ml_pipeline.sh` and `bridge_server.py` automate this):
    ```bash
    pkill -TERM -f "[g]z sim|[p]arameter_bridge|[r]oute_runner|[r]viz2|[n]av2_|[e]kf_node"
    ```

---

## 7. `tf2` Bridge Shutdown Crash (Core Dump on Exit / Mode Switch)

- **Symptom:** Shutting down [`app/bridge/bridge_server.py`](../../app/bridge/bridge_server.py) or switching modes (`POST /api/mode`) triggers a segmentation fault / core dump inside `rclpy` / `tf2_ros`.
- **Root Cause:** `tf2_ros.TransformListener` holds internal C++/subscription callbacks bound to the ROS 2 node. Calling `executor.shutdown()` or `node.destroy_node()` while `_tf_listener` is still referenced causes a destructor order crash.
- **Verified Fix in [`app/bridge/bridge_server.py`](../../app/bridge/bridge_server.py) (`_ros_thread`)**:
  Explicitly release `node._tf_listener = None` and `node._tf_buffer = None` **before** calling `executor.shutdown(timeout_sec=1.0)` and `node.destroy_node()`.

---

## 8. `pytest` ML Perception Timing Note (CPU Contention)

- **Symptom:** Running `pytest src/agv_navigation/test/test_ml_perception.py` occasionally fails `test_moving_pedestrian_classification_and_2s_prediction` at `assert np.median(latencies) < 4.0` when Gazebo Harmonic or heavy builds are running concurrently.
- **Root Cause:** The test measures wall-clock per-scan inference latency (`4.0 ms` threshold) across 20 scans. Heavy background CPU contention (e.g., `gz sim` + Mesa `llvmpipe` software rendering) increases scheduling jitter above `4.0 ms` even though the NumPy runtime itself is unchanged.
- **Verified Fix:** Run `pytest` on an idle CPU (without concurrent Gazebo simulation):
  ```bash
  cd ~/AMR/AMR-main
  .venv_ml/bin/python -m pytest src/agv_navigation/test/test_ml_perception.py -v
  ```

---

## 9. URDF Odometry TF Ownership & Gazebo Bridge Traps

- **Symptom:** Robot jumps or `/tf` reports multiple competing publishers for `odom -> base_link`, or `/scan` is absent in ROS 2.
- **Root Cause:**
  1. Enabling `<publish_odom_tf>true</publish_odom_tf>` in Gazebo's `DiffDrive` plugin conflicts with `ekf_node` (`publish_tf: true` in [`ekf.yaml`](../../src/agv_description/config/ekf.yaml)).
  2. Gazebo Harmonic publishes lidar scans on world-scoped topics (`/world/<world>/model/warehouse_agv/link/lidar_link/sensor/laser/scan`).
- **Verified Fix:**
  - [`warehouse_agv.urdf`](../../src/agv_description/urdf/warehouse_agv.urdf) sets `<publish_odom>true</publish_odom>` and `<publish_odom_tf>false</publish_odom_tf>` so `ekf_node` is the single TF authority for `odom -> base_link`.
  - [`bridge.yaml`](../../src/agv_description/config/bridge.yaml) explicitly maps `/scan` across `test1`, `warehouse`, and `hospital` worlds alongside `/clock`, `/odom`, `/cmd_vel`, `/imu/data`, and `/joint_states`.
