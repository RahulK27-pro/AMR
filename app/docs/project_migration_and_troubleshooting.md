# Project Migration & Troubleshooting Documentation

This document provides a consolidated, comprehensive reference for the migration of the AGV project to modern environments, along with resolutions for critical simulation, mapping, and navigation bugs encountered.

---

## 1. Migration to Gazebo Harmonic
**Transitioning from Gazebo Classic (ROS 2 Humble/Galactic) to Gazebo Harmonic (ROS 2 Jazzy)**

### 1.1 The Initial Failure: `gazebo` Not Found
* **Symptoms:** When attempting to launch the simulation using the legacy `gazebo.launch.py` script, the launch process crashed with:
  ```text
  FileNotFoundError: [Errno 2] No such file or directory: 'gazebo'
  [ERROR] [launch]: Caught exception in launch: "package 'gazebo_ros' not found"
  ```
* **Root Cause:** The legacy files were written for Gazebo Classic (`gazebo` executable and `gazebo_ros` integration package). ROS 2 Jazzy (running on Ubuntu 24.04) has dropped support for Gazebo Classic. The system uses **Gazebo Harmonic** (`gz sim` executable and `ros_gz_sim` integration).
* **Resolution:** 
  1. Updated `gazebo.launch.py` to invoke the command `['gz', 'sim', '-r', '-v', '2']` rather than `['gazebo', '--verbose']`.
  2. Replaced the classic `spawn_entity.py` tool with the entity `create` node provided by `ros_gz_sim`.
  3. Added a `bridge.yaml` configuration to bridge ROS 2 and Gazebo Harmonic topics (such as `/scan` and `/cmd_vel`) via `ros_gz_bridge`.

### 1.2 Missing Sun and Floor
* **Symptoms:** When Gazebo Harmonic launched, the simulation environment was empty, leaving the robot falling into a void, accompanied by terminal errors:
  ```text
  [gz-1] [Err] [Server.cc:86] Error Code 14: Msg: Unable to find uri[model://sun]
  [gz-1] [Err] [Server.cc:86] Error Code 14: Msg: Unable to find uri[model://ground_plane]
  ```
* **Root Cause:** Gazebo Classic cached standard assets locally or loaded them automatically. Gazebo Harmonic fails to resolve standard model URIs like `model://sun` or `model://ground_plane` if there is no internet connection to the Ignition Fuel server or if the local cache is empty.
* **Resolution:** Edited `warehouse.world` to replace `<include>` tags with explicit SDF XML code defining a directional light (sun) and a static ground plane.

### 1.3 Broken URDF Plugins
* **Symptoms:** Gazebo Harmonic crashed or failed to move the robot when parsing the robot's physical description in `warehouse_agv.urdf`.
* **Root Cause:** The legacy URDF referenced classic-only plugins (e.g., `libgazebo_ros_diff_drive.so` and `libgazebo_ros_ray_sensor.so`) and classic materials tags (e.g., `Gazebo/Black`), which the Ogre 2 rendering engine in Gazebo Harmonic cannot parse.
* **Resolution:**
  1. Replaced all `<material>Gazebo/Color</material>` tags with standard `<ambient>` and `<diffuse>` RGB values inside the visual elements.
  2. Changed the LiDAR sensor type from classic ray sensor to Harmonic's native `<sensor type="gpu_lidar">`.
  3. Replaced the old diff-drive plugin with `gz-sim-diff-drive-system`.

---

## 2. Mapping & SLAM Toolbox Integration

### 2.1 Missing Odometry Transform (TF)
* **Symptoms:** The robot's LiDAR scan was working, but SLAM was unable to generate a map. In RViz, the `/map` frame was missing and the status read **"No map received"**.
* **Root Cause:** The `slam_toolbox` node requires wheel odometry transforms (`odom` to `base_link` TF frame) to project LiDAR scans accurately. Although Gazebo Harmonic's Diff Drive plugin calculated odometry, it kept the information internal to Gazebo (on the topic `/model/warehouse_agv/tf`) and did not publish it to ROS 2's global `/tf` topic.
* **Resolution:** Modified the Diff Drive plugin configuration inside `warehouse_agv.urdf` to explicitly broadcast the transform to `/tf`:
  ```xml
  <tf_topic>/tf</tf_topic>
  <publish_odom>true</publish_odom>
  <publish_odom_tf>true</publish_odom_tf>
  ```
  This topic was subsequently bridged in `bridge.yaml` to make the `odom` -> `base_link` transform visible to ROS 2.

### 2.2 Silent SLAM Toolbox (Lifecycle Node Trap)
* **Symptoms:** Even with the odometry transform corrected, running `slam_launch.py` resulted in the node starting, but remaining silent. It failed to subscribe to incoming sensor/TF topics and never published a `/map` topic.
* **Root Cause:** In ROS 2 Jazzy, the `slam_toolbox` node is designed as a **Lifecycle Node**. Unlike standard nodes, it remains dormant in an `Unconfigured` state until explicitly commanded to transition to the `Active` state.
* **Resolution:** Rewrote `slam_launch.py` to use `IncludeLaunchDescription` pointing to the official `online_async_launch.py` launch script:
  ```python
  IncludeLaunchDescription(
      PythonLaunchDescriptionSource(slam_launch_file),
      launch_arguments={
          'slam_params_file': slam_params,
          'use_sim_time': 'true'
      }.items()
  )
  ```
  The official launch file starts a "Lifecycle Manager" that automatically transitions `slam_toolbox` from `Unconfigured` to `Configured` (loading the `mapper_params.yaml`) and finally to `Active`, enabling its subscriptions and map generation.

---

## 3. Autonomous Exploration Costmap Failures
**Troubleshooting why the robot fails to move automatically during exploration.**

### 3.1 The Circular Dependency & Lifecycle Timeout
* **Symptoms:** The robot does not move during exploration, and the `explore_lite` node hangs indefinitely with:
  ```text
  Waiting for costmap to become available, topic: /global_costmap/costmap
  ```
* **Root Cause:** A circular dependency exists during system startup:
  1. The navigation stack (`explore_launch.py`) starts the `planner_server` which hosts the `global_costmap`.
  2. The `global_costmap` uses a `static_layer` configured to wait for the `/map` topic from the SLAM node before activating.
  3. The `slam_toolbox` node does not immediately publish the `/map` topic until it receives enough scan data or the robot moves.
  4. The `nav2_lifecycle_manager` times out and aborts the bringup of the navigation stack because the global costmap node remains blocked.
  5. As a result, the `/global_costmap/costmap` is never published, halting the frontier exploration node (`explore_lite`).

* **Failed Attempt:** Attempting to remove the `static_layer` from the global costmap configuration parameters resulted in a costmap size of 0x0 meters because Nav2 could not determine the environment boundaries.

### 3.2 The Solution: Rolling Window Costmap
* **Resolution:** To eliminate the blocking behavior while preserving exploration capabilities, convert the global costmap to a rolling window costmap:
  1. Remove the `static_layer` from `global_costmap` in `nav2_params_explore.yaml`.
  2. Configure fixed dimensions for the global costmap and enable rolling windows:
     * Set `width: 30` and `height: 30`.
     * Set `rolling_window: true`.
  
  Since the simulated warehouse environment is 16x12 meters, a 30x30 meter rolling window centered on the robot covers the entire workspace at all times without requiring static map initialization from SLAM. This breaks the circular dependency, allowing the navigation stack to boot immediately, which in turn allows `explore_lite` to drive the robot.
