# Phase 1: Robot Design & Autonomous Mapping

This document details the configuration and implementation of the autonomous mapping phase.

## 1. Physical Robot Geometry (`warehouse_agv.urdf`)
The robot is modeled to mathematically reflect a compact, differential drive platform.

- **Footprint:** The `base_link` cylinder has a radius of `0.11m` (22cm diameter) and height of `0.10m`.
- **Ground Clearance:** A `1cm` gap is mathematically enforced underneath the chassis (`z=0.01`) to prevent Gazebo from bogging down the physics solver with floor friction. The casters and wheels correctly sit at `z=0`.
- **Sensors:** 
  - **LiDAR:** Mounted on top-center of body at `z=0.125m`. Minimum range clamped to `0.15m` so it does not scan the robot's own chassis.
  - **Camera:** Front-facing, mounted at `x=0.11m`.
  - **IMU:** Placed in the dead center of the chassis for accurate orientation data.
- **Kinematics:** Differential drive plugin configured with exactly `0.26m` wheel separation and `0.025m` wheel radius.

## 2. Nav2 Configuration (`nav2_params_explore.yaml`)
Autonomous exploration relies on the tight integration of MPPI local planning and SLAM.

### Costmaps and Safety
- **Robot Radius:** `0.13m` (0.11m chassis + 0.02m margin).
- **Inflation Radius:** `0.25m` (Provides a 12cm safety buffer from walls).
- **Costmap Broadcasts:** `always_send_full_costmap: true` ensures the global map is constantly streamed to `explore_lite`.
- **Message Filter:** Added `transform_tolerance: 0.5` to prevent LiDAR frames from being dropped due to simulation clock jitter.

### MPPI Local Planner
- **Velocity Limits:** 
  - Max forward velocity: `0.65 m/s` (rapid cruising in open corridors).
  - Max reverse velocity: `-0.15 m/s` (allows safe reversing to escape tight corners).
  - Max angular velocity: `1.4 rad/s` (smooth, agile heading alignment).
- **Behavior Critics:** Calibrated `TwirlingCritic` (weight 1.0) and `PathAlignCritic` (weight 5.0) to eliminate twirling hesitation and allow swift pivot turns.

### Global Planner
- Uses **SmacPlanner2D** (A* 2D grid planner with Euclidean heuristic) rather than NavfnPlanner. This provides high robustness on unknown space borders, handles narrow doorways ($0.9\text{ m}$) reliably, and eliminates legacy potential gradient failures.

## 3. Autonomous Exploration (`explore_lite`)
The frontier explorer is configured for progressive room-by-room coverage:
- **`local_radius`:** Set to `3.5m`. Enforces strict local section prioritization: all frontiers in the active room/section receive a major priority bonus ($-100.0$) and are $100\%$ resolved before the robot transitions to adjacent sections or distant corridors.
- **Safe Free-Space Goal Projection:** Goals are automatically projected $0.25\text{ m}$ into confirmed `FREE_SPACE` rather than targeting unknown frontier boundaries or wall centroids, guaranteeing legal planning paths.
- **Metric Homogeneous Cost Scaling:** Distance penalty is evaluated in physical meters ($d^{1.25}$) against frontier physical length, preventing distant boundaries from overpowering nearby areas.
- **`min_frontier_size`:** Set to `0.30m`.
- **`planner_frequency`:** Set to `0.33 Hz` (every 3 seconds), eliminating the previous 20-second idle lock while maintaining trajectory stability.
- **`track_unknown_space`:** Enabled in the global costmap obstacle layer. This ensures the LiDAR raytracing does not maliciously erase unknown boundaries (frontiers) as the robot approaches them.

## 4. Saving the Map
When the robot completes exploration, the map is saved via the command line:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/AMR/AMR-main/src/agv_description/maps/warehouse_map
```
