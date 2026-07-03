# Dynamic RViz Goals and MPPI Controller Implementation Plan

This plan details how to upgrade your existing custom topological navigation system (`route_runner.py`) to support dynamic goal selection via RViz, pathfinding via Dijkstra, and local trajectory control using a Model Predictive Path Integral (MPPI) controller with collision avoidance.

---

## 1. Core Architecture Assumptions

Based on our discussion, the MPPI implementation will assume the following baseline configurations for the AMR:

- **Kinematic Limits:**
  - Max Linear Velocity ($v_{max}$): `1.0 m/s` (or as per your robot's config)
  - Min Linear Velocity ($v_{min}$): `0.0 m/s` (no reversing in standard operation)
  - Max Angular Velocity ($\omega_{max}$): `1.5 rad/s`
- **MPPI Tuning Parameters:**
  - Control Frequency: `10 Hz`
  - Prediction Horizon ($T$): `2.0 seconds`
  - Number of Trajectory Samples ($K$): `~100-200 samples` per iteration
- **Collision Avoidance:**
  - The node will subscribe to a 2D `LaserScan` (or local `OccupancyGrid` costmap).
  - The MPPI cost function will heavily penalize simulated trajectories that collide with or come dangerously close to sensed obstacles.

---

## 2. Proposed Changes

### `agv_navigation`

#### [MODIFY] route_runner.py

1. **New ROS 2 Subscriptions:**
   - **`/goal_pose` (geometry_msgs/PoseStamped):** 
     - Subscribe to the RViz "2D Goal Pose" tool.
     - Extract `x`, `y` coordinates.
     - Use `find_closest_node(x, y)` to set the target topological node.
     - Trigger `calculate_dijkstra()` to find the global path.
   - **`/scan` (sensor_msgs/LaserScan) OR `/local_costmap/costmap`:**
     - Subscribe to the sensor data.
     - Maintain an internal representation of local obstacles for the MPPI cost evaluator.

2. **State Management Refactoring:**
   - Introduce state flags: `IDLE`, `PLANNING`, `NAVIGATING`.
   - Start in `IDLE`.
   - On receiving `/goal_pose`, transition to `PLANNING`, compute the path, then transition to `NAVIGATING`.
   - Stop (`/cmd_vel` = 0) and transition to `IDLE` upon reaching the final node.

3. **MPPI Local Controller Implementation:**
   - Remove the simple P-controller.
   - Create a timer running at 10 Hz that executes the MPPI loop when in the `NAVIGATING` state:
     - **Sample:** Generate $K$ random control sequences (linear & angular velocities) of length $N$ (where $N = 2.0s \times 10Hz = 20$ timesteps).
     - **Rollout:** Simulate the robot's future poses (unicycle model) for each sequence, starting from the latest Odometry ping.
     - **Evaluate Cost:** For each simulated trajectory, calculate the cost:
       - *Path Tracking Cost:* Distance to the active segment of the Dijkstra path.
       - *Heading Cost:* Alignment with the target path direction.
       - *Collision Cost:* If any simulated pose hits an obstacle (based on the latest LaserScan/Costmap data), apply a massive penalty cost (near infinity).
     - **Update:** Compute the optimal control sequence by taking a cost-weighted average of the valid sampled control sequences (using the exponential path integral formulation).
     - **Execute:** Publish the first timestep's optimal velocity command to `/cmd_vel`.

---

## 3. Verification Plan

### Manual Verification
1. **Launch the Navigation Stack:** Start the `route_runner` node along with your sensor/simulation nodes.
2. **Open RViz2:** Load the Map, RobotModel, and configure the "2D Goal Pose" tool. Also visualize the `/scan` or costmap.
3. **Dynamic Goal Testing:** Click a point near an existing topological node. Ensure Dijkstra logs a successful path and the robot begins moving.
4. **MPPI Path Tracking Verification:** Ensure the robot smoothly follows the topological segments without oscillation.
5. **Collision Avoidance Verification:** Place a dynamic obstacle (or simulate one) on the path. The MPPI controller should naturally swerve around it or stop if the path is completely blocked.
