# Phase 3: Navigation Architecture

## Overview
Phase 3 establishes the complete navigation stack for the AMR. It takes the output from Phase 2 (the JSON topological graph) and uses it to enable autonomous, collision-free movement from any point in the warehouse to a user-specified goal. 

The architecture is divided into two primary layers:
1. **Global Planner (Dijkstra):** Operates on the topological graph to find the shortest path of nodes.
2. **Local Planner (MPPI):** Operates on real-time sensor data (LiDAR) and odometry to execute the global path while dynamically avoiding dynamic obstacles.

All of these features are implemented inside the `agv_navigation` ROS 2 package.

---

## Core Components

### 1. The Route Runner (`route_runner.py`)
This is the primary navigation node and the brain of the AMR. It handles both global pathfinding and local motion control.

#### Global Planning (Dijkstra's Algorithm)
When the user publishes a goal pose (typically via RViz's "2D Goal Pose" tool), the `RouteRunner`:
1. **Snapping:** Finds the closest topological node in `warehouse_graph.json` to the robot's current odometry position, and the closest node to the goal coordinate.
2. **Pathfinding:** Uses a priority-queue implementation of Dijkstra's algorithm to search the JSON graph.
3. **Waypoint Generation:** Extracts the shortest sequence of nodes leading to the destination and stores them as active waypoints.

#### Local Planning (MPPI - Model Predictive Path Integral)
Instead of a standard PID controller or DWA, this stack uses MPPI, a state-of-the-art predictive controller. Running at 10 Hz, the controller:
1. **Sampling:** Generates 150 random control sequences (linear and angular velocities) over a 2.0-second prediction horizon.
2. **Rollout Simulation:** Uses a kinematic model to predict exactly where the robot would end up for every single sequence based on its current pose (`/odom`).
3. **Cost Evaluation:** Evaluates every predicted path against three criteria:
   - **Distance Cost:** How close does the path get to the active waypoint?
   - **Heading Cost:** Is the robot facing the correct direction?
   - **Collision Cost:** Uses real-time LiDAR (`/scan`) mapped into global coordinates. If a predicted path comes within 0.4m of an obstacle, it is heavily penalized.
4. **Execution:** Uses a softmax weighting function (temperature $\lambda$) to blend the best, collision-free trajectories into a single optimal `Twist` command published to `/cmd_vel`.

### 2. The Path Tracer (`path_tracer.py`)
This node is used for telemetry, debugging, and visualization.
- It subscribes to wheel odometry (`/odom`) and the IMU (`/imu`).
- It constructs a continuous `nav_msgs/Path` message representing the historical trail of where the robot has actually driven.
- By comparing the wheel encoder translation with the IMU yaw rate, you can visualize and debug wheel slip or odometry drift over time directly in RViz.

### 3. Graph Visualizer (`graph_visualizer.py`)
Because the global planner uses a custom JSON graph instead of standard Nav2 costmaps, this node parses `warehouse_graph.json` and converts it into ROS 2 `MarkerArray` messages. This allows the user to see the exact nodes and edges overlaid on top of the map in RViz.

---

## Summary of the Data Flow
1. **Graph Loader:** `route_runner` loads `warehouse_graph.json` into memory.
2. **Goal Input:** User clicks RViz -> `/goal_pose`.
3. **Path Generation:** Dijkstra computes Node A -> Node B -> Node C.
4. **Perception:** `/scan` updates obstacle arrays; `/odom` updates position.
5. **Control Loop:** MPPI evaluates 150 paths avoiding `/scan` obstacles towards Node A.
6. **Actuation:** Optimal velocity sent to `/cmd_vel` until Node A is reached, then targets Node B.
