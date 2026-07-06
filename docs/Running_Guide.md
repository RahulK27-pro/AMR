# AMR Project Running Guide

This guide details the steps to launch the autonomous mobile robot (AMR), visualize its navigation network, and execute autonomous routing using the custom Dijkstra + MPPI navigation stack.

## Prerequisites
Open a terminal, navigate to your workspace (e.g., `~/AMR/AMR-main`), and ensure it is built and sourced. You must do this in **every** terminal window you open:
```bash
cd ~/AMR/AMR-main
colcon build
source install/setup.bash
```

---

## Step-by-Step Execution Order

You will need to open multiple terminals to run the system cleanly.

### 1. Launch the Simulation (Terminal 1)
Start the Gazebo simulation environment, spawn the robot, and launch RViz.
```bash
ros2 launch agv_description gazebo.launch.py
```
*(Note: If you have a specific custom launch file for the final navigation world, use that instead. Ensure RViz is open.)*

### 2. Visualize the Topological Graph (Terminal 2)
The navigation stack relies on the `warehouse_graph.json` generated during Phase 2. To render this graph in RViz so you can see the valid paths, run the visualizer node:
```bash
ros2 run agv_navigation graph_visualizer
```
*In RViz:* Ensure you have a `MarkerArray` display added and subscribed to the `/graph_markers` topic to see the nodes and edges.

### 3. Start the Route Runner (Terminal 3)
This is the core navigation node containing the Dijkstra global planner and the MPPI local controller. It will wait idly until it receives a goal.
```bash
ros2 run agv_navigation route_runner
```
You should see terminal output indicating: `"Route Runner Active. Waiting for Goal in RViz..."`

### 4. Optional: Start the Path Tracer (Terminal 4)
If you want to debug odometry drift or simply draw a cool trail showing exactly where the robot has driven over time, launch the tracer:
```bash
ros2 run agv_navigation tracer
```
*In RViz:* Add a `Path` display and subscribe to the `/agv_path` topic.

---

## Executing Autonomous Navigation

Once the above nodes are running, the system is waiting for your command.

1. Go to the **RViz** window.
2. At the top toolbar, click the **"2D Goal Pose"** button (or "Nav2 Goal" depending on your RViz setup).
3. Click and drag anywhere on the map (preferably near a valid topological node) to set the destination.
4. Watch the `route_runner` terminal. You should see it:
   - Snap your goal to the closest node.
   - Print the `DIJKSTRA PATH FOUND`.
   - Transition state to `NAVIGATING`.
5. The MPPI controller will automatically begin publishing velocity commands (`/cmd_vel`) to drive the robot along the graph while avoiding LiDAR (`/scan`) obstacles.

## Troubleshooting

- **No module named networkx:** If you attempt to re-run the `graph_extractor.py` offline map generator and receive this error, ensure your Python dependencies are installed: `sudo apt install python3-opencv python3-yaml` (We updated the script to no longer require `networkx`).
- **Robot spins or hits obstacles:** Check the MPPI weights in `route_runner.py`. You may need to tune `w_collision` or `noise_v` / `noise_w` depending on the robot's actual kinematics.
- **Odometry not ready:** If `route_runner` rejects goals with "Waiting for Odometry", ensure `/odom` messages are actually being published by your robot or simulator plugin.
