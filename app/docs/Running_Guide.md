# AMR Project — Complete Running & App Integration Guide

This guide details the complete workflow to launch the Gazebo simulation, AMCL localization, Dijkstra + MPPI navigation stack, FastAPI bridge server, and the interactive Web Dashboard & Mobile Control App.

---

## 🏗️ Architecture & Component Overview

```
ROS 2 (Gazebo + Nav2 + MPPI)  ←→  FastAPI Bridge (:8000)  ←→  Web Dashboard (:5173)
  /odometry/filtered, /scan            bridge_server.py             (React + Canvas 60 FPS)
  /agv_state, /agv_dense_path                 ↑                             ↑
  /imu/data, /mission_progress                ↓                             ↓
  /cmd_vel, /goal_pose, /agv_estop      REST & WebSocket           Flutter Android App
  /goal_sequence, /dynamic_obstacle      (/ws/telemetry)              (Local Network)
```

---

## 📋 Prerequisites

Open a terminal, navigate to the workspace root, and build & source the workspace:

```bash
cd ~/AMR/AMR-main
colcon build
source install/setup.bash
```

Check required Python packages:
```bash
python3 -c "from scipy.spatial import KDTree; import py_trees, fastapi, uvicorn; print('All core dependencies OK')"
```

---

## 🚀 Step-by-Step Launch Order

You will open **4–5 terminals** for a complete autonomous simulation + control dashboard session.

### Terminal 1 — Simulation & Localization
Launches Gazebo Harmonic, robot state publishers, EKF odometry fusion, map server, AMCL localization, and RViz2:

```bash
cd ~/AMR/AMR-main
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch agv_description navigation_launch.py
```

> **Note:** Wait ~10 seconds until Gazebo and RViz are fully loaded and AMCL is initialized.

---

### Terminal 2 — Route Runner (Autonomous Brain)
Runs the Dijkstra global planner, MPPI dynamic obstacle evasion controller, and state machine:

```bash
cd ~/AMR/AMR-main
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run agv_navigation route_runner
```

*Output:* Look for `Route Runner Active with KD-Tree (756 nodes) & Dynamic Obstacle Avoidance.`

---

### Terminal 3 — Graph Visualizer (RViz Marker Overlay)
*(Optional but recommended for RViz monitoring)*

```bash
cd ~/AMR/AMR-main
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run agv_navigation graph_visualizer
```

*In RViz:* Add a `MarkerArray` subscribed to `/agv_graph_markers` to view topological nodes and edges.

---

### Terminal 4 — FastAPI Bridge Server
Starts the async ROS 2 ↔ Web/Mobile bridge on port 8000:

```bash
cd ~/AMR/AMR-main/app/bridge
chmod +x run_bridge.sh
./run_bridge.sh
```

*Verification:*
```bash
curl http://localhost:8000/api/status
curl http://localhost:8000/api/map/metadata
```

---

### Terminal 5 — React Web Dashboard
Starts the modern 60 FPS Vector Map dashboard:

```bash
cd ~/AMR/AMR-main/app/web_dashboard
chmod +x run_dashboard.sh
./run_dashboard.sh
```

*(Alternatively, run `npm run dev` directly if native Linux Node is in your PATH).*

*Open in Browser:* **`http://localhost:5173`**

---

### Optional — Mobile / Tablet (Flutter Android App)
To monitor and teleoperate from an Android phone on the same Wi-Fi:

```bash
cd ~/AMR/AMR-main/app/flutter_app
flutter pub get
flutter run
```
*In the App:* Enter `http://<YOUR_LAPTOP_IP>:8000` (find IP using `hostname -I`).

---

## 🎮 How to Control & Monitor Using the Web Dashboard

### 1. Interactive 60 FPS Vector Map
- **Smooth Navigation**: Scroll wheel to zoom ($0.5\times$ to $8\times$), right-click / shift-drag to pan the map, and click `[⊙]` to reset view.
- **Click-to-Goal**: Left-click anywhere on the map floorplan to dispatch an autonomous navigation goal.
- **Topological Node Picker**:
  1. Click the **"🗺️ Nodes"** toggle in the top map HUD to show all 756 graph nodes.
  2. Click any node (e.g. `N0`, `N5`, `N12`) to automatically add it to your mission queue.
- **Dynamic Overlays**:
  - **Robot Footprint**: Shows the robot's real-time position, $0.18\text{ m}$ clearance bubble, and heading direction arrow.
  - **MPPI Dense Path**: Renders the dynamic path (`/agv_dense_path`) in glowing neon magenta.
  - **2D LiDAR Hits**: Projects 360° laser obstacle reflections directly in warehouse world coordinates.

---

### 2. Control Interlocks (Manual vs. Autonomous)
- **Automatic Protection**: When a goal or sequence is dispatched, the dashboard enters **`AUTONOMOUS NAV`** mode, locking the joystick to prevent accidental conflicting commands.
- **Manual Teleop Override**: Tap **"✋ Take Manual Control"** or toggle to **`Manual Drive`** to unlock the virtual joystick, directional pad, and speed slider.

---

### 3. Topological Mission Sequence Dispatch
1. In the **Mission Waypoints** card, view your selected nodes (e.g. `[1] N0 ➔ [2] N5 ➔ [3] N12`).
2. Use quick preset buttons:
   - **North Aisle**: `['N0', 'N5', 'N12']`
   - **Aisle Loop**: `['N3', 'N8', 'N15', 'N2']`
   - **Docking**: `['N1', 'N10', 'N4']`
3. Click **"▶ Dispatch Mission Sequence"** to begin autonomous execution across all waypoints.

---

### 4. Dynamic Obstacle Simulation Controls
Test how the AMR reacts to traffic directly from the dashboard:
1. Expand the **"🚧 Dynamic Obstacle Tester"** card on the right panel.
2. Use the directional buttons to drive the simulated obstacle (`/dynamic_obstacle/cmd_vel`) into the robot's path.
3. Observe autonomous behaviors in real time:
   - **Tactical Evasion**: MPPI swerves around moving obstacles when within $0.85\text{ m}$.
   - **Corridor Yielding**: Robot stops (`YIELDING` state) if an aisle is blocked.
   - **Dijkstra Rerouting**: If blocked for $>4.5\text{ s}$, global planner recalculates an alternate route!

---

### 5. Live Diagnostics & Event Timeline
- Located in the center bottom column.
- Automatically captures state changes (`IDLE` ➔ `PLANNING` ➔ `NAVIGATING` ➔ `YIELDING` ➔ `ARRIVED` ➔ `ESTOP`), obstacle warnings, and mission progress.
- Filter by `All`, `Alerts`, or `Missions`.

---

### 6. Emergency Stop (E-STOP)
- Click the prominent red **"EMERGENCY STOP"** button at any time.
- Instantly sends zero velocity (`/cmd_vel`) and asserts `/agv_estop: True`.
- Latches until you click **"✓ Clear E-Stop & Resume"**.

---

## 🛠️ Key Topics Reference

| Topic | Type | Direction | Purpose |
|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | Subscribed | 360° LiDAR obstacle detection |
| `/odometry/filtered` | `nav_msgs/Odometry` | Subscribed | EKF robot pose and velocity |
| `/imu/data` | `sensor_msgs/Imu` | Subscribed | 3-axis robot orientation |
| `/agv_state` | `std_msgs/String` | Published | FSM state (IDLE, NAVIGATING, YIELDING, etc.) |
| `/agv_dense_path` | `nav_msgs/Path` | Published | Current dense MPPI trajectory |
| `/goal_pose` | `geometry_msgs/PoseStamped` | Subscribed | Single navigation goal |
| `/goal_sequence` | `std_msgs/String` (JSON) | Subscribed | Multi-stop mission queue |
| `/mission_progress` | `std_msgs/String` (JSON) | Published | Active waypoint & mission state |
| `/cmd_vel` | `geometry_msgs/Twist` | Published | Robot drive velocity commands |
| `/agv_estop` | `std_msgs/Bool` | Subscribed | Emergency stop latch |
| `/dynamic_obstacle/cmd_vel` | `geometry_msgs/Twist` | Published | Simulated obstacle movement |

---

## 🔍 Troubleshooting

| Problem | Cause | Solution |
|---|---|---|
| Map canvas black / waiting | Bridge server not running | Start `bridge_server.py` in `app/bridge/` |
| Robot does not move to goal | AMCL localization not initialized | Wait for AMCL particle cloud or click "Init AMCL" |
| Joystick does not respond | Autonomous mode interlock active | Click "Override to Manual" in the Control card |
| Mobile app cannot connect | Firewall or wrong IP | Use laptop LAN IP (`hostname -I`), not `localhost`; check port 8000 |
| `NO VALID PATH FOUND` | Target node disconnected | Pick a node connected to the active corridor graph |
