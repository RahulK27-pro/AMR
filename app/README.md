# AMR Control App — Setup & Usage Guide

The `app/` directory contains the full control and monitoring stack for the Warehouse AMR.
It is **separate from `src/`** and connects to the ROS 2 system via a FastAPI bridge server.

---

## 📐 Architecture

```
ROS 2 (Gazebo + Nav)              ←→  bridge_server.py  ←→  Flutter Android App
  /scan, /odometry/filtered                 (port 8000)       (same WiFi)
  /agv_state, /agv_dense_path                    ↑
  /imu/data, /obstacle_alert                     ↓
  /cmd_vel, /goal_pose, /agv_estop        Web Dashboard
  /initialpose, /goal_sequence            (port 5173, browser)
```

---

## 📁 Directory Layout

```
app/
├── bridge/
│   ├── bridge_server.py      Enhanced FastAPI + rclpy bridge (v2.0)
│   ├── requirements.txt      Python dependencies
│   └── run_bridge.sh         One-command startup script
├── flutter_app/
│   ├── lib/
│   │   ├── screens/
│   │   │   ├── connect_screen.dart   IP/port entry
│   │   │   ├── home_screen.dart      3-tab shell + nav state badge
│   │   │   ├── map_screen.dart       Live map + tap-to-goal
│   │   │   ├── control_tab.dart      D-pad teleop + E-STOP
│   │   │   └── monitor_tab.dart      Full telemetry dashboard
│   │   └── services/
│   │       └── amr_bridge_service.dart  WebSocket + REST client
│   └── android/app/src/main/
│       └── AndroidManifest.xml      INTERNET + cleartext permissions
└── web_dashboard/
    └── src/components/
        ├── StatusBar, TelemetryPanel, MapView, ScanRing, ControlPanel
```

---

## 🚀 Step-by-Step Startup

### Step 1 — Build & source the ROS 2 workspace
```bash
cd ~/AMR/AMR-main
colcon build
source install/setup.bash
```

### Step 2 — Start the ROS 2 simulation
```bash
# Terminal 1
ros2 launch agv_description navigation_launch.py

# Terminal 2 (after "Managed nodes are active")
ros2 run agv_navigation route_runner

# Terminal 3 (optional — RViz graph overlay)
ros2 run agv_navigation graph_visualizer
```

### Step 3 — Start the bridge server
```bash
cd ~/AMR/AMR-main/app/bridge
chmod +x run_bridge.sh
./run_bridge.sh
# Bridge starts at http://0.0.0.0:8000
```

Verify:
```bash
curl http://localhost:8000/api/status
```

### Step 4 — Web dashboard
```bash
cd ~/AMR/AMR-main/app/web_dashboard
npm run dev
# Open http://localhost:5173
```

### Step 5 — Flutter Android app

Requires Flutter SDK on your machine.

```bash
cd ~/AMR/AMR-main/app/flutter_app

# Generate Android platform files (FIRST TIME ONLY)
flutter create --platforms=android .

# Install dependencies
flutter pub get

# Find your machine's LAN IP
hostname -I   # e.g. 192.168.1.42

# Run on connected Android phone (USB debugging ON)
flutter run
```

In the app: enter `http://192.168.1.42:8000`

---

## 🎮 Features

### Flutter Android

| Screen | Features |
|---|---|
| **Map** | 2s auto-refresh PNG, robot dot overlay, tap-to-goal, AMCL initial pose |
| **Control** | D-pad (hold for continuous 10 Hz cmd_vel), speed slider, goal input, sequence, E-STOP |
| **Monitor** | Pose, velocity, IMU, proximity bar, mission progress, obstacle alert |

### Web Dashboard

| Panel | Features |
|---|---|
| **Header** | Connection status dot, nav state badge, bridge host config, clock |
| **Left** | Full telemetry — pose, velocity, IMU, proximity, mission |
| **Centre** | Map click-to-goal + LiDAR 360° ring |
| **Right** | Nipplejs joystick, E-STOP, goal input, sequence runner |

---

## 🌐 Bridge API

| Endpoint | Method | Description |
|---|---|---|
| `/api/status` | GET | Full telemetry snapshot |
| `/api/map` | GET | Static map JPEG (base64) with robot+path overlay |
| `/api/cmd_vel` | POST `{linear, angular}` | Teleop command |
| `/api/goal` | POST `{x, y}` | Navigate to world coordinates |
| `/api/stop` | POST | Zero velocity + E-STOP |
| `/api/estop/clear` | POST | Clear E-STOP |
| `/api/initial_pose` | POST `{x, y, theta}` | Set AMCL initial pose |
| `/api/goal_sequence` | POST `{nodes:[...]}` | Multi-waypoint mission |
| `/ws/telemetry` | WebSocket | 5 Hz JSON stream |

---

## ⚠️ Troubleshooting

| Problem | Fix |
|---|---|
| Flutter can't connect | Use LAN IP (not localhost); `sudo ufw allow 8000` |
| Map blank | Check `curl http://localhost:8000/api/map`; ensure `graph_visualization.png` exists |
| nav_state UNKNOWN | Ensure `route_runner` is running; check `ros2 topic echo /agv_state` |
| Joystick not working | Drag (not click) the joystick zone; check browser console |
| Bridge can't find map PNG | Set `export AMR_WS=/path/to/AMR-main` before running bridge |
