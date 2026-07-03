# Autonomous Mobile Robot (AMR) Workspace

Welcome to the **Autonomous Mobile Robot (AMR)** development workspace. This project contains a simulated Automated Guided Vehicle (AGV) designed for autonomous mapping, navigation, and frontier exploration in a warehouse environment.

The codebase is fully migrated to run on **Ubuntu 24.04** using **ROS 2 Jazzy Jalisco** and **Gazebo Harmonic**.

---

## 📂 Repository Structure

The workspace consists of the following ROS 2 packages under the `src/` directory:

| Package | Description |
| :--- | :--- |
| **[`agv_description`](file:///home/rahul/AMR/AMR-main/src/agv_description)** | Contains the robot URDF (`warehouse_agv.urdf`), warehouse simulation worlds, RViz configurations, launch scripts, and nav2 configuration parameters. |
| **[`agv_navigation`](file:///home/rahul/AMR/AMR-main/src/agv_navigation)** | Contains Python nodes for path-following, topological map building, route planning, and visualization. |
| **[`agv_vision`](file:///home/rahul/AMR/AMR-main/src/agv_vision)** | Contains computer vision nodes, specifically the camera-based obstacle detector. |
| **[`m-explore-ros2`](file:///home/rahul/AMR/AMR-main/src/m-explore-ros2)** | A submodule containing `explore_lite` for frontier-based autonomous exploration and map merging. |

### 📂 File & Directory Structure

```text
AMR-main/
├── README.md
├── docs/                               # Project-wide documentation
│   ├── Phase1_Mapping.md
│   ├── implementation.md
│   └── project_migration_and_troubleshooting.md
└── src/                                # ROS 2 Packages
    ├── agv_description/                # URDF, Simulation, Config & Launch files
    │   ├── config/                     # Configuration parameters for EKF, SLAM, Bridge, Nav2
    │   │   ├── bridge.yaml
    │   │   ├── ekf.yaml
    │   │   ├── mapper_params.yaml
    │   │   └── nav2_params_explore.yaml
    │   ├── launch/                     # ROS 2 launch scripts
    │   │   ├── mapping_session.launch.py   # Launch complete autonomous mapping session
    │   │   ├── gazebo.launch.py
    │   │   ├── slam_launch.py
    │   │   ├── navigation_launch.py
    │   │   └── auto_explore.launch.py
    │   ├── maps/                       # Generated occupancy grid maps (.yaml, .pgm)
    │   ├── scripts/                    # Helper scripts (e.g., save_map.sh)
    │   ├── urdf/                       # warehouse_agv.urdf robot physical description
    │   └── worlds/                     # warehouse.world warehouse simulator world
    ├── agv_navigation/                 # Custom navigation nodes and path trackers
    │   ├── agv_navigation/
    │   │   ├── graph_visualizer.py
    │   │   ├── map_builder.py
    │   │   ├── path_tracer.py
    │   │   └── route_runner.py
    │   └── maps/
    ├── agv_vision/                     # Vision/perception nodes
    │   └── agv_vision/
    │       └── obstacle_detector.py    # Camera-based obstacle detection
    └── m-explore-ros2/                 # Submodule for frontier exploration (explore_lite)
```

---

## 📖 Project Documentation & Phases

Detailed design documentation, configuration guides, and phase-by-phase implementations are stored in the `docs` folder:

1. **[Phase 1: Robot Design & Autonomous Mapping](file:///home/rahul/AMR/AMR-main/docs/Phase1_Mapping.md)**
   - Custom URDF physical modeling (30cm footprint).
   - Sensor integration (LiDAR, Camera, IMU).
   - Differential drive kinematics.
   - SLAM Toolbox setup and Nav2 `explore_lite` frontier exploration.
2. **Phase 2: Autonomous Navigation** *(Pending)*
   - Utilizing the saved maps to perform `navigate_to_pose`.
   - Obstacle avoidance and dynamic re-routing.
3. **Phase 3: High-Level Task Execution** *(Pending)*
   - Integration with behavior trees.
   - Dispatching tasks (e.g., patrolling, warehouse logic).

### Additional Documentation:
- **[System Implementation Details](file:///home/rahul/AMR/AMR-main/docs/implementation.md)**: Deep dive into config parameters, MPPI planner parameters, and frontier explorer tuning.
- **[Project Migration & Troubleshooting Guide](file:///home/rahul/AMR/AMR-main/docs/project_migration_and_troubleshooting.md)**: Details on migrating from Gazebo Classic to Gazebo Harmonic and troubleshooting SLAM Lifecycle/Nav2 Costmap errors.

---

## 🛠️ Build & Installation

### 1. Prerequisites
Ensure you have **ROS 2 Jazzy** and **Gazebo Harmonic** installed on Ubuntu 24.04:
```bash
# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash
```

### 2. Compiling the Workspace
Use `colcon` to build the workspace from the root directory:
```bash
colcon build --symlink-install
```

### 3. Sourcing the Local Workspace
After a successful build, source the setup script to overlay your local packages:
```bash
source install/setup.bash
```

---

## 🚀 How to Run

### Complete Autonomous Exploration Session
To launch the full autonomous session (Gazebo Harmonic, SLAM Toolbox, Nav2, RViz2, and frontier exploration):
```bash
ros2 launch agv_description mapping_session.launch.py
```

#### Running Headlessly
If you are running in a Virtual Machine, WSL, or Docker without GUI acceleration, you can run the Gazebo server headlessly (without the GUI window) by adding the `headless:=true` parameter:
```bash
ros2 launch agv_description mapping_session.launch.py headless:=true
```

#### Saving the Map after Exploration
Once the robot has explored the entire warehouse, open a new terminal and run:
```bash
bash ~/AMR/AMR-main/src/agv_description/scripts/save_map.sh
```

---

### Manual/Component Launch
If you want to spin up the nodes individually:

1. **Launch Gazebo & Spawn the AGV:**
   ```bash
   ros2 launch agv_description gazebo.launch.py
   ```
   *(To run Gazebo server only, use `ros2 launch agv_description gazebo.launch.py headless:=true`)*

2. **Launch SLAM (Mapping):**
   ```bash
   ros2 launch agv_description slam_launch.py
   ```

3. **Launch Navigation (Nav2):**
   ```bash
   ros2 launch agv_description navigation_launch.py
   ```

4. **Launch Frontier Explorer (`explore_lite`):**
   ```bash
   ros2 launch agv_description auto_explore.launch.py
   ```

### 🎮 Manual Keyboard Teleoperation
To drive the AGV manually using your keyboard, run the standard ROS 2 keyboard teleop node in a new terminal:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Use the keys shown in your terminal (typically `i`/`,` for forward/reverse, `j`/`l` for turning left/right, and `k` or `space` to stop) to control the robot.

---

## 📝 Key Migration & Architecture Notes

During the transition from Gazebo Classic to Gazebo Harmonic, several changes were introduced:

* **Gazebo Harmonic Integration**: Replaced `gazebo_ros` with `ros_gz_sim`. A ROS-Gazebo bridge configuration ([`bridge.yaml`](file:///home/rahul/AMR/AMR-main/src/agv_description/config/bridge.yaml)) handles topic transitions for `/scan`, `/cmd_vel`, `/tf`, and odometry data.
* **Odometry Broadcasting**: The diff-drive plugin inside the URDF ([`warehouse_agv.urdf`](file:///home/rahul/AMR/AMR-main/src/agv_description/urdf/warehouse_agv.urdf)) was updated to explicitly broadcast the `odom -> base_link` transform on `/tf`.
* **SLAM Toolbox Lifecycle**: In ROS 2 Jazzy, `slam_toolbox` runs as a Lifecycle Node. Launching it requires a Lifecycle Manager to transition the node into an `Active` state, which is handled via the official launch includes in [`slam_launch.py`](file:///home/rahul/AMR/AMR-main/src/agv_description/launch/slam_launch.py).
* **Self-Contained World**: The warehouse world was updated to include explicit light and collision plane representations rather than relying on external web resources (`model://sun`, `model://ground_plane`).
