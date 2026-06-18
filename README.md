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
ros2 launch agv_description explore_launch.py
```

#### Running Headlessly
If you are running in a Virtual Machine, WSL, or Docker without GUI acceleration, you can run the Gazebo server headlessly (without the GUI window) by adding the `headless:=true` parameter:
```bash
ros2 launch agv_description explore_launch.py headless:=true
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


---

## 📝 Key Migration & Architecture Notes

During the transition from Gazebo Classic to Gazebo Harmonic, several changes were introduced:

* **Gazebo Harmonic Integration**: Replaced `gazebo_ros` with `ros_gz_sim`. A ROS-Gazebo bridge configuration ([`bridge.yaml`](file:///home/rahul/AMR/AMR-main/src/agv_description/config/bridge.yaml)) handles topic transitions for `/scan`, `/cmd_vel`, `/tf`, and odometry data.
* **Odometry Broadcasting**: The diff-drive plugin inside the URDF ([`warehouse_agv.urdf`](file:///home/rahul/AMR/AMR-main/src/agv_description/urdf/warehouse_agv.urdf)) was updated to explicitly broadcast the `odom -> base_link` transform on `/tf`.
* **SLAM Toolbox Lifecycle**: In ROS 2 Jazzy, `slam_toolbox` runs as a Lifecycle Node. Launching it requires a Lifecycle Manager to transition the node into an `Active` state, which is handled via the official launch includes in [`slam_launch.py`](file:///home/rahul/AMR/AMR-main/src/agv_description/launch/slam_launch.py).
* **Self-Contained World**: The warehouse world was updated to include explicit light and collision plane representations rather than relying on external web resources (`model://sun`, `model://ground_plane`).
