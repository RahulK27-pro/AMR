# Project Migration & Debugging Documentation
**Transitioning from Gazebo Classic (ROS 2 Humble/Galactic) to Gazebo Harmonic (ROS 2 Jazzy)**

This document provides a detailed breakdown of every error encountered during the migration of the AGV project, why the map was not forming, why SLAM Toolbox got stuck, and exactly what changes were made to resolve these issues.

---

## 1. The Initial Failure: `gazebo` Not Found
**The Error:**
When attempting to run the provided "working version" of `gazebo.launch.py`, the terminal threw the following error:
```text
FileNotFoundError: [Errno 2] No such file or directory: 'gazebo'
[ERROR] [launch]: Caught exception in launch: "package 'gazebo_ros' not found"
```

**Why it happened:**
The provided code was built for **Gazebo Classic**, which uses the executable command `gazebo` and the ROS 2 integration package `gazebo_ros`. However, your system runs **Ubuntu 24.04 and ROS 2 Jazzy**. ROS 2 Jazzy has completely dropped support for Gazebo Classic. The only simulator available on your system is the modern architecture: **Gazebo Harmonic** (executed using `gz sim`).

**How it was solved:**
We had to abandon the old `gazebo_ros` packages and rewrite the launch files to use `ros_gz_sim`. 
1. In `gazebo.launch.py`, the execution command was changed from `['gazebo', '--verbose']` to `['gz', 'sim', '-r', '-v', '2']`.
2. The entity spawner was changed from `spawn_entity.py` (a Classic tool) to the `create` node provided by `ros_gz_sim`.
3. We introduced `bridge.yaml` to run `ros_gz_bridge`. Gazebo Harmonic does not automatically talk to ROS 2; it requires an explicit bridge to translate topics (like `/scan` and `/cmd_vel`) back and forth.

---

## 2. The Missing Sun and Floor
**The Error:**
When Gazebo Harmonic finally launched, it threw these errors in the terminal:
```text
[gz-1] [Err] [Server.cc:86] Error Code 14: Msg: Unable to find uri[model://sun]
[gz-1] [Err] [Server.cc:86] Error Code 14: Msg: Unable to find uri[model://ground_plane]
```

**Why it happened:**
In Gazebo Classic, `model://sun` and `model://ground_plane` were cached locally or fetched automatically from the internet. In Gazebo Harmonic, if you aren't connected to the Ignition Fuel server or lack the local cache, the simulator refuses to render them, leaving the robot falling in an empty void.

**How it was solved:**
We edited `warehouse.world` and replaced the `<include>` tags with the actual explicit SDF code (XML definitions) for a directional `<light>` (the sun) and a static `<model>` with a collision `<plane>` (the ground). This guarantees the world will load regardless of internet connection.

---

## 3. The Broken URDF Plugins
**The Error:**
Gazebo Harmonic crashed or failed to move the robot when reading the `warehouse_agv.urdf` file.

**Why it happened:**
The URDF was heavily reliant on Classic plugins like `libgazebo_ros_diff_drive.so` and `libgazebo_ros_ray_sensor.so`. Furthermore, it used tags like `<material>Gazebo/Black</material>`, which Harmonic's rendering engine (`ogre2`) cannot parse.

**How it was solved:**
1. **Materials:** We removed all `Gazebo/Color` tags and replaced them with standard `<ambient>` and `<diffuse>` RGB values inside `<visual>` blocks.
2. **LiDAR:** We removed the ROS-specific ray sensor plugin and simply used Harmonic's native `<sensor type="gpu_lidar">`.
3. **Diff Drive:** We replaced the old diff-drive plugin with `gz-sim-diff-drive-system`. 

---

## 4. The Silent SLAM Toolbox Bug (Why the Map Was Not Received)
**The Error:**
When running `slam_launch.py`, the terminal outputted:
```text
[INFO] [async_slam_toolbox_node-1]: process started with pid [11027]
```
...and then completely froze. It never printed that it was using a solver, it never published the `/map` topic, and RViz continually displayed **"No map received"**.

**The Debugging Investigation:**
To figure out why SLAM was frozen, we performed three critical tests:
1. `ros2 topic hz /scan` confirmed the LiDAR was working and sending data at 5Hz.
2. `ros2 topic hz /tf` confirmed the odometry and transform trees were flowing at 45Hz.
3. `ros2 node info /slam_toolbox` revealed the smoking gun:
   ```text
   Subscribers:
     /clock: rosgraph_msgs/msg/Clock
     /parameter_events: rcl_interfaces/msg/ParameterEvent
   ```
   **SLAM Toolbox was not subscribing to the `/scan` or `/tf` topics!**

**Why it happened:**
In ROS 2 Jazzy, `slam_toolbox` is architected as a **Lifecycle Node**. A standard node turns on and immediately starts working. A Lifecycle Node turns on in an `Unconfigured` state—it acts dormant and refuses to subscribe to or publish any data until a "Lifecycle Manager" commands it to wake up, load its parameters, and transition to the `Active` state. Because our custom `slam_launch.py` just ran the node directly without a lifecycle manager, SLAM stayed asleep forever.

**How it was solved:**
We rewrote `slam_launch.py`. Instead of launching the node directly, we used `IncludeLaunchDescription` to execute the official `online_async_launch.py` script provided by the developers of `slam_toolbox`. This official script automatically brings up a Lifecycle Manager, passes it our `mapper_params.yaml`, and safely transitions the node to `Active`.

---

## 5. The Odometry TF Failure
*(Note: This occurred just prior to the Lifecycle Node fix, and was a secondary reason the map would not form).*

**The Error:**
Even if SLAM was awake, it would have rejected the LiDAR data because the TF (Transform) tree was broken. RViz could not draw a line between where the robot started (`odom`) and where the robot currently was (`base_link`).

**Why it happened:**
Gazebo Harmonic's Diff Drive plugin calculates the robot's odometry, but by default, it keeps that odometry internal to Gazebo (publishing on `/model/warehouse_agv/tf`). It was not sending it out to the ROS 2 `/tf` topic, meaning ROS 2 was completely blind to the robot's movement.

**How it was solved:**
We modified the Diff Drive plugin inside `warehouse_agv.urdf` to explicitly broadcast the odometry to the global `/tf` topic:
```xml
<tf_topic>/tf</tf_topic>
<publish_odom>true</publish_odom>
<publish_odom_tf>true</publish_odom_tf>
```
Once added, Gazebo began broadcasting the `odom -> base_link` transform. We bridged this topic in `bridge.yaml`, and RViz was finally able to connect the physical LiDAR scans to the global coordinate map.

---

### Conclusion
By migrating the code to Harmonic standards, setting up a solid ROS-to-Gazebo bridge, forcing the simulator to share its odometry, and respecting ROS 2 Jazzy's strict Lifecycle Node requirements, the simulation is now structurally sound, stable, and capable of autonomous mapping.
