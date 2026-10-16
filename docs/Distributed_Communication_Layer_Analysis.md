# Distributed Communication Layer & Failure Analysis

This document provides an exhaustive, layer-by-layer architectural analysis of the communication pipeline implemented for the Autonomous Mobile Robot (AMR) in **Distributed Simulation Mode (Option C)**, detailing how each layer is implemented, where failures occur, and the underlying root causes.

---

## 1. System Topology Overview

The distributed architecture splits the system across two machines connected via local Wi-Fi:

* **Node A: Laptop (Digital Twin Simulator)**
  * **OS/Host**: Windows 11 with WSL2 (Ubuntu 24.04).
  * **IP Address**: `10.87.204.140`.
  * **Roles**: Gazebo Harmonic physics engine, GPU LiDAR shader rendering (`gpu_lidar`), and ROS-Gazebo bridge (`parameter_bridge`).
* **Node B: Raspberry Pi 4/5 (Autonomous Edge Brain)**
  * **OS**: Linux ARM64 (Ubuntu 24.04 / Debian Bookworm).
  * **IP Address**: `10.87.204.118`.
  * **Roles**: Autonomy brain, `robot_state_publisher`, `ekf_node`, `nav2_map_server`, `nav2_amcl`, MPPI `route_runner`, `bt_manager`, FastAPI Bridge (`:8000`), and Web Dashboard (`:5173`).

```
┌──────────────────────────────────────────────────────────────────────────────────┐
│                           5-LAYER COMMUNICATION STACK                            │
├──────────────────────────────────────────────────────────────────────────────────┤
│ Layer 5: Application / Autonomy (TF Tree, EKF, AMCL, MPPI Route Runner)           │
├──────────────────────────────────────────────────────────────────────────────────┤
│ Layer 4: Simulation Bridge (Gazebo Transport ➔ ros_gz_bridge ➔ ROS 2)             │
├──────────────────────────────────────────────────────────────────────────────────┤
│ Layer 3: ROS 2 Node Graph & RMW (rmw_cyclonedds_cpp, Domain ID 7)                │
├──────────────────────────────────────────────────────────────────────────────────┤
│ Layer 2: Transport & DDS Middleware (RTPS, SPDP Discovery, Unicast/Multicast UDP)│
├──────────────────────────────────────────────────────────────────────────────────┤
│ Layer 1: Operating System & Hypervisor (WSL2 Mirrored Mode, Hyper-V Firewall)    │
└──────────────────────────────────────────────────────────────────────────────────┘
```

---

## 2. Layer 1: Operating System & Hypervisor Layer

### 2.1 Implementation
* To allow external physical devices (Raspberry Pi) to reach WSL2 without complicated NAT port-forwarding proxies, WSL2 is configured in **mirrored networking mode** via `C:\Users\<user>\.wslconfig`:
  ```ini
  [wsl2]
  networkingMode=mirrored
  ```
* In mirrored mode, WSL2’s virtual network interface (`eth0`) attaches directly to the host Windows Wi-Fi adapter, mirroring its IP address (`10.87.204.140`).

### 2.2 Failure Mechanism & Root Cause
Inspection of the CycloneDDS subsystem logs on the laptop revealed socket transmission failures:
```text
[robot_state_publisher] ddsi_udp_conn_write to udp/10.87.204.118:9174 failed with retcode -1
[ekf_node]             ddsi_udp_conn_write to udp/239.255.0.1:9150 failed with retcode -1
[ekf_node]             ddsi_udp_conn_write to udp/10.87.204.118:9160 failed with retcode -1
```

1. **Hyper-V Virtual Switch Packet Filter:**
   In Windows 11 mirrored mode, all outbound/inbound traffic crossing between WSL2 and external physical LAN devices passes through the Windows Filtering Platform (WFP) and the Hyper-V Virtual Switch packet filter.
2. **`retcode -1` Socket Rejection:**
   When CycloneDDS calls the Linux kernel syscall `sendto(sock, buf, len, 0, (struct sockaddr *)&dst, ...)`, the underlying driver rejects the packet, returning `-1` (`EPERM` or `ENETUNREACH`).
3. **Multicast Packet Dropping:**
   CycloneDDS by default attempts to broadcast discovery packets to multicast IP `239.255.0.1:9150`. Most standard Wi-Fi routers drop or throttle UDP multicast (via IGMP snooping / wireless multicast suppression), and Windows Defender Firewall drops outbound multicast packets originating from mirrored virtual network interfaces unless explicit Hyper-V firewall bypass rules exist.

---

## 3. Layer 2: Transport & DDS Middleware Layer

### 3.1 Implementation
* **Middleware**: Eclipse CycloneDDS (`rmw_cyclonedds_cpp`).
* **Protocol**: Real-Time Publish-Subscribe (RTPS v2.3 / OMG DDS specification).
* **Domain**: `ROS_DOMAIN_ID=7`.
* **Port Allocation Formula**:
  $$\text{Discovery Multicast Port} = 7400 + (250 \times \text{Domain ID}) = 7400 + 1750 = 9150$$
  $$\text{Unicast Data Port} = 7400 + (250 \times \text{Domain ID}) + 10 + (2 \times \text{Participant ID}) = 9160, 9162, 9164 \dots$$
* Both machines utilize explicit XML configuration for Unicast Peer Discovery:
  * Laptop: `<Peer Address="10.87.204.118"/>`
  * Raspberry Pi: `<Peer Address="10.87.204.140"/>`

### 3.2 Failure Mechanism & Root Cause

#### 1. Asymmetric Execution State (Split-Brain Discovery)
* For two DDS participants to establish an RTPS session over unicast UDP without multicast:
  $$\text{Participant A (Laptop)} \xleftrightarrow{\text{SPDP Discovery Handshake}} \text{Participant B (Pi)}$$
  **Both participants must be actively transmitting and listening at the exact same time.**
* On the Raspberry Pi, when `./connect_pi_to_laptop.sh 10.87.204.140` was executed, it created `/tmp/cyclonedds_pi.xml` and ran diagnostics.
* However, the **autonomy stack already running on the Pi** (PIDs `1606`, `1607`, `1634`, `1635`, `1470`) was started by an earlier script that **did NOT have `CYCLONEDDS_URI` exported in its environment**.
* Therefore, the Pi's active ROS 2 nodes were only listening on default interfaces using multicast. They were completely unaware of the unicast peer configuration pointing to the laptop.

#### 2. Multicast Failure Cascade in CycloneDDS
* Because `<AllowMulticast>false</AllowMulticast>` was not explicitly configured in CycloneDDS XML, CycloneDDS attempted to write to `239.255.0.1:9150` in addition to unicast peers.
* When `sendto()` returned `-1`, CycloneDDS's internal transmit thread (`ddsi_udp_conn_write`) threw fatal socket errors and backed up its transmission queue, refusing to route user topic packets (`/clock`, `/odom`, `/scan`).

---

## 4. Layer 3: ROS 2 Node Graph & RMW Layer

### 4.1 Implementation
* The ROS 2 RMW abstraction layer translates ROS 2 primitives (`create_publisher`, `create_subscription`) into DDS DataWriters and DataReaders.
* Topics exposed across the network boundary:
  * `/clock` (`rosgraph_msgs/msg/Clock`)
  * `/odom` (`nav_msgs/msg/Odometry`)
  * `/scan` (`sensor_msgs/msg/LaserScan`)
  * `/cmd_vel` (`geometry_msgs/msg/Twist`)

### 4.2 Failure Mechanism & Root Cause

#### 1. The "Topic List" False Positive Trap
* In ROS 2, `ros2 topic list` returns any topic that has **either an active publisher OR an active subscriber**.
* On the Raspberry Pi, because `robot_state_publisher` and `ekf_node` are running with `use_sim_time: true`, they subscribe to `/clock`, `/odom`, and `/scan`.
* When connection scripts check:
  ```bash
  ros2 topic list | grep -q "/clock"
  ```
  It returns `true` **locally on the Pi**, because the subscriber exists on the Pi, leading to a false report that the simulation is connected.
* When actual message arrival is measured over a 2.0-second window, 0 messages are received:
  `• /clock : ❌ DEAD (0 Hz)`

#### 2. Stale ROS 2 Daemon Socket
* ROS 2 runs a background Python XMLRPC daemon (`ros2 daemon`) that caches node graph states.
* When WSL2 switched IP configuration from NAT to mirrored, the ROS 2 daemon process remained bound to an invalid socket. When CLI commands (`ros2 topic hz`, `ros2 topic info`) were invoked, they hung for 90 seconds attempting to connect to the dead daemon before throwing:
  ```text
  TimeoutError: [Errno 110] Connection timed out
  ```

---

## 5. Layer 4: Simulation Bridge Layer (Gazebo Transport ➔ ROS 2)

### 5.1 Implementation
* Gazebo Harmonic uses internal `gz-transport` (ZeroMQ/protobuf):
  * `/clock` (`gz.msgs.Clock`)
  * `/odom` (`gz.msgs.Odometry`)
  * `/world/test1/model/warehouse_agv/link/lidar_link/sensor/laser/scan` (`gz.msgs.LaserScan`)
* The `ros_gz_bridge` (`parameter_bridge`) node bridges these `gz-transport` topics into ROS 2 DDS messages.

### 5.2 Failure Mechanism & Root Cause
* Process table inspection on the laptop revealed:
  ```bash
  ps aux | grep -E "gz|parameter_bridge|ros2"
  ```
  Two separate instances of `/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch agv_description gazebo.launch.py` were running concurrently.
* When `./stop_sim.sh` was previously run, it terminated `gz sim` and `parameter_bridge` by process name, but **did not terminate the parent Python ROS 2 launch wrapper**.
* The parent launch process remained alive as an orphaned process holding locks on the ROS 2 domain. When a second launcher started, port collisions occurred, resulting in the actual simulation server (`gz sim server`) and `parameter_bridge` failing to stay alive.

---

## 6. Layer 5: Autonomous Edge & Robot Logic Layer

### 6.1 Implementation
* All autonomy nodes on the Pi (`robot_state_publisher`, `ekf_node`, `map_server`, `amcl`, `route_runner`) run with:
  ```python
  use_sim_time: True
  ```
* Under `use_sim_time: True`, the ROS 2 system clock (`rclcpp::Clock` / `rclpy.clock`) stops using the physical hardware clock and synchronizes entirely with timestamps received on `/clock`.

### 6.2 Failure Mechanism: The Cascade Freeze
Because Layer 1 and Layer 2 dropped `/clock`, a cascading stall occurred through the entire autonomy stack:

```
/clock is 0 Hz (DEAD)
       │
       ▼
ROS 2 Sim Time Freezes at 0.000s
       │
       ▼
EKF Filter Node (`ekf_node`) cannot tick
       │
       ▼
`odom ➔ base_link` TF transform is NEVER published
       │
       ▼
AMCL cannot match LiDAR particles without odometry
       │
       ▼
`map ➔ odom` TF transform is NEVER published
       │
       ▼
MPPI Route Runner attempts `lookup_transform("map", "base_link")`
       │
       ▼
Transform Lookup Fails ➔ Route Runner refuses to move:
"[WARN] [route_runner]: Waiting for localization/odometry before accepting goals."
Robot held at 0.0 m/s.
```

---

## 7. Diagnostic Matrix & Root Cause Summary

| Layer | Component | Exact Failure Mechanism | Impact |
| :--- | :--- | :--- | :--- |
| **Layer 1 (OS/Hypervisor)** | Windows Hyper-V / WSL2 | Hyper-V packet filter rejects outbound UDP sockets (`ddsi_udp_conn_write failed with retcode -1`). | Packets cannot leave laptop physical NIC. |
| **Layer 2 (Transport/DDS)** | CycloneDDS | Multicast packets to `239.255.0.1` dropped; Pi's background nodes were started without `CYCLONEDDS_URI` pointing to laptop. | DDS participants cannot handshake. |
| **Layer 3 (ROS 2/RMW)** | ROS 2 Discovery | Pi detects `/clock` in topic list due to local subscribers, but 0 bytes arrive from remote publisher. | False positive reporting in naive scripts. |
| **Layer 4 (Simulation)** | Gazebo & Bridge | Duplicate orphaned Python launch processes on laptop caused port contention on the bridge. | Bridge processes fail to stream topics. |
| **Layer 5 (Autonomy)** | Autonomy Stack | `use_sim_time: True` freezes the Pi's internal node clocks, preventing TF generation and path planning. | Robot holds 0.0 velocity indefinitely. |
