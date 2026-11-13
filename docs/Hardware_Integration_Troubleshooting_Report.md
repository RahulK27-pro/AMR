# AMR Physical Hardware & Cross-Machine ROS 2 Integration: Comprehensive Engineering & Troubleshooting Report

**Project**: Autonomous Mobile Robot (AMR) Warehouse AGV  
**Hardware Platform**: Raspberry Pi 5 (RP1 Southbridge, Ubuntu 24.04 LTS, ROS 2 Jazzy, ARM64)  
**Remote Visualizer / Workstation**: Laptop (Windows 11 + WSL2 Ubuntu 24.04 LTS, ROS 2 Jazzy, x86_64)  
**Middleware**: CycloneDDS (`rmw_cyclonedds_cpp`, ROS Domain ID `7`)  
**Sensors & Actuators**: RMCS-2305 Dual DC Motor Driver, 6-wire Quadrature Encoders ($r=0.070\text{ m}$, $L=0.240\text{ m}$, $4713\text{ ticks/rev}$), GY-91 MPU-6500 IMU, YDLIDAR X2 LiDAR.

---

## 1. Executive Summary & Hardware Architecture

During physical deployment, the AMR stack transitions from Gazebo Harmonic simulation to real-world embedded hardware. The software architecture partitions low-level sensor acquisition, motor actuation, and high-frequency state estimation onto the onboard single-board computer (Raspberry Pi 5), while offloading heavy graphical rendering (RViz2) and remote operator controls (teleoperation) to an external laptop over a wireless local area network (Wi-Fi).

```
+-------------------------------------------------------------------------------------------------+
|                                 PHYSICAL HARDWARE ROBOT (Raspberry Pi 5)                        |
|                                       IP: 10.73.81.208 (amr@amrita)                             |
|                                                                                                 |
|   +----------------------------+   +------------------------+   +---------------------------+   |
|   | amr_hardware_drive_node.py |   |    amr_imu_node.py     |   |    amr_lidar_node.py      |   |
|   |  - RMCS-2305 (PWM/DIR)     |   |  - MPU-6500 (I2C 0x68) |   |  - YDLIDAR X2 (/dev/ttyUSB0)|
|   |  - Dual Encoders (4713 cpr)|   |  - 6-DOF Accel / Gyro  |   |  - 360 deg 2D LaserScan   |   |
|   |  - Publishes /odom (50 Hz) |   |  - Pub /imu/data(50Hz) |   |  - Publishes /scan (24 Hz)|   |
|   +--------------+-------------+   +-----------+------------+   +-------------+-------------+   |
|                  |                             |                              |                 |
|                  +--------------------+        |                              |                 |
|                                       v        v                              |                 |
|                            +-----------------------+                          |                 |
|                            |   ekf_node (robot_    |                          |                 |
|                            |   localization fusion)|                          |                 |
|                            |   Pub /odometry/      |                          |                 |
|                            |   filtered (50 Hz)    |                          |                 |
|                            +-----------+-----------+                          |                 |
|                                        |                                      |                 |
|   +----------------------------+       |                                      |                 |
|   |   robot_state_publisher    |       |                                      |                 |
|   |   Publishes /tf, /tf_static|       |                                      |                 |
|   +--------------+-------------+       |                                      |                 |
|                  |                     |                                      |                 |
|                  v                     v                                      v                 |
|     =======================================================================================     |
|                      CycloneDDS Middleware (ROS_DOMAIN_ID=7, rmw_cyclonedds_cpp)                |
|                      Unicast Peer: 127.0.0.1 + Remote Peer: 10.73.81.140                        |
|     =======================================================================================     |
+------------------------------------------------|------------------------------------------------+
                                                 | (Wi-Fi LAN: Unicast UDP Ports 7400-9500)
                                                 v
+------------------------------------------------|------------------------------------------------+
|     =======================================================================================     |
|                      CycloneDDS Middleware (ROS_DOMAIN_ID=7, rmw_cyclonedds_cpp)                |
|                      NetworkInterface: 10.73.81.140, Peers: 127.0.0.1, 10.73.81.208             |
|     =======================================================================================     |
|                                                |                                                |
|                                                v                                                |
|   +--------------------------------------------+--------------------------------------------+   |
|   |                                 DEVELOPMENT LAPTOP (WSL2 Ubuntu 24.04)                  |   |
|   |                                       IP: 10.73.81.140 (rahul@Rahul)                    |   |
|   |                                                                                         |   |
|   |   +------------------------------------+    +---------------------------------------+   |   |
|   |   |        connect_laptop_rviz.sh      |    |           teleop_hardware.sh          |   |   |
|   |   |  - RViz2 (Fixed Frame: odom)       |    |  - teleop_twist_keyboard              |   |   |
|   |   |  - Visualizes /scan, /odom, Robot  |    |  - Publishes /cmd_vel -> Robot        |   |   |
|   |   +------------------------------------+    +---------------------------------------+   |   |
+-------------------------------------------------------------------------------------------------+
```

While deploying and testing this setup across the two nodes, several critical blockers emerged across six layers:
1. DDS XML Schema Specification
2. DDS Discovery Loopback Topology
3. WSL2 Mirrored Networking Loopback RPC Timeout
4. Multi-NIC Interface Selection in Hypervisor Environments
5. Windows Defender Packet Filtering & Firewall Policy
6. Linux Kernel GPIO Exclusivity (`lgpio` on RP1)
7. ROS 2 Transform Tree Root Inconsistency in RViz2

This report provides the technical mechanisms, diagnostic steps, and resolutions for each issue.

---

## 2. Comprehensive Root-Cause Matrix

| # | Subsystem / Layer | Observed Symptom | Root Cause | Engineering Resolution |
|---|---|---|---|---|
| **1** | **DDS XML Schema** | `RCLError: failed to create domain` on Pi launch. Exit code -6 / 1. | `<NetworkInterface autoselect="true"/>` is invalid in CycloneDDS schema; parser rejected the configuration. | Replaced with dynamically generated `<NetworkInterface address="$PI_MY_IP"/>` in `start_hardware.sh`. |
| **2** | **DDS Discovery Topology** | Terminal 1 runs nodes on Pi, but `ros2 node list` in Terminal 2 on Pi is empty. | Pure unicast peering to remote laptop bypassed local loopback; local processes could not discover one another. | Added `<Peer Address="127.0.0.1"/>` explicitly to `<Peers>` list in all CycloneDDS configuration files. |
| **3** | **WSL2 Networking / Python CLI** | `ros2 node list` hangs for 60 seconds on laptop, crashing with `TimeoutError: [Errno 110]`. | WSL2 mirrored mode drops SYN packets to closed ports on `127.0.0.1:11518` instead of returning TCP RST; `ros2cli` daemon client hangs. | Implemented a smart bash function wrapper in `env_hardware.sh` routing CLI list/hz/echo verbs with `--no-daemon`. |
| **4** | **WSL2 Command Syntax** | `ros2: error: unrecognized arguments: --no-daemon`. | An initial alias placed `--no-daemon` at the root command rather than passing it to specific sub-verbs. | Converted alias into a multi-branch bash function that inserts `--no-daemon` directly after verbs (`node list`, `topic list`). |
| **5** | **Hypervisor Multi-NIC Routing** | Laptop discovers only its own static TF node; 0 nodes visible from Pi over Wi-Fi. | CycloneDDS arbitrarily bound its UDP sockets to dummy adapter `zeth` (`192.0.2.2`) instead of physical Wi-Fi `eth0` (`10.73.81.140`). | Bound `<NetworkInterface address="10.73.81.140"/>` explicitly in `cyclonedds.xml` and `connect_laptop_rviz.sh`. |
| **6** | **Windows Defender Firewall** | `test_udp_link.py` timed out on laptop; UDP packets from Pi dropped at network boundary. | Windows Firewall drops unsolicited incoming UDP packets in the dynamic DDS range (7400–9500) directed to WSL2. | Created Windows Defender Firewall Inbound Allow rule for UDP 7400–9500 in PowerShell. |
| **7** | **Linux Kernel GPIO Subsystem** | Running `test_motor_teleop.py` while hardware stack is running produced no wheel rotation. | `amr_hardware_drive_node.py` held exclusive locks on RP1 GPIO lines; second script silently defaulted to mock mode. | Built `teleop_hardware.sh` publishing `/cmd_vel` over DDS Domain 7 to the running drive node. |
| **8** | **TF Tree / RViz2 Rendering** | Blank black viewport in RViz2; error `Frame [map] does not exist`. | RViz defaulted to `Fixed Frame: map`, but map server/SLAM is not active during pure hardware bringup (root is `odom`). | Created `agv_hardware.rviz` with `Fixed Frame: odom` and added a background fallback static TF publisher (`map -> odom`). |

---

## 3. Deep-Dive Failure Analyses & Solutions

### Issue 1: CycloneDDS XML Schema Violation (`autoselect: unknown attribute`)

#### Symptom
When executing `./start_hardware.sh 10.73.81.140` on the Raspberry Pi 5, the launch command aborted instantaneously with the following terminal output:
```text
[robot_state_publisher-1] 1794590849.646124 [7] robot_stat: config: //CycloneDDS/Domain/General/Interfaces/NetworkInterface: autoselect: unknown attribute (file:///tmp/cyclonedds_pi.xml line 7)
[robot_state_publisher-1] [ERROR] [1794590849.647798519] [rmw_cyclonedds_cpp]: rmw_create_node: failed to create domain, error Error
[robot_state_publisher-1] terminate called after throwing an instance of 'rclcpp::exceptions::RCLError'
[robot_state_publisher-1]   what():  failed to initialize rcl node: error not set, at ./src/rcl/node.c:252
[ERROR] [robot_state_publisher-1]: process has died [pid 4127, exit code -6]
```

#### Technical Mechanism
The CycloneDDS parser parses the XML file specified in `$CYCLONEDDS_URI` against its XSD schema (`https://cdds.io/config`). In an earlier iteration, `start_hardware.sh` generated an XML node:
```xml
<NetworkInterface autoselect="true"/>
```
The CycloneDDS schema does not permit an `autoselect` attribute under `NetworkInterface`. When the parser encountered this unexpected attribute, it aborted immediately and returned an error code to the ROS 2 RMW abstraction layer (`rmw_cyclonedds_cpp`). Because `rmw_create_node` received `Error`, `rclcpp` threw an unhandled `RCLError`, terminating all five hardware nodes with exit code -6 (`SIGABRT`).

#### Resolution
The startup script was updated to auto-detect the Pi's active IP address on the default routing interface via `ip route get "$LAPTOP_IP"` and write a strictly compliant `<NetworkInterface address="$PI_MY_IP"/>` tag:
```bash
PI_MY_IP=$(ip route get "$LAPTOP_IP" 2>/dev/null | awk '{for(i=1;i<=NF;i++) if($i=="src") print $(i+1)}' || hostname -I | awk '{print $1}')
cat <<EOF > "$CFG_FILE"
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain Id="7">
    <General>
      <Interfaces>
        <NetworkInterface address="$PI_MY_IP"/>
      </Interfaces>
    </General>
    ...
```

---

### Issue 2: CycloneDDS Pure Unicast Peering & Intra-Machine Loopback Isolation

#### Symptom
After fixing Issue 1, all 5 nodes started successfully on the Pi in Terminal 1 (`hardware_bringup.launch.py`). However, opening Terminal 2 on the Pi and executing `ros2 node list` or `./verify_hardware_topics.sh` returned zero active nodes and empty topic lists:
```text
amr@amrita:~/AMR$ ros2 node list
amr@amrita:~/AMR$ 
```

#### Technical Mechanism
By default, DDS relies on Simple Participant Discovery Protocol (SPDP) using UDP multicast on `239.255.0.1`. When CycloneDDS is configured with a `<Peers>` element listing only external unicast addresses (e.g., `<Peer Address="10.73.81.140"/>`), CycloneDDS disables automatic broadcast/multicast discovery on `lo` (`127.0.0.1`). 

Consequently, Terminal 1 (running the hardware launch file) and Terminal 2 (running `ros2 node list`) were running on the same computer, but each sent discovery participant announcements **only** to the external IP (`10.73.81.140`). Neither process was transmitting discovery packets to `127.0.0.1`. Thus, the two local processes were completely blind to one another.

#### Resolution
In `start_hardware.sh`, `connect_laptop_rviz.sh`, and `cyclonedds.xml`, the loopback address was explicitly added as a permanent discovery peer:
```xml
<Discovery>
  <Peers>
    <Peer Address="127.0.0.1"/>
    <Peer Address="$REMOTE_IP"/>
  </Peers>
</Discovery>
```
With `127.0.0.1` included, local processes announce themselves across loopback while simultaneously streaming across the LAN to the remote machine. Terminal 2 immediately listed all nodes:
```text
/amr_hardware_drive_node
/amr_imu_node
/amr_lidar_node
/ekf_node
/robot_state_publisher
```

---

### Issue 3: WSL2 Mirrored Networking Daemon Socket Timeout (`TimeoutError: [Errno 110]`)

#### Symptom
On the development laptop (Ubuntu 24.04 inside Windows 11 WSL2), running `ros2 node list` or `ros2 topic list` would hang for 60 seconds and fail with a Python exception:
```text
Traceback (most recent call last):
  File "/opt/ros/jazzy/bin/ros2", line 33, in <module>
    sys.exit(load_entry_point('ros2cli==0.32.10', 'console_scripts', 'ros2')())
  File "/opt/ros/jazzy/lib/python3.12/site-packages/ros2cli/cli.py", line 91, in main
    rc = extension.main(parser=parser, args=args)
  File "/opt/ros/jazzy/lib/python3.12/site-packages/ros2cli/node/daemon.py", line 46, in connected
    for method in self._proxy.system.listMethods():
  File "/usr/lib/python3.12/xmlrpc/client.py", line 1122, in __call__
    return self.__send(self.__name, args)
  File "/usr/lib/python3.12/socket.py", line 837, in create_connection
    raise err
TimeoutError: [Errno 110] Connection timed out
```

#### Technical Mechanism
Under ROS 2 Jazzy, `ros2cli` defaults to using a background daemon process (`_ros2_daemon`) running an XML-RPC server listening on a dynamic or static loopback port (`127.0.0.1:11518`). 
- On native Linux, if no daemon is running, connecting to `127.0.0.1:11518` causes the kernel network stack to instantly return a `TCP RST` packet. Python immediately catches `ConnectionRefusedError` (taking < 2 ms) and spawns a direct DDS discovery query.
- In **Windows 11 WSL2 with `networkingMode=mirrored`**, loopback TCP packets are intercepted by the Windows Hyper-V Virtual Switch / Windows Filtering Platform (`loopback0`). Unopened ports do not return a `TCP RST` segment; instead, the connection requests are silently dropped.
- Python's standard `socket.create_connection` waits for the kernel timeout (60 seconds) before raising `TimeoutError: [Errno 110] Connection timed out`.

#### Diagnostic Step & Initial Pitfall
We tested running `ros2 node list --no-daemon`. It completed in **0.18 seconds**, demonstrating that the daemon RPC socket was the sole failure point.

However, creating a simple bash alias:
```bash
alias ros2='ros2 --no-daemon'
```
Failed because `ros2cli` does not permit `--no-daemon` as a root command flag:
```text
usage: ros2 [-h] [--use-python-default-buffering]
ros2: error: unrecognized arguments: --no-daemon
```

#### Resolution
We engineered a context-aware shell wrapper in `env_hardware.sh`:
```bash
# Smart wrapper to bypass WSL2 loopback daemon socket timeouts
ros2() {
    if [ "$1" = "node" ] && [ "$2" = "list" ]; then
        shift 2
        command ros2 node list --no-daemon "$@"
    elif [ "$1" = "topic" ] && [ "$2" = "list" ]; then
        shift 2
        command ros2 topic list --no-daemon "$@"
    elif [ "$1" = "topic" ] && [ "$2" = "hz" ]; then
        shift 2
        command ros2 topic hz --no-daemon "$@"
    elif [ "$1" = "topic" ] && [ "$2" = "echo" ]; then
        shift 2
        command ros2 topic echo --no-daemon "$@"
    else
        command ros2 "$@"
    fi
}
```
When a user types `ros2 node list` or `ros2 topic list`, the function transparently injects `--no-daemon` after the verb, achieving immediate sub-second query response.

---

### Issue 4: Hypervisor Multi-Interface Selection (`zeth` vs `eth0`)

#### Symptom
On the laptop, running `ros2 node list` returned only:
```text
/static_transform_publisher_E8WWK4z7WqCUc94B
```
This was the local static TF node launched by `connect_laptop_rviz.sh`. None of the robot's physical nodes on the Raspberry Pi (`10.73.81.208`) were discovered, despite both devices being connected to the same Wi-Fi network and reachable via ICMP `ping`.

#### Technical Mechanism
Running `ip -br addr` inside WSL2 revealed multiple virtual network adapters:
```text
lo               UNKNOWN        127.0.0.1/8 ::1/128 
eth0             UP             10.73.81.140/22 fe80::...
zeth             UP             192.0.2.2/24 
docker0          DOWN           172.17.0.1/16 
```
When CycloneDDS initializes without an explicit `<NetworkInterface>` directive, its heuristic selects the first active non-loopback network interface. In WSL2 with mirrored networking, it selected `zeth` (`192.0.2.2`, an internal Windows container management link). 

CycloneDDS bound all UDP discovery and RTPS data sockets to `192.0.2.2`. Because the Raspberry Pi was sending packets to `10.73.81.140`, the laptop's CycloneDDS instance ignored all incoming packets arriving on `eth0`.

#### Resolution
We anchored CycloneDDS strictly to the physical Wi-Fi IP address in `cyclonedds.xml` and `connect_laptop_rviz.sh`:
```xml
<Domain Id="7">
  <General>
    <Interfaces>
      <NetworkInterface address="10.73.81.140"/>
    </Interfaces>
  </General>
  <Discovery>
    <Peers>
      <Peer Address="127.0.0.1"/>
      <Peer Address="10.73.81.208"/>
    </Peers>
  </Discovery>
</Domain>
```

---

### Issue 5: Windows Defender Firewall Packet Filtering for WSL2 Ports 7400–9500

#### Symptom
Even with IP addresses properly configured in CycloneDDS, cross-machine topic discovery failed. To isolate whether DDS or the operating system network stack was causing the drop, we created a diagnostic script: `test_udp_link.py`.

Running:
- Laptop: `python3 test_udp_link.py listen` (listening on UDP 7410)
- Pi: `python3 test_udp_link.py ping 10.73.81.140`

Resulted in:
```text
==================================================================
  [LISTENER] Waiting for UDP packet on 0.0.0.0:7410...
==================================================================
❌ TIMEOUT: No UDP packets arrived within 20 seconds.
   -> Windows Defender Firewall or router client isolation is blocking incoming UDP!
```

#### Technical Mechanism
In Windows 11 mirrored networking, WSL2 shares the host Windows network stack. However, all incoming traffic from the external physical Wi-Fi interface passes through the **Windows Filtering Platform (WFP)** and Windows Defender Firewall. 

By default, Windows Defender blocks unsolicited inbound UDP traffic across the Public and Private profiles. Because ROS 2 RTPS/DDS communicates over dynamic UDP ports derived from Domain ID ($7400 + 250 \times \text{Domain ID} + \dots$, spanning ports 7400 through 9500), Windows Defender silently dropped all UDP packets transmitted by the Raspberry Pi before they reached WSL2.

#### Resolution
In Windows PowerShell (Run as Administrator), we created an explicit inbound rule allowing UDP traffic across the entire DDS port range:
```powershell
New-NetFirewallRule -DisplayName "ROS2 WSL2 DDS" -Direction Inbound -Action Allow -Protocol UDP -LocalPort 7400-9500 -Profile Any
```
Once this firewall rule was committed:
1. `test_udp_link.py` confirmed bidirectional communication instantly (`✅ SUCCESS! Received reply from ('10.73.81.140', 7410)`).
2. CycloneDDS completed the RTPS participant handshake.
3. Running `ros2 node list` on the laptop immediately displayed all 5 remote robot nodes from the Pi.

---

### Issue 6: Linux Kernel GPIO Exclusivity (`lgpio` on RP1) & Teleoperation

#### Symptom
While the hardware stack was running on the Pi, running `test_motor_teleop.py` produced no physical movement of the robot's wheels, despite keyboard presses registering in the terminal.

#### Technical Mechanism
The Raspberry Pi 5 does not use the legacy BCM2835 peripheral registers; all 40-pin GPIO lines are managed by the **Raspberry Pi RP1 I/O Controller Southbridge** accessed via the Linux kernel character device interface (`/dev/gpiochip*` through the `lgpio` library).

When `./start_hardware.sh` launches `amr_hardware_drive_node.py`, the drive node calls:
```python
self.chip = lgpio.gpiochip_open(0)
lgpio.gpio_claim_output(self.chip, PIN_LEFT_PWM)
lgpio.gpio_claim_output(self.chip, PIN_RIGHT_PWM)
...
```
The Linux kernel assigns exclusive ownership of those GPIO pins to the `amr_hardware_drive_node` PID. When `test_motor_teleop.py` was started in another terminal, its attempt to call `lgpio.gpio_claim_output` threw a `lgpio.error: 'GPIO busy'`. The standalone script caught this exception and silently switched to mock simulation mode (`[WARN] Hardware not detected. Running in SIMULATION mode`).

#### Resolution
Instead of attempting raw GPIO manipulation while the hardware driver was active, teleoperation was integrated cleanly into the ROS 2 computational graph. We created `teleop_hardware.sh`:
```bash
export ROS_DOMAIN_ID=7
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$WS_DIR/cyclonedds.xml"

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/cmd_vel
```
This feeds standard `geometry_msgs/msg/Twist` messages to `/cmd_vel` over DDS. The running `amr_hardware_drive_node.py` receives `/cmd_vel`, translates linear and angular velocities to wheel setpoints via differential drive kinematics:
$$v_L = v_x - \frac{\omega_z \cdot L}{2}, \quad v_R = v_x + \frac{\omega_z \cdot L}{2}$$
and applies closed-loop PID control using the 4713 CPR quadrature encoders.

---

### Issue 7: RViz2 Transform Hierarchy & Fixed Frame Inconsistency

#### Symptom
When RViz2 opened on the laptop, the 3D viewport was completely black with zero laser scans or robot models rendered. RViz displayed global warnings:
`Fixed Frame [map] does not exist` and `Transform [odom] to [base_link] does not exist`.

#### Technical Mechanism
The pre-existing simulation RViz configuration (`agv_nav.rviz`) was designed for the digital twin navigation stack where AMCL or Cartographer is active and publishes the `map -> odom` transform. 

In pure physical hardware bringup, AMCL and map servers are intentionally not launched; only raw hardware drivers, `robot_state_publisher`, and `ekf_node` are running. The physical TF tree is rooted at `odom`:
$$\text{odom} \longrightarrow \text{base_link} \longrightarrow \{\text{laser\_frame}, \text{imu\_link}, \text{left\_wheel}, \text{right\_wheel}\}$$
Because RViz had `Fixed Frame: map` specified in its display options and could not find a transform chain connecting `map` to `base_link`, RViz's TF listener dropped all sensor messages (`/scan` on `laser_frame` and `/odometry/filtered` on `odom`).

#### Resolution
1. We authored a dedicated hardware visualization profile:  
   `src/agv_description/config/agv_hardware.rviz`:
   - Configured `Fixed Frame: odom`.
   - Pre-configured `RobotModel` (`/robot_description`), `LaserScan` (`/scan`, flat color red), `Odometry` (`/odometry/filtered`, covariance enabled), and `TF`.
2. In `connect_laptop_rviz.sh`, we added a background static transform publisher providing a fallback `map -> odom` transform:
   ```bash
   ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --yaw 0 --pitch 0 --roll 0 --frame-id map --child-frame-id odom >/dev/null 2>&1 &
   ```
As verified in the test screenshot, RViz loaded with all displays green:
- **Global Status**: `Ok`
- **Fixed Frame**: `odom`
- **Grid**: `Ok`
- **RobotModel**: `Ok`
- **LaserScan**: `Ok` (live 24 Hz scans visualizable in real time).

---

## 4. Final Operational Workflow & Run Guide

To operate the physical AMR system reliably, follow this exact startup sequence:

### Step 1: On the Raspberry Pi 5 (Robot)
In the robot terminal:
```bash
cd ~/AMR
./start_hardware.sh 10.73.81.140
```
This starts:
- `robot_state_publisher` (URDF model & joint states)
- `amr_hardware_drive_node.py` (RMCS-2305 driver + encoder odometry at 50 Hz)
- `amr_imu_node.py` (MPU-6500 IMU at 50 Hz)
- `ekf_node` (fuses encoder `/odom` + IMU `/imu/data` $\to$ `/odometry/filtered` at 50 Hz)
- `amr_lidar_node.py` (YDLIDAR X2 publishing `/scan` at ~24 Hz)

To verify the stack on the Pi at any time:
```bash
cd ~/AMR
./verify_hardware_topics.sh
```

### Step 2: On the Laptop (Visualization in RViz2)
Open Terminal 1 on your laptop:
```bash
cd ~/AMR/AMR-main
./connect_laptop_rviz.sh 10.73.81.208
```
RViz2 opens with `agv_hardware.rviz`. All displays (`RobotModel`, `LaserScan`, `Odometry`, `TF`) will display `Status: OK`.

### Step 3: On the Laptop (Keyboard Teleoperation)
Open Terminal 2 on your laptop:
```bash
cd ~/AMR/AMR-main
source env_hardware.sh
./teleop_hardware.sh
```
Drive the robot using:
- `i` : Drive forward
- `,` : Drive reverse
- `j` / `l` : Turn left / turn right in place
- `u` / `o` : Forward turn left / right
- `k` : Stop all motion
- `q` / `z` : Increase / decrease linear speed

---

## 5. Architectural Verification & Quality Control Artifacts

### Active ROS 2 Hardware Topics & Verified Frequencies
| ROS 2 Topic | Type | Verified Rate | Source Node | Function |
| :--- | :--- | :--- | :--- | :--- |
| `/odom` | `nav_msgs/msg/Odometry` | **50.0 Hz** | `amr_hardware_drive_node` | High-res wheel odometry ($4713\text{ ticks/rev}$) |
| `/imu/data` | `sensor_msgs/msg/Imu` | **50.0 Hz** | `amr_imu_node` | 6-DOF linear acceleration and angular velocity |
| `/odometry/filtered`| `nav_msgs/msg/Odometry` | **50.0 Hz** | `ekf_node` | Fused EKF state estimate (`odom -> base_link`) |
| `/scan` | `sensor_msgs/msg/LaserScan` | **24.2 Hz** | `amr_lidar_node` | 360-degree planar range measurements |
| `/robot_description` | `std_msgs/msg/String` | Latched | `robot_state_publisher` | URDF geometric & kinematic description |
| `/tf` & `/tf_static`| `tf2_msgs/msg/TFMessage` | Streaming | `robot_state_publisher`, `ekf_node` | Full coordinate frame transforms |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Event-driven | `teleop_hardware.sh` | Velocity demand inputs to motor driver |

---

## 6. Summary of Key Files Modified & Created

1. [start_hardware.sh](file:///home/rahul/AMR/AMR-main/start_hardware.sh): Pi 5 master bringup script with dynamic CycloneDDS IP binding and loopback discovery.
2. [connect_laptop_rviz.sh](file:///home/rahul/AMR/AMR-main/connect_laptop_rviz.sh): Laptop visualizer launcher with automatic fallback TF, IP binding, and RViz initialization.
3. [env_hardware.sh](file:///home/rahul/AMR/AMR-main/env_hardware.sh): Terminal setup helper with smart `--no-daemon` CLI wrapper for WSL2.
4. [teleop_hardware.sh](file:///home/rahul/AMR/AMR-main/teleop_hardware.sh): Unified ROS 2 keyboard teleop controller publishing to `/cmd_vel`.
5. [verify_hardware_topics.sh](file:///home/rahul/AMR/AMR-main/verify_hardware_topics.sh): Live frequency and health validation script for the hardware stack.
6. [test_udp_link.py](file:///home/rahul/AMR/AMR-main/test_udp_link.py): Two-way socket ping diagnostic utility for UDP connectivity verification across Windows Defender Firewall.
7. [cyclonedds.xml](file:///home/rahul/AMR/AMR-main/cyclonedds.xml): CycloneDDS configuration file for laptop terminal processes.
8. [agv_hardware.rviz](file:///home/rahul/AMR/AMR-main/src/agv_description/config/agv_hardware.rviz): Dedicated RViz2 configuration rooted at `odom`.
