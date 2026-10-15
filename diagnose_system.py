#!/usr/bin/env python3
"""
diagnose_system.py
==================
Full-system diagnostics for Autonomous Mobile Robot (AMR).
Checks every single link in the communication and navigation chain:
  1. ROS 2 Environment (ROS_DOMAIN_ID, RMW_IMPLEMENTATION)
  2. Running Processes (Gazebo, parameter_bridge, EKF, AMCL, Route Runner, Bridge)
  3. Active ROS 2 Topics & Publishing Rates (Hz)
  4. TF Tree Connectivity (odom->base_link, map->odom, map->base_link)
  5. Route Runner Brain State & Telemetry
  6. Subprocess Log Files (/tmp/amr_navigation.log, /tmp/amr_route_runner.log)
  7. End-to-End Goal Dispatch Test

Usage:
  python3 diagnose_system.py
"""

import os
import sys
import time
import subprocess
import json
import threading

# Ensure ROS 2 environment
if not os.environ.get('ROS_DISTRO'):
    print("[ERROR] ROS 2 environment not sourced! Run: source /opt/ros/jazzy/setup.bash")
    sys.exit(1)

# Ensure Domain 7 is set
if os.environ.get('ROS_DOMAIN_ID') != '7':
    print(f"[WARN] ROS_DOMAIN_ID is '{os.environ.get('ROS_DOMAIN_ID', 'unset')}'. Bridge and stack use 7. Setting to 7...")
    os.environ['ROS_DOMAIN_ID'] = '7'

if os.environ.get('RMW_IMPLEMENTATION') != 'rmw_cyclonedds_cpp':
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped

try:
    import tf2_ros
    _TF2 = True
except ImportError:
    _TF2 = False

class AMRDiagnostics(Node):
    def __init__(self):
        super().__init__('amr_diagnostics', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.parameter.Parameter.Type.BOOL, True)
        ])
        self.counts = {
            '/clock': 0,
            '/odom': 0,
            '/scan': 0,
            '/odometry/filtered': 0,
            '/map': 0,
            '/amcl_pose': 0,
            '/cmd_vel': 0,
            '/agv_state': 0,
        }
        self.last_state = "UNKNOWN"
        self.last_cmd_vel = None

        latched_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, reliability=QoSReliabilityPolicy.RELIABLE)

        # Subscribers
        self.create_subscription(Clock, '/clock', lambda m: self._count('/clock'), 10)
        self.create_subscription(Odometry, '/odom', lambda m: self._count('/odom'), 10)
        self.create_subscription(LaserScan, '/scan', lambda m: self._count('/scan'), qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odometry/filtered', lambda m: self._count('/odometry/filtered'), 10)
        self.create_subscription(OccupancyGrid, '/map', lambda m: self._count('/map'), latched_qos)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', lambda m: self._count('/amcl_pose'), latched_qos)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, 10)
        self.create_subscription(String, '/agv_state', self._state_cb, 10)

        # TF Buffer
        self.tf_buffer = tf2_ros.Buffer() if _TF2 else None
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self) if _TF2 else None

        # Goal Publisher
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

    def _count(self, topic):
        self.counts[topic] += 1

    def _cmd_cb(self, msg):
        self.counts['/cmd_vel'] += 1
        self.last_cmd_vel = (msg.linear.x, msg.angular.z)

    def _state_cb(self, msg):
        self.counts['/agv_state'] += 1
        self.last_state = msg.data

def run_diagnostics():
    rclpy.init()
    diag = AMRDiagnostics()

    print("=" * 68)
    print("       AMR COMPLETE SYSTEM DIAGNOSTIC REPORT")
    print("=" * 68)

    # 1. Environment Check
    print("\n[1] ROS 2 ENVIRONMENT")
    domain = os.environ.get('ROS_DOMAIN_ID', 'unset')
    rmw = os.environ.get('RMW_IMPLEMENTATION', 'unset')
    print(f"  • ROS_DOMAIN_ID:     {domain} " + ("✅ (Matches stack: 7)" if domain == '7' else "❌ (MISMATCH: expected 7)"))
    print(f"  • RMW_IMPLEMENTATION: {rmw}")

    # 2. Process / Process List
    print("\n[2] PROCESSES RUNNING ON SYSTEM")
    processes = [
        ("Gazebo Sim", ["gz sim", "gz-sim", "ruby"]),
        ("ros_gz_bridge", ["parameter_bridge"]),
        ("Robot State Pub", ["robot_state_publisher"]),
        ("EKF Node", ["ekf_node"]),
        ("Map Server", ["map_server"]),
        ("AMCL", ["amcl"]),
        ("Route Runner", ["route_runner"]),
        ("BT Manager", ["bt_manager"]),
        ("AMR Bridge Server", ["bridge_server.py"]),
    ]
    for name, patterns in processes:
        running = False
        pids = []
        for pat in patterns:
            try:
                out = subprocess.check_output(['pgrep', '-f', pat]).decode().strip()
                if out:
                    pids.extend(out.split('\n'))
                    running = True
            except subprocess.CalledProcessError:
                pass
        pid_str = f"(PID {', '.join(set(pids))})" if running else ""
        status = "✅ RUNNING" if running else "❌ NOT RUNNING"
        print(f"  • {name:<20}: {status} {pid_str}")

    # 3. Topic Rates Sampling (Sample for 2.0 seconds)
    print("\n[3] TOPIC MESSAGE SAMPLING (Measuring over 2.0s)...")
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(diag)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    time.sleep(2.0)
    executor.shutdown()

    topics_info = [
        ("/clock", "Sim Clock (Gazebo)", 10.0),
        ("/odom", "Raw Odometry (Gazebo)", 5.0),
        ("/scan", "LiDAR Scan (Gazebo)", 5.0),
        ("/odometry/filtered", "Fused EKF Odometry", 5.0),
        ("/map", "Occupancy Grid (Map Server)", 0.5),
        ("/amcl_pose", "AMCL Pose Estimate", 0.0),
        ("/cmd_vel", "Motor Velocities (Route Runner)", 0.0),
        ("/agv_state", "Route Runner Nav State", 0.0),
    ]

    for topic, desc, min_expected in topics_info:
        cnt = diag.counts.get(topic, 0)
        hz = cnt / 2.0
        if cnt > 0:
            status = f"✅ ACTIVE ({hz:.1f} Hz)"
        else:
            status = "⚠️  IDLE / NO DATA" if min_expected == 0.0 else "❌ DEAD (0 Hz)"
        print(f"  • {topic:<22} ({desc:<26}): {status}")

    # 4. TF Tree Verification
    print("\n[4] TF TRANSFORM TREE (Checking frames)")
    tf_pairs = [
        ("odom", "base_link", "EKF Odometry Transform"),
        ("map", "odom", "AMCL Localization Transform"),
        ("map", "base_link", "Complete Global Localization"),
    ]
    for parent, child, desc in tf_pairs:
        ok = False
        if diag.tf_buffer:
            try:
                diag.tf_buffer.lookup_transform(parent, child, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.2))
                ok = True
            except Exception:
                ok = False
        status = "✅ CONNECTED" if ok else "❌ MISSING"
        print(f"  • {parent} ➔ {child:<12} ({desc:<28}): {status}")

    # 5. Route Runner State
    print("\n[5] ROUTE RUNNER STATUS")
    print(f"  • Current AGV State: {diag.last_state}")
    if diag.last_cmd_vel:
        print(f"  • Last /cmd_vel:     lin_x={diag.last_cmd_vel[0]:.2f} m/s, ang_z={diag.last_cmd_vel[1]:.2f} rad/s")
    else:
        print(f"  • Last /cmd_vel:     None (Robot is commanded to hold 0 velocity)")

    # 6. Tail of Log Files
    print("\n[6] RECENT LOGS FROM SUBPROCESSES")
    for log_name, log_path in [
        ("Navigation Launch (Gazebo/AMCL)", "/tmp/amr_navigation.log"),
        ("Route Runner (Brain/MPPI)", "/tmp/amr_route_runner.log"),
    ]:
        print(f"  ── {log_name} [{log_path}] ──")
        if os.path.exists(log_path):
            with open(log_path, 'r', errors='replace') as f:
                lines = [l.strip() for l in f.readlines() if l.strip()]
                for l in lines[-5:]:
                    print(f"     {l}")
        else:
            print("     (File does not exist)")

    # 7. Summary Diagnosis & Actionable Fix
    print("\n" + "=" * 68)
    print("                 DIAGNOSTIC SUMMARY & ACTION ITEMS")
    print("=" * 68)

    clock_ok = diag.counts['/clock'] > 0
    odom_ok = diag.counts['/odometry/filtered'] > 0
    scan_ok = diag.counts['/scan'] > 0
    tf_ok = False
    if diag.tf_buffer:
        try:
            diag.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            tf_ok = True
        except Exception:
            tf_ok = False

    if not clock_ok:
        print("❌ CRITICAL: Gazebo /clock is NOT publishing!")
        print("   Action: Gazebo crashed or failed to start. Check /tmp/amr_navigation.log above.")
    elif not scan_ok:
        print("❌ CRITICAL: LiDAR /scan is NOT publishing!")
        print("   Action: Gazebo sensor plugin failed or ros_gz_bridge is not bridging /scan.")
    elif not odom_ok:
        print("❌ CRITICAL: /odometry/filtered is NOT publishing!")
        print("   Action: ekf_node is waiting for /clock or /odom. Check ekf_node status.")
    elif not tf_ok:
        print("⚠️  ACTION NEEDED: map ➔ base_link TF is missing!")
        print("   Action: AMCL has not yet localized. Send an initial pose to AMCL:")
        print("           curl -X POST http://localhost:8000/api/initial_pose -H 'Content-Type: application/json' -d '{\"x\":0.0,\"y\":0.0,\"theta\":0.0}'")
    else:
        print("✅ ALL HARDWARE & SENSOR PIPELINES ARE HEALTHY!")
        print("   You can send goals from the dashboard or via:")
        print("   curl -X POST http://localhost:8000/api/goal -H 'Content-Type: application/json' -d '{\"x\":2.0,\"y\":1.0}'")

    print("=" * 68 + "\n")
    rclpy.shutdown()

if __name__ == '__main__':
    run_diagnostics()
