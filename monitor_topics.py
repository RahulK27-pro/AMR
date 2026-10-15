#!/usr/bin/env python3
"""
monitor_topics.py
=================
Live AMR Terminal Telemetry & Sensor Monitor.
Streams all critical ROS 2 topics continuously in a single live-updating terminal:
  - /cmd_vel               (Motor Velocity Commands)
  - /odometry/filtered     (Fused EKF Robot Position & Speed)
  - /odom                  (Raw Wheel Odometry)
  - /scan                  (LiDAR LaserScan: Front/Left/Right/Closest distances)
  - /clock                 (Gazebo Simulation Clock & Frequency)
  - /amcl_pose             (Global AMCL Map Localization Pose)
  - /agv_state             (Route Runner Navigation State)
  - /mission_progress      (Waypoint Tracking Progress)
  - /dynamic_obstacle/odom (Active Dynamic Obstacle Position & Speed)

Usage:
  python3 monitor_topics.py
  python3 monitor_topics.py --stream   # Continuous scrolling log mode
"""

import sys
import os
import time
import math
import argparse
from datetime import datetime

# Auto-set ROS environment defaults if not set
if 'ROS_DOMAIN_ID' not in os.environ:
    os.environ['ROS_DOMAIN_ID'] = '7'
if 'RMW_IMPLEMENTATION' not in os.environ:
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, qos_profile_sensor_data

from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path as NavPath
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String


def quat_to_yaw(q):
    """Convert Quaternion (x, y, z, w) to yaw in degrees and radians."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw_rad = math.atan2(siny_cosp, cosy_cosp)
    yaw_deg = math.degrees(yaw_rad)
    return yaw_deg, yaw_rad


class TopicMonitorNode(Node):
    def __init__(self):
        super().__init__('amr_topic_monitor', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.parameter.Parameter.Type.BOOL, True)
        ])

        # Telemetry data containers
        self.sim_time = 0.0
        self.clock_count = 0
        self.last_clock_calc = time.time()
        self.clock_hz = 0.0

        # cmd_vel
        self.cmd_vel = {'v': 0.0, 'w': 0.0, 'ts': 0.0, 'hz': 0.0, 'count': 0}

        # ekf odom
        self.ekf_odom = {
            'x': 0.0, 'y': 0.0, 'yaw_deg': 0.0, 'yaw_rad': 0.0,
            'vx': 0.0, 'wz': 0.0, 'speed': 0.0,
            'ts': 0.0, 'hz': 0.0, 'count': 0
        }

        # raw odom
        self.raw_odom = {
            'x': 0.0, 'y': 0.0, 'yaw_deg': 0.0,
            'vx': 0.0, 'wz': 0.0,
            'ts': 0.0, 'hz': 0.0, 'count': 0
        }

        # lidar scan
        self.lidar = {
            'samples': 0, 'valid_count': 0,
            'min_dist': 99.0, 'min_angle_deg': 0.0,
            'front_dist': 99.0, 'left_dist': 99.0, 'right_dist': 99.0, 'rear_dist': 99.0,
            'range_min': 0.0, 'range_max': 0.0,
            'ts': 0.0, 'hz': 0.0, 'count': 0
        }

        # amcl pose
        self.amcl_pose = {
            'x': 0.0, 'y': 0.0, 'yaw_deg': 0.0,
            'ts': 0.0, 'count': 0, 'received': False
        }

        # agv state & mission
        self.agv_state = "UNKNOWN"
        self.mission_progress = "IDLE"
        self.planned_path_len = 0

        # dynamic obstacle
        self.dyn_obs = {
            'x': 0.0, 'y': 0.0, 'speed': 0.0, 'ts': 0.0, 'active': False
        }

        # Rate tracking
        self.rate_timer_sec = time.time()

        latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        # Subscriptions
        self.create_subscription(Clock, '/clock', self._clock_cb, 10)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self._ekf_cb, 10)
        self.create_subscription(Odometry, '/odom', self._raw_odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, latched_qos)
        self.create_subscription(String, '/agv_state', self._state_cb, 10)
        self.create_subscription(String, '/mission_progress', self._mission_cb, 10)
        self.create_subscription(NavPath, '/agv_dense_path', self._path_cb, 10)
        self.create_subscription(Odometry, '/dynamic_obstacle/odom', self._dyn_obs_cb, 10)

    def _clock_cb(self, msg: Clock):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9
        self.clock_count += 1

    def _cmd_vel_cb(self, msg: Twist):
        self.cmd_vel['v'] = msg.linear.x
        self.cmd_vel['w'] = msg.angular.z
        self.cmd_vel['ts'] = time.time()
        self.cmd_vel['count'] += 1

    def _ekf_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        yaw_deg, yaw_rad = quat_to_yaw(o)
        self.ekf_odom['x'] = p.x
        self.ekf_odom['y'] = p.y
        self.ekf_odom['yaw_deg'] = yaw_deg
        self.ekf_odom['yaw_rad'] = yaw_rad
        self.ekf_odom['vx'] = msg.twist.twist.linear.x
        self.ekf_odom['wz'] = msg.twist.twist.angular.z
        self.ekf_odom['speed'] = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.ekf_odom['ts'] = time.time()
        self.ekf_odom['count'] += 1

    def _raw_odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        yaw_deg, _ = quat_to_yaw(o)
        self.raw_odom['x'] = p.x
        self.raw_odom['y'] = p.y
        self.raw_odom['yaw_deg'] = yaw_deg
        self.raw_odom['vx'] = msg.twist.twist.linear.x
        self.raw_odom['wz'] = msg.twist.twist.angular.z
        self.raw_odom['ts'] = time.time()
        self.raw_odom['count'] += 1

    def _scan_cb(self, msg: LaserScan):
        ranges = msg.ranges
        num_samples = len(ranges)
        if num_samples == 0:
            return

        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        rmin = msg.range_min
        rmax = msg.range_max

        valid_pts = []
        closest_dist = 99.0
        closest_angle = 0.0

        for i, r in enumerate(ranges):
            if rmin <= r <= rmax and not math.isinf(r) and not math.isnan(r):
                valid_pts.append(r)
                if r < closest_dist:
                    closest_dist = r
                    closest_angle = angle_min + i * angle_inc

        # Sector samples (approximate angle sectors)
        def get_sector_dist(target_angle_rad, tolerance_rad=0.25):
            candidates = []
            for i, r in enumerate(ranges):
                if rmin <= r <= rmax and not math.isinf(r) and not math.isnan(r):
                    ang = angle_min + i * angle_inc
                    # Normalize angle difference to [-pi, pi]
                    diff = (ang - target_angle_rad + math.pi) % (2.0 * math.pi) - math.pi
                    if abs(diff) <= tolerance_rad:
                        candidates.append(r)
            return min(candidates) if candidates else 99.0

        self.lidar['samples'] = num_samples
        self.lidar['valid_count'] = len(valid_pts)
        self.lidar['min_dist'] = closest_dist
        self.lidar['min_angle_deg'] = math.degrees(closest_angle)
        self.lidar['front_dist'] = get_sector_dist(0.0)
        self.lidar['left_dist'] = get_sector_dist(math.pi / 2.0)
        self.lidar['right_dist'] = get_sector_dist(-math.pi / 2.0)
        self.lidar['rear_dist'] = get_sector_dist(math.pi)
        self.lidar['range_min'] = rmin
        self.lidar['range_max'] = rmax
        self.lidar['ts'] = time.time()
        self.lidar['count'] += 1

    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        yaw_deg, _ = quat_to_yaw(o)
        self.amcl_pose['x'] = p.x
        self.amcl_pose['y'] = p.y
        self.amcl_pose['yaw_deg'] = yaw_deg
        self.amcl_pose['ts'] = time.time()
        self.amcl_pose['received'] = True
        self.amcl_pose['count'] += 1

    def _state_cb(self, msg: String):
        self.agv_state = msg.data

    def _mission_cb(self, msg: String):
        self.mission_progress = msg.data

    def _path_cb(self, msg: NavPath):
        self.planned_path_len = len(msg.poses)

    def _dyn_obs_cb(self, msg: Odometry):
        self.dyn_obs['x'] = msg.pose.pose.position.x
        self.dyn_obs['y'] = msg.pose.pose.position.y
        self.dyn_obs['speed'] = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.dyn_obs['ts'] = time.time()
        self.dyn_obs['active'] = True

    def update_hz_estimates(self, elapsed):
        if elapsed <= 0.0:
            return
        self.clock_hz = self.clock_count / elapsed
        self.cmd_vel['hz'] = self.cmd_vel['count'] / elapsed
        self.ekf_odom['hz'] = self.ekf_odom['count'] / elapsed
        self.raw_odom['hz'] = self.raw_odom['count'] / elapsed
        self.lidar['hz'] = self.lidar['count'] / elapsed

        # Reset counts for next second
        self.clock_count = 0
        self.cmd_vel['count'] = 0
        self.ekf_odom['count'] = 0
        self.raw_odom['count'] = 0
        self.lidar['count'] = 0


def format_dist(val):
    if val >= 90.0:
        return " >10 m (CLEAR)"
    elif val <= 0.40:
        return f"\033[91m{val:5.2f} m [CRITICAL]\033[0m"
    elif val <= 1.00:
        return f"\033[93m{val:5.2f} m [CAUTION]\033[0m"
    else:
        return f"\033[92m{val:5.2f} m [OK]\033[0m"


def main():
    parser = argparse.ArgumentParser(description="Live continuous telemetry and sensor monitor for AMR.")
    parser.add_argument('--stream', action='store_true', help='Continuous scrolling line mode instead of full dashboard screen.')
    args = parser.parse_args()

    rclpy.init()
    node = TopicMonitorNode()

    domain = os.environ.get('ROS_DOMAIN_ID', '7')
    rmw = os.environ.get('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')

    last_hz_calc = time.time()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        while rclpy.ok():

            now = time.time()
            dt = now - last_hz_calc
            if dt >= 1.0:
                node.update_hz_estimates(dt)
                last_hz_calc = now

            # Freshness indicators
            cmd_fresh = (now - node.cmd_vel['ts']) < 1.0 if node.cmd_vel['ts'] > 0 else False
            ekf_fresh = (now - node.ekf_odom['ts']) < 1.0 if node.ekf_odom['ts'] > 0 else False
            lidar_fresh = (now - node.lidar['ts']) < 1.0 if node.lidar['ts'] > 0 else False
            clock_fresh = node.clock_hz > 0.0

            if args.stream:
                # Streaming single-line mode
                line = (
                    f"[{datetime.now().strftime('%H:%M:%S')}] "
                    f"SimTime={node.sim_time:6.1f}s | "
                    f"Pose=({node.ekf_odom['x']:5.2f}, {node.ekf_odom['y']:5.2f}, {node.ekf_odom['yaw_deg']:+5.1f}°) | "
                    f"Cmd=(v={node.cmd_vel['v']:+4.2f}, w={node.cmd_vel['w']:+4.2f}) | "
                    f"LiDAR: Front={node.lidar['front_dist']:4.2f}m Closest={node.lidar['min_dist']:4.2f}m@{node.lidar['min_angle_deg']:+4.0f}° | "
                    f"State={node.agv_state}"
                )
                print(line)
                time.sleep(0.1)
                continue

            # Clear screen & reposition cursor for live full dashboard
            sys.stdout.write("\033[H\033[J")
            sys.stdout.write("=================================================================================\n")
            sys.stdout.write(f"   \033[1;36mAMR COMPLETE LIVE TELEMETRY & SENSOR DASHBOARD\033[0m   [Domain {domain} | {rmw}]\n")
            sys.stdout.write(f"   Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}  |  Sim Clock: \033[1;33m{node.sim_time:7.2f}s\033[0m ({node.clock_hz:4.0f} Hz)\n")
            sys.stdout.write("=================================================================================\n\n")

            # 1. ROBOT MOTION & CONTROLLER (/cmd_vel)
            cmd_status = "\033[92m● ACTIVE\033[0m" if cmd_fresh else "\033[93m○ IDLE (0.0)\033[0m"
            sys.stdout.write(f"\033[1;34m[1] MOTOR COMMANDS (/cmd_vel)\033[0m  {cmd_status} ({node.cmd_vel['hz']:.1f} Hz)\n")
            sys.stdout.write(f"    • Linear Velocity (v)  : \033[1m{node.cmd_vel['v']:+6.2f} m/s\033[0m  (Forward / Reverse)\n")
            sys.stdout.write(f"    • Angular Velocity (w) : \033[1m{node.cmd_vel['w']:+6.2f} rad/s\033[0m (Steering Turn Rate)\n\n")

            # 2. LOCALIZATION & POSE (/odometry/filtered & /amcl_pose)
            ekf_status = "\033[92m● ACTIVE\033[0m" if ekf_fresh else "\033[91m✖ NO DATA\033[0m"
            sys.stdout.write(f"\033[1;34m[2] ROBOT LOCALIZATION & ODOMETRY\033[0m  {ekf_status}\n")
            sys.stdout.write(f"    • EKF Fused Pose (/odometry/filtered @ {node.ekf_odom['hz']:.1f} Hz):\n")
            sys.stdout.write(f"        X: \033[1;32m{node.ekf_odom['x']:+7.3f} m\033[0m  |  Y: \033[1;32m{node.ekf_odom['y']:+7.3f} m\033[0m  |  Heading: \033[1;32m{node.ekf_odom['yaw_deg']:+6.1f}°\033[0m ({node.ekf_odom['yaw_rad']:+5.2f} rad)\n")
            sys.stdout.write(f"        Linear Speed: {node.ekf_odom['speed']:.2f} m/s  |  Turn Speed: {node.ekf_odom['wz']:+5.2f} rad/s\n")
            
            amcl_str = f"X={node.amcl_pose['x']:+6.2f}m, Y={node.amcl_pose['y']:+6.2f}m, Yaw={node.amcl_pose['yaw_deg']:+5.1f}°" if node.amcl_pose['received'] else "Waiting for initial pose / scan match"
            sys.stdout.write(f"    • AMCL Map Localization (/amcl_pose):\n")
            sys.stdout.write(f"        Global Pose: {amcl_str}\n\n")

            # 3. LIDAR SENSOR (/scan)
            scan_status = "\033[92m● STREAMING\033[0m" if lidar_fresh else "\033[91m✖ NO SCAN\033[0m"
            sys.stdout.write(f"\033[1;34m[3] 2D LIDAR SENSOR (/scan)\033[0m  {scan_status} ({node.lidar['hz']:.1f} Hz | {node.lidar['valid_count']}/{node.lidar['samples']} rays)\n")
            sys.stdout.write(f"    • FRONT (  0°): {format_dist(node.lidar['front_dist'])}\n")
            sys.stdout.write(f"    • LEFT  (+90°): {format_dist(node.lidar['left_dist'])}\n")
            sys.stdout.write(f"    • RIGHT (-90°): {format_dist(node.lidar['right_dist'])}\n")
            sys.stdout.write(f"    • REAR (180°): {format_dist(node.lidar['rear_dist'])}\n")
            sys.stdout.write(f"    • \033[1mClosest Obstacle\033[0m  : {format_dist(node.lidar['min_dist'])} at angle \033[1m{node.lidar['min_angle_deg']:+5.1f}°\033[0m\n\n")

            # 4. NAVIGATION STATE & BRAIN
            sys.stdout.write(f"\033[1;34m[4] NAVIGATION BRAIN (Route Runner / MPPI)\033[0m\n")
            sys.stdout.write(f"    • AGV State         : \033[1;33m{node.agv_state}\033[0m\n")
            sys.stdout.write(f"    • Mission Progress  : {node.mission_progress}\n")
            sys.stdout.write(f"    • Dense Path Length : {node.planned_path_len} waypoints\n")
            if node.dyn_obs['active']:
                sys.stdout.write(f"    • Dynamic Obstacle  : At ({node.dyn_obs['x']:.2f}, {node.dyn_obs['y']:.2f}) Speed={node.dyn_obs['speed']:.2f} m/s\n")
            sys.stdout.write("\n")

            sys.stdout.write("=================================================================================\n")
            sys.stdout.write("   Press [Ctrl+C] to exit monitor.\n")
            sys.stdout.flush()

            time.sleep(0.05)

    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        print("\n[Topic Monitor] Exiting cleanly...")
    finally:
        try:
            executor.shutdown()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    main()
