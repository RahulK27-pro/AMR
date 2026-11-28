#!/usr/bin/env python3
"""
run_logger.py
=============
Dedicated Real-Time Navigation & Localization Telemetry Logger for AMR:
  1. Subscribes to:
     - /tf (map -> base_link robot pose)
     - /amcl_pose (AMCL localization estimate & covariance)
     - /cmd_vel (Motor velocity commands)
     - /odom & /odometry/filtered (Wheel encoder and EKF odometry)
     - /scan (LiDAR obstacle proximity)
     - /planned_path (Active roadmap path)
     - /agv_state (Navigation FSM state: IDLE, PLANNING, NAVIGATING, YIELDING, DETOURING, ARRIVED)
  2. Logs real-time 10 Hz telemetry to:
     - CSV file: run_logs/navigation_run_<timestamp>.csv
     - Human-readable milestone log: run_logs/navigation_run_<timestamp>.log
     - Symlinks: run_logs/latest_run.csv and run_logs/latest_run.log
"""

import os
import sys
import time
import math
import csv
import json
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from tf2_ros import Buffer, TransformListener, TransformException
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


def get_yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class AMRRunLogger(Node):
    def __init__(self):
        super().__init__('amr_run_logger')

        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)

        # Prepare log directory
        workspace_root = os.path.expanduser('~/AMR/AMR-main')
        self.log_dir = os.path.join(workspace_root, 'run_logs')
        os.makedirs(self.log_dir, exist_ok=True)

        timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(self.log_dir, f'navigation_run_{timestamp_str}.csv')
        self.log_path = os.path.join(self.log_dir, f'navigation_run_{timestamp_str}.log')

        # CSV Writer setup
        self.csv_file = open(self.csv_path, 'w', newline='', buffering=1)
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'ros_time_sec', 'state',
            'map_x', 'map_y', 'map_yaw_deg',
            'amcl_x', 'amcl_y', 'amcl_yaw_deg', 'amcl_cov_xx', 'amcl_cov_yy', 'amcl_cov_yaw',
            'ekf_x', 'ekf_y', 'ekf_vx', 'ekf_wz',
            'cmd_v', 'cmd_w',
            'path_len', 'current_target_idx', 'dist_to_goal',
            'min_scan_dist_m', 'num_valid_beams',
            'ml_person_count', 'ml_clutter_count', 'ml_wall_count', 'ml_infer_ms'
        ])

        # Text logger
        self.text_file = open(self.log_path, 'w', buffering=1)
        self._write_milestone("SESSION_START", f"AMR Navigation Run Logging Started | Target CSV: {self.csv_path}")

        # Symlinks to latest
        self._update_latest_symlinks()

        # TF Buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Telemetry State Variables
        self.current_state = "IDLE"
        self.map_x = 0.0
        self.map_y = 0.0
        self.map_yaw = 0.0
        self.has_map_tf = False

        self.amcl_x = 0.0
        self.amcl_y = 0.0
        self.amcl_yaw = 0.0
        self.amcl_cov_xx = 0.0
        self.amcl_cov_yy = 0.0
        self.amcl_cov_yaw = 0.0
        self.has_amcl_pose = False

        self.ekf_x = 0.0
        self.ekf_y = 0.0
        self.ekf_vx = 0.0
        self.ekf_wz = 0.0

        self.cmd_v = 0.0
        self.cmd_w = 0.0

        self.path_points = []
        self.current_target_idx = 0
        self.dist_to_goal = 0.0

        self.min_scan_dist = 99.0
        self.valid_beams = 0

        self.ml_person_count = 0
        self.ml_clutter_count = 0
        self.ml_wall_count = 0
        self.ml_infer_ms = 0.0

        # Subscriptions
        self.create_subscription(String, '/agv_state', self.state_cb, 10)
        self.create_subscription(String, '/agv_obstacles_ml', self.ml_obs_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_cb, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.ekf_cb, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.create_subscription(Path, '/planned_path', self.path_cb, 10)

        scan_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.create_subscription(LaserScan, '/scan', self.scan_cb, scan_qos)

        # Logging Timer at 10 Hz
        self.log_timer = self.create_timer(0.10, self.timer_tick)
        self.get_logger().info(f"[RUN_LOGGER] Telemetry active. Logging to {self.csv_path}")

    def _update_latest_symlinks(self):
        try:
            latest_csv = os.path.join(self.log_dir, 'latest_run.csv')
            latest_log = os.path.join(self.log_dir, 'latest_run.log')
            if os.path.islink(latest_csv) or os.path.exists(latest_csv):
                os.remove(latest_csv)
            if os.path.islink(latest_log) or os.path.exists(latest_log):
                os.remove(latest_log)
            os.symlink(self.csv_path, latest_csv)
            os.symlink(self.log_path, latest_log)
        except Exception:
            pass

    def _write_milestone(self, event_name: str, message: str):
        now_str = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        line = f"[{now_str}] [{event_name:18s}] {message}\n"
        self.text_file.write(line)
        self.text_file.flush()

    def state_cb(self, msg: String):
        prev_state = self.current_state
        self.current_state = msg.data.strip()
        if self.current_state != prev_state:
            self._write_milestone(
                "STATE_CHANGE",
                f"{prev_state} -> {self.current_state} | Pose=({self.map_x:.2f}, {self.map_y:.2f}, {math.degrees(self.map_yaw):.1f}°)"
            )

    def amcl_cb(self, msg: PoseWithCovarianceStamped):
        self.has_amcl_pose = True
        self.amcl_x = msg.pose.pose.position.x
        self.amcl_y = msg.pose.pose.position.y
        self.amcl_yaw = get_yaw_from_quaternion(msg.pose.pose.orientation)
        cov = msg.pose.covariance
        self.amcl_cov_xx = cov[0]
        self.amcl_cov_yy = cov[7]
        self.amcl_cov_yaw = cov[35]

    def ekf_cb(self, msg: Odometry):
        self.ekf_x = msg.pose.pose.position.x
        self.ekf_y = msg.pose.pose.position.y
        self.ekf_vx = msg.twist.twist.linear.x
        self.ekf_wz = msg.twist.twist.angular.z

    def cmd_vel_cb(self, msg: Twist):
        self.cmd_v = msg.linear.x
        self.cmd_w = msg.angular.z

    def path_cb(self, msg: Path):
        self.path_points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if len(self.path_points) > 0:
            target_pt = self.path_points[-1]
            self._write_milestone(
                "PATH_RECEIVED",
                f"New path with {len(self.path_points)} waypoints -> Final Goal=({target_pt[0]:.2f}, {target_pt[1]:.2f})"
            )

    def scan_cb(self, msg: LaserScan):
        ranges = [r for r in msg.ranges if msg.range_min < r < msg.range_max and math.isfinite(r)]
        self.valid_beams = len(ranges)
        self.min_scan_dist = min(ranges) if len(ranges) > 0 else 99.0

    def ml_obs_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            counts = data.get('counts', {})
            self.ml_person_count = int(counts.get('PERSON', 0))
            self.ml_clutter_count = int(counts.get('CLUTTER', 0))
            self.ml_wall_count = int(counts.get('WALL', 0))
            self.ml_infer_ms = float(data.get('infer_ms', 0.0))
        except Exception:
            pass

    def timer_tick(self):
        # 1. Update Map Frame Pose from TF
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.04))
            self.map_x = t.transform.translation.x
            self.map_y = t.transform.translation.y
            self.map_yaw = get_yaw_from_quaternion(t.transform.rotation)
            if not self.has_map_tf:
                self.has_map_tf = True
                self._write_milestone(
                    "TF_LOCALIZATION_OK",
                    f"Acquired map -> base_link transform at ({self.map_x:.2f}, {self.map_y:.2f}, {math.degrees(self.map_yaw):.1f}°)"
                )
        except TransformException:
            pass

        # 2. Compute Distance to Goal & Active Waypoint Index
        if len(self.path_points) > 0:
            goal_x, goal_y = self.path_points[-1]
            self.dist_to_goal = math.hypot(goal_x - self.map_x, goal_y - self.map_y)
            if self.has_map_tf:
                best_i = 0
                best_d2 = float('inf')
                for idx_p, (px, py) in enumerate(self.path_points):
                    d2 = (px - self.map_x) ** 2 + (py - self.map_y) ** 2
                    if d2 < best_d2:
                        best_d2 = d2
                        best_i = idx_p
                self.current_target_idx = best_i
        else:
            self.dist_to_goal = 0.0
            self.current_target_idx = 0

        # 3. Write CSV row
        ros_time_sec = self.get_clock().now().nanoseconds / 1e9
        self.csv_writer.writerow([
            f"{ros_time_sec:.3f}",
            self.current_state,
            f"{self.map_x:.3f}", f"{self.map_y:.3f}", f"{math.degrees(self.map_yaw):.1f}",
            f"{self.amcl_x:.3f}", f"{self.amcl_y:.3f}", f"{math.degrees(self.amcl_yaw):.1f}",
            f"{self.amcl_cov_xx:.4f}", f"{self.amcl_cov_yy:.4f}", f"{self.amcl_cov_yaw:.4f}",
            f"{self.ekf_x:.3f}", f"{self.ekf_y:.3f}", f"{self.ekf_vx:.3f}", f"{self.ekf_wz:.3f}",
            f"{self.cmd_v:.3f}", f"{self.cmd_w:.3f}",
            len(self.path_points), self.current_target_idx, f"{self.dist_to_goal:.3f}",
            f"{self.min_scan_dist:.3f}", self.valid_beams,
            self.ml_person_count, self.ml_clutter_count, self.ml_wall_count, f"{self.ml_infer_ms:.3f}"
        ])

    def destroy_node(self):
        try:
            self._write_milestone("SESSION_END", "Navigation Run Ended. Closing log files.")
            self.csv_file.close()
            self.text_file.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AMRRunLogger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
