#!/usr/bin/env python3
"""
verify_sensors.py
=================
Live interactive verification tool to validate physical sensor orientations
BEFORE starting SLAM:
  1. LiDAR Quadrant Test: Stand in Front, Left, Right, Back -> Verifies non-inverted scan.
  2. Odometry & IMU Direction Test: Turn left by hand -> Verifies matching positive yaw signs.
  3. EKF Health Check: Verifies covariance stability (no explosion).

Usage:
  python3 verify_sensors.py
"""

import os
import sys
import time
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry


class SensorVerifier(Node):
    def __init__(self):
        super().__init__('amr_sensor_verifier')

        self.latest_scan = None
        self.latest_odom = None
        self.latest_imu = None
        self.latest_ekf = None

        self.create_subscription(LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(Imu, '/imu/data', self._imu_cb, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self._ekf_cb, 10)

    def _scan_cb(self, msg):
        self.latest_scan = msg

    def _odom_cb(self, msg):
        self.latest_odom = msg

    def _imu_cb(self, msg):
        self.latest_imu = msg

    def _ekf_cb(self, msg):
        self.latest_ekf = msg


def main():
    rclpy.init()
    node = SensorVerifier()

    print("════════════════════════════════════════════════════════════════════════════════")
    print("                AMR LIVE SENSOR ORIENTATION & DIRECTION VERIFIER                ")
    print("════════════════════════════════════════════════════════════════════════════════")
    print(" Press Ctrl+C to exit.\n")

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)

            # Clear screen & home cursor
            print("\033[2J\033[H", end="")
            print("╔════════════════════════════════════════════════════════════════════════════════╗")
            print("║                    AMR SENSOR DIRECTION VALIDATION HUD                         ║")
            print("╚════════════════════════════════════════════════════════════════════════════════╝")

            # 1. LiDAR Quadrant Test
            print("\n\033[1;36m[1] LIDAR QUADRANT TEST (/scan)\033[0m")
            if node.latest_scan:
                scan = node.latest_scan
                n = len(scan.ranges)
                quad_front = []
                quad_left = []
                quad_right = []
                quad_rear = []

                for i, r in enumerate(scan.ranges):
                    if not math.isnan(r) and not math.isinf(r) and r > 0.05:
                        ang = scan.angle_min + i * scan.angle_increment
                        deg = math.degrees(ang)
                        if -45.0 <= deg <= 45.0:
                            quad_front.append(r)
                        elif 45.0 < deg <= 135.0:
                            quad_left.append(r)
                        elif -135.0 <= deg < -45.0:
                            quad_right.append(r)
                        else:
                            quad_rear.append(r)

                min_f = min(quad_front) if quad_front else float('inf')
                min_l = min(quad_left) if quad_left else float('inf')
                min_r = min(quad_right) if quad_right else float('inf')
                min_b = min(quad_rear) if quad_rear else float('inf')

                # Identify closest direction
                closest_dist = min(min_f, min_l, min_r, min_b)
                if closest_dist < float('inf'):
                    if closest_dist == min_f:
                        closest_sector = "\033[1;32mFRONT (+X)\033[0m"
                    elif closest_dist == min_l:
                        closest_sector = "\033[1;34mLEFT (+Y)\033[0m"
                    elif closest_dist == min_r:
                        closest_sector = "\033[1;33mRIGHT (-Y)\033[0m"
                    else:
                        closest_sector = "\033[1;35mBACK (-X)\033[0m"
                else:
                    closest_sector = "None (No obstacles in range)"

                print(f"  • Front Sector : {min_f:>5.2f} m  │  Back Sector  : {min_b:>5.2f} m")
                print(f"  • Left Sector  : {min_l:>5.2f} m  │  Right Sector : {min_r:>5.2f} m")
                print(f"  • Closest Obstacle Detected In: {closest_sector} at {closest_dist:.2f} m")
                print("  \033[0;37m-> Stand on the robot's physical RIGHT: verify closest says RIGHT.\033[0m")
                print("  \033[0;37m-> Stand on the robot's physical LEFT : verify closest says LEFT.\033[0m")
            else:
                print("  Waiting for /scan messages...")

            # 2. Wheel Odometry vs IMU Yaw Agreement & Linear Drive Test
            print("\n\033[1;36m[2] ODOMETRY & IMU DIRECTION TEST\033[0m")
            odom_wz = node.latest_odom.twist.twist.angular.z if node.latest_odom else 0.0
            odom_vx = node.latest_odom.twist.twist.linear.x if node.latest_odom else 0.0
            imu_wz = node.latest_imu.angular_velocity.z if node.latest_imu else 0.0

            odom_yaw_dir = "STOPPED"
            if abs(odom_wz) > 0.05:
                odom_yaw_dir = "\033[1;32mTURNING LEFT (+)\033[0m" if odom_wz > 0 else "\033[1;33mTURNING RIGHT (-)\033[0m"

            imu_yaw_dir = "STOPPED"
            if abs(imu_wz) > 0.05:
                imu_yaw_dir = "\033[1;32mTURNING LEFT (+)\033[0m" if imu_wz > 0 else "\033[1;33mTURNING RIGHT (-)\033[0m"

            odom_lin_dir = "STOPPED"
            if abs(odom_vx) > 0.02:
                odom_lin_dir = "\033[1;32mFORWARD (+)\033[0m" if odom_vx > 0 else "\033[1;33mREVERSE (-)\033[0m"

            print(f"  • Wheel Odom Linear Vx (/odom)    : {odom_vx:>+6.2f} m/s   -> {odom_lin_dir}")
            print(f"  • Wheel Odom Yaw Rate (/odom)     : {odom_wz:>+6.2f} rad/s -> {odom_yaw_dir}")
            print(f"  • IMU Gyroscope Z Rate (/imu/data): {imu_wz:>+6.2f} rad/s -> {imu_yaw_dir}")

            if abs(odom_wz) > 0.10 and abs(imu_wz) > 0.10:
                if (odom_wz > 0 and imu_wz > 0) or (odom_wz < 0 and imu_wz < 0):
                    match_status = "\033[1;32m✅ PERFECT MATCH (Both agree on direction)\033[0m"
                else:
                    match_status = "\033[1;31m❌ REVERSED POLARITY! (One is + and other is -)\033[0m"
                print(f"  • Yaw Direction Alignment : {match_status}")

            # 3. EKF Covariance Watchdog
            print("\n\033[1;36m[3] EKF STATE & COVARIANCE WATCHDOG (/odometry/filtered)\033[0m")
            if node.latest_ekf:
                cov_x = node.latest_ekf.pose.covariance[0]
                cov_y = node.latest_ekf.pose.covariance[7]
                cov_yaw = node.latest_ekf.pose.covariance[35]

                if cov_x < 1.0 and cov_y < 1.0:
                    cov_status = "\033[1;32m✅ HEALTHY (Low covariance, stable filter)\033[0m"
                else:
                    cov_status = f"\033[1;31m⚠️ HIGH COVARIANCE (sigma_x={cov_x:.1f}, sigma_y={cov_y:.1f})\033[0m"
                print(f"  • Position Covariance (X, Y) : {cov_x:.4f}, {cov_y:.4f} -> {cov_status}")
                print(f"  • Yaw Covariance              : {cov_yaw:.4f}")
            else:
                print("  Waiting for /odometry/filtered...")

            print("\n════════════════════════════════════════════════════════════════════════════════")
            print("  Controls: Wave hand or walk around robot to test sectors  │  Ctrl+C to Exit")
            print("════════════════════════════════════════════════════════════════════════════════")

            time.sleep(0.2)

    except KeyboardInterrupt:
        print("\nVerifier closed.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
