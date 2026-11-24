#!/usr/bin/env python3
"""
amr_auto_localizer.py
=====================
Zero-Prior Global Scan-Matching Auto-Localizer for Physical AMR.

Solves the Kidnapped Robot / Arbitrary Initial Starting Pose Problem:
  1. Computes Likelihood Field / Distance Transform over all traversable map free space.
  2. Subscribes to physical LiDAR (/scan).
  3. Executes vectorized 2D Correlative Scan Matching (< 100ms) to identify global (x, y, yaw).
  4. Automatically publishes /initialpose (geometry_msgs/PoseWithCovarianceStamped).
  5. Triggers AMCL no-motion update service so particles snap onto the real physical pose.
  6. Provides '/trigger_auto_relocalization' (std_srvs/Trigger) service for dynamic repositioning.
"""

import os
import sys
import math
import time
import json
import yaml
import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Trigger, Empty
from ament_index_python.packages import get_package_share_directory


def euler_to_quaternion(yaw: float):
    return [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]


class AMRAutoLocalizer(Node):
    def __init__(self):
        super().__init__('amr_auto_localizer')

        self.declare_parameter('map_name', '')
        self.declare_parameter('use_sim_time', False)
        self.declare_parameter('auto_on_startup', True)
        self.declare_parameter('sigma_hit', 0.15)
        self.declare_parameter('min_clearance', 0.20)

        self.map_name = self.get_parameter('map_name').value.strip()
        self.auto_on_startup = self.get_parameter('auto_on_startup').value
        self.sigma = float(self.get_parameter('sigma_hit').value)
        self.min_clearance = float(self.get_parameter('min_clearance').value)

        # Map representation
        self.map_loaded = False
        self.map_res = 0.05
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_w = 0
        self.map_h = 0
        self.dist_map = None
        self.likelihood_map = None
        self.cand_x = None
        self.cand_y = None
        self.cand_yaws = np.linspace(0, 2 * np.pi, 36, endpoint=False)  # 10 deg steps

        # Publishers & Services
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10
        )
        self.reloc_srv = self.create_service(
            Trigger, '/trigger_auto_relocalization', self.relocalize_service_cb
        )
        self.nomotion_client = self.create_client(Empty, '/request_nomotion_update')

        # Map subscription (latched)
        map_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, map_qos
        )

        # Sensor data QoS for LiDAR
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, sensor_qos
        )

        self.latest_scan = None
        self.has_localized = False

        # Attempt to load static map file directly from disk for instant readiness
        self._load_static_map_file()

        self.get_logger().info(
            "AMR Zero-Prior Global Auto-Localizer Active! "
            "Listening for LiDAR scan to estimate arbitrary robot starting pose..."
        )

    def _load_static_map_file(self):
        """Loads static map YAML and constructs initial likelihood field before /map topic arrives."""
        candidates = []
        ws_root = os.path.join(os.environ.get('HOME', ''), 'AMR', 'AMR-main')
        ws_root_maps = os.path.join(ws_root, 'maps')
        ws_pkg_maps = os.path.join(ws_root, 'src', 'agv_description', 'maps')

        if self.map_name:
            candidates.append(os.path.join(ws_root_maps, f'{self.map_name}.yaml'))
            candidates.append(os.path.join(ws_pkg_maps, f'{self.map_name}.yaml'))

        try:
            pkg_share = get_package_share_directory('agv_description')
            if self.map_name:
                candidates.append(os.path.join(pkg_share, 'maps', f'{self.map_name}.yaml'))
            candidates.append(os.path.join(pkg_share, 'maps', 'final_map.yaml'))
            candidates.append(os.path.join(pkg_share, 'maps', 'prakash_room.yaml'))
        except Exception:
            pass

        candidates.append(os.path.join(ws_root_maps, 'final_map.yaml'))
        candidates.append(os.path.join(ws_root_maps, 'prakash_room.yaml'))

        yaml_path = None
        for c in candidates:
            if os.path.exists(c):
                yaml_path = c
                break

        if not yaml_path:
            return

        try:
            with open(yaml_path, 'r') as f:
                meta = yaml.safe_load(f)
            pgm_file = meta.get('image', '')
            if not os.path.isabs(pgm_file):
                pgm_file = os.path.join(os.path.dirname(yaml_path), pgm_file)

            if not os.path.exists(pgm_file):
                return

            img = Image.open(pgm_file)
            arr = np.array(img)
            arr = np.flipud(arr)  # ROS origin bottom-left

            self.map_res = float(meta.get('resolution', 0.05))
            orig = meta.get('origin', [0.0, 0.0, 0.0])
            self.map_origin_x = float(orig[0])
            self.map_origin_y = float(orig[1])
            self.map_h, self.map_w = arr.shape

            self._build_likelihood_field(arr)
            self.get_logger().info(
                f"[AUTO_LOCALIZE] Initialized likelihood field from {os.path.basename(pgm_file)} "
                f"({self.map_w}x{self.map_h}, res={self.map_res}m, {len(self.cand_x)} candidate poses)."
            )
        except Exception as e:
            self.get_logger().warn(f"[AUTO_LOCALIZE] Could not preload map YAML: {e}")

    def map_callback(self, msg: OccupancyGrid):
        """Update map and likelihood field dynamically if received via topic."""
        try:
            w = msg.info.width
            h = msg.info.height
            grid = np.array(msg.data, dtype=np.int8).reshape((h, w))
            # ROS OccupancyGrid: -1=unknown, 0=free, 100=occupied
            arr = np.full((h, w), 205, dtype=np.uint8)
            arr[grid == 0] = 254
            arr[grid > 50] = 0

            self.map_res = float(msg.info.resolution)
            self.map_origin_x = float(msg.info.origin.position.x)
            self.map_origin_y = float(msg.info.origin.position.y)
            self.map_w = w
            self.map_h = h

            self._build_likelihood_field(arr)
            self.get_logger().info(f"[AUTO_LOCALIZE] Updated likelihood field from /map topic ({w}x{h}).")
        except Exception as e:
            self.get_logger().warn(f"[AUTO_LOCALIZE] Error processing /map: {e}")

    def _build_likelihood_field(self, arr: np.ndarray):
        """Constructs distance transform and candidate poses in free space."""
        # Strictly free space (white pixels >= 240)
        is_free = (arr >= 240).astype(np.uint8)
        # Wall obstacles (black pixels < 50)
        occ = (arr < 50).astype(np.uint8)

        # Distance transform: distance in meters to nearest wall
        self.dist_map = distance_transform_edt(occ == 0) * self.map_res
        self.likelihood_map = np.exp(-0.5 * (self.dist_map / self.sigma) ** 2)
        # Wall cells have peak likelihood 1.0
        self.likelihood_map[occ == 1] = 1.0

        # Candidate free-space poses with sufficient physical clearance from walls
        valid_y, valid_x = np.where((is_free == 1) & (self.dist_map >= self.min_clearance))
        # Downsample candidate positions to every 2nd cell (~10cm grid) for speed
        self.cand_x = valid_x[::2] * self.map_res + self.map_origin_x
        self.cand_y = valid_y[::2] * self.map_res + self.map_origin_y
        self.map_loaded = True

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg
        if self.auto_on_startup and self.map_loaded and not self.has_localized:
            self.has_localized = True
            # Allow a short 0.5s pause for LiDAR driver to deliver full 360-degree sweep
            self.create_timer(0.5, self._run_startup_localizer, clock=self.get_clock())

    def _run_startup_localizer(self):
        # One-shot trigger
        self.find_and_publish_global_pose()

    def relocalize_service_cb(self, request, response):
        """Service callback to manually or programmatically trigger global relocalization."""
        success, msg = self.find_and_publish_global_pose()
        response.success = success
        response.message = msg
        return response

    def find_and_publish_global_pose(self):
        """Executes vectorized correlative scan matching and publishes /initialpose."""
        if not self.map_loaded or self.dist_map is None:
            self.get_logger().warn("[AUTO_LOCALIZE] Cannot localize: Map not loaded yet.")
            return False, "Map not loaded yet."

        if self.latest_scan is None:
            self.get_logger().warn("[AUTO_LOCALIZE] Cannot localize: No LiDAR scan received yet.")
            return False, "No LiDAR scan received yet."

        scan = self.latest_scan
        ranges = np.array(scan.ranges)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment

        # Filter valid physical range returns (exclude chassis self-reflections < 0.12m)
        valid = (ranges > max(0.12, scan.range_min)) & (ranges < min(6.5, scan.range_max)) & np.isfinite(ranges)
        ranges = ranges[valid]
        angles = angles[valid]

        if len(ranges) < 30:
            self.get_logger().warn("[AUTO_LOCALIZE] Scan has too few valid beams (<30) to localize.")
            return False, "Scan has too few valid beams."

        # Subsample to 120 points for high speed (<100ms) with dense coverage
        step = max(1, len(ranges) // 120)
        ranges = ranges[::step]
        angles = angles[::step]

        local_x = ranges * np.cos(angles)
        local_y = ranges * np.sin(angles)

        t0 = time.time()

        # Vectorized Coarse Scan Matching
        c_y = np.cos(self.cand_yaws)[:, None]
        s_y = np.sin(self.cand_yaws)[:, None]
        rot_x = c_y * local_x[None, :] - s_y * local_y[None, :]
        rot_y = s_y * local_x[None, :] + c_y * local_y[None, :]

        best_score = -1.0
        best_pose = None

        for yi in range(len(self.cand_yaws)):
            rx = rot_x[yi]
            ry = rot_y[yi]
            gx = ((self.cand_x[:, None] + rx[None, :] - self.map_origin_x) / self.map_res).astype(int)
            gy = ((self.cand_y[:, None] + ry[None, :] - self.map_origin_y) / self.map_res).astype(int)

            in_b = (gx >= 0) & (gx < self.map_w) & (gy >= 0) & (gy < self.map_h)
            gx_c = np.clip(gx, 0, self.map_w - 1)
            gy_c = np.clip(gy, 0, self.map_h - 1)

            lk = self.likelihood_map[gy_c, gx_c]
            lk[~in_b] = 0.0
            scores = np.mean(lk, axis=1)
            max_i = np.argmax(scores)
            if scores[max_i] > best_score:
                best_score = scores[max_i]
                best_pose = (self.cand_x[max_i], self.cand_y[max_i], self.cand_yaws[yi])

        if best_pose is None:
            self.get_logger().error("[AUTO_LOCALIZE] Global localization failed to converge.")
            return False, "Global search failed."

        # Fine Grid Refinement (0.02m spatial step, 2.0 deg angular step)
        bx, by, byaw = best_pose
        fine_x = np.linspace(bx - 0.12, bx + 0.12, 7)
        fine_y = np.linspace(by - 0.12, by + 0.12, 7)
        fine_yaws = np.linspace(byaw - np.radians(12), byaw + np.radians(12), 13)

        refined_best_score = best_score
        refined_pose = best_pose

        for yaw in fine_yaws:
            c_y_f = np.cos(yaw)
            s_y_f = np.sin(yaw)
            rot_x_f = local_x * c_y_f - local_y * s_y_f
            rot_y_f = local_x * s_y_f + local_y * c_y_f
            for px in fine_x:
                for py in fine_y:
                    gx = ((px + rot_x_f - self.map_origin_x) / self.map_res).astype(int)
                    gy = ((py + rot_y_f - self.map_origin_y) / self.map_res).astype(int)
                    in_b = (gx >= 0) & (gx < self.map_w) & (gy >= 0) & (gy < self.map_h)
                    gx_c = np.clip(gx, 0, self.map_w - 1)
                    gy_c = np.clip(gy, 0, self.map_h - 1)
                    lk = self.likelihood_map[gy_c, gx_c]
                    lk[~in_b] = 0.0
                    score = np.mean(lk)
                    if score > refined_best_score:
                        refined_best_score = score
                        refined_pose = (px, py, yaw)

        elapsed_ms = (time.time() - t0) * 1000.0
        final_x, final_y, final_yaw = refined_pose
        yaw_deg = math.degrees(final_yaw) % 360.0

        # Construct and publish /initialpose
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.pose.pose.position.x = float(final_x)
        msg.pose.pose.position.y = float(final_y)
        msg.pose.pose.position.z = 0.0

        q = euler_to_quaternion(final_yaw)
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]

        # Covariance: small confident covariance
        cov = [0.0] * 36
        cov[0] = 0.05    # X variance (m^2)
        cov[7] = 0.05    # Y variance (m^2)
        cov[35] = 0.04   # Yaw variance (rad^2, ~11 deg)
        msg.pose.covariance = cov

        self.initial_pose_pub.publish(msg)

        # Trigger AMCL no-motion update so particle filter updates weights while stationary
        self._trigger_amcl_nomotion_update()

        log_str = (
            f"[AUTO_LOCALIZE] ✅ GLOBAL LOCALIZATION SUCCESS! "
            f"Physical Pose: X={final_x:.3f}m, Y={final_y:.3f}m, Yaw={yaw_deg:.1f}° | "
            f"Confidence Score={refined_best_score:.3f} | Solved in {elapsed_ms:.1f}ms"
        )
        self.get_logger().info("=" * 72)
        self.get_logger().info(log_str)
        self.get_logger().info("=" * 72)

        return True, log_str

    def _trigger_amcl_nomotion_update(self):
        """Calls AMCL's /request_nomotion_update service to converge particles immediately."""
        if self.nomotion_client.wait_for_service(timeout_sec=1.0):
            req = Empty.Request()
            for _ in range(3):
                self.nomotion_client.call_async(req)
                time.sleep(0.05)


def main(args=None):
    rclpy.init(args=args)
    node = AMRAutoLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
