#!/usr/bin/env python3
"""ROS 2 dataset recorder for capturing /scan, /tf, and optional /dynamic_obstacle/odom."""

import argparse
import math
import os
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


class LidarDatasetRecorder(Node):
    """Records synchronized 2D LiDAR scans, robot poses, and optional GT obstacle poses."""

    def __init__(self, out_path: str, gt_odom_topic: str = "/dynamic_obstacle/odom"):
        super().__init__("lidar_dataset_recorder")
        self.out_path = out_path
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_gt = None
        self.frames_ranges = []
        self.frames_poses = []
        self.frames_ts = []
        self.frames_gt = []
        self.angle_min = -math.pi
        self.angle_inc = 2.0 * math.pi / 360.0

        self.create_subscription(LaserScan, "/scan", self._scan_cb, qos_profile_sensor_data)
        if gt_odom_topic:
            self.create_subscription(Odometry, gt_odom_topic, self._gt_cb, 10)
        self.get_logger().info(f"[RECORDER] Recording /scan + TF -> {self.out_path} (Ctrl-C to save)")

    def _gt_cb(self, msg: Odometry):
        self.latest_gt = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            float(msg.twist.twist.linear.x),
            float(msg.twist.twist.linear.y),
        )

    @staticmethod
    def _quat_to_yaw(q) -> float:
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _scan_cb(self, msg: LaserScan):
        try:
            t = self.tf_buffer.lookup_transform(
                "map",
                msg.header.frame_id if msg.header.frame_id else "base_link",
                msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
            px = float(t.transform.translation.x)
            py = float(t.transform.translation.y)
            pyaw = self._quat_to_yaw(t.transform.rotation)
        except TransformException:
            return

        self.angle_min = float(msg.angle_min)
        self.angle_inc = float(msg.angle_increment)
        self.frames_ranges.append(np.asarray(msg.ranges, dtype=np.float32))
        self.frames_poses.append([px, py, pyaw])
        self.frames_ts.append(float(msg.header.stamp.sec) + 1e-9 * float(msg.header.stamp.nanosec))
        self.frames_gt.append(list(self.latest_gt) if self.latest_gt is not None else [np.nan] * 4)

    def save(self):
        if not self.frames_ranges:
            self.get_logger().warn("[RECORDER] No frames captured; skipping save.")
            return
        os.makedirs(os.path.dirname(os.path.abspath(self.out_path)), exist_ok=True)
        np.savez_compressed(
            self.out_path,
            ranges=np.asarray(self.frames_ranges, dtype=np.float32),
            poses=np.asarray(self.frames_poses, dtype=np.float32),
            timestamps=np.asarray(self.frames_ts, dtype=np.float64),
            gt_odom=np.asarray(self.frames_gt, dtype=np.float32),
            angle_min=np.float32(self.angle_min),
            angle_inc=np.float32(self.angle_inc),
        )
        self.get_logger().info(f"[RECORDER] Saved {len(self.frames_ranges)} frames to {self.out_path}")


def main(args=None):
    parser = argparse.ArgumentParser(description="Record /scan + TF episodes to compressed .npz")
    parser.add_argument(
        "--out",
        default=os.path.expanduser(f"~/AMR/AMR-main/ml_datasets/capture_{int(time.time())}.npz"),
    )
    parser.add_argument("--gt-odom-topic", default="/dynamic_obstacle/odom")
    known, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = LidarDatasetRecorder(known.out, gt_odom_topic=known.gt_odom_topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
