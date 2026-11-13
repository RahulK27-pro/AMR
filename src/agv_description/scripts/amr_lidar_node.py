#!/usr/bin/env python3
"""
AMR Native YDLIDAR X2 ROS 2 Driver Node
=======================================
Reads serial data directly from YDLIDAR X2 without external SDK or C++ dependencies.
Publishes standard sensor_msgs/LaserScan to /scan at ~10 Hz.

Hardware:
  - Port: /dev/ttyUSB0
  - Baudrate: 115200
  - Frame ID: laser_frame
"""

import math
import time
import struct
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class AMRLidarNode(Node):
    def __init__(self):
        super().__init__('amr_lidar_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'laser_frame')
        self.declare_parameter('range_min', 0.10)
        self.declare_parameter('range_max', 8.0)
        self.declare_parameter('num_bins', 360)

        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.num_bins = self.get_parameter('num_bins').value

        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)

        self.running = True
        self.mock_mode = False
        self.ser = None

        # 360-degree range bin accumulator (default to inf)
        self.current_ranges = [float('inf')] * self.num_bins
        self.last_start_angle = 0.0

        self.thread = threading.Thread(target=self._lidar_worker_thread, daemon=True)
        self.thread.start()

        self.get_logger().info(
            f"AMR Native YDLIDAR X2 Node started on {self.port} @ {self.baudrate} baud (publishing /scan)"
        )

    def _lidar_worker_thread(self):
        try:
            import serial
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1.0,
                bytesize=8,
                parity='N',
                stopbits=1
            )
            self.ser.reset_input_buffer()
            self.get_logger().info(f"Connected to LiDAR serial port: {self.port}")
        except Exception as e:
            self.get_logger().warn(f"Failed to open LiDAR serial port {self.port}: {e}. Running in MOCK mode.")
            self.mock_mode = True

        if self.mock_mode:
            self._mock_loop()
        else:
            self._serial_parse_loop()

    def _mock_loop(self):
        rate = 10.0
        while self.running and rclpy.ok():
            scan = self._create_scan_msg([2.5] * self.num_bins)
            self.scan_pub.publish(scan)
            time.sleep(1.0 / rate)

    def _serial_parse_loop(self):
        while self.running and rclpy.ok():
            try:
                # Sync header: 0xAA 0x55
                b1 = self.ser.read(1)
                if not b1 or b1[0] != 0xAA:
                    continue
                b2 = self.ser.read(1)
                if not b2 or b2[0] != 0x55:
                    continue

                # Read 8 header bytes: CT(1), LSN(1), FSA(2), LSA(2), CS(2)
                hdr = self.ser.read(8)
                if len(hdr) < 8:
                    continue

                pkt_type = hdr[0]
                sample_count = hdr[1]
                raw_fsa = struct.unpack_from('<H', hdr, 2)[0]
                raw_lsa = struct.unpack_from('<H', hdr, 4)[0]

                start_angle = (raw_fsa >> 1) / 64.0
                end_angle = (raw_lsa >> 1) / 64.0

                if sample_count == 0:
                    continue

                # Read distance samples (2 bytes each)
                data_bytes = self.ser.read(sample_count * 2)
                if len(data_bytes) < sample_count * 2:
                    continue

                # Check if this packet begins a new 360-degree rotation
                # (either CT bit 0 is set OR angle wrapped around from ~350 to ~0)
                is_new_scan = (pkt_type & 0x01 != 0) or (start_angle < self.last_start_angle and self.last_start_angle > 300.0)

                if is_new_scan:
                    # Publish the completed 360-degree scan
                    scan_msg = self._create_scan_msg(self.current_ranges)
                    self.scan_pub.publish(scan_msg)
                    # Reset bin accumulator
                    self.current_ranges = [float('inf')] * self.num_bins

                self.last_start_angle = start_angle

                # Interpolate sample angles and populate bins
                angle_diff = end_angle - start_angle
                if angle_diff < 0.0:
                    angle_diff += 360.0

                step = angle_diff / (sample_count - 1) if sample_count > 1 else 0.0

                for i in range(sample_count):
                    raw_dist = struct.unpack_from('<H', data_bytes, i * 2)[0]
                    dist_meters = (raw_dist / 4.0) / 1000.0  # mm to meters

                    if self.range_min <= dist_meters <= self.range_max:
                        cur_ang = (start_angle + step * i) % 360.0
                        # Bin index [0, num_bins - 1]
                        bin_idx = int((cur_ang / 360.0) * self.num_bins) % self.num_bins
                        # Store closest obstacle in bin
                        if dist_meters < self.current_ranges[bin_idx]:
                            self.current_ranges[bin_idx] = dist_meters

            except Exception as e:
                self.get_logger().debug(f"Serial stream glitch: {e}")
                time.sleep(0.01)

    def _create_scan_msg(self, ranges) -> LaserScan:
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.frame_id

        scan.angle_min = -math.pi
        scan.angle_max = math.pi - (2.0 * math.pi / self.num_bins)
        scan.angle_increment = (2.0 * math.pi) / self.num_bins
        scan.time_increment = (1.0 / 10.0) / self.num_bins
        scan.scan_time = 0.1
        scan.range_min = float(self.range_min)
        scan.range_max = float(self.range_max)

        # Shift [0, 360) degrees to [-pi, +pi) for ROS standard laser scan
        half = self.num_bins // 2
        shifted_ranges = ranges[half:] + ranges[:half]
        scan.ranges = [float(r) for r in shifted_ranges]
        return scan

    def destroy_node(self):
        self.running = False
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AMRLidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down AMR Lidar Node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

