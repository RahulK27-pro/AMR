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
        self.declare_parameter('range_min', 0.15)
        self.declare_parameter('range_max', 8.0)
        self.declare_parameter('num_bins', 360)
        self.declare_parameter('inverted', True)
        self.declare_parameter('angle_offset', 180.0)

        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.num_bins = self.get_parameter('num_bins').value
        self.inverted = self.get_parameter('inverted').value
        self.angle_offset = float(self.get_parameter('angle_offset').value)

        self.scan_pub = self.create_publisher(LaserScan, '/scan', 5)

        self.running = True
        self.mock_mode = False
        self.ser = None

        # 360-degree range bin accumulator (default to inf)
        self.current_ranges = [float('inf')] * self.num_bins
        self.last_start_angle = 0.0
        self._seen_ct_marker = False
        self._scan_pkt_count = 0
        self._last_scan_pub_time = time.monotonic()
        self._last_scan_duration = 1.0 / 10.0
        self._cs_verified_count = 0
        self._cs_fail_streak = 0
        self._cs_disabled = False

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

    def _verify_packet_checksum(self, pkt_type, sample_count, raw_fsa, raw_lsa, cs, data_bytes):
        """Verify YDLIDAR X2 16-bit XOR checksum; auto-bypass if hardware omits standard CS."""
        if self._cs_disabled:
            return True
        calc = 0x55AA ^ ((sample_count << 8) | pkt_type) ^ raw_fsa ^ raw_lsa
        for idx in range(sample_count):
            calc ^= struct.unpack_from('<H', data_bytes, idx * 2)[0]
        if (calc & 0xFFFF) == cs:
            self._cs_verified_count += 1
            self._cs_fail_streak = 0
            return True
        self._cs_fail_streak += 1
        if self._cs_verified_count == 0 and self._cs_fail_streak >= 25:
            self._cs_disabled = True
            self.get_logger().warn(
                "YDLIDAR checksum format mismatch over 25 consecutive packets; disabling CS gate."
            )
            return True
        return False

    def _should_flush_scan(self, pkt_type, start_angle, now_mono):
        """Determine scan revolution boundary without double-flushing near 0/360 deg."""
        is_ct_start = (pkt_type & 0x01) != 0
        dt_pub = now_mono - self._last_scan_pub_time
        if is_ct_start:
            self._seen_ct_marker = True
            return self._scan_pkt_count >= 2 or dt_pub >= 0.05

        angle_wrapped = (start_angle < self.last_start_angle and self.last_start_angle > 300.0)
        if not angle_wrapped:
            return False
        # Only flush on angle wrap if CT markers are absent or a CT=1 packet dropped
        if not self._seen_ct_marker:
            return self._scan_pkt_count >= 2
        return self._scan_pkt_count >= 4 and dt_pub >= 0.08

    def _reopen_serial(self):
        """Close and reopen the LiDAR serial port after a serial/USB error."""
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        while self.running and rclpy.ok():
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
                self.current_ranges = [float('inf')] * self.num_bins
                self._scan_pkt_count = 0
                self.get_logger().info(f"Reconnected to LiDAR serial port: {self.port}")
                return True
            except Exception as e:
                self.get_logger().warn(f"Waiting to reconnect LiDAR port {self.port}: {e}")
                time.sleep(1.0)
        return False

    def _serial_parse_loop(self):
        while self.running and rclpy.ok():
            try:
                # Sync header: 0xAA 0x55 (preserve back-to-back 0xAA bytes during resync)
                b1 = self.ser.read(1)
                if not b1 or b1[0] != 0xAA:
                    continue
                b2 = self.ser.read(1)
                while b2 and b2[0] == 0xAA:
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
                cs = struct.unpack_from('<H', hdr, 6)[0]

                start_angle = (raw_fsa >> 1) / 64.0
                end_angle = (raw_lsa >> 1) / 64.0

                if sample_count == 0 or start_angle >= 360.0 or end_angle >= 360.0:
                    continue

                # Read distance samples (2 bytes each)
                data_bytes = self.ser.read(sample_count * 2)
                if len(data_bytes) < sample_count * 2:
                    continue

                if not self._verify_packet_checksum(pkt_type, sample_count, raw_fsa, raw_lsa, cs, data_bytes):
                    continue

                now_mono = time.monotonic()
                if self._should_flush_scan(pkt_type, start_angle, now_mono):
                    dt_scan = now_mono - self._last_scan_pub_time
                    if 0.04 <= dt_scan <= 0.30:
                        self._last_scan_duration = dt_scan
                    scan_msg = self._create_scan_msg(self.current_ranges, self._last_scan_duration)
                    self.scan_pub.publish(scan_msg)
                    self.current_ranges = [float('inf')] * self.num_bins
                    self._scan_pkt_count = 0
                    self._last_scan_pub_time = now_mono

                self.last_start_angle = start_angle
                self._scan_pkt_count += 1

                # Interpolate sample angles and populate bins
                angle_diff = end_angle - start_angle
                if angle_diff < 0.0:
                    angle_diff += 360.0

                step = angle_diff / (sample_count - 1) if sample_count > 1 else 0.0

                for i in range(sample_count):
                    raw_dist = struct.unpack_from('<H', data_bytes, i * 2)[0]
                    dist_meters = (raw_dist / 4.0) / 1000.0  # mm to meters

                    if self.range_min <= dist_meters <= self.range_max:
                        raw_ang = (start_angle + step * i) % 360.0
                        if self.inverted:
                            # Invert Clockwise laser sweep to Counter-Clockwise (ROS REP-103 convention)
                            cur_ang = (360.0 - raw_ang + self.angle_offset) % 360.0
                        else:
                            cur_ang = (raw_ang + self.angle_offset) % 360.0

                        bin_idx = int((cur_ang / 360.0) * self.num_bins) % self.num_bins
                        if dist_meters < self.current_ranges[bin_idx]:
                            self.current_ranges[bin_idx] = dist_meters

            except Exception as e:
                self.get_logger().warn(f"LiDAR serial read error ({e}); attempting port reconnect...")
                self._reopen_serial()

    def _create_scan_msg(self, ranges, scan_duration=0.1) -> LaserScan:
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.frame_id

        scan.angle_min = -math.pi
        scan.angle_max = math.pi - (2.0 * math.pi / self.num_bins)
        scan.angle_increment = (2.0 * math.pi) / self.num_bins
        scan.scan_time = float(scan_duration)
        scan.time_increment = float(scan_duration) / self.num_bins
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

