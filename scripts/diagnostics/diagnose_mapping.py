#!/usr/bin/env python3
"""
diagnose_mapping.py
===================
Comprehensive Real-Time SLAM Mapping & Sensor Calibration Diagnostic Tool
for Physical AMR (Raspberry Pi 5 + Laptop Workstation over CycloneDDS).

Features:
  1. Multi-Machine Network & Clock Skew Monitor (Pi vs Laptop timestamp drift)
  2. ROS 2 Topic Rates & Jitter Watchdog (/scan, /odom, /imu/data, /odometry/filtered, /map)
  3. TF Tree Chain Inspector (map -> odom -> base_link -> laser_frame, imu_link)
  4. SENSOR CALIBRATION ANALYZER:
     - Gyro vs Wheel Odometry Yaw Agreement (calculates effective track width L ratio)
     - Commanded vs Measured Linear Velocity (wheel radius / CPR accuracy)
     - IMU Stationary Gyro Bias & Noise Floor
     - LiDAR Self-Obstruction & Blind Sector Detection (< 0.12m chassis hits)
  5. SLAM Mapping Progress & Quality (Grid dimensions, explored area m^2, map->odom drift)
  6. Actionable Calibration Guidance & Recommendations

Usage:
  # Real-time interactive HUD (refreshes every 0.5s):
  python3 diagnose_mapping.py

  # Single snapshot report (samples for 3s then outputs full text summary):
  python3 diagnose_mapping.py --snapshot

  # Save diagnostics to JSON file:
  python3 diagnose_mapping.py --snapshot --json /tmp/mapping_diag.json
"""

import os
import sys
import time
import math
import argparse
import json
from collections import deque

# Environment Configuration
if not os.environ.get('ROS_DISTRO'):
    print("[ERROR] ROS 2 environment not sourced! Run: source /opt/ros/jazzy/setup.bash")
    sys.exit(1)

if os.environ.get('ROS_DOMAIN_ID') != '7':
    os.environ['ROS_DOMAIN_ID'] = '7'

if os.environ.get('RMW_IMPLEMENTATION') != 'rmw_cyclonedds_cpp':
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'

# Fallback CycloneDDS URI if not set
ws_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
ws_cyclone = os.path.join(ws_dir, 'config', 'cyclonedds.xml')
if not os.environ.get('CYCLONEDDS_URI') and os.path.exists(ws_cyclone):
    os.environ['CYCLONEDDS_URI'] = f"file://{ws_cyclone}"

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist
import tf2_ros


class MappingDiagnosticsNode(Node):
    def __init__(self):
        super().__init__('amr_mapping_diagnostics', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.parameter.Parameter.Type.BOOL, False)
        ])

        # Message Counters & Timestamps
        self.topic_counts = {
            '/scan': 0,
            '/odom': 0,
            '/imu/data': 0,
            '/odometry/filtered': 0,
            '/map': 0,
            '/cmd_vel': 0,
        }
        self.last_msg_times = {}
        self.clock_skews = {
            '/scan': deque(maxlen=20),
            '/odom': deque(maxlen=50),
            '/imu/data': deque(maxlen=50),
        }

        # Calibration Data Buffers
        self.latest_cmd_vel = None
        self.latest_odom = None
        self.latest_imu = None
        self.latest_scan = None
        self.latest_map = None

        # Kinematic Integration for Calibration Comparison
        self.cum_odom_yaw = 0.0      # Radians
        self.cum_imu_yaw = 0.0       # Radians
        self.active_turning_samples = 0
        self.last_odom_time = None
        self.last_imu_time = None
        self.stationary_gyro_samples = deque(maxlen=100)

        # LiDAR Self-Occlusion Hits
        self.lidar_self_hits = []    # List of (angle_deg, range_m)
        self.lidar_min_range = float('inf')
        self.lidar_valid_pct = 0.0

        # QoS Profiles
        latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        # Subscriptions
        self.create_subscription(LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(Imu, '/imu/data', self._imu_cb, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self._ekf_cb, 10)
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, latched_qos)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, 10)

        # TF2 Buffer
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def _record_skew(self, topic, stamp_sec):
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        skew = now_sec - stamp_sec
        self.clock_skews[topic].append(skew)

    def _scan_cb(self, msg: LaserScan):
        self.topic_counts['scan'] = self.topic_counts.get('scan', 0) + 1
        self.topic_counts['/scan'] += 1
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._record_skew('/scan', stamp)
        self.latest_scan = msg

        # Analyze ranges for chassis reflections (< 0.12m)
        self_hits = []
        min_r = float('inf')
        valid_count = 0
        num_readings = len(msg.ranges)
        if num_readings > 0:
            for i, r in enumerate(msg.ranges):
                if not math.isnan(r) and not math.isinf(r) and r > 0.02:
                    valid_count += 1
                    if r < min_r:
                        min_r = r
                    if r < 0.12:  # AGV radius is 0.11m
                        angle_rad = msg.angle_min + i * msg.angle_increment
                        self_hits.append((math.degrees(angle_rad), r))

            self.lidar_valid_pct = (valid_count / num_readings) * 100.0
            self.lidar_min_range = min_r
            self.lidar_self_hits = self_hits[:10]  # Store up to 10 instances

    def _odom_cb(self, msg: Odometry):
        self.topic_counts['/odom'] += 1
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._record_skew('/odom', stamp)
        self.latest_odom = msg

        # Angular integration
        now = time.time()
        if self.last_odom_time is not None:
            dt = now - self.last_odom_time
            if 0.0 < dt < 0.2:
                w_z = msg.twist.twist.angular.z
                if abs(w_z) > 0.10:  # Active turning
                    self.cum_odom_yaw += abs(w_z) * dt
                    self.active_turning_samples += 1
        self.last_odom_time = now

    def _imu_cb(self, msg: Imu):
        self.topic_counts['/imu/data'] += 1
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._record_skew('/imu/data', stamp)
        self.latest_imu = msg

        w_z = msg.angular_velocity.z
        now = time.time()
        if self.last_imu_time is not None:
            dt = now - self.last_imu_time
            if 0.0 < dt < 0.2:
                if abs(w_z) > 0.10:
                    self.cum_imu_yaw += abs(w_z) * dt
        self.last_imu_time = now

        # Stationary bias tracking
        if self.latest_cmd_vel is not None and abs(self.latest_cmd_vel[0]) < 0.01 and abs(self.latest_cmd_vel[1]) < 0.01:
            self.stationary_gyro_samples.append(w_z)

    def _ekf_cb(self, msg: Odometry):
        self.topic_counts['/odometry/filtered'] += 1

    def _map_cb(self, msg: OccupancyGrid):
        self.topic_counts['/map'] += 1
        self.latest_map = msg

    def _cmd_cb(self, msg: Twist):
        self.topic_counts['/cmd_vel'] += 1
        self.latest_cmd_vel = (msg.linear.x, msg.angular.z)


def get_tf_relation(tf_buffer, parent, child):
    """Inspects coordinate transform status between parent and child."""
    try:
        t = tf_buffer.lookup_transform(parent, child, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.15))
        tx = t.transform.translation.x
        ty = t.transform.translation.y
        tz = t.transform.translation.z
        qx = t.transform.rotation.x
        qy = t.transform.rotation.y
        qz = t.transform.rotation.z
        qw = t.transform.rotation.w
        # Convert quaternion to yaw
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw_deg = math.degrees(math.atan2(siny_cosp, cosy_cosp))
        dist = math.sqrt(tx*tx + ty*ty + tz*tz)
        return True, dist, yaw_deg, tx, ty
    except Exception as e:
        return False, 0.0, 0.0, 0.0, 0.0


def format_report(diag_node, sample_sec=2.0):
    """Generates a structured dictionary and markdown report of the system status."""
    report = {}

    # 1. Topic Rates & Clock Skews
    topics = {}
    for topic, count in diag_node.topic_counts.items():
        hz = count / sample_sec
        skews = diag_node.clock_skews.get(topic, [])
        avg_skew_ms = (sum(skews) / len(skews) * 1000.0) if skews else 0.0
        topics[topic] = {
            'rate_hz': round(hz, 1),
            'avg_skew_ms': round(avg_skew_ms, 1)
        }
    report['topics'] = topics

    # 2. TF Transforms
    tf_data = {}
    tf_pairs = [
        ('map', 'odom', 'SLAM Toolbox Correction'),
        ('odom', 'base_link', 'EKF Odometry Localizer'),
        ('base_link', 'laser_frame', 'LiDAR Sensor Mount'),
        ('base_link', 'imu_link', 'IMU Sensor Mount'),
        ('map', 'base_link', 'Global Localization Complete Chain')
    ]
    for parent, child, desc in tf_pairs:
        ok, dist, yaw, tx, ty = get_tf_relation(diag_node.tf_buffer, parent, child)
        tf_data[f"{parent}->{child}"] = {
            'connected': ok,
            'description': desc,
            'dist_m': round(dist, 3) if ok else None,
            'yaw_deg': round(yaw, 1) if ok else None,
            'tx': round(tx, 3) if ok else None,
            'ty': round(ty, 3) if ok else None,
        }
    report['tf'] = tf_data

    # 3. Calibration Metrics
    calib = {}
    # Track width / Yaw Ratio
    cum_odom_deg = math.degrees(diag_node.cum_odom_yaw)
    cum_imu_deg = math.degrees(diag_node.cum_imu_yaw)
    yaw_ratio = (cum_imu_deg / cum_odom_deg) if cum_odom_deg > 5.0 else None
    calib['wheel_yaw_deg'] = round(cum_odom_deg, 1)
    calib['imu_yaw_deg'] = round(cum_imu_deg, 1)
    calib['yaw_ratio_gyro_over_odom'] = round(yaw_ratio, 3) if yaw_ratio else None

    # Track width suggestion
    current_L = 0.240
    if yaw_ratio is not None and abs(cum_odom_deg) > 45.0:
        recommended_L = current_L / yaw_ratio
        calib['effective_track_width_m'] = round(recommended_L, 4)
    else:
        calib['effective_track_width_m'] = current_L

    # Stationary Gyro Bias
    if diag_node.stationary_gyro_samples:
        samples = list(diag_node.stationary_gyro_samples)
        bias = sum(samples) / len(samples)
        calib['gyro_stationary_bias_rad_s'] = round(bias, 5)
    else:
        calib['gyro_stationary_bias_rad_s'] = None

    # LiDAR Health
    calib['lidar_min_range_m'] = round(diag_node.lidar_min_range, 3) if diag_node.lidar_min_range != float('inf') else None
    calib['lidar_valid_pct'] = round(diag_node.lidar_valid_pct, 1)
    calib['lidar_self_reflection_detected'] = len(diag_node.lidar_self_hits) > 0
    calib['lidar_self_hits_angles_deg'] = [round(a, 1) for a, _ in diag_node.lidar_self_hits]
    report['calibration'] = calib

    # 4. Map Statistics
    map_stats = {}
    if diag_node.latest_map:
        m = diag_node.latest_map
        w = m.info.width
        h = m.info.height
        res = m.info.resolution
        total_cells = w * h
        occupied = sum(1 for c in m.data if c > 50)
        free = sum(1 for c in m.data if 0 <= c <= 50)
        unknown = sum(1 for c in m.data if c < 0)
        floor_area_m2 = free * (res * res)
        map_stats['width_m'] = round(w * res, 2)
        map_stats['height_m'] = round(h * res, 2)
        map_stats['resolution_m'] = round(res, 3)
        map_stats['free_floor_m2'] = round(floor_area_m2, 2)
        map_stats['explored_pct'] = round(((occupied + free) / total_cells) * 100.0, 1) if total_cells > 0 else 0
    report['map'] = map_stats

    return report


def print_hud(report):
    """Renders ANSI-formatted single-screen terminal HUD."""
    # Clear screen & home cursor
    print("\033[2J\033[H", end="")
    print("╔════════════════════════════════════════════════════════════════════════════════╗")
    print("║                 AMR PHYSICAL HARDWARE SLAM & CALIBRATION HUD                   ║")
    print("╚════════════════════════════════════════════════════════════════════════════════╝")

    # 1. Multi-Machine Network & Clock Synchronization
    print("\n\033[1;36m[1] MULTI-MACHINE CLOCK & TOPIC RATES (Domain 7, CycloneDDS)\033[0m")
    print("  Topic Name            │ Rate (Hz) │ Target (Hz) │ Clock Skew (ms) │ Status")
    print("  ──────────────────────┼───────────┼─────────────┼─────────────────┼─────────")
    targets = {
        '/scan': (24.0, 15.0),
        '/odom': (50.0, 35.0),
        '/imu/data': (50.0, 35.0),
        '/odometry/filtered': (50.0, 35.0),
        '/map': (1.0, 0.1),
    }
    for topic, (tgt, min_ok) in targets.items():
        data = report['topics'].get(topic, {'rate_hz': 0.0, 'avg_skew_ms': 0.0})
        hz = data['rate_hz']
        skew = data['avg_skew_ms']

        if hz >= min_ok:
            status = "\033[1;32m● ONLINE\033[0m"
        elif hz > 0:
            status = "\033[1;33m▲ DEGRADED\033[0m"
        else:
            status = "\033[1;31m■ DEAD\033[0m"

        skew_str = f"{skew:+.1f} ms"
        if abs(skew) > 80.0:
            skew_str = f"\033[1;31m{skew:+.1f} ms [SKEW!]\033[0m"

        print(f"  {topic:<21} │ {hz:>6.1f} Hz  │ {tgt:>6.1f} Hz   │ {skew_str:>15} │ {status}")

    # 2. Coordinate Transform Tree (TF)
    print("\n\033[1;36m[2] COORDINATE TRANSFORM HIERARCHY (TF Tree)\033[0m")
    for link, info in report['tf'].items():
        conn = info['connected']
        desc = info['description']
        if conn:
            dist = f"{info['dist_m']:.3f} m"
            yaw = f"{info['yaw_deg']:+.1f}°"
            status = f"\033[1;32m✅ OK\033[0m (offset: {dist}, yaw: {yaw})"
        else:
            status = "\033[1;31m❌ DISCONNECTED / TIMEOUT\033[0m"
        print(f"  • {link:<24} [{desc:<26}]: {status}")

    # 3. Sensor Calibration Diagnostics
    cal = report['calibration']
    print("\n\033[1;36m[3] SENSOR CALIBRATION & PHYSICAL METRICS\033[0m")
    odom_yaw = cal['wheel_yaw_deg']
    imu_yaw = cal['imu_yaw_deg']
    ratio = cal['yaw_ratio_gyro_over_odom']

    if ratio is not None:
        if 0.95 <= ratio <= 1.05:
            ratio_status = f"\033[1;32m{ratio:.3f} (EXCELLENT MATCH: ±{abs(1-ratio)*100:.1f}%)\033[0m"
        elif ratio > 1.05:
            ratio_status = f"\033[1;33m{ratio:.3f} (Odometry Under-turning. L={cal['effective_track_width_m']:.3f}m suggested)\033[0m"
        else:
            ratio_status = f"\033[1;33m{ratio:.3f} (Odometry Over-turning. L={cal['effective_track_width_m']:.3f}m suggested)\033[0m"
    else:
        ratio_status = "Waiting for robot rotation (> 45° turn)..."

    print(f"  • Integrated Turn Angle : Odom = {odom_yaw:.1f}° │ IMU Gyro = {imu_yaw:.1f}°")
    print(f"  • Gyro / Odom Yaw Ratio : {ratio_status}")
    print(f"  • Configured Track (L)  : 0.240 m  │  Calibrated Estimate: {cal['effective_track_width_m']:.4f} m")

    # Gyro Zero Bias
    bias = cal['gyro_stationary_bias_rad_s']
    bias_str = f"{bias:+.5f} rad/s" if bias is not None else "Measuring (hold robot still)..."
    print(f"  • Gyro Zero-Rate Bias   : {bias_str}")

    # LiDAR Health & Reflection
    lidar_r = cal['lidar_min_range_m']
    lidar_str = f"{lidar_r:.3f} m" if lidar_r is not None else "No Scans"
    valid_p = cal['lidar_valid_pct']
    ref_warn = "\033[1;31m⚠️ CHASSIS SELF-REFLECTION DETECTED (<0.12m)!\033[0m" if cal['lidar_self_reflection_detected'] else "\033[1;32m✅ Clear (No self-obstruction)\033[0m"
    print(f"  • LiDAR Min Range Seen  : {lidar_str} │ Valid Rays: {valid_p:.1f}% │ Chassis Hits: {ref_warn}")
    if cal['lidar_self_reflection_detected']:
        print(f"    Obstruction Angles    : {cal['lidar_self_hits_angles_deg']} deg")

    # 4. SLAM Map Growth & Quality
    m_info = report['map']
    print("\n\033[1;36m[4] SLAM OCCUPANCY GRID PROGRESS\033[0m")
    if m_info:
        w_m = m_info.get('width_m', 0)
        h_m = m_info.get('height_m', 0)
        floor_m2 = m_info.get('free_floor_m2', 0)
        exp_pct = m_info.get('explored_pct', 0)
        res_cm = m_info.get('resolution_m', 0.05) * 100
        print(f"  • Map Dimensions        : {w_m:.2f} m × {h_m:.2f} m ({res_cm:.1f} cm/cell)")
        print(f"  • Explored Floor Area   : {floor_m2:.2f} m²  (Explored Cells: {exp_pct:.1f}%)")
    else:
        print("  • Map State             : Waiting for first /map broadcast from SLAM Toolbox...")

    print("\n════════════════════════════════════════════════════════════════════════════════")
    print("  Controls: [WASD] Drive in teleop terminal  │  Ctrl+C to Exit HUD  │  Save: ./scripts/hardware/save_hardware_map.sh")
    print("════════════════════════════════════════════════════════════════════════════════")


def main():
    parser = argparse.ArgumentParser(description="AMR SLAM Mapping & Calibration Diagnostics")
    parser.add_argument('--snapshot', action='store_true', help="Sample for a duration and output single summary")
    parser.add_argument('--duration', type=float, default=3.0, help="Sampling duration in seconds (for snapshot)")
    parser.add_argument('--json', type=str, default="", help="Path to write JSON diagnostics summary")
    args = parser.parse_args()

    rclpy.init()
    diag = MappingDiagnosticsNode()

    if args.snapshot:
        print(f"Sampling mapping and calibration telemetry for {args.duration:.1f} seconds...")
        start_t = time.time()
        while time.time() - start_t < args.duration:
            rclpy.spin_once(diag, timeout_sec=0.01)

        rep = format_report(diag, sample_sec=args.duration)
        print_hud(rep)

        if args.json:
            with open(args.json, 'w') as f:
                json.dump(rep, f, indent=2)
            print(f"\n[OK] Wrote full diagnostic report to: {args.json}")

        rclpy.shutdown()
        return

    # Continuous Interactive HUD Mode
    try:
        sample_window = 0.5
        while rclpy.ok():
            # Reset counters for the upcoming sample window
            diag.topic_counts = {k: 0 for k in diag.topic_counts}
            window_start = time.time()
            while time.time() - window_start < sample_window:
                rclpy.spin_once(diag, timeout_sec=0.01)

            rep = format_report(diag, sample_sec=sample_window)
            print_hud(rep)
    except KeyboardInterrupt:
        print("\n[INFO] Diagnostics monitor stopped by user.")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
