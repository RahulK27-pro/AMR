#!/usr/bin/env python3
"""
AMR Isolated LiDAR Test & Data Recorder for Raspberry Pi
=========================================================
Tests Gazebo simulation and 2D LiDAR (/scan) topic in isolation:
  - Spawns only Gazebo (headless Mesa software rendering), robot description,
    and ros_gz_bridge.
  - No Nav2, AMCL, Costmaps, SLAM, EKF, or Dashboard nodes.
  - Monitors `/scan` and `/clock` topics.
  - Computes update rate (Hz), RTF, ray validity, and cardinal obstacle distances.
  - Records detailed logs to a text report and JSON data file.
"""

import os
import sys
import time
import math
import json
import signal
import argparse
import subprocess
import threading
from datetime import datetime

# Check ROS 2 environment
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import LaserScan
    from rosgraph_msgs.msg import Clock
except ImportError as e:
    print(f"[ERROR] ROS 2 Python libraries not found ({e}).")
    print("Please run: source /opt/ros/jazzy/setup.bash && source ~/AMR/install/setup.bash")
    sys.exit(1)


class LidarScanRecorder(Node):
    def __init__(self, record_duration=0, report_file="lidar_pi_test_report.txt", json_file="lidar_pi_test_data.json"):
        super().__init__('amr_lidar_test_recorder')
        self.record_duration = record_duration
        self.report_file = report_file
        self.json_file = json_file

        self.start_wall_time = time.time()
        self.first_scan_time = None
        self.last_scan_time = None
        self.scan_count = 0
        self.clock_count = 0
        self.sim_time = 0.0
        self.initial_sim_time = None

        self.last_scan_msg = None
        self.sample_scans = []  # Store up to 5 full scans for JSON export
        self.lock = threading.RLock()

        # Rolling window for Hz calculation
        self.timestamps = []

        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            sensor_qos
        )
        self.clock_sub = self.create_subscription(
            Clock,
            '/clock',
            self.clock_callback,
            10
        )

    def clock_callback(self, msg: Clock):
        with self.lock:
            self.clock_count += 1
            sec = msg.clock.sec + msg.clock.nanosec * 1e-9
            self.sim_time = sec
            if self.initial_sim_time is None:
                self.initial_sim_time = sec

    def scan_callback(self, msg: LaserScan):
        now = time.time()
        with self.lock:
            self.scan_count += 1
            if self.first_scan_time is None:
                self.first_scan_time = now
            self.last_scan_time = now
            self.last_scan_msg = msg

            self.timestamps.append(now)
            cutoff = now - 3.0
            self.timestamps = [t for t in self.timestamps if t >= cutoff]

            # Save a few full scans for detailed analysis (first, middle, etc.)
            if len(self.sample_scans) < 5 or self.scan_count % 50 == 0:
                scan_dict = {
                    "scan_id": self.scan_count,
                    "timestamp": now,
                    "header_stamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                    "frame_id": msg.header.frame_id,
                    "angle_min": msg.angle_min,
                    "angle_max": msg.angle_max,
                    "angle_increment": msg.angle_increment,
                    "range_min": msg.range_min,
                    "range_max": msg.range_max,
                    "ranges_count": len(msg.ranges),
                    "ranges": [round(float(r), 4) if not math.isinf(r) and not math.isnan(r) else ("inf" if math.isinf(r) else "nan") for r in msg.ranges]
                }
                if len(self.sample_scans) >= 10:
                    self.sample_scans.pop(1)  # keep first, remove second
                self.sample_scans.append(scan_dict)

    def get_hz(self):
        with self.lock:
            if len(self.timestamps) < 2:
                return 0.0
            dt = self.timestamps[-1] - self.timestamps[0]
            if dt <= 0.001:
                return 0.0
            return (len(self.timestamps) - 1) / dt

    def get_avg_hz(self):
        with self.lock:
            if self.scan_count < 2 or not self.first_scan_time or not self.last_scan_time:
                return 0.0
            dt = self.last_scan_time - self.first_scan_time
            if dt <= 0.001:
                return 0.0
            return (self.scan_count - 1) / dt

    def analyze_scan(self):
        with self.lock:
            msg = self.last_scan_msg
            if msg is None or len(msg.ranges) == 0:
                return None

            ranges = list(msg.ranges)
            total_rays = len(ranges)
            rmin = msg.range_min
            rmax = msg.range_max
            amin = msg.angle_min
            ainc = msg.angle_increment

            valid_rays = []
            inf_count = 0
            nan_count = 0
            below_min_count = 0
            above_max_count = 0

            for i, r in enumerate(ranges):
                if math.isnan(r):
                    nan_count += 1
                elif math.isinf(r):
                    inf_count += 1
                elif r < rmin:
                    below_min_count += 1
                elif r > rmax:
                    above_max_count += 1
                else:
                    angle = amin + i * ainc
                    valid_rays.append((angle, r))

            # Helper function to get cone average around a given target angle in degrees
            def get_cardinal_dist(target_deg, cone_deg=6.0):
                target_rad = math.radians(target_deg)
                cone_rad = math.radians(cone_deg)
                matches = []
                for i, r in enumerate(ranges):
                    angle = amin + i * ainc
                    # Normalize angle difference to [-pi, pi]
                    diff = math.atan2(math.sin(angle - target_rad), math.cos(angle - target_rad))
                    if abs(diff) <= cone_rad:
                        if not math.isnan(r) and not math.isinf(r) and rmin <= r <= rmax:
                            matches.append(r)
                if matches:
                    return sum(matches) / len(matches)
                return None

            front_dist = get_cardinal_dist(0.0)
            left_dist = get_cardinal_dist(90.0)
            right_dist = get_cardinal_dist(-90.0)
            rear_dist = get_cardinal_dist(180.0)

            min_valid = min(valid_rays, key=lambda x: x[1]) if valid_rays else None
            max_valid = max(valid_rays, key=lambda x: x[1]) if valid_rays else None

            return {
                "frame_id": msg.header.frame_id,
                "total_rays": total_rays,
                "valid_rays": len(valid_rays),
                "valid_pct": (len(valid_rays) / total_rays * 100.0) if total_rays > 0 else 0.0,
                "inf_count": inf_count,
                "nan_count": nan_count,
                "range_min_spec": rmin,
                "range_max_spec": rmax,
                "angle_min_deg": math.degrees(amin),
                "angle_max_deg": math.degrees(msg.angle_max),
                "angle_inc_deg": math.degrees(ainc),
                "front_dist": front_dist,
                "left_dist": left_dist,
                "right_dist": right_dist,
                "rear_dist": rear_dist,
                "closest": min_valid,  # (angle_rad, dist)
                "furthest": max_valid,
            }

    def save_reports(self, test_meta):
        with self.lock:
            analysis = self.analyze_scan()
            elapsed_wall = time.time() - self.start_wall_time
            sim_elapsed = (self.sim_time - self.initial_sim_time) if self.initial_sim_time is not None else 0.0
            avg_hz = self.get_avg_hz()

            # Determine verdict
            if self.scan_count == 0:
                verdict = "FAIL - NO SCAN MESSAGES RECEIVED (0.0 Hz)"
            elif analysis is None or analysis["valid_rays"] == 0:
                verdict = "WARNING - SCANS RECEIVED BUT 0 VALID RAYS (All Inf/NaN)"
            elif avg_hz < 5.0:
                verdict = f"DEGRADED - SCANS ACTIVE BUT LOW RATE ({avg_hz:.1f} Hz < 10 Hz target)"
            else:
                verdict = f"PASS - LIDAR FULLY FUNCTIONAL ({avg_hz:.1f} Hz, {analysis['valid_pct']:.1f}% valid rays)"

            # Build human-readable text report
            lines = [
                "=" * 78,
                "          AMR RASPBERRY PI ISOLATED LIDAR TEST REPORT",
                "=" * 78,
                f"Generated At       : {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
                f"Test Mode          : {test_meta.get('mode', 'Standalone')}",
                f"World File         : {test_meta.get('world', 'test1.world')}",
                f"ROS_DOMAIN_ID      : {os.environ.get('ROS_DOMAIN_ID', 'not set')}",
                f"RMW_IMPLEMENTATION : {os.environ.get('RMW_IMPLEMENTATION', 'default')}",
                f"MESA Software LibGL: {os.environ.get('LIBGL_ALWAYS_SOFTWARE', 'not set')}",
                "-" * 78,
                "EXECUTION TIMING & CLOCK:",
                f"  • Total Wall Time  : {elapsed_wall:.2f} s",
                f"  • Sim Elapsed Time : {sim_elapsed:.2f} s",
                f"  • Real-Time Factor : {(sim_elapsed / elapsed_wall):.2f}x" if elapsed_wall > 0 else "  • Real-Time Factor : N/A",
                f"  • Clock msgs count : {self.clock_count}",
                "-" * 78,
                "LIDAR SENSOR (/scan) PERFORMANCE:",
                f"  • Total Scans Recv : {self.scan_count}",
                f"  • Average Hz       : {avg_hz:.2f} Hz",
                f"  • Target Rate      : 10.0 Hz",
                f"  • Scan Frame ID    : {analysis['frame_id'] if analysis else 'None'}",
                "-" * 78,
            ]

            if analysis:
                lines.extend([
                    "SCAN SPECIFICATIONS & RAYS:",
                    f"  • Total Rays / Beam : {analysis['total_rays']} samples",
                    f"  • Angular Coverage  : {analysis['angle_min_deg']:.1f}° to {analysis['angle_max_deg']:.1f}° (step: {analysis['angle_inc_deg']:.2f}°)",
                    f"  • Spec Range Limits : {analysis['range_min_spec']:.2f} m to {analysis['range_max_spec']:.2f} m",
                    f"  • Valid Obstacle Rays: {analysis['valid_rays']} ({analysis['valid_pct']:.1f}%)",
                    f"  • Free/Inf Rays     : {analysis['inf_count']}",
                    f"  • NaN / Error Rays  : {analysis['nan_count']}",
                    "-" * 78,
                    "CARDINAL DISTANCE MEASUREMENTS (Obstacles Detected):",
                    f"  • FRONT  (  0°): {analysis['front_dist']:.3f} m" if analysis['front_dist'] else "  • FRONT  (  0°): Clear / > 12.0 m",
                    f"  • LEFT   (+90°): {analysis['left_dist']:.3f} m" if analysis['left_dist'] else "  • LEFT   (+90°): Clear / > 12.0 m",
                    f"  • RIGHT  (-90°): {analysis['right_dist']:.3f} m" if analysis['right_dist'] else "  • RIGHT  (-90°): Clear / > 12.0 m",
                    f"  • REAR  (±180°): {analysis['rear_dist']:.3f} m" if analysis['rear_dist'] else "  • REAR  (±180°): Clear / > 12.0 m",
                ])
                if analysis['closest']:
                    ang_deg = math.degrees(analysis['closest'][0])
                    lines.append(f"  • Closest Obstacle  : {analysis['closest'][1]:.3f} m at {ang_deg:+.1f}°")
                if analysis['furthest']:
                    ang_deg = math.degrees(analysis['furthest'][0])
                    lines.append(f"  • Furthest Obstacle : {analysis['furthest'][1]:.3f} m at {ang_deg:+.1f}°")
            else:
                lines.append("  [NO VALID SCAN DATA AVAILABLE]")

            lines.extend([
                "=" * 78,
                f"FINAL VERDICT: {verdict}",
                "=" * 78,
            ])

            report_str = "\n".join(lines)
            with open(self.report_file, 'w') as f:
                f.write(report_str)

            # JSON export
            json_data = {
                "test_metadata": test_meta,
                "timestamp": datetime.now().isoformat(),
                "verdict": verdict,
                "wall_time_sec": elapsed_wall,
                "sim_time_sec": sim_elapsed,
                "total_scans": self.scan_count,
                "avg_hz": avg_hz,
                "analysis": analysis,
                "sample_scans": self.sample_scans
            }
            with open(self.json_file, 'w') as f:
                json.dump(json_data, f, indent=2)

            return verdict, report_str


def find_file(filename, search_dirs):
    for d in search_dirs:
        cand = os.path.join(d, filename)
        if os.path.exists(cand):
            return cand
    return None


def run_isolated_test(args):
    ws_dir = os.path.abspath(os.path.dirname(__file__))
    search_dirs = [
        ws_dir,
        os.path.join(ws_dir, "src", "agv_description"),
        os.path.join(ws_dir, "install", "agv_description", "share", "agv_description"),
        os.path.expanduser("~/AMR/install/agv_description/share/agv_description"),
        os.path.expanduser("~/AMR/src/agv_description"),
    ]

    # Resolve world file
    world_filename = args.world if args.world.endswith(".world") else args.world + ".world"
    world_path = None
    if os.path.isabs(world_filename) and os.path.exists(world_filename):
        world_path = world_filename
    else:
        for base in search_dirs:
            cand = os.path.join(base, "worlds", world_filename)
            if os.path.exists(cand):
                world_path = cand
                break
            cand = os.path.join(base, world_filename)
            if os.path.exists(cand):
                world_path = cand
                break

    if not world_path:
        print(f"[ERROR] Could not find world file: {world_filename}")
        sys.exit(1)

    # Resolve URDF file
    urdf_path = None
    for base in search_dirs:
        cand = os.path.join(base, "urdf", "warehouse_agv.urdf")
        if os.path.exists(cand):
            urdf_path = cand
            break
    if not urdf_path:
        print("[ERROR] Could not find warehouse_agv.urdf")
        sys.exit(1)

    # Resolve bridge.yaml
    bridge_path = None
    for base in search_dirs:
        cand = os.path.join(base, "config", "bridge.yaml")
        if os.path.exists(cand):
            bridge_path = cand
            break
    if not bridge_path:
        print("[ERROR] Could not find bridge.yaml")
        sys.exit(1)

    with open(urdf_path, "r") as f:
        robot_desc = f.read()

    world_stem = os.path.splitext(os.path.basename(world_path))[0]

    # Prepare environment for Gazebo software rendering
    env = os.environ.copy()
    env["LIBGL_ALWAYS_SOFTWARE"] = "1"
    env["MESA_GL_VERSION_OVERRIDE"] = "3.3"
    if "ROS_DOMAIN_ID" not in env:
        env["ROS_DOMAIN_ID"] = "7"
    if "RMW_IMPLEMENTATION" not in env:
        env["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"

    procs = []

    def cleanup(signum=None, frame=None):
        print("\n[INFO] Terminating isolated test processes cleanly...")
        for p in procs:
            if p.poll() is None:
                try:
                    p.terminate()
                except Exception:
                    pass
        time.sleep(1.0)
        for p in procs:
            if p.poll() is None:
                try:
                    p.kill()
                except Exception:
                    pass
        # Extra safeguard: kill matching processes
        subprocess.run(["pkill", "-f", "ros_gz_bridge"], stderr=subprocess.DEVNULL)
        subprocess.run(["pkill", "-f", "gz sim"], stderr=subprocess.DEVNULL)

    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    print("=" * 78)
    print("      AMR ISOLATED LIDAR VALIDATION & DATA RECORDER (PI)")
    print("=" * 78)
    print(f"World:  {world_path}")
    print(f"URDF:   {urdf_path}")
    print(f"Bridge: {bridge_path}")
    print(f"Mesa:   LIBGL_ALWAYS_SOFTWARE=1, MESA_GL_VERSION_OVERRIDE=3.3")
    print("-" * 78)

    if not args.monitor_only:
        print("[1/4] Starting Gazebo Harmonic (headless, software rendering)...")
        gz_cmd = ["gz", "sim", "-s", "--headless-rendering", "-r", "-v", "2", world_path]
        gz_proc = subprocess.Popen(gz_cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        procs.append(gz_proc)
        time.sleep(2.5)

        print("[2/4] Starting Robot State Publisher...")
        rsp_cmd = [
            "ros2", "run", "robot_state_publisher", "robot_state_publisher",
            "--ros-args", "-p", "use_sim_time:=true", "-p", f"robot_description:={robot_desc}"
        ]
        rsp_proc = subprocess.Popen(rsp_cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        procs.append(rsp_proc)
        time.sleep(1.0)

        print(f"[3/4] Spawning warehouse_agv into world '{world_stem}' from {os.path.basename(urdf_path)}...")
        spawn_cmd = [
            "ros2", "run", "ros_gz_sim", "create",
            "-name", "warehouse_agv",
            "-file", urdf_path,
            "-world", world_stem,
            "-z", "0.05"
        ]
        spawn_res = subprocess.run(spawn_cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        if spawn_res.returncode != 0:
            print(f"[WARN] Spawner output: {spawn_res.stderr.strip() or spawn_res.stdout.strip()}")
        else:
            print("[3/4] Robot spawned successfully into Gazebo.")
        time.sleep(1.0)

        print("[4/4] Starting ros_gz_bridge for /clock and /scan...")
        bridge_cmd = [
            "ros2", "run", "ros_gz_bridge", "parameter_bridge",
            "--ros-args", "-p", f"config_file:={bridge_path}"
        ]
        bridge_proc = subprocess.Popen(bridge_cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        procs.append(bridge_proc)
        time.sleep(2.0)

    # Start ROS 2 node
    print("-" * 78)
    print("Connecting ROS 2 subscriber to /scan ...")
    rclpy.init(args=None)
    recorder = LidarScanRecorder(
        record_duration=args.duration,
        report_file=args.output,
        json_file=args.json
    )

    # Spinner thread
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(recorder)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print("Listening to /scan topic (press Ctrl+C to stop)...")
    print("=" * 78)

    start_time = time.time()
    try:
        while rclpy.ok():
            time.sleep(1.0)
            elapsed = time.time() - start_time
            hz = recorder.get_hz()
            avg_hz = recorder.get_avg_hz()
            analysis = recorder.analyze_scan()

            if analysis:
                status_str = f"ACTIVE ({hz:.1f} Hz | avg: {avg_hz:.1f} Hz)"
                valid_str = f"{analysis['valid_rays']} / {analysis['total_rays']} ({analysis['valid_pct']:.1f}%)"
                f_str = f"{analysis['front_dist']:.2f} m" if analysis['front_dist'] else ">12m"
                l_str = f"{analysis['left_dist']:.2f} m" if analysis['left_dist'] else ">12m"
                r_str = f"{analysis['right_dist']:.2f} m" if analysis['right_dist'] else ">12m"
                rear_str = f"{analysis['rear_dist']:.2f} m" if analysis['rear_dist'] else ">12m"

                closest_str = f"{analysis['closest'][1]:.2f}m @ {math.degrees(analysis['closest'][0]):+.0f}°" if analysis['closest'] else "None"

                print(f"[{elapsed:4.0f}s] /scan: {status_str} | Scans: {recorder.scan_count} | Rays: {valid_str}")
                print(f"        FRONT: {f_str:>5} | LEFT: {l_str:>5} | RIGHT: {r_str:>5} | REAR: {rear_str:>5} | Nearest: {closest_str}")
            else:
                print(f"[{elapsed:4.0f}s] /scan: WAITING FOR DATA (0.0 Hz) | Sim Clock: {recorder.sim_time:.1f}s (msgs: {recorder.clock_count})")

            if args.duration > 0 and elapsed >= args.duration:
                print(f"\n[INFO] Reached requested test duration ({args.duration}s). Finishing...")
                break

    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user (Ctrl+C).")
    finally:
        test_meta = {
            "mode": "Monitor-Only" if args.monitor_only else "Isolated Standalone",
            "world": world_path,
            "duration_requested": args.duration,
            "urdf": urdf_path,
        }
        verdict, report_str = recorder.save_reports(test_meta)
        print("\n" + report_str)
        print(f"\n[SAVED] Human-readable report : {os.path.abspath(args.output)}")
        print(f"[SAVED] Raw JSON dataset     : {os.path.abspath(args.json)}")

        try:
            rclpy.shutdown()
        except Exception:
            pass
        cleanup()


def main():
    parser = argparse.ArgumentParser(description="AMR Raspberry Pi Isolated LiDAR Test & Recorder")
    parser.add_argument("--world", default="test1.world", help="World file name or path (default: test1.world)")
    parser.add_argument("--duration", type=int, default=0, help="Test duration in seconds (0 = run until Ctrl+C)")
    parser.add_argument("--output", default="lidar_pi_test_report.txt", help="Output text report filename")
    parser.add_argument("--json", default="lidar_pi_test_data.json", help="Output JSON dataset filename")
    parser.add_argument("--monitor-only", action="store_true", help="Do not launch Gazebo; only record /scan topic")
    args = parser.parse_args()

    run_isolated_test(args)


if __name__ == "__main__":
    main()
