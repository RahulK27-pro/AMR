"""
AMR Control Bridge Server  (v2.0 — enhanced)
============================================
FastAPI service that sits between the Flutter / web dashboard and ROS 2.
Runs an rclpy node in a background thread; FastAPI runs in the main thread.

REST endpoints:
  GET  /api/status          → one-shot telemetry snapshot
  GET  /api/map             → static warehouse map PNG (base64 JPEG, robot overlay)
  POST /api/cmd_vel         → {linear, angular} → publishes /cmd_vel
  POST /api/goal            → {x, y}            → publishes /goal_pose (PoseStamped)
  POST /api/stop            → zero-velocity + /agv_estop True
  POST /api/estop/clear     → /agv_estop False (resume)
  POST /api/initial_pose    → {x, y, theta}     → /initialpose (AMCL init)
  POST /api/goal_sequence   → {nodes: [...]}     → /goal_sequence (JSON array)

WebSocket:
  /ws/telemetry  → JSON snapshot pushed at 5 Hz

Run (after sourcing ROS 2 workspace):
  source /opt/ros/jazzy/setup.bash
  source ~/AMR/AMR-main/install/setup.bash
  pip install -r requirements.txt
  python3 bridge_server.py
"""

import asyncio
import base64
import io
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import List, Optional, Tuple, Dict, Any

from PIL import Image, ImageDraw
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Odometry, Path as NavPath, OccupancyGrid
from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import String, Bool

# Ensure local install packages (like explore_lite_msgs) are on sys.path
_WS_BOOTSTRAP = Path(os.environ.get("AMR_WS", Path.home() / "AMR" / "AMR-main"))
_py_ver = f"python{sys.version_info.major}.{sys.version_info.minor}"
_explore_site = _WS_BOOTSTRAP / "install" / "explore_lite_msgs" / "lib" / _py_ver / "site-packages"
if _explore_site.exists() and str(_explore_site) not in sys.path:
    sys.path.insert(0, str(_explore_site))

# Exploration messages (explore_lite)
try:
    from explore_lite_msgs.msg import ExploreStatus
    _EXPLORE_MSGS_AVAILABLE = True
except ImportError:
    _EXPLORE_MSGS_AVAILABLE = False

try:
    from visualization_msgs.msg import MarkerArray, Marker
    _MARKER_MSGS_AVAILABLE = True
except ImportError:
    _MARKER_MSGS_AVAILABLE = False

# TF2 for map-frame pose lookup
try:
    import tf2_ros
    _TF2_AVAILABLE = True
except ImportError:
    _TF2_AVAILABLE = False

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from fastapi.responses import JSONResponse, FileResponse
from pydantic import BaseModel
import uvicorn

# ---------------------------------------------------------------------------
# Config — map image path (static pre-computed PNG)
# ---------------------------------------------------------------------------
_WORKSPACE_ROOT = Path(os.environ.get("AMR_WS", Path.home() / "AMR" / "AMR-main"))
# Use the clean architectural floorplan (no graph clutter burned in)
_MAP_PNG_PATH       = _WORKSPACE_ROOT / "src" / "agv_description" / "maps" / "clean_warehouse_map.png"
_MAP_PNG_GRAPH_PATH = _WORKSPACE_ROOT / "src" / "agv_description" / "maps" / "graph_visualization.png"
_GRAPH_JSON_PATH    = _WORKSPACE_ROOT / "src" / "agv_description" / "maps" / "warehouse_graph.json"

# Import graph extractor engine
_MAPS_DIR = _WORKSPACE_ROOT / "src" / "agv_description" / "maps"
if str(_MAPS_DIR) not in sys.path:
    sys.path.insert(0, str(_MAPS_DIR))
try:
    import graph_extractor
except Exception as e:
    print(f"[bridge] Warning: Failed to import graph_extractor: {e}")
    graph_extractor = None

# Calibrated Map metadata from warehouse_map.yaml (resolution, origin)
# resolution: metres per pixel
MAP_RESOLUTION = 0.05          # 0.05 m/px
MAP_ORIGIN_X   = -7.397        # map frame X at pixel (0,0)
MAP_ORIGIN_Y   = -6.596        # map frame Y at pixel (0,0)
MAP_WIDTH_PX   = 329           # cols in the PNG
MAP_HEIGHT_PX  = 275           # rows in the PNG


def world_to_pixel(wx: float, wy: float):
    """Convert world (x,y) metres → (col, row) pixel in the PNG."""
    col = int((wx - MAP_ORIGIN_X) / MAP_RESOLUTION)
    row = MAP_HEIGHT_PX - int((wy - MAP_ORIGIN_Y) / MAP_RESOLUTION)
    return col, row


# ---------------------------------------------------------------------------
# Mapping Process & State Manager (Phase 1)
# ---------------------------------------------------------------------------

class MappingManager:
    """Manages the lifecycle of mapping sessions (SLAM Toolbox + explore_lite) and map saving."""
    def __init__(self, workspace_root: Path):
        self.ws_root = workspace_root
        self.state = 'READY'  # 'READY' | 'MAPPING' | 'STOPPED' | 'SAVED'
        self.map_name = 'warehouse_01'
        self.start_time: Optional[float] = None
        self.stop_time: Optional[float] = None
        self.proc: Optional[subprocess.Popen] = None
        self.explore_proc: Optional[subprocess.Popen] = None
        self.target_world = 'test1.world'
        self.run_explore = False
        self.is_saved: bool = False
        self.saved_map_name: Optional[str] = None
        self.saved_paths: Dict[str, str] = {}
        self.verification_report: Optional[dict] = None
        self.last_extracted_graph: Optional[dict] = None
        self.lock = threading.Lock()

    def start(self, map_name: str = 'warehouse_01', world: str = 'test1.world', run_explore: bool = False):
        with self.lock:
            if map_name and map_name.strip():
                self.map_name = map_name.strip()
            self.state = 'MAPPING'
            self.start_time = time.time()
            self.stop_time = None
            self.is_saved = False
            self.saved_map_name = None
            self.saved_paths = {}
            self.verification_report = None
            self.target_world = world if world.endswith('.world') else f"{world}.world"
            self.run_explore = run_explore

            # Check if gz sim is already running on the machine
            gz_already_running = False
            try:
                res = subprocess.run(['pgrep', '-f', 'gz sim'], stdout=subprocess.PIPE)
                if res.returncode == 0 and res.stdout.strip():
                    gz_already_running = True
            except Exception:
                pass

            run_sim_flag = 'false' if gz_already_running else 'true'
            run_explore_flag = 'true' if run_explore else 'false'

            # Spawn mapping_session.launch.py in background if not already started
            if self.proc is None or self.proc.poll() is not None:
                cmd = (
                    f"source /opt/ros/jazzy/setup.bash && "
                    f"source {self.ws_root}/install/setup.bash && "
                    f"ros2 launch agv_description mapping_session.launch.py "
                    f"world:={self.target_world} run_sim:={run_sim_flag} run_explore:={run_explore_flag} use_rviz:=false"
                )
                try:
                    self.proc = subprocess.Popen(
                        cmd,
                        shell=True,
                        executable='/bin/bash',
                        cwd=str(self.ws_root),
                        preexec_fn=os.setsid,
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL
                    )
                except Exception as e:
                    print(f"[MappingManager] Failed to launch mapping_session: {e}")
            return True

    def start_auto_explore(self, ros_node=None):
        """Dynamically activate explore_lite if not already running."""
        with self.lock:
            self.run_explore = True
            is_running = False
            try:
                res = subprocess.run(['pgrep', '-f', 'explore_node'], stdout=subprocess.PIPE)
                if res.returncode == 0 and res.stdout.strip():
                    is_running = True
            except Exception:
                pass

            if is_running:
                if ros_node:
                    ros_node.publish_explore_resume(True)
                return True

            if self.state == 'MAPPING':
                cmd = (
                    f"source /opt/ros/jazzy/setup.bash && "
                    f"source {self.ws_root}/install/setup.bash && "
                    f"ros2 run explore_lite explore --ros-args -r __node:=explore_node "
                    f"--params-file {self.ws_root}/src/agv_description/config/nav2_params_explore.yaml "
                    f"-p use_sim_time:=true"
                )
                try:
                    self.explore_proc = subprocess.Popen(
                        cmd,
                        shell=True,
                        executable='/bin/bash',
                        cwd=str(self.ws_root),
                        preexec_fn=os.setsid,
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL
                    )
                    print("[MappingManager] Spawning explore_node dynamically...")
                except Exception as e:
                    print(f"[MappingManager] Failed to spawn explore_node: {e}")
            return True

    def pause_auto_explore(self, ros_node=None):
        """Pause exploration without killing the node, allowing manual driving."""
        with self.lock:
            self.run_explore = False
            if ros_node:
                ros_node.publish_explore_resume(False)
            return True

    def resume_auto_explore(self, ros_node=None):
        """Resume exploration."""
        with self.lock:
            self.run_explore = True
            if ros_node:
                ros_node.publish_explore_resume(True)
            return True

    def stop(self):
        with self.lock:
            self.state = 'STOPPED'
            self.stop_time = time.time()
            self.run_explore = False
            if self.explore_proc and self.explore_proc.poll() is None:
                try:
                    os.killpg(os.getpgid(self.explore_proc.pid), signal.SIGTERM)
                except Exception:
                    pass
                self.explore_proc = None
            if self.proc and self.proc.poll() is None:
                try:
                    os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
                    try:
                        self.proc.wait(timeout=3.0)
                    except subprocess.TimeoutExpired:
                        os.killpg(os.getpgid(self.proc.pid), signal.SIGTERM)
                except Exception as e:
                    print(f"[MappingManager] Stop proc error: {e}")
                self.proc = None
            try:
                subprocess.run(['pkill', '-f', 'explore_node'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except Exception:
                pass
            return True

    def redo(self):
        with self.lock:
            self.run_explore = False
            if self.explore_proc and self.explore_proc.poll() is None:
                try:
                    os.killpg(os.getpgid(self.explore_proc.pid), signal.SIGTERM)
                except Exception:
                    pass
                self.explore_proc = None
            if self.proc and self.proc.poll() is None:
                try:
                    os.killpg(os.getpgid(self.proc.pid), signal.SIGTERM)
                except Exception:
                    pass
                self.proc = None
            try:
                subprocess.run(['pkill', '-f', 'explore_node'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except Exception:
                pass
            self.state = 'READY'
            self.start_time = None
            self.stop_time = None
            self.is_saved = False
            self.saved_map_name = None
            self.saved_paths = {}
            self.verification_report = None
            self.last_extracted_graph = None
            return True

    def save_map(self, map_name: str, ros_node=None) -> dict:
        """Saves active SLAM map into src/agv_description/maps/<map_name>.yaml + .pgm + .png
        and performs strict on-disk verification checks (Phase 2).
        """
        with self.lock:
            if not ros_node or not ros_node._has_live_map or ros_node._last_raw_occupancy is None:
                return {
                    'ok': False,
                    'error': 'No active SLAM map data available to save. Run mapping before saving.'
                }

            import re
            raw_name = (map_name or self.map_name or 'warehouse_01').strip()
            clean_name = re.sub(r'[^a-zA-Z0-9_-]', '_', raw_name)
            if not clean_name:
                clean_name = 'warehouse_01'

            maps_dir = self.ws_root / "src" / "agv_description" / "maps"
            maps_dir.mkdir(parents=True, exist_ok=True)

            yaml_path = maps_dir / f"{clean_name}.yaml"
            pgm_path = maps_dir / f"{clean_name}.pgm"
            png_path = maps_dir / f"{clean_name}.png"

            w = ros_node._live_map_width
            h = ros_node._live_map_height
            res = float(ros_node._live_map_resolution)
            ox = float(ros_node._live_map_origin_x)
            oy = float(ros_node._live_map_origin_y)
            raw = ros_node._last_raw_occupancy

            # 1. Generate PGM data (ROS standard trinary values: free=254, occupied=0, unknown=205)
            # Row 0 in PGM image is top, but row 0 in OccupancyGrid is bottom, so flip vertically
            pgm_arr = np.full((h, w), 205, dtype=np.uint8)
            pgm_arr[raw == 0] = 254
            pgm_arr[raw > 50] = 0
            pgm_flipped = np.flipud(pgm_arr)

            pil_pgm = Image.fromarray(pgm_flipped, mode='L')
            pil_pgm.save(str(pgm_path))

            # 2. Write YAML specification (matching standard Nav2 / map_server)
            yaml_content = (
                f"image: {clean_name}.pgm\n"
                f"mode: trinary\n"
                f"resolution: {res:.4f}\n"
                f"origin: [{ox:.3f}, {oy:.3f}, 0]\n"
                f"negate: 0\n"
                f"occupied_thresh: 0.65\n"
                f"free_thresh: 0.196\n"
            )
            with open(yaml_path, 'w', encoding='utf-8') as f:
                f.write(yaml_content)

            # 3. Write Clean Dark Preview PNG for browser dashboard
            png_arr = np.zeros((h, w, 3), dtype=np.uint8)
            png_arr[raw == -1] = [26, 32, 46]     # unknown slate
            png_arr[raw == 0]  = [12, 16, 28]     # free space
            png_arr[raw > 50]  = [0, 212, 255]    # wall / occupied cyan
            png_flipped = np.flipud(png_arr)
            pil_png = Image.fromarray(png_flipped)
            pil_png.save(str(png_path))

            # 4. Optional background call to map_saver_cli if topic is active
            if ros_node and ros_node._has_live_map:
                try:
                    cmd = (
                        f"source /opt/ros/jazzy/setup.bash && "
                        f"source {self.ws_root}/install/setup.bash && "
                        f"ros2 run nav2_map_server map_saver_cli -f {maps_dir / clean_name} --ros-args -p use_sim_time:=true"
                    )
                    subprocess.run(cmd, shell=True, executable='/bin/bash', timeout=2.0, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                except Exception:
                    pass

            # 5. On-disk Verification
            yaml_ok = yaml_path.is_file() and yaml_path.stat().st_size > 0
            pgm_ok = pgm_path.is_file() and pgm_path.stat().st_size > 0
            png_ok = png_path.is_file() and png_path.stat().st_size > 0

            if not (yaml_ok and pgm_ok):
                return {
                    'ok': False,
                    'error': f"Failed to verify saved files on disk: yaml_ok={yaml_ok}, pgm_ok={pgm_ok}"
                }

            total_cells = w * h
            free_cells = int(np.sum(raw == 0))
            occupied_cells = int(np.sum(raw > 50))
            unknown_cells = int(np.sum(raw == -1))
            free_area = round(free_cells * (res ** 2), 2)
            occupied_area = round(occupied_cells * (res ** 2), 2)
            coverage_pct = round(((free_cells + occupied_cells) / max(1, total_cells)) * 100, 1)

            self.is_saved = True
            self.saved_map_name = clean_name
            self.saved_paths = {
                'yaml': str(yaml_path.relative_to(self.ws_root)),
                'pgm': str(pgm_path.relative_to(self.ws_root)),
                'png': str(png_path.relative_to(self.ws_root)),
            }
            self.state = 'SAVED'

            report = {
                'verified': True,
                'map_name': clean_name,
                'status': 'VERIFIED & SAVED',
                'files': {
                    'yaml': self.saved_paths['yaml'],
                    'pgm': self.saved_paths['pgm'],
                    'png': self.saved_paths['png'],
                    'yaml_size_bytes': yaml_path.stat().st_size,
                    'pgm_size_bytes': pgm_path.stat().st_size,
                },
                'metrics': {
                    'width_px': w,
                    'height_px': h,
                    'resolution': res,
                    'origin': [ox, oy, 0.0],
                    'real_width_m': round(w * res, 2),
                    'real_height_m': round(h * res, 2),
                    'free_area_sqm': free_area,
                    'occupied_area_sqm': occupied_area,
                    'coverage_pct': coverage_pct,
                    'free_cells': free_cells,
                    'occupied_cells': occupied_cells,
                    'unknown_cells': unknown_cells,
                    'quality_grade': 'EXCELLENT' if free_cells > 200 and occupied_cells > 30 else 'GOOD',
                }
            }
            self.verification_report = report
            return {'ok': True, 'report': report}

    def get_status(self, has_map_data: bool = False, map_info: Optional[dict] = None, ros_node=None) -> dict:
        with self.lock:
            elapsed = 0.0
            if self.state == 'MAPPING' and self.start_time:
                elapsed = time.time() - self.start_time
            elif (self.state in ('STOPPED', 'SAVED')) and self.start_time and self.stop_time:
                elapsed = self.stop_time - self.start_time

            mins = int(elapsed // 60)
            secs = int(elapsed % 60)
            elapsed_str = f"{mins:02d}:{secs:02d}"

            if has_map_data and self.state == 'MAPPING':
                map_status_text = "Receiving map data... (SLAM Toolbox active)"
            elif self.state == 'MAPPING':
                map_status_text = "Mapping in progress... Initializing SLAM Toolbox..."
            elif self.state == 'SAVED':
                map_status_text = f"Map '{self.saved_map_name or self.map_name}' verified and saved to disk."
            elif self.state == 'STOPPED':
                map_status_text = "Mapping stopped. Inspect map and verify before saving."
            else:
                map_status_text = "Ready to start mapping"

            exp_info = {
                'active': getattr(self, 'run_explore', False),
                'raw_status': ros_node.explore_raw_status if ros_node else 'idle',
                'status_label': ros_node.explore_status_label if ros_node else 'Not started',
                'is_paused': ros_node.is_explore_paused if ros_node else False,
                'is_exploring': ros_node.is_exploring if ros_node else False,
                'frontiers_count': ros_node.frontiers_count if ros_node else 0,
                'frontiers': ros_node.frontiers if ros_node else [],
            }

            return {
                'state': self.state,
                'map_name': self.map_name,
                'world': getattr(self, 'target_world', 'test1.world'),
                'run_explore': getattr(self, 'run_explore', False),
                'elapsed_sec': round(elapsed, 1),
                'elapsed_str': elapsed_str,
                'has_map_data': has_map_data,
                'map_status_text': map_status_text,
                'map_info': map_info or {},
                'exploration': exp_info,
                'is_saved': getattr(self, 'is_saved', False),
                'saved_map_name': getattr(self, 'saved_map_name', None),
                'saved_paths': getattr(self, 'saved_paths', {}),
                'verification_report': getattr(self, 'verification_report', None),
                'last_extracted_graph': getattr(self, 'last_extracted_graph', None),
            }


mapping_mgr = MappingManager(_WORKSPACE_ROOT)


# ---------------------------------------------------------------------------
# ROS 2 bridge node
# ---------------------------------------------------------------------------

class AmrBridgeNode(Node):
    def __init__(self):
        super().__init__('amr_web_bridge')

        # --- Publishers ---
        self.cmd_pub          = self.create_publisher(Twist,                      '/cmd_vel',       10)
        self.goal_pub         = self.create_publisher(PoseStamped,                '/goal_pose',     10)
        self.estop_pub        = self.create_publisher(Bool,                       '/agv_estop',     10)
        self.init_pose_pub    = self.create_publisher(PoseWithCovarianceStamped,  '/initialpose',   10)
        self.seq_pub          = self.create_publisher(String,                     '/goal_sequence', 10)
        self.obstacle_cmd_pub = self.create_publisher(Twist,                      '/dynamic_obstacle/cmd_vel', 10)
        self.explore_resume_pub = self.create_publisher(Bool,                     '/explore/resume', 10)

        # --- Subscribers ---
        self.create_subscription(Odometry,      '/odometry/filtered', self._odom_cb,    10)
        self.create_subscription(LaserScan,     '/scan',              self._scan_cb,    10)
        self.create_subscription(String,        '/agv_state',         self._state_cb,   10)
        self.create_subscription(NavPath,       '/agv_dense_path',    self._path_cb,    10)
        self.create_subscription(String,        '/obstacle_alert',    self._alert_cb,   10)
        self.create_subscription(Imu,           '/imu/data',          self._imu_cb,     10)
        self.create_subscription(String,        '/mission_progress',  self._mission_cb, 10)
        # AMCL pose — authoritative map-frame localization (replaces raw odom for pose display)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, 10
        )
        # Live /map subscription from SLAM Toolbox
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, 10)

        # Exploration status & frontiers subscriptions
        if _EXPLORE_MSGS_AVAILABLE:
            self.create_subscription(
                ExploreStatus, '/explore/status', self._explore_status_cb, 10
            )
        if _MARKER_MSGS_AVAILABLE:
            self.create_subscription(
                MarkerArray, '/explore/frontiers', self._explore_frontiers_cb, 10
            )

        # TF2 buffer for map->base_link lookups
        self._amcl_pose_received = False
        self._tf_buffer   = None
        self._tf_listener = None
        if _TF2_AVAILABLE:
            try:
                self._tf_buffer   = tf2_ros.Buffer()
                self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
                # Timer: try TF lookup at 10 Hz when no AMCL yet
                self.create_timer(0.1, self._tf_pose_cb)
            except Exception as e:
                self.get_logger().warn(f'tf2_ros init failed: {e} — falling back to /odometry/filtered')
        else:
            self.get_logger().warn('tf2_ros not available — falling back to /odometry/filtered')

        # --- Telemetry state ---
        self.pose             = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.velocity         = {'linear': 0.0, 'angular': 0.0}
        self.min_obstacle_dist= float('inf')
        self.scan_ranges: List[float] = []
        self.scan_angle_min   = 0.0
        self.scan_angle_inc   = 0.0
        self.nav_state        = 'IDLE'
        self.path_waypoints: List[List[float]] = []
        self.obstacle_alert: Optional[str] = None
        self.imu              = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        self.mission          = {}
        self._cached_graph: Optional[dict] = None

        # --- Autonomous exploration state ---
        self.explore_raw_status: str = 'idle'
        self.explore_status_label: str = 'Not started'
        self.is_exploring: bool = False
        self.is_explore_paused: bool = False
        self.frontiers: List[dict] = []
        self.frontiers_count: int = 0

        # --- Live SLAM map state ---
        self._live_map_width = 0
        self._live_map_height = 0
        self._live_map_resolution = 0.05
        self._live_map_origin_x = 0.0
        self._live_map_origin_y = 0.0
        self._has_live_map = False
        self._last_map_time = 0.0
        self._map_msg_count = 0
        self._live_map_b64: Optional[str] = None
        self._clean_map_b64: Optional[str] = None
        self._last_raw_occupancy: Optional[np.ndarray] = None

    # -------- Subscriber callbacks --------

    def _odom_cb(self, msg):
        # Use odom only for velocity; pose comes from AMCL / TF (map frame)
        self.velocity['linear']  = msg.twist.twist.linear.x
        self.velocity['angular'] = msg.twist.twist.angular.z
        # Fallback: update pose from odom only if AMCL has never fired
        if not self._amcl_pose_received:
            self.pose['x']   = msg.pose.pose.position.x
            self.pose['y']   = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            self.pose['yaw'] = self._quat_to_yaw(q)

    def _amcl_cb(self, msg):
        """AMCL pose — map-frame, highest-confidence source for robot position."""
        self._amcl_pose_received = True
        self.pose['x']   = msg.pose.pose.position.x
        self.pose['y']   = msg.pose.pose.position.y
        self.pose['yaw'] = self._quat_to_yaw(msg.pose.pose.orientation)

    def _tf_pose_cb(self):
        """10 Hz TF lookup: map->base_link as secondary map-frame source."""
        if not _TF2_AVAILABLE or self._amcl_pose_received or self._tf_buffer is None:
            return
        try:
            tf = self._tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time()
            )
            t = tf.transform.translation
            self.pose['x']   = t.x
            self.pose['y']   = t.y
            self.pose['yaw'] = self._quat_to_yaw(tf.transform.rotation)
        except Exception:
            pass  # Quietly fall through to odom fallback

    def _scan_cb(self, msg):
        self.scan_ranges    = list(msg.ranges)
        self.scan_angle_min = msg.angle_min
        self.scan_angle_inc = msg.angle_increment
        valid = [r for r in msg.ranges if not math.isnan(r) and not math.isinf(r) and r > 0]
        self.min_obstacle_dist = min(valid) if valid else float('inf')

    def _state_cb(self, msg):
        self.nav_state = msg.data

    def _path_cb(self, msg):
        self.path_waypoints = [
            [ps.pose.position.x, ps.pose.position.y]
            for ps in msg.poses
        ]

    def _alert_cb(self, msg):
        self.obstacle_alert = msg.data if msg.data else None

    def _imu_cb(self, msg):
        q = msg.orientation
        # Roll, pitch, yaw from quaternion
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll  = math.atan2(sinr_cosp, cosr_cosp)
        sinp  = 2 * (q.w * q.y - q.z * q.x)
        pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
        yaw   = self._quat_to_yaw(q)
        self.imu = {'roll': round(roll, 4), 'pitch': round(pitch, 4), 'yaw': round(yaw, 4)}

    def _mission_cb(self, msg):
        try:
            self.mission = json.loads(msg.data)
        except Exception:
            pass

    def _explore_status_cb(self, msg):
        status_map = {
            'exploration_started': 'Initializing exploration...',
            'exploration_in_progress': 'Exploring frontier...',
            'exploration_paused': 'Exploration paused',
            'exploration_complete': 'All frontiers explored (Complete)',
            'returning_to_origin': 'Returning to starting pose...',
            'returned_to_origin': 'Returned to starting pose',
        }
        self.explore_raw_status = msg.status
        self.explore_status_label = status_map.get(msg.status, msg.status)
        self.is_exploring = msg.status in ['exploration_started', 'exploration_in_progress']
        self.is_explore_paused = (msg.status == 'exploration_paused')

    def _explore_frontiers_cb(self, msg):
        pts = []
        for m in msg.markers:
            m_type = getattr(m, 'type', None)
            if m_type == 2 or (_MARKER_MSGS_AVAILABLE and m_type == Marker.SPHERE):
                pts.append({
                    'id': int(m.id),
                    'x': round(float(m.pose.position.x), 2),
                    'y': round(float(m.pose.position.y), 2),
                    'cost': round(float(getattr(m.scale, 'x', 0.5)), 2)
                })
        self.frontiers = pts
        self.frontiers_count = len(pts)

    def _world_to_map_px(self, wx: float, wy: float) -> Optional[Tuple[int, int]]:
        if self._live_map_width <= 0 or self._live_map_height <= 0 or self._live_map_resolution <= 0:
            return None
        cx = int((wx - self._live_map_origin_x) / self._live_map_resolution)
        cy = int((wy - self._live_map_origin_y) / self._live_map_resolution)
        if 0 <= cx < self._live_map_width and 0 <= cy < self._live_map_height:
            img_x = cx
            img_y = (self._live_map_height - 1) - cy
            return (img_x, img_y)
        return None

    def _map_cb(self, msg: OccupancyGrid):
        """Callback for live /map occupancy grid (from SLAM Toolbox or map_server)."""
        try:
            self._live_map_width = msg.info.width
            self._live_map_height = msg.info.height
            self._live_map_resolution = msg.info.resolution
            self._live_map_origin_x = msg.info.origin.position.x
            self._live_map_origin_y = msg.info.origin.position.y
            self._has_live_map = True
            self._last_map_time = time.time()
            self._map_msg_count += 1

            w = msg.info.width
            h = msg.info.height
            if w > 0 and h > 0 and len(msg.data) == w * h:
                raw_data = np.array(msg.data, dtype=np.int8).reshape((h, w))
                self._last_raw_occupancy = raw_data.copy()

                # -1 = unknown (dark slate), 0 = free (deep blue/black), >50 = occupied (cyan)
                rgb = np.zeros((h, w, 3), dtype=np.uint8)
                rgb[raw_data == -1] = [26, 32, 46]     # unknown
                rgb[raw_data == 0]  = [12, 16, 28]     # free space
                rgb[raw_data > 50]  = [0, 212, 255]    # wall / occupied
                rgb = np.flipud(rgb)
                pil_img = Image.fromarray(rgb)

                # Save clean map image (without dynamic markers or robot dot)
                clean_buf = io.BytesIO()
                pil_img.save(clean_buf, format='PNG')
                self._clean_map_b64 = base64.b64encode(clean_buf.getvalue()).decode()

                # Overlay active frontiers and robot pose on the live map
                overlay_img = pil_img.copy()
                draw = ImageDraw.Draw(overlay_img)

                # Draw frontiers as bright gold diamond targets
                for f in self.frontiers:
                    f_px = self._world_to_map_px(f.get('x', 0), f.get('y', 0))
                    if f_px:
                        fx, fy = f_px
                        draw.rectangle([fx - 2, fy - 2, fx + 2, fy + 2], fill=(255, 215, 0), outline=(255, 255, 255))

                # Draw robot pose (green dot with heading pointer)
                r_px = self._world_to_map_px(self.pose['x'], self.pose['y'])
                if r_px:
                    rx, ry = r_px
                    rad = max(3, int(0.18 / self._live_map_resolution))
                    draw.ellipse([rx - rad, ry - rad, rx + rad, ry + rad], fill=(0, 255, 157), outline=(255, 255, 255))
                    yaw = self.pose.get('yaw', 0.0)
                    hx = rx + int((rad + 4) * math.cos(yaw))
                    hy = ry - int((rad + 4) * math.sin(yaw))
                    draw.line([rx, ry, hx, hy], fill=(255, 255, 255), width=2)

                buf = io.BytesIO()
                overlay_img.save(buf, format='PNG')
                self._live_map_b64 = base64.b64encode(buf.getvalue()).decode()
        except Exception as e:
            self.get_logger().error(f"Error processing /map: {e}")

    # -------- Publisher helpers --------

    def publish_cmd_vel(self, linear: float, angular: float):
        t = Twist()
        t.linear.x  = float(linear)
        t.angular.z = float(angular)
        self.cmd_pub.publish(t)

    def publish_goal(self, x: float, y: float, yaw: float = 0.0):
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.orientation = self._yaw_to_quat(yaw)
        self.goal_pub.publish(ps)

    def publish_explore_resume(self, resume: bool):
        b = Bool()
        b.data = bool(resume)
        self.explore_resume_pub.publish(b)
        if not resume:
            self.is_explore_paused = True
            self.is_exploring = False
            self.explore_raw_status = 'exploration_paused'
            self.explore_status_label = 'Exploration paused'
        else:
            self.is_explore_paused = False
            self.is_exploring = True
            self.explore_raw_status = 'exploration_in_progress'
            self.explore_status_label = 'Exploring frontier...'

    def publish_estop(self, active: bool):
        b = Bool()
        b.data = active
        self.estop_pub.publish(b)
        if active:
            self.publish_explore_resume(False)

    def publish_initial_pose(self, x: float, y: float, theta: float):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation = self._yaw_to_quat(theta)
        # Standard AMCL covariance
        cov = [0.0] * 36
        cov[0]  = 0.25   # x
        cov[7]  = 0.25   # y
        cov[35] = 0.0685 # yaw
        msg.pose.covariance = cov
        self.init_pose_pub.publish(msg)

    def publish_goal_sequence(self, nodes: list):
        s = String()
        s.data = json.dumps(nodes)
        self.seq_pub.publish(s)

    def publish_obstacle_cmd_vel(self, linear: float, angular: float):
        t = Twist()
        t.linear.x  = float(linear)
        t.angular.z = float(angular)
        self.obstacle_cmd_pub.publish(t)

    def set_active_graph(self, graph_dict: dict):
        self._cached_graph = {
            'nodes': graph_dict.get('nodes', []),
            'edges': graph_dict.get('edges', []),
            'total_nodes': graph_dict.get('total_nodes', len(graph_dict.get('nodes', []))),
            'total_edges': graph_dict.get('total_edges', len(graph_dict.get('edges', []))),
            'map_name': graph_dict.get('map_name', 'warehouse_map'),
            'metrics': graph_dict.get('metrics', {}),
            'vis_image_b64': graph_dict.get('vis_image_b64', None),
        }

    def get_graph_data(self) -> dict:
        if self._cached_graph is None and _GRAPH_JSON_PATH.exists():
            try:
                with open(_GRAPH_JSON_PATH, 'r', encoding='utf-8') as f:
                    raw = json.load(f)
                nodes = raw.get('nodes', [])
                edges = raw.get('edges', [])
                vis_b64 = None
                vis_candidates = [
                    _WORKSPACE_ROOT / "src" / "agv_description" / "maps" / "warehouse_01_graph_vis.png",
                    _WORKSPACE_ROOT / "src" / "agv_description" / "maps" / "graph_visualization.png",
                    _WORKSPACE_ROOT / "src" / "agv_description" / "maps" / "warehouse_map_graph_vis.png",
                ]
                for vc in vis_candidates:
                    if vc.exists():
                        try:
                            with open(vc, 'rb') as img_f:
                                vis_b64 = base64.b64encode(img_f.read()).decode('utf-8')
                            break
                        except Exception:
                            pass
                self._cached_graph = {
                    'nodes': nodes,
                    'edges': edges,
                    'total_nodes': len(nodes),
                    'total_edges': len(edges),
                    'vis_image_b64': vis_b64,
                }
            except Exception as e:
                self.get_logger().error(f"Failed to load graph: {e}")
                return {'nodes': [], 'edges': [], 'total_nodes': 0, 'total_edges': 0}
        return self._cached_graph or {'nodes': [], 'edges': [], 'total_nodes': 0, 'total_edges': 0}

    def get_live_map_info(self) -> dict:
        return {
            'width': self._live_map_width,
            'height': self._live_map_height,
            'resolution': self._live_map_resolution,
            'origin_x': round(self._live_map_origin_x, 3),
            'origin_y': round(self._live_map_origin_y, 3),
            'msg_count': self._map_msg_count,
            'last_update': round(self._last_map_time, 2)
        }

    def get_live_map_image_b64(self, clean: bool = False) -> Optional[str]:
        if clean and self._clean_map_b64:
            return self._clean_map_b64
        return self._live_map_b64

    def get_map_verification_data(self) -> dict:
        if not self._has_live_map or self._last_raw_occupancy is None:
            return {'has_map': False, 'error': 'No map data available yet'}
        w = self._live_map_width
        h = self._live_map_height
        res = float(self._live_map_resolution)
        raw = self._last_raw_occupancy

        total_cells = w * h
        free_cells = int(np.sum(raw == 0))
        occupied_cells = int(np.sum(raw > 50))
        unknown_cells = int(np.sum(raw == -1))

        free_area_sqm = round(free_cells * (res ** 2), 2)
        occupied_area_sqm = round(occupied_cells * (res ** 2), 2)
        total_area_sqm = round(total_cells * (res ** 2), 2)
        coverage_pct = round(((free_cells + occupied_cells) / max(1, total_cells)) * 100, 1)

        has_free_space = free_cells > 50
        has_obstacles = occupied_cells > 10
        is_valid = has_free_space and has_obstacles

        return {
            'has_map': True,
            'width': w,
            'height': h,
            'resolution': res,
            'origin_x': round(float(self._live_map_origin_x), 3),
            'origin_y': round(float(self._live_map_origin_y), 3),
            'real_width_m': round(w * res, 2),
            'real_height_m': round(h * res, 2),
            'total_cells': total_cells,
            'free_cells': free_cells,
            'occupied_cells': occupied_cells,
            'unknown_cells': unknown_cells,
            'free_area_sqm': free_area_sqm,
            'occupied_area_sqm': occupied_area_sqm,
            'total_area_sqm': total_area_sqm,
            'coverage_pct': coverage_pct,
            'has_free_space': has_free_space,
            'has_obstacles': has_obstacles,
            'is_valid': is_valid,
            'health_status': 'PASS' if is_valid else 'WARNING',
            'clean_image_b64': self._clean_map_b64,
            'overlay_image_b64': self._live_map_b64,
            'map_msg_count': self._map_msg_count
        }

    # -------- Snapshot --------

    def telemetry_snapshot(self) -> dict:
        obs_dist = self.min_obstacle_dist if not math.isinf(self.min_obstacle_dist) else None
        # Downsample scan to 72 points (every 5°) for the app
        scan_ds: List[float] = []
        if self.scan_ranges:
            step = max(1, len(self.scan_ranges) // 72)
            for i in range(0, len(self.scan_ranges), step):
                r = self.scan_ranges[i]
                scan_ds.append(round(r, 3) if not (math.isnan(r) or math.isinf(r)) else 0.0)
        return {
            'pose':            self.pose,
            'velocity':        self.velocity,
            'min_obstacle_dist': round(obs_dist, 3) if obs_dist is not None else None,
            'nav_state':       self.nav_state,
            'path':            self.path_waypoints,
            'obstacle_alert':  self.obstacle_alert,
            'imu':             self.imu,
            'mission':         self.mission,
            'scan':            scan_ds,
            'scan_angle_min':  round(self.scan_angle_min, 4),
            'scan_angle_inc':  round(self.scan_angle_inc, 4),
            'mapping':         mapping_mgr.get_status(self._has_live_map, self.get_live_map_info(), ros_node=self),
            'ts':              round(time.time(), 3),
        }

    # -------- Map PNG with robot overlay --------

    def get_map_image_b64(self) -> Optional[str]:
        """Return base64-encoded JPEG of static map PNG with robot dot overlaid."""
        try:
            from PIL import Image, ImageDraw
        except ImportError:
            # Return raw map without overlay if PIL not installed
            if _MAP_PNG_PATH.exists():
                with open(_MAP_PNG_PATH, 'rb') as f:
                    return base64.b64encode(f.read()).decode()
            return None

        if not _MAP_PNG_PATH.exists():
            return None

        img = Image.open(_MAP_PNG_PATH).convert('RGBA')
        draw = ImageDraw.Draw(img)

        # Robot position dot (red circle)
        cx, cy = world_to_pixel(self.pose['x'], self.pose['y'])
        r = 6
        draw.ellipse([cx - r, cy - r, cx + r, cy + r], fill=(255, 60, 60, 230))

        # Robot heading line
        length = 16
        yaw = self.pose['yaw']
        ex = int(cx + length * math.cos(yaw))
        ey = int(cy - length * math.sin(yaw))
        draw.line([cx, cy, ex, ey], fill=(255, 200, 0, 220), width=3)

        # Path overlay (magenta polyline)
        if len(self.path_waypoints) >= 2:
            pix_path = [world_to_pixel(p[0], p[1]) for p in self.path_waypoints]
            draw.line(pix_path, fill=(255, 80, 220, 200), width=2)

        buf = io.BytesIO()
        img.convert('RGB').save(buf, format='JPEG', quality=82)
        return base64.b64encode(buf.getvalue()).decode()

    # -------- Utils --------

    @staticmethod
    def _quat_to_yaw(q) -> float:
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _yaw_to_quat(yaw: float):
        q = Quaternion()
        q.w = math.cos(yaw / 2)
        q.z = math.sin(yaw / 2)
        return q


# ---------------------------------------------------------------------------
# FastAPI app
# ---------------------------------------------------------------------------

app  = FastAPI(title='AMR Control Bridge', version='2.0')
node: Optional[AmrBridgeNode] = None

app.add_middleware(
    CORSMiddleware,
    allow_origins=['*'],
    allow_credentials=True,
    allow_methods=['*'],
    allow_headers=['*'],
)

# -------- Pydantic models --------

class CmdVelRequest(BaseModel):
    linear:  float = 0.0
    angular: float = 0.0

class GoalRequest(BaseModel):
    x:   float
    y:   float
    yaw: float = 0.0

class InitialPoseRequest(BaseModel):
    x:     float
    y:     float
    theta: float = 0.0

class GoalSequenceRequest(BaseModel):
    nodes: list

class StartMappingRequest(BaseModel):
    map_name: str = 'warehouse_01'
    world: str = 'test1.world'
    run_explore: bool = False

class SaveMapRequest(BaseModel):
    map_name: Optional[str] = 'warehouse_01'

class ExtractGraphRequest(BaseModel):
    map_name: Optional[str] = None
    robot_radius: float = 0.11
    safety_margin: float = 0.10
    search_radius: float = 2.5
    step_corridor: float = 0.40
    step_medium: float = 0.50
    step_open: float = 0.80

# -------- REST endpoints --------

@app.get('/api/status')
async def get_status():
    return node.telemetry_snapshot() if node else {'error': 'ROS node not ready'}

@app.get('/api/map')
async def get_map():
    if node is None:
        return JSONResponse({'error': 'ROS node not ready'}, status_code=503)
    data = node.get_map_image_b64()
    if data is None:
        return JSONResponse({'error': 'Map image not found'}, status_code=404)
    return {'image': data, 'encoding': 'jpeg/base64'}

@app.post('/api/cmd_vel')
async def post_cmd_vel(req: CmdVelRequest):
    if node:
        node.publish_cmd_vel(req.linear, req.angular)
    return {'ok': True}

@app.post('/api/goal')
async def post_goal(req: GoalRequest):
    if node:
        node.publish_goal(req.x, req.y, req.yaw)
    return {'ok': True}

@app.post('/api/stop')
async def post_stop():
    if node:
        node.publish_cmd_vel(0.0, 0.0)
        node.publish_estop(True)
    return {'ok': True}

@app.post('/api/estop/clear')
async def post_estop_clear():
    if node:
        node.publish_estop(False)
    return {'ok': True}

@app.post('/api/initial_pose')
async def post_initial_pose(req: InitialPoseRequest):
    if node:
        node.publish_initial_pose(req.x, req.y, req.theta)
    return {'ok': True}

@app.post('/api/goal_sequence')
async def post_goal_sequence(req: GoalSequenceRequest):
    if node:
        node.publish_goal_sequence(req.nodes)
    return {'ok': True}

@app.get('/api/map/metadata')
async def get_map_metadata():
    return {
        'resolution': MAP_RESOLUTION,
        'origin_x': MAP_ORIGIN_X,
        'origin_y': MAP_ORIGIN_Y,
        'width': MAP_WIDTH_PX,
        'height': MAP_HEIGHT_PX,
    }

@app.get('/api/map/raw')
async def get_map_raw():
    """Serve the clean architectural floorplan (no graph overlay)."""
    if not _MAP_PNG_PATH.exists():
        return JSONResponse({'error': 'Map file not found'}, status_code=404)
    return FileResponse(_MAP_PNG_PATH, media_type='image/png')

@app.get('/api/map/clean')
async def get_map_clean():
    """Alias for /api/map/raw — explicitly returns the clean floorplan."""
    return await get_map_raw()

@app.get('/api/graph')
async def get_graph():
    if node is None:
        return JSONResponse({'error': 'ROS node not ready'}, status_code=503)
    return node.get_graph_data()

@app.post('/api/obstacle/cmd_vel')
async def post_obstacle_cmd_vel(req: CmdVelRequest):
    if node:
        node.publish_obstacle_cmd_vel(req.linear, req.angular)
    return {'ok': True}

# -------- Mapping REST endpoints (Phase 1) --------

@app.get('/api/mapping/status')
async def get_mapping_status():
    has_map = node._has_live_map if node else False
    map_info = node.get_live_map_info() if node else {}
    return mapping_mgr.get_status(has_map, map_info, ros_node=node)

@app.post('/api/mapping/start')
async def post_mapping_start(req: StartMappingRequest = StartMappingRequest()):
    mapping_mgr.start(map_name=req.map_name, world=req.world, run_explore=req.run_explore)
    has_map = node._has_live_map if node else False
    map_info = node.get_live_map_info() if node else {}
    return {'ok': True, 'mapping': mapping_mgr.get_status(has_map, map_info, ros_node=node)}

@app.post('/api/mapping/stop')
async def post_mapping_stop():
    mapping_mgr.stop()
    has_map = node._has_live_map if node else False
    map_info = node.get_live_map_info() if node else {}
    return {'ok': True, 'mapping': mapping_mgr.get_status(has_map, map_info, ros_node=node)}

@app.post('/api/mapping/redo')
async def post_mapping_redo():
    mapping_mgr.redo()
    has_map = node._has_live_map if node else False
    map_info = node.get_live_map_info() if node else {}
    return {'ok': True, 'mapping': mapping_mgr.get_status(has_map, map_info, ros_node=node)}

@app.post('/api/mapping/explore/start')
async def post_explore_start():
    if node is None:
        return JSONResponse({'error': 'ROS node not ready'}, status_code=503)
    ok = mapping_mgr.start_auto_explore(ros_node=node)
    has_map = node._has_live_map if node else False
    map_info = node.get_live_map_info() if node else {}
    return {'ok': ok, 'mapping': mapping_mgr.get_status(has_map, map_info, ros_node=node)}

@app.post('/api/mapping/explore/pause')
async def post_explore_pause():
    if node is None:
        return JSONResponse({'error': 'ROS node not ready'}, status_code=503)
    ok = mapping_mgr.pause_auto_explore(ros_node=node)
    has_map = node._has_live_map if node else False
    map_info = node.get_live_map_info() if node else {}
    return {'ok': ok, 'mapping': mapping_mgr.get_status(has_map, map_info, ros_node=node)}

@app.post('/api/mapping/explore/resume')
async def post_explore_resume():
    if node is None:
        return JSONResponse({'error': 'ROS node not ready'}, status_code=503)
    ok = mapping_mgr.resume_auto_explore(ros_node=node)
    has_map = node._has_live_map if node else False
    map_info = node.get_live_map_info() if node else {}
    return {'ok': ok, 'mapping': mapping_mgr.get_status(has_map, map_info, ros_node=node)}

@app.get('/api/mapping/live_map')
async def get_mapping_live_map(clean: bool = False):
    if node is None:
        return JSONResponse({'error': 'ROS node not ready'}, status_code=503)
    b64 = node.get_live_map_image_b64(clean=clean)
    return {
        'has_data': node._has_live_map and b64 is not None,
        'image': b64,
        'is_clean': clean,
        'encoding': 'png/base64',
        'info': node.get_live_map_info(),
        'pose': node.pose
    }

@app.get('/api/mapping/verify')
async def get_mapping_verify():
    if node is None:
        return JSONResponse({'error': 'ROS node not ready'}, status_code=503)
    verif = node.get_map_verification_data()
    status = mapping_mgr.get_status(node._has_live_map, node.get_live_map_info(), ros_node=node)
    return {'verification': verif, 'mapping': status}

@app.post('/api/mapping/save')
async def post_mapping_save(req: SaveMapRequest = SaveMapRequest()):
    if node is None:
        return JSONResponse({'error': 'ROS node not ready'}, status_code=503)
    target_name = req.map_name or mapping_mgr.map_name or 'warehouse_01'
    res = mapping_mgr.save_map(target_name, ros_node=node)
    return res

@app.get('/api/mapping/saved_maps')
async def get_saved_maps():
    maps_dir = _WORKSPACE_ROOT / "src" / "agv_description" / "maps"
    maps = []
    if maps_dir.exists():
        for yaml_file in sorted(maps_dir.glob("*.yaml")):
            name = yaml_file.stem
            pgm_file = maps_dir / f"{name}.pgm"
            png_file = maps_dir / f"{name}.png"
            maps.append({
                'name': name,
                'yaml': str(yaml_file.relative_to(_WORKSPACE_ROOT)),
                'has_pgm': pgm_file.exists(),
                'has_png': png_file.exists(),
                'size_kb': round(yaml_file.stat().st_size / 1024, 2),
            })
    return {'maps': maps}

@app.post('/api/mapping/graph/extract')
async def post_extract_graph(req: ExtractGraphRequest = ExtractGraphRequest()):
    """Phase 3: Extracts topological navigation roadmap (nodes & edges) from map YAML."""
    if graph_extractor is None:
        return JSONResponse({'error': 'Graph extractor module not available'}, status_code=500)

    target_name = (req.map_name or mapping_mgr.saved_map_name or mapping_mgr.map_name or 'warehouse_01').strip()
    maps_dir = _WORKSPACE_ROOT / "src" / "agv_description" / "maps"

    yaml_path = maps_dir / f"{target_name}.yaml"
    if not yaml_path.exists():
        candidates = [
            maps_dir / f"{target_name}_map.yaml",
            maps_dir / "warehouse_map.yaml",
            maps_dir / "warehouse_01.yaml"
        ]
        for c in candidates:
            if c.exists():
                yaml_path = c
                break

    if not yaml_path.exists():
        return JSONResponse({'error': f'Map YAML file not found for "{target_name}" in {maps_dir}'}, status_code=404)

    try:
        report = graph_extractor.extract_graph(
            str(yaml_path),
            robot_radius=req.robot_radius,
            safety_margin=req.safety_margin,
            global_search_radius=req.search_radius,
            step_corridor=req.step_corridor,
            step_medium=req.step_medium,
            step_open=req.step_open,
        )

        # Update install directory if present so downstream ROS nodes immediately see the graph
        install_maps_dir = _WORKSPACE_ROOT / "install" / "agv_description" / "share" / "agv_description" / "maps"
        if install_maps_dir.exists():
            try:
                import shutil
                json_p = Path(report['json_path'])
                if json_p.exists():
                    shutil.copy2(json_p, install_maps_dir / json_p.name)
                    shutil.copy2(json_p, install_maps_dir / "warehouse_graph.json")
            except Exception as e:
                print(f"[bridge] Note: Could not copy graph to install dir: {e}")

        # Update in-memory graph cache in node
        if node:
            node.set_active_graph(report)

        mapping_mgr.last_extracted_graph = {
            'map_name': report['map_name'],
            'total_nodes': report['total_nodes'],
            'total_edges': report['total_edges'],
            'json_path': str(Path(report['json_path']).relative_to(_WORKSPACE_ROOT)),
            'metrics': report['metrics'],
        }

        return {'ok': True, 'report': report}
    except Exception as e:
        return JSONResponse({'error': f'Graph extraction failed: {str(e)}'}, status_code=500)

@app.get('/api/mapping/graph/latest')
async def get_latest_graph(map_name: Optional[str] = None):
    maps_dir = _WORKSPACE_ROOT / "src" / "agv_description" / "maps"
    if map_name:
        cand_json = maps_dir / f"{map_name}_graph.json"
        if not cand_json.exists():
            cand_json = maps_dir / f"{map_name}.json"
        if cand_json.exists():
            try:
                with open(cand_json, 'r', encoding='utf-8') as f:
                    raw = json.load(f)
                vis_b64 = None
                cand_vis = maps_dir / f"{map_name}_graph_vis.png"
                if cand_vis.exists():
                    try:
                        with open(cand_vis, 'rb') as vf:
                            vis_b64 = base64.b64encode(vf.read()).decode('utf-8')
                    except Exception:
                        pass
                return {
                    'ok': True,
                    'graph': {
                        'map_name': map_name,
                        'nodes': raw.get('nodes', []),
                        'edges': raw.get('edges', []),
                        'total_nodes': len(raw.get('nodes', [])),
                        'total_edges': len(raw.get('edges', [])),
                        'vis_image_b64': vis_b64,
                        'metrics': {
                            'avg_connectivity': round(len(raw.get('edges', [])) / max(1, len(raw.get('nodes', []))), 2),
                            'connected_components': 1
                        }
                    }
                }
            except Exception as e:
                return JSONResponse({'error': str(e)}, status_code=500)

    if node:
        active = node.get_graph_data()
        if active and active.get('nodes'):
            return {'ok': True, 'graph': active}

    # Fallback to reading warehouse_graph.json
    if _GRAPH_JSON_PATH.exists():
        try:
            with open(_GRAPH_JSON_PATH, 'r', encoding='utf-8') as f:
                raw = json.load(f)
            vis_b64 = None
            vis_candidates = [
                maps_dir / "warehouse_01_graph_vis.png",
                maps_dir / "graph_visualization.png",
                maps_dir / "warehouse_map_graph_vis.png",
            ]
            for vc in vis_candidates:
                if vc.exists():
                    try:
                        with open(vc, 'rb') as img_f:
                            vis_b64 = base64.b64encode(img_f.read()).decode('utf-8')
                        break
                    except Exception:
                        pass
            return {
                'ok': True,
                'graph': {
                    'map_name': 'warehouse',
                    'nodes': raw.get('nodes', []),
                    'edges': raw.get('edges', []),
                    'total_nodes': len(raw.get('nodes', [])),
                    'total_edges': len(raw.get('edges', [])),
                    'vis_image_b64': vis_b64,
                    'metrics': {
                        'avg_connectivity': round(len(raw.get('edges', [])) / max(1, len(raw.get('nodes', []))), 2),
                        'connected_components': 1
                    }
                }
            }
        except Exception as e:
            return JSONResponse({'error': str(e)}, status_code=500)

    return JSONResponse({'error': 'No graph found'}, status_code=404)

# -------- WebSocket --------

active_ws: list = []

@app.websocket('/ws/telemetry')
async def ws_telemetry(websocket: WebSocket):
    await websocket.accept()
    active_ws.append(websocket)
    try:
        while True:
            # Keep connection alive; actual pushes come from the broadcast task
            await asyncio.sleep(60)
    except WebSocketDisconnect:
        pass
    finally:
        if websocket in active_ws:
            active_ws.remove(websocket)


async def _broadcast_loop():
    """Push telemetry to all connected WebSocket clients at 5 Hz."""
    while True:
        await asyncio.sleep(0.2)
        if not active_ws or node is None:
            continue
        payload = json.dumps(node.telemetry_snapshot())
        dead = []
        for ws in list(active_ws):
            try:
                await ws.send_text(payload)
            except Exception:
                dead.append(ws)
        for ws in dead:
            if ws in active_ws:
                active_ws.remove(ws)


@app.on_event('startup')
async def _startup():
    asyncio.create_task(_broadcast_loop())


# ---------------------------------------------------------------------------
# Entry point — spin ROS 2 in a daemon thread
# ---------------------------------------------------------------------------

def _ros_thread():
    global node
    try:
        rclpy.init()
        node = AmrBridgeNode()
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        try:
            executor.spin()
        except Exception:
            pass
        finally:
            # Null out tf2 listener BEFORE destroying node to prevent core dump
            try:
                if node._tf_listener is not None:
                    node._tf_listener = None
                    node._tf_buffer   = None
            except Exception:
                pass
            try:
                node.destroy_node()
            except Exception:
                pass
    except Exception as e:
        print(f'[bridge] ROS thread error: {e}')
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    t = threading.Thread(target=_ros_thread, daemon=True)
    t.start()
    time.sleep(1.0)  # Give ROS a moment to initialise
    uvicorn.run(app, host='0.0.0.0', port=8000, log_level='info')
