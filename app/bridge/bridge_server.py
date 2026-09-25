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
import threading
import time
from pathlib import Path
from typing import List, Optional

from PIL import Image, ImageDraw
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Odometry, Path as NavPath, OccupancyGrid
from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import String, Bool

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
    """Manages the lifecycle of mapping sessions (SLAM Toolbox + explore_lite)."""
    def __init__(self, workspace_root: Path):
        self.ws_root = workspace_root
        self.state = 'READY'  # 'READY' | 'MAPPING' | 'STOPPED'
        self.map_name = 'warehouse_01'
        self.start_time: Optional[float] = None
        self.stop_time: Optional[float] = None
        self.proc: Optional[subprocess.Popen] = None
        self.lock = threading.Lock()

    def start(self, map_name: str = 'warehouse_01'):
        with self.lock:
            if map_name and map_name.strip():
                self.map_name = map_name.strip()
            self.state = 'MAPPING'
            self.start_time = time.time()
            self.stop_time = None

            # Spawn mapping_session.launch.py in background if not already started
            if self.proc is None or self.proc.poll() is not None:
                cmd = (
                    f"source /opt/ros/jazzy/setup.bash && "
                    f"source {self.ws_root}/install/setup.bash && "
                    f"ros2 launch agv_description mapping_session.launch.py use_rviz:=false"
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

    def stop(self):
        with self.lock:
            self.state = 'STOPPED'
            self.stop_time = time.time()
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
            return True

    def redo(self):
        with self.lock:
            if self.proc and self.proc.poll() is None:
                try:
                    os.killpg(os.getpgid(self.proc.pid), signal.SIGTERM)
                except Exception:
                    pass
                self.proc = None
            self.state = 'READY'
            self.start_time = None
            self.stop_time = None
            return True

    def get_status(self, has_map_data: bool = False, map_info: Optional[dict] = None) -> dict:
        with self.lock:
            elapsed = 0.0
            if self.state == 'MAPPING' and self.start_time:
                elapsed = time.time() - self.start_time
            elif self.state == 'STOPPED' and self.start_time and self.stop_time:
                elapsed = self.stop_time - self.start_time

            mins = int(elapsed // 60)
            secs = int(elapsed % 60)
            elapsed_str = f"{mins:02d}:{secs:02d}"

            if has_map_data and self.state == 'MAPPING':
                map_status_text = "Receiving map data... (SLAM Toolbox active)"
            elif self.state == 'MAPPING':
                map_status_text = "Mapping in progress... Initializing SLAM Toolbox..."
            elif self.state == 'STOPPED':
                map_status_text = "Mapping stopped. Ready for map verification."
            else:
                map_status_text = "Ready to start mapping"

            return {
                'state': self.state,
                'map_name': self.map_name,
                'elapsed_sec': round(elapsed, 1),
                'elapsed_str': elapsed_str,
                'has_map_data': has_map_data,
                'map_status_text': map_status_text,
                'map_info': map_info or {},
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
                # -1 = unknown (dark slate), 0 = free (deep blue/black), >50 = occupied (cyan)
                rgb = np.zeros((h, w, 3), dtype=np.uint8)
                rgb[raw_data == -1] = [26, 32, 46]     # unknown
                rgb[raw_data == 0]  = [12, 16, 28]     # free space
                rgb[raw_data > 50]  = [0, 212, 255]    # wall / occupied
                rgb = np.flipud(rgb)
                pil_img = Image.fromarray(rgb)
                buf = io.BytesIO()
                pil_img.save(buf, format='PNG')
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

    def publish_estop(self, active: bool):
        b = Bool()
        b.data = active
        self.estop_pub.publish(b)

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

    def get_graph_data(self) -> dict:
        if self._cached_graph is None and _GRAPH_JSON_PATH.exists():
            try:
                with open(_GRAPH_JSON_PATH, 'r') as f:
                    raw = json.load(f)
                nodes = [
                    {
                        'id': n['id'],
                        'x': n['x'],
                        'y': n['y'],
                        'px': n.get('px'),
                        'py': n.get('py')
                    }
                    for n in raw.get('nodes', [])
                ]
                self._cached_graph = {
                    'nodes': nodes,
                    'total_nodes': len(nodes),
                    'total_edges': len(raw.get('edges', []))
                }
            except Exception as e:
                self.get_logger().error(f"Failed to load graph: {e}")
                return {'nodes': [], 'total_nodes': 0, 'total_edges': 0}
        return self._cached_graph or {'nodes': [], 'total_nodes': 0, 'total_edges': 0}

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

    def get_live_map_image_b64(self) -> Optional[str]:
        return self._live_map_b64

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
            'mapping':         mapping_mgr.get_status(self._has_live_map, self.get_live_map_info()),
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
    return mapping_mgr.get_status(has_map, map_info)

@app.post('/api/mapping/start')
async def post_mapping_start(req: StartMappingRequest = StartMappingRequest()):
    mapping_mgr.start(req.map_name)
    has_map = node._has_live_map if node else False
    map_info = node.get_live_map_info() if node else {}
    return {'ok': True, 'mapping': mapping_mgr.get_status(has_map, map_info)}

@app.post('/api/mapping/stop')
async def post_mapping_stop():
    mapping_mgr.stop()
    has_map = node._has_live_map if node else False
    map_info = node.get_live_map_info() if node else {}
    return {'ok': True, 'mapping': mapping_mgr.get_status(has_map, map_info)}

@app.post('/api/mapping/redo')
async def post_mapping_redo():
    mapping_mgr.redo()
    has_map = node._has_live_map if node else False
    map_info = node.get_live_map_info() if node else {}
    return {'ok': True, 'mapping': mapping_mgr.get_status(has_map, map_info)}

@app.get('/api/mapping/live_map')
async def get_mapping_live_map():
    if node is None:
        return JSONResponse({'error': 'ROS node not ready'}, status_code=503)
    b64 = node.get_live_map_image_b64()
    return {
        'has_data': node._has_live_map and b64 is not None,
        'image': b64,
        'encoding': 'png/base64',
        'info': node.get_live_map_info(),
        'pose': node.pose
    }

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
