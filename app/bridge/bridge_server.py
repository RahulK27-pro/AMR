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
import threading
import time
from pathlib import Path
from typing import List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Odometry, Path as NavPath
from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import String, Bool

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
_MAP_PNG_PATH = _WORKSPACE_ROOT / "src" / "agv_description" / "maps" / "graph_visualization.png"

# Map metadata from warehouse_map.yaml (resolution, origin) — used to convert
# robot (x,y) in metres to pixel coordinates for the overlay.
# resolution: metres per pixel
MAP_RESOLUTION = 0.05          # 0.05 m/px
MAP_ORIGIN_X   = -8.3          # map frame X at pixel (0,0)
MAP_ORIGIN_Y   = -7.3          # map frame Y at pixel (0,0)
MAP_HEIGHT_PX  = 275           # rows in the PGM


def world_to_pixel(wx: float, wy: float):
    """Convert world (x,y) metres → (col, row) pixel in the PNG."""
    col = int((wx - MAP_ORIGIN_X) / MAP_RESOLUTION)
    row = MAP_HEIGHT_PX - int((wy - MAP_ORIGIN_Y) / MAP_RESOLUTION)
    return col, row


# ---------------------------------------------------------------------------
# ROS 2 bridge node
# ---------------------------------------------------------------------------

class AmrBridgeNode(Node):
    def __init__(self):
        super().__init__('amr_web_bridge')

        # --- Publishers ---
        self.cmd_pub        = self.create_publisher(Twist,                      '/cmd_vel',       10)
        self.goal_pub       = self.create_publisher(PoseStamped,                '/goal_pose',     10)
        self.estop_pub      = self.create_publisher(Bool,                       '/agv_estop',     10)
        self.init_pose_pub  = self.create_publisher(PoseWithCovarianceStamped,  '/initialpose',   10)
        self.seq_pub        = self.create_publisher(String,                     '/goal_sequence', 10)

        # --- Subscribers ---
        self.create_subscription(Odometry,  '/odometry/filtered', self._odom_cb,    10)
        self.create_subscription(LaserScan, '/scan',              self._scan_cb,    10)
        self.create_subscription(String,    '/agv_state',         self._state_cb,   10)
        self.create_subscription(NavPath,   '/agv_dense_path',    self._path_cb,    10)
        self.create_subscription(String,    '/obstacle_alert',    self._alert_cb,   10)
        self.create_subscription(Imu,       '/imu/data',          self._imu_cb,     10)
        self.create_subscription(String,    '/mission_progress',  self._mission_cb, 10)

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

    # -------- Subscriber callbacks --------

    def _odom_cb(self, msg):
        self.pose['x']   = msg.pose.pose.position.x
        self.pose['y']   = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.pose['yaw'] = self._quat_to_yaw(q)
        self.velocity['linear']  = msg.twist.twist.linear.x
        self.velocity['angular'] = msg.twist.twist.angular.z

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
    rclpy.init()
    node = AmrBridgeNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    t = threading.Thread(target=_ros_thread, daemon=True)
    t.start()
    time.sleep(1.0)  # Give ROS a moment to initialise
    uvicorn.run(app, host='0.0.0.0', port=8000, log_level='info')
