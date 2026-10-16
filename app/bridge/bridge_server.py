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
os.environ['RCUTILS_LOGGING_SEVERITY_THRESHOLD'] = 'ERROR'
import signal
import subprocess
import sys
import threading
import time
import heapq
import logging
import shutil
import yaml
from collections import deque
from pathlib import Path
from typing import List, Optional, Tuple, Dict, Any

logger = logging.getLogger("amr_bridge")
logging.basicConfig(level=logging.INFO)


from PIL import Image, ImageDraw
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, qos_profile_sensor_data
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
# Floorplan image and graph paths
_MAP_PNG_PATH       = _WORKSPACE_ROOT / "src" / "agv_description" / "maps" / "warehouse_map.png"
_MAP_PNG_GRAPH_PATH = _WORKSPACE_ROOT / "src" / "agv_description" / "maps" / "warehouse_map_graph_vis.png"
_GRAPH_JSON_PATH    = _WORKSPACE_ROOT / "src" / "agv_description" / "maps" / "warehouse_map_graph.json"

# Import graph extractor engine
_MAPS_DIR = _WORKSPACE_ROOT / "src" / "agv_description" / "maps"
if str(_MAPS_DIR) not in sys.path:
    sys.path.insert(0, str(_MAPS_DIR))
try:
    import graph_extractor
except Exception as e:
    print(f"[bridge] Warning: Failed to import graph_extractor: {e}")
    graph_extractor = None

# Calibrated Map metadata defaults
MAP_RESOLUTION = 0.05          # default 0.05 m/px
MAP_ORIGIN_X   = -7.397        # default map frame X at pixel (0,0)
MAP_ORIGIN_Y   = -6.596        # default map frame Y at pixel (0,0)
MAP_WIDTH_PX   = 329           # default cols
MAP_HEIGHT_PX  = 275           # default rows


def get_map_metadata_by_name(map_name: Optional[str] = None) -> dict:
    """Read map metadata (resolution, origin, width, height) dynamically from <map_name>.yaml and image."""
    maps_dir = _WORKSPACE_ROOT / "src" / "agv_description" / "maps"
    target = (map_name or 'warehouse_01').strip()
    yaml_path = maps_dir / f"{target}.yaml"
    if not yaml_path.exists():
        if target in ('warehouse_map', 'warehouse'):
            yaml_path = maps_dir / "warehouse_map.yaml"
        elif target == 'warehouse_01':
            yaml_path = maps_dir / "warehouse_01.yaml"
        else:
            matches = list(maps_dir.glob(f"*{target}*.yaml"))
            if matches:
                yaml_path = matches[0]

    # Map-specific calibrated fallbacks
    if target == 'warehouse_01':
        res = 0.05
        ox = -6.976
        oy = -4.976
        width = 279
        height = 199
    else:
        res = 0.05
        ox = -7.397
        oy = -6.596
        width = 329
        height = 275

    if yaml_path.exists():
        try:
            with open(yaml_path, 'r', encoding='utf-8') as f:
                ydata = yaml.safe_load(f)
            if isinstance(ydata, dict):
                res = float(ydata.get('resolution', res))
                orig = ydata.get('origin', [ox, oy, 0.0])
                if isinstance(orig, list) and len(orig) >= 2:
                    ox = float(orig[0])
                    oy = float(orig[1])
                img_name = ydata.get('image', f"{target}.png")
                img_file = yaml_path.parent / img_name
                if not img_file.exists():
                    img_file = maps_dir / f"{target}.png"
                if not img_file.exists():
                    img_file = maps_dir / f"{yaml_path.stem}.png"
                
                if img_file.exists():
                    try:
                        from PIL import Image
                        with Image.open(img_file) as pimg:
                            width, height = pimg.size
                    except Exception:
                        pass
        except Exception as e:
            logger.warning(f"Error parsing map yaml {yaml_path}: {e}")

    return {
        'map_name': target,
        'resolution': res,
        'origin_x': ox,
        'origin_y': oy,
        'width': width,
        'height': height,
    }


def resolve_map_image_path(map_name: Optional[str] = None) -> Optional[Path]:
    """Find or convert the clean floorplan PNG for a given map."""
    maps_dir = _WORKSPACE_ROOT / "src" / "agv_description" / "maps"
    target = (map_name or 'warehouse_01').strip()
    
    # 1. Direct png file
    direct_png = maps_dir / f"{target}.png"
    if direct_png.exists():
        return direct_png
        
    # 2. Check yaml
    yaml_path = maps_dir / f"{target}.yaml"
    if yaml_path.exists():
        try:
            with open(yaml_path, 'r', encoding='utf-8') as f:
                yd = yaml.safe_load(f)
            if isinstance(yd, dict) and 'image' in yd:
                im_path = maps_dir / yd['image']
                if im_path.suffix.lower() == '.pgm' and not direct_png.exists():
                    try:
                        from PIL import Image
                        with Image.open(im_path) as pimg:
                            pimg.save(direct_png, format='PNG')
                        return direct_png
                    except Exception:
                        pass
                if im_path.exists() and im_path.suffix.lower() == '.png':
                    return im_path
        except Exception:
            pass
            
    # 3. If .pgm exists, convert to png
    pgm_path = maps_dir / f"{target}.pgm"
    if pgm_path.exists():
        try:
            from PIL import Image
            with Image.open(pgm_path) as pimg:
                pimg.save(direct_png, format='PNG')
            return direct_png
        except Exception:
            pass

    return direct_png if direct_png.exists() else None


def world_to_pixel(wx: float, wy: float, map_name: Optional[str] = None):
    """Convert world (x,y) metres → (col, row) pixel in the map image."""
    meta = get_map_metadata_by_name(map_name or (node.active_map_name if 'node' in globals() and node else 'warehouse_01'))
    col = int((wx - meta['origin_x']) / meta['resolution'])
    row = meta['height'] - int((wy - meta['origin_y']) / meta['resolution'])
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

    def _cleanup_processes(self):
        if self.explore_proc and self.explore_proc.poll() is None:
            try:
                os.killpg(os.getpgid(self.explore_proc.pid), signal.SIGTERM)
                try:
                    self.explore_proc.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    os.killpg(os.getpgid(self.explore_proc.pid), signal.SIGKILL)
            except Exception:
                pass
        self.explore_proc = None

        if self.proc and self.proc.poll() is None:
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
                try:
                    self.proc.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
            except Exception:
                pass
        self.proc = None

        # Clean up any lingering mapping sub-processes
        try:
            subprocess.run(
                ['pkill', '-f', 'async_slam_toolbox_node|explore_lite|rviz2'],
                check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
            )
        except Exception:
            pass

        for pattern in [
            'mapping_session', 'async_slam_toolbox_node', 'slam_toolbox',
            'explore_node', 'explore_lite', 'controller_server', 'planner_server',
            'behavior_server', 'bt_navigator', 'waypoint_follower', 'lifecycle_manager',
            'gz sim', 'parameter_bridge', 'ros_gz_bridge', 'ekf_node', 'robot_state_publisher'
        ]:
            try:
                subprocess.run(['pkill', '-9', '-f', pattern], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except Exception:
                pass

    def start(self, map_name: str = 'warehouse_01', world: str = 'test1.world', run_explore: bool = False, headless: bool = True, use_rviz: bool = False):
        with self.lock:
            # 1. Stop any active navigation stack first to prevent conflicts
            if 'system_mgr' in globals() and globals()['system_mgr'] is not None:
                try:
                    globals()['system_mgr'].stop()
                except Exception as e:
                    logger.warning(f"[MappingManager] Warning stopping system_mgr: {e}")

            # 2. Clean up any leftover navigation / mapping processes
            self._cleanup_processes()

            # 3. Reset bridge node state for clean mapping session
            if 'node' in globals() and globals()['node'] is not None:
                try:
                    globals()['node'].reset_simulation_state()
                    globals()['node']._has_live_map = False
                    globals()['node']._live_map_b64 = None
                    globals()['node']._clean_map_b64 = None
                except Exception as e:
                    logger.warning(f"[MappingManager] Node reset warning: {e}")

            if map_name and map_name.strip():
                self.map_name = map_name.strip()
            self.state = 'MAPPING'
            self.start_time = time.time()
            self.stop_time = None
            self.is_saved = False
            self.saved_map_name = None
            self.saved_paths = {}
            self.verification_report = None
            self.target_world = world if (world.endswith('.world') or world.endswith('.sdf')) else f"{world}.world"
            self.run_explore = run_explore

            run_sim_flag = 'true'
            run_explore_flag = 'true' if run_explore else 'false'
            headless_flag = 'true' if headless else 'false'
            rviz_flag = 'true' if use_rviz else 'false'

            cmd = (
                f"export LIBGL_ALWAYS_SOFTWARE=1 && "
                f"export MESA_GL_VERSION_OVERRIDE=3.3 && "
                f"source /opt/ros/jazzy/setup.bash && "
                f"source {self.ws_root}/install/setup.bash && "
                f"ros2 launch agv_description mapping_session.launch.py "
                f"world:={self.target_world} run_sim:={run_sim_flag} run_explore:={run_explore_flag} "
                f"headless:={headless_flag} use_rviz:={rviz_flag}"
            )
            logger.info(f"[MappingManager] Starting mapping_session: {cmd}")
            try:
                self.mapping_log = open('/tmp/amr_mapping.log', 'w')
                self.proc = subprocess.Popen(
                    cmd,
                    shell=True,
                    executable='/bin/bash',
                    cwd=str(self.ws_root),
                    preexec_fn=os.setsid,
                    stdout=self.mapping_log,
                    stderr=subprocess.STDOUT
                )
            except Exception as e:
                logger.error(f"[MappingManager] Failed to launch mapping_session: {e}")
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
                    logger.info("[MappingManager] Spawning explore_node dynamically...")
                except Exception as e:
                    logger.error(f"[MappingManager] Failed to spawn explore_node: {e}")
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
            self._cleanup_processes()
            if 'node' in globals() and globals()['node'] is not None:
                try:
                    globals()['node'].publish_cmd_vel(0.0, 0.0)
                except Exception:
                    pass
            logger.info("[MappingManager] Mapping session stopped cleanly.")
            return True

    def redo(self):
        with self.lock:
            self.run_explore = False
            self._cleanup_processes()
            self.state = 'READY'
            self.start_time = None
            self.stop_time = None
            self.is_saved = False
            self.saved_map_name = None
            self.saved_paths = {}
            self.verification_report = None
            self.last_extracted_graph = None
            if 'node' in globals() and globals()['node'] is not None:
                try:
                    globals()['node'].reset_simulation_state()
                    globals()['node']._has_live_map = False
                    globals()['node']._live_map_b64 = None
                    globals()['node']._clean_map_b64 = None
                except Exception:
                    pass
            logger.info("[MappingManager] Mapping session reset (redo).")
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


class SystemLaunchManager:
    """Manages the full navigation stack lifecycle:
       1. navigation_launch.py (Gazebo + RViz + AMCL + map_server)
       2. route_runner (MPPI Controller)
       3. bt_manager (Behavior Tree Orchestrator)
    """
    def __init__(self, workspace_root: Path):
        self.ws_root = workspace_root
        self.state = 'STOPPED'  # 'STOPPED' | 'STARTING' | 'RUNNING' | 'STOPPING' | 'ERROR'
        self.active_map = 'warehouse_01'
        self.stage = 0  # 0: Stopped, 1: Launching Nav & Gazebo, 2: Starting MPPI, 3: Starting BT Manager, 4: Running
        self.stage_text = 'Stopped'
        self.start_time: Optional[float] = None
        self.nav_proc: Optional[subprocess.Popen] = None
        self.runner_proc: Optional[subprocess.Popen] = None
        self.bt_proc: Optional[subprocess.Popen] = None
        self.graph_proc: Optional[subprocess.Popen] = None
        self.nav_log = None
        self.runner_log = None
        self.bt_log = None
        self.graph_log = None
        self.error_msg: Optional[str] = None
        self.lock = threading.Lock()
        self._worker_thread: Optional[threading.Thread] = None

    @staticmethod
    def _tail_log(filepath: str, max_lines: int = 5) -> str:
        try:
            if not os.path.exists(filepath):
                return "(no log file found)"
            with open(filepath, 'r', errors='replace') as f:
                lines = [l.strip() for l in f.readlines() if l.strip()]
                return " | ".join(lines[-max_lines:]) if lines else "(empty log)"
        except Exception as e:
            return f"(error reading log: {e})"

    def start(self, map_name: str = 'warehouse_01', world: Optional[str] = None, headless: bool = True, use_rviz: bool = False) -> dict:
        with self.lock:
            if self.state in ('STARTING', 'RUNNING'):
                return {'ok': False, 'error': f'System is already {self.state.lower()}'}

            target_map = (map_name or self.active_map or 'warehouse_01').strip()
            self.active_map = target_map
            self.state = 'STARTING'
            self.stage = 1
            self.stage_text = f'1/3 Initializing simulation and map ({target_map})...'
            self.start_time = time.time()
            self.error_msg = None

            # Clean any old stray processes first
            self._cleanup_processes()

            # Reset bridge node telemetry state & TF buffer for fresh simulation run
            if 'node' in globals() and globals()['node'] is not None:
                try:
                    globals()['node'].reset_simulation_state()
                except Exception as e:
                    logger.warning(f"[SystemLaunchManager] Node reset warning: {e}")

            self._worker_thread = threading.Thread(
                target=self._start_sequence,
                args=(target_map, world, headless, use_rviz),
                daemon=True
            )
            self._worker_thread.start()

            return {'ok': True, 'state': self.state, 'map_name': target_map}

    def _start_sequence(self, map_name: str, world: Optional[str], headless: bool = True, use_rviz: bool = False):
        try:
            # Stop any active mapping session first
            if 'mapping_mgr' in globals() and globals()['mapping_mgr'] is not None:
                try:
                    globals()['mapping_mgr'].stop()
                except Exception as e:
                    logger.warning(f"[SystemLaunchManager] Warning stopping mapping_mgr: {e}")

            maps_dir = self.ws_root / "src" / "agv_description" / "maps"
            src_graph = maps_dir / f"{map_name}_graph.json"
            if src_graph.exists():
                dest_graph = maps_dir / "warehouse_graph.json"
                try:
                    shutil.copy2(src_graph, dest_graph)
                    logger.info(f"[SystemLaunchManager] Synced {map_name}_graph.json -> warehouse_graph.json")
                except Exception as e:
                    logger.warning(f"[SystemLaunchManager] Sync graph warning: {e}")

            # 1. Launch navigation_launch.py
            world_arg = f"world:={world}" if world else ""
            headless_arg = f"headless:={'true' if headless else 'false'}"
            rviz_arg = f"use_rviz:={'true' if use_rviz else 'false'}"
            nav_cmd = (
                f"export LIBGL_ALWAYS_SOFTWARE=1 && "
                f"export MESA_GL_VERSION_OVERRIDE=3.3 && "
                f"source /opt/ros/jazzy/setup.bash && "
                f"source {self.ws_root}/install/setup.bash && "
                f"ros2 launch agv_description navigation_launch.py map:={map_name} {world_arg} {headless_arg} {rviz_arg}"
            )
            logger.info(f"[SystemLaunchManager] Starting navigation_launch: {nav_cmd}")
            self.nav_log = open('/tmp/amr_navigation.log', 'w')
            self.nav_proc = subprocess.Popen(
                nav_cmd,
                shell=True,
                executable='/bin/bash',
                cwd=str(self.ws_root),
                preexec_fn=os.setsid,
                stdout=self.nav_log,
                stderr=subprocess.STDOUT
            )

            # Wait ~6.0s for Gazebo and map_server to start
            for _ in range(6):
                time.sleep(1.0)
                if self.state == 'STOPPING':
                    return
                if self.nav_proc.poll() is not None:
                    tail = self._tail_log('/tmp/amr_navigation.log')
                    raise RuntimeError(f"navigation_launch.py terminated unexpectedly: {tail}")

            # 2. Launch route_runner
            with self.lock:
                self.stage = 2
                self.stage_text = '2/3 Starting MPPI Route Runner...'

            runner_cmd = (
                f"source /opt/ros/jazzy/setup.bash && "
                f"source {self.ws_root}/install/setup.bash && "
                f"ros2 run agv_navigation route_runner --ros-args -p map_name:={map_name} -p use_sim_time:=true"
            )
            logger.info("[SystemLaunchManager] Starting route_runner...")
            self.runner_log = open('/tmp/amr_route_runner.log', 'w')
            self.runner_proc = subprocess.Popen(
                runner_cmd,
                shell=True,
                executable='/bin/bash',
                cwd=str(self.ws_root),
                preexec_fn=os.setsid,
                stdout=self.runner_log,
                stderr=subprocess.STDOUT
            )

            time.sleep(2.0)
            if self.state == 'STOPPING':
                return
            if self.runner_proc.poll() is not None:
                tail = self._tail_log('/tmp/amr_route_runner.log')
                raise RuntimeError(f"route_runner terminated unexpectedly: {tail}")

            # 3. Launch bt_manager
            with self.lock:
                self.stage = 3
                self.stage_text = '3/3 Starting Behavior Tree Manager...'

            bt_cmd = (
                f"source /opt/ros/jazzy/setup.bash && "
                f"source {self.ws_root}/install/setup.bash && "
                f"ros2 run agv_navigation bt_manager --ros-args -p map_name:={map_name} -p use_sim_time:=true"
            )
            logger.info("[SystemLaunchManager] Starting bt_manager...")
            self.bt_log = open('/tmp/amr_bt_manager.log', 'w')
            self.bt_proc = subprocess.Popen(
                bt_cmd,
                shell=True,
                executable='/bin/bash',
                cwd=str(self.ws_root),
                preexec_fn=os.setsid,
                stdout=self.bt_log,
                stderr=subprocess.STDOUT
            )

            # 4. Launch graph_visualizer
            graph_cmd = (
                f"source /opt/ros/jazzy/setup.bash && "
                f"source {self.ws_root}/install/setup.bash && "
                f"ros2 run agv_navigation graph_visualizer --ros-args -p map_name:={map_name} -p use_sim_time:=true"
            )
            logger.info("[SystemLaunchManager] Starting graph_visualizer...")
            self.graph_log = open('/tmp/amr_graph_visualizer.log', 'w')
            self.graph_proc = subprocess.Popen(
                graph_cmd,
                shell=True,
                executable='/bin/bash',
                cwd=str(self.ws_root),
                preexec_fn=os.setsid,
                stdout=self.graph_log,
                stderr=subprocess.STDOUT
            )

            time.sleep(1.0)
            with self.lock:
                if self.state != 'STOPPING':
                    self.state = 'RUNNING'
                    self.stage = 4
                    self.stage_text = f'Running ({map_name})'
                    logger.info(f"[SystemLaunchManager] Navigation stack is fully RUNNING on {map_name}!")

            # Auto-spawn dynamic obstacle into Gazebo if one was chosen
            if 'node' in globals() and globals()['node'] is not None:
                n = globals()['node']
                if getattr(n, 'obstacle_pose', None) and n.obstacle_pose.get('active'):
                    try:
                        n.spawn_or_move_obstacle(n.obstacle_pose['x'], n.obstacle_pose['y'], n.obstacle_pose.get('yaw', 0.0))
                    except Exception as err:
                        logger.warning(f"[SystemLaunchManager] Auto-spawn obstacle error: {err}")
        except Exception as e:
            with self.lock:
                self.state = 'ERROR'
                self.error_msg = str(e)
                self.stage_text = f'Error: {e}'
                logger.error(f"[SystemLaunchManager] Startup failed: {e}")
                self._cleanup_processes()

    def stop(self) -> dict:
        with self.lock:
            self.state = 'STOPPING'
            self.stage_text = 'Stopping navigation stack...'
            self._cleanup_processes()
            self.state = 'STOPPED'
            self.stage = 0
            self.stage_text = 'Stopped'
            self.start_time = None
            if 'node' in globals() and globals()['node'] is not None:
                try:
                    globals()['node'].reset_simulation_state()
                except Exception as e:
                    logger.warning(f"[SystemLaunchManager] Node reset warning: {e}")
            logger.info("[SystemLaunchManager] System stopped successfully.")
            return {'ok': True, 'state': self.state}

    def _cleanup_processes(self):
        for proc, name in [
            (self.graph_proc, 'graph_visualizer'),
            (self.bt_proc, 'bt_manager'),
            (self.runner_proc, 'route_runner'),
            (self.nav_proc, 'navigation_launch')
        ]:
            if proc and proc.poll() is None:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                    try:
                        proc.wait(timeout=2.0)
                    except subprocess.TimeoutExpired:
                        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except Exception as e:
                    logger.warning(f"[SystemLaunchManager] Error terminating {name}: {e}")

        for log_f in [self.nav_log, self.runner_log, self.bt_log, self.graph_log]:
            if log_f:
                try:
                    log_f.close()
                except Exception:
                    pass
        self.nav_log = None
        self.runner_log = None
        self.bt_log = None
        self.graph_log = None

        self.graph_proc = None
        self.bt_proc = None
        self.runner_proc = None
        self.nav_proc = None

        for pattern in [
            'graph_visualizer', 'bt_manager', 'route_runner', 'navigation_launch',
            'gz sim', 'map_server', 'amcl', 'lifecycle_manager',
            'parameter_bridge', 'ros_gz_bridge', 'ekf_node', 'robot_state_publisher'
        ]:
            try:
                subprocess.run(['pkill', '-9', '-f', pattern], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except Exception:
                pass

    def get_status(self) -> dict:
        with self.lock:
            if self.state == 'RUNNING':
                if self.nav_proc and self.nav_proc.poll() is not None:
                    self.state = 'ERROR'
                    tail = self._tail_log('/tmp/amr_navigation.log')
                    self.error_msg = f'navigation_launch terminated: {tail}'
                    logger.error(f"[SystemLaunchManager] {self.error_msg}")
                elif self.runner_proc and self.runner_proc.poll() is not None:
                    self.state = 'ERROR'
                    tail = self._tail_log('/tmp/amr_route_runner.log')
                    self.error_msg = f'route_runner terminated: {tail}'
                    logger.error(f"[SystemLaunchManager] {self.error_msg}")
                elif self.bt_proc and self.bt_proc.poll() is not None:
                    self.state = 'ERROR'
                    tail = self._tail_log('/tmp/amr_bt_manager.log')
                    self.error_msg = f'bt_manager terminated: {tail}'
                    logger.error(f"[SystemLaunchManager] {self.error_msg}")

            uptime = round(time.time() - self.start_time, 1) if (self.start_time and self.state == 'RUNNING') else 0.0

            return {
                'ok': True,
                'state': self.state,
                'stage': self.stage,
                'stage_text': self.stage_text,
                'active_map': self.active_map,
                'uptime_sec': uptime,
                'error': self.error_msg,
                'processes': {
                    'navigation_launch': self.nav_proc.pid if (self.nav_proc and self.nav_proc.poll() is None) else None,
                    'route_runner': self.runner_proc.pid if (self.runner_proc and self.runner_proc.poll() is None) else None,
                    'bt_manager': self.bt_proc.pid if (self.bt_proc and self.bt_proc.poll() is None) else None,
                }
            }


system_mgr = SystemLaunchManager(_WORKSPACE_ROOT)


# ---------------------------------------------------------------------------
# ROS 2 bridge node
# ---------------------------------------------------------------------------

class AmrBridgeNode(Node):
    def __init__(self):
        super().__init__(
            'amr_web_bridge',
            parameter_overrides=[
                rclpy.parameter.Parameter('use_sim_time', rclpy.parameter.Parameter.Type.BOOL, True)
            ]
        )
        try:
            rclpy.logging.get_logger('tf2_buffer').set_level(rclpy.logging.LoggingSeverity.FATAL)
        except Exception:
            pass

        # --- Publishers ---
        self.cmd_pub          = self.create_publisher(Twist,                      '/cmd_vel',       10)
        self.goal_pub         = self.create_publisher(PoseStamped,                '/goal_pose',     10)
        self.estop_pub        = self.create_publisher(Bool,                       '/agv_estop',     10)
        self.init_pose_pub    = self.create_publisher(PoseWithCovarianceStamped,  '/initialpose',   10)
        self.seq_pub          = self.create_publisher(String,                     '/goal_sequence', 10)
        self.obstacle_cmd_pub = self.create_publisher(Twist,                      '/dynamic_obstacle/cmd_vel', 10)
        self.explore_resume_pub = self.create_publisher(Bool,                     '/explore/resume', 10)

        latched_map_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, reliability=QoSReliabilityPolicy.RELIABLE)

        # --- Subscribers ---
        self.create_subscription(Odometry,      '/dynamic_obstacle/odom', self._obstacle_odom_cb, 10)
        self.create_subscription(Odometry,      '/odometry/filtered', self._odom_cb,    10)
        self.create_subscription(LaserScan,     '/scan',              self._scan_cb,    qos_profile_sensor_data)
        self.create_subscription(String,        '/agv_state',         self._state_cb,   10)
        self.create_subscription(NavPath,       '/agv_dense_path',    self._path_cb,    10)
        self.create_subscription(String,        '/obstacle_alert',    self._alert_cb,   10)
        self.create_subscription(Imu,           '/imu/data',          self._imu_cb,     qos_profile_sensor_data)
        self.create_subscription(String,        '/mission_progress',  self._mission_cb, 10)
        # AMCL pose — authoritative map-frame localization (replaces raw odom for pose display)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, latched_map_qos
        )
        # Live /map subscription from SLAM Toolbox
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, latched_map_qos)

        # Exploration status & frontiers subscriptions
        if _EXPLORE_MSGS_AVAILABLE:
            self.create_subscription(
                ExploreStatus, '/explore/status', self._explore_status_cb, 10
            )
        if _MARKER_MSGS_AVAILABLE:
            self.create_subscription(
                MarkerArray, '/explore/frontiers', self._explore_frontiers_cb, 10
            )

        # TF2 buffer for map->base_link lookups (bound to node clock for time-jump handling)
        self._amcl_pose_received = False
        self._last_sim_time = 0.0
        self._tf_buffer   = None
        self._tf_listener = None
        if _TF2_AVAILABLE:
            try:
                self._tf_buffer   = tf2_ros.Buffer(node=self)
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
        self.active_map_name: str = 'warehouse_01'
        self._cached_graphs: Dict[str, dict] = {}
        self._cached_graph: Optional[dict] = None
        self.active_place_nav: Optional[dict] = None
        self.obstacle_pose: Optional[dict] = None
        self._obs_spawn_x: Optional[float] = None
        self._obs_spawn_y: Optional[float] = None
        self._obs_spawn_yaw: float = 0.0

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

    def reset_simulation_state(self):
        """Reset all telemetry state, waypoints, and TF buffer for clean new run."""
        logger.info("[bridge] Resetting simulation state and clearing buffers...")
        self.pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.velocity = {'linear': 0.0, 'angular': 0.0}
        self.min_obstacle_dist = float('inf')
        self.scan_ranges = []
        self.scan_angle_min = 0.0
        self.scan_angle_inc = 0.0
        self.nav_state = 'IDLE'
        self.path_waypoints = []
        self.obstacle_alert = None
        self.mission = {}
        self.active_place_nav = None
        self.obstacle_pose = None
        self._obs_spawn_x = None
        self._obs_spawn_y = None
        self._obs_spawn_yaw = 0.0
        self._amcl_pose_received = False
        self._last_sim_time = 0.0
        if self._tf_buffer is not None:
            try:
                self._tf_buffer.clear()
            except Exception as e:
                logger.warning(f"Error clearing TF buffer: {e}")

    # -------- Subscriber callbacks --------

    def _get_active_gz_world(self) -> str:
        """Dynamically detect running Gazebo world name."""
        try:
            res = subprocess.run(
                ["gz", "service", "-s", "/gazebo/worlds", "--reqtype", "gz.msgs.Empty", "--reptype", "gz.msgs.StringMsg_V", "--timeout", "1000", "--req", ""],
                capture_output=True, text=True, timeout=1.5
            )
            if res.returncode == 0 and "data:" in res.stdout:
                for line in res.stdout.splitlines():
                    if "data:" in line:
                        w = line.split("data:")[-1].strip().strip('"').strip("'")
                        if w:
                            return w
        except Exception:
            pass
        active = getattr(self, 'active_map_name', None)
        if active == 'warehouse_01':
            return 'test1'
        if active == 'warehouse_map':
            return 'warehouse'
        return active if active else 'test1'

    def _obstacle_odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        siny_cosp = 2.0 * (o.w * o.z + o.x * o.y)
        cosy_cosp = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
        local_yaw = math.atan2(siny_cosp, cosy_cosp)
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        speed = math.hypot(vx, vy)

        # Map local DiffDrive odometry back to global world coordinates
        spawn_x = getattr(self, '_obs_spawn_x', None)
        spawn_y = getattr(self, '_obs_spawn_y', None)
        spawn_yaw = getattr(self, '_obs_spawn_yaw', 0.0) or 0.0

        if spawn_x is None or spawn_y is None:
            # Obstacle has been removed or not yet spawned; do not revive pose
            return

        cos_sy = math.cos(spawn_yaw)
        sin_sy = math.sin(spawn_yaw)
        world_x = spawn_x + (p.x * cos_sy - p.y * sin_sy)
        world_y = spawn_y + (p.x * sin_sy + p.y * cos_sy)
        world_yaw = spawn_yaw + local_yaw

        self.obstacle_pose = {
            'x': round(float(world_x), 3),
            'y': round(float(world_y), 3),
            'yaw': round(float(world_yaw), 3),
            'speed': round(float(speed), 3),
            'active': True,
            'ts': time.time(),
        }

    def _odom_cb(self, msg):
        # Use odom only for velocity; pose comes from AMCL / TF (map frame)
        self.velocity['linear']  = msg.twist.twist.linear.x
        self.velocity['angular'] = msg.twist.twist.angular.z

        # Detect simulation restart (time jump backward > 2.0s)
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if hasattr(self, '_last_sim_time') and stamp_sec > 0 and self._last_sim_time > 0:
            if stamp_sec < (self._last_sim_time - 2.0):
                logger.warning(f"[bridge] Detected simulation time jump backward ({self._last_sim_time:.2f}s -> {stamp_sec:.2f}s)! Resetting simulation state.")
                self.reset_simulation_state()
        self._last_sim_time = stamp_sec

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
        """10 Hz TF lookup: map->base_link as authoritative map-frame source."""
        if not _TF2_AVAILABLE or self._tf_buffer is None:
            return
        try:
            tf = self._tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time()
            )
            t = tf.transform.translation
            self.pose['x']   = t.x
            self.pose['y']   = t.y
            self.pose['yaw'] = self._quat_to_yaw(tf.transform.rotation)
            self._amcl_pose_received = True
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
        if msg.data == 'IDLE' and not self.active_place_nav:
            self.path_waypoints = []
        if self.active_place_nav:
            self.active_place_nav['nav_state'] = msg.data
            if msg.data == 'IDLE' and self.active_place_nav.get('status') == 'NAVIGATING':
                tx = self.active_place_nav.get('target_x')
                ty = self.active_place_nav.get('target_y')
                if tx is not None and ty is not None:
                    d = math.hypot(self.pose['x'] - tx, self.pose['y'] - ty)
                    if d < 0.6:
                        self.active_place_nav['status'] = 'ARRIVED'

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
            if self.active_place_nav:
                st = self.mission.get('state')
                if st == 'MISSION_COMPLETE':
                    self.active_place_nav['status'] = 'COMPLETED'
                elif st:
                    self.active_place_nav['status'] = st
                if 'current' in self.mission:
                    self.active_place_nav['current_stop'] = self.mission['current']
                if 'total' in self.mission:
                    self.active_place_nav['total_stops'] = self.mission['total']
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
        if self.obstacle_pose:
            self.obstacle_pose['speed'] = round(abs(float(linear)), 3)
            self.obstacle_pose['ts'] = time.time()

    def spawn_or_move_obstacle(self, x: float, y: float, yaw: float = 0.0) -> dict:
        """Spawn or relocate dynamic obstacle in Gazebo Harmonic simulation using native gz services."""
        self._obs_spawn_x = float(x)
        self._obs_spawn_y = float(y)
        self._obs_spawn_yaw = float(yaw)

        self.obstacle_pose = {
            'x': round(float(x), 3),
            'y': round(float(y), 3),
            'yaw': round(float(yaw), 3),
            'speed': 0.0,
            'active': True,
            'ts': time.time(),
        }

        # Model SDF path
        sdf_path = _WORKSPACE_ROOT / "src" / "agv_description" / "models" / "dynamic_obstacle" / "model.sdf"
        if not sdf_path.exists():
            sdf_path = _WORKSPACE_ROOT / "install" / "agv_description" / "share" / "agv_description" / "models" / "dynamic_obstacle" / "model.sdf"

        def _worker():
            try:
                # Check if Gazebo Harmonic is running
                pg = subprocess.run(["pgrep", "-f", "gz sim|gz-sim"], capture_output=True, text=True)
                if pg.returncode != 0:
                    logger.info(f"[DynamicObstacle] Gazebo not running. Pose recorded at ({x:.2f}, {y:.2f}).")
                    return

                world = self._get_active_gz_world()
                qz = math.sin(yaw / 2.0)
                qw = math.cos(yaw / 2.0)

                # Remove existing obstacle entity first (takes ~0.05s) to reset odometry & physics momentum
                rem_cmd = [
                    "gz", "service", "-s", f"/world/{world}/remove",
                    "--reqtype", "gz.msgs.Entity",
                    "--reptype", "gz.msgs.Boolean",
                    "--timeout", "1500",
                    "--req", 'name: "dynamic_obstacle", type: MODEL'
                ]
                subprocess.run(rem_cmd, capture_output=True, text=True, timeout=2.0)
                time.sleep(0.1)

                # Spawn dynamic obstacle via native Gazebo service
                create_req = (
                    f'sdf_filename: "{str(sdf_path)}", name: "dynamic_obstacle", '
                    f'pose: {{position: {{x: {x}, y: {y}, z: 0.5}}, orientation: {{z: {qz:.5f}, w: {qw:.5f}}}}}'
                )
                create_cmd = [
                    "gz", "service", "-s", f"/world/{world}/create",
                    "--reqtype", "gz.msgs.EntityFactory",
                    "--reptype", "gz.msgs.Boolean",
                    "--timeout", "3000",
                    "--req", create_req
                ]
                logger.info(f"[DynamicObstacle] Spawning obstacle in world '{world}' at ({x:.2f}, {y:.2f})")
                res = subprocess.run(create_cmd, capture_output=True, text=True, timeout=4.0)

                if res.returncode == 0 and "data: true" in res.stdout:
                    logger.info(f"[DynamicObstacle] Successfully spawned dynamic obstacle in world '{world}' at ({x:.2f}, {y:.2f})")
                else:
                    logger.warning(f"[DynamicObstacle] Create returned: {res.stdout.strip()} {res.stderr.strip()}. Attempting set_pose...")
                    set_req = f'name: "dynamic_obstacle", position: {{x: {x}, y: {y}, z: 0.5}}, orientation: {{z: {qz:.5f}, w: {qw:.5f}}}'
                    set_cmd = [
                        "gz", "service", "-s", f"/world/{world}/set_pose",
                        "--reqtype", "gz.msgs.Pose",
                        "--reptype", "gz.msgs.Boolean",
                        "--timeout", "1500",
                        "--req", set_req
                    ]
                    subprocess.run(set_cmd, capture_output=True, text=True, timeout=2.5)
            except Exception as e:
                logger.error(f"[DynamicObstacle] Spawn worker error: {e}")

        threading.Thread(target=_worker, daemon=True).start()
        return {'ok': True, 'msg': f'Dynamic obstacle placed at ({x:.2f}, {y:.2f})', 'obstacle': self.obstacle_pose}

    def remove_obstacle(self) -> dict:
        """Remove dynamic obstacle from simulation."""
        self.obstacle_pose = None
        self._obs_spawn_x = None
        self._obs_spawn_y = None
        self._obs_spawn_yaw = 0.0

        def _worker():
            try:
                world = self._get_active_gz_world()
                rem_cmd = [
                    "gz", "service", "-s", f"/world/{world}/remove",
                    "--reqtype", "gz.msgs.Entity",
                    "--reptype", "gz.msgs.Boolean",
                    "--timeout", "1500",
                    "--req", 'name: "dynamic_obstacle", type: MODEL'
                ]
                res = subprocess.run(rem_cmd, capture_output=True, text=True, timeout=2.5)
                logger.info(f"[DynamicObstacle] Removed dynamic obstacle entity from Gazebo world '{world}': {res.stdout.strip()}")
            except Exception as e:
                logger.warning(f"[DynamicObstacle] Remove obstacle error: {e}")

        threading.Thread(target=_worker, daemon=True).start()
        return {'ok': True, 'msg': 'Dynamic obstacle removed from simulation'}

    def set_active_graph(self, graph_dict: dict):
        map_name = graph_dict.get('map_name', self.active_map_name)
        cached = {
            'nodes': graph_dict.get('nodes', []),
            'edges': graph_dict.get('edges', []),
            'places': graph_dict.get('places', []),
            'total_nodes': graph_dict.get('total_nodes', len(graph_dict.get('nodes', []))),
            'total_edges': graph_dict.get('total_edges', len(graph_dict.get('edges', []))),
            'map_name': map_name,
            'metrics': graph_dict.get('metrics', {}),
            'vis_image_b64': graph_dict.get('vis_image_b64', None),
        }
        self._cached_graphs[map_name] = cached
        if map_name == self.active_map_name:
            self._cached_graph = cached

    def set_active_map(self, map_name: str):
        target_name = map_name.strip()
        maps_dir = _WORKSPACE_ROOT / "src" / "agv_description" / "maps"
        valid_map = (
            (maps_dir / f"{target_name}.yaml").exists() or 
            (maps_dir / f"{target_name}_graph.json").exists() or 
            (maps_dir / f"{target_name}.png").exists()
        )
        if not valid_map:
            raise FileNotFoundError(f"Map '{target_name}' not found on disk.")
            
        self.active_map_name = target_name
        # Pre-cache graph for active map
        self.get_graph_data(target_name)
        
        # 1. Synchronize active graph to warehouse_graph.json for ROS2 route_runner
        src_graph = maps_dir / f"{target_name}_graph.json"
        if src_graph.exists():
            dest_graph = maps_dir / "warehouse_graph.json"
            try:
                shutil.copy2(src_graph, dest_graph)
                self.get_logger().info(f"Synchronized active map '{target_name}' graph to warehouse_graph.json")
            except Exception as e:
                self.get_logger().warning(f"Could not sync graph to warehouse_graph.json: {e}")

    def get_graph_data(self, map_name: Optional[str] = None) -> dict:
        target_name = (map_name or self.active_map_name or 'warehouse_01').strip()
        if target_name in self._cached_graphs:
            return self._cached_graphs[target_name]

        maps_dir = _WORKSPACE_ROOT / "src" / "agv_description" / "maps"
        graph_path = maps_dir / f"{target_name}_graph.json"
        if not graph_path.exists() and target_name == 'warehouse':
            graph_path = maps_dir / "warehouse_graph.json"

        if graph_path.exists():
            try:
                with open(graph_path, 'r', encoding='utf-8') as f:
                    raw = json.load(f)
                nodes = raw.get('nodes', [])
                edges = raw.get('edges', [])
                places = raw.get('places', [])
                vis_b64 = None
                vis_candidates = [
                    maps_dir / f"{target_name}_graph_vis.png",
                    maps_dir / f"{target_name}_vis.png",
                ]
                for vc in vis_candidates:
                    if vc.exists():
                        try:
                            with open(vc, 'rb') as img_f:
                                vis_b64 = base64.b64encode(img_f.read()).decode('utf-8')
                            break
                        except Exception:
                            pass
                graph_data = {
                    'nodes': nodes,
                    'edges': edges,
                    'places': places,
                    'total_nodes': len(nodes),
                    'total_edges': len(edges),
                    'vis_image_b64': vis_b64,
                    'map_name': target_name,
                }
                self._cached_graphs[target_name] = graph_data
                if target_name == self.active_map_name:
                    self._cached_graph = graph_data
                return graph_data
            except Exception as e:
                self.get_logger().error(f"Failed to load graph for {target_name}: {e}")
                return {'nodes': [], 'edges': [], 'places': [], 'total_nodes': 0, 'total_edges': 0, 'map_name': target_name}

        return {'nodes': [], 'edges': [], 'places': [], 'total_nodes': 0, 'total_edges': 0, 'map_name': target_name}

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
        dyn_obs = None
        if getattr(self, 'obstacle_pose', None):
            dyn_obs = dict(self.obstacle_pose)
            rx = self.pose.get('x', 0.0)
            ry = self.pose.get('y', 0.0)
            d_robot = math.hypot(dyn_obs['x'] - rx, dyn_obs['y'] - ry)
            dyn_obs['dist_to_robot'] = round(d_robot, 2)
            if d_robot < 0.85:
                dyn_obs['avoidance_status'] = 'TACTICAL_EVASION'
            elif d_robot < 1.5:
                dyn_obs['avoidance_status'] = 'PROXIMITY_SLOWDOWN'
            elif d_robot < 2.5:
                dyn_obs['avoidance_status'] = 'DETECTED'
            else:
                dyn_obs['avoidance_status'] = 'CLEAR'

        return {
            'pose':            self.pose,
            'velocity':        self.velocity,
            'min_obstacle_dist': round(obs_dist, 3) if obs_dist is not None else None,
            'nav_state':       self.nav_state,
            'path':            self.path_waypoints,
            'obstacle_alert':  self.obstacle_alert,
            'imu':             self.imu,
            'mission':         self.mission,
            'active_map':      getattr(self, 'active_map_name', 'warehouse_01'),
            'active_place_nav': getattr(self, 'active_place_nav', None),
            'dynamic_obstacle': dyn_obs,
            'scan':            scan_ds,
            'scan_angle_min':  round(self.scan_angle_min, 4),
            'scan_angle_inc':  round(self.scan_angle_inc, 4),
            'mapping':         mapping_mgr.get_status(self._has_live_map, self.get_live_map_info(), ros_node=self),
            'system_status':   system_mgr.get_status() if 'system_mgr' in globals() else {},
            'ts':              round(time.time(), 3),
        }

    # -------- Map PNG with robot overlay --------

    def get_map_image_b64(self, map_name: Optional[str] = None) -> Optional[str]:
        """Return base64-encoded JPEG of map PNG with robot dot overlaid for specified map."""
        target = (map_name or self.active_map_name or 'warehouse_01').strip()
        img_path = resolve_map_image_path(target)
        if not img_path or not img_path.exists():
            return None

        try:
            from PIL import Image, ImageDraw
            img = Image.open(img_path).convert('RGBA')
            draw = ImageDraw.Draw(img)

            # Robot position dot (red circle)
            cx, cy = world_to_pixel(self.pose['x'], self.pose['y'], map_name=target)
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
                pix_path = [world_to_pixel(p[0], p[1], map_name=target) for p in self.path_waypoints]
                draw.line(pix_path, fill=(255, 80, 220, 200), width=2)

            buf = io.BytesIO()
            img.convert('RGB').save(buf, format='JPEG', quality=82)
            return base64.b64encode(buf.getvalue()).decode()
        except Exception as e:
            self.get_logger().warning(f"Error drawing map overlay for {target}: {e}")
            try:
                with open(img_path, 'rb') as f:
                    return base64.b64encode(f.read()).decode()
            except Exception:
                return None

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

class SpawnObstacleRequest(BaseModel):
    x:   float
    y:   float
    yaw: float = 0.0
    name: str = 'dynamic_obstacle'

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

class SystemStartRequest(BaseModel):
    map_name: Optional[str] = 'warehouse_01'
    world: Optional[str] = None
    headless: bool = True
    use_rviz: bool = False

class StartMappingRequest(BaseModel):
    map_name: str = 'warehouse_01'
    world: str = 'test1.world'
    run_explore: bool = False
    headless: bool = True
    use_rviz: bool = False

class GuiLaunchRequest(BaseModel):
    app: str = "rviz"  # "rviz" | "gazebo"
    mode: Optional[str] = None  # "nav" | "explore"

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

class PlanRouteRequest(BaseModel):
    start_node: str
    goal_node: str
    map_name: Optional[str] = None

class NamedPlace(BaseModel):
    id: str
    name: str
    node_id: str
    type: Optional[str] = "place"
    x: float = 0.0
    y: float = 0.0
    px: int = 0
    py: int = 0
    theta: Optional[float] = 0.0
    icon: Optional[str] = "📍"
    color: Optional[str] = "#06b6d4"
    description: Optional[str] = ""


class SavePlacesRequest(BaseModel):
    map_name: Optional[str] = None
    places: List[NamedPlace]


class DispatchPlaceRequest(BaseModel):
    place_id: Optional[str] = None
    place_name: str
    node_id: str
    x: float
    y: float
    yaw: Optional[float] = 0.0
    map_name: Optional[str] = None


class DispatchSequenceRequest(BaseModel):
    places: List[DispatchPlaceRequest]
    map_name: Optional[str] = None


class SetActiveMapRequest(BaseModel):
    map_name: str


# -------- REST endpoints --------

@app.get('/api/status')
async def get_status():
    return node.telemetry_snapshot() if node else {'error': 'ROS node not ready'}

@app.get('/api/map/active')
async def get_active_map():
    curr = node.active_map_name if node else 'warehouse_01'
    return {'ok': True, 'active_map': curr}

@app.post('/api/map/active')
async def post_active_map(req: SetActiveMapRequest):
    if not node:
        return JSONResponse({'error': 'ROS node not ready'}, status_code=503)
    try:
        node.set_active_map(req.map_name)
        return {'ok': True, 'active_map': node.active_map_name}
    except Exception as e:
        return JSONResponse({'error': str(e)}, status_code=400)

@app.get('/api/map')
async def get_map(map_name: Optional[str] = None):
    if node is None:
        return JSONResponse({'error': 'ROS node not ready'}, status_code=503)
    target = (map_name or node.active_map_name or 'warehouse_01').strip()
    data = node.get_map_image_b64(target)
    if data is None:
        return JSONResponse({'error': f'Map image for {target} not found'}, status_code=404)
    return {'image': data, 'encoding': 'jpeg/base64', 'map_name': target}

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
async def get_map_metadata(map_name: Optional[str] = None):
    target = (map_name or (node.active_map_name if node else None) or 'warehouse_01').strip()
    return get_map_metadata_by_name(target)

@app.get('/api/map/raw')
async def get_map_raw(map_name: Optional[str] = None):
    """Serve the clean architectural floorplan (no graph overlay)."""
    target = (map_name or (node.active_map_name if node else None) or 'warehouse_01').strip()
    img_path = resolve_map_image_path(target)
    if not img_path or not img_path.exists():
        return JSONResponse({'error': f'Map image for {target} not found'}, status_code=404)
    return FileResponse(img_path, media_type='image/png')

@app.get('/api/map/clean')
async def get_map_clean(map_name: Optional[str] = None):
    """Alias for /api/map/raw — explicitly returns the clean floorplan."""
    return await get_map_raw(map_name)

@app.get('/api/graph')
async def get_graph(map_name: Optional[str] = None):
    if node is None:
        return JSONResponse({'error': 'ROS node not ready'}, status_code=503)
    target = (map_name or node.active_map_name or 'warehouse_01').strip()
    return node.get_graph_data(target)

@app.post('/api/obstacle/cmd_vel')
async def post_obstacle_cmd_vel(req: CmdVelRequest):
    if node:
        node.publish_obstacle_cmd_vel(req.linear, req.angular)
    return {'ok': True}

@app.post('/api/obstacle/spawn')
async def post_obstacle_spawn(req: SpawnObstacleRequest):
    if not node:
        return JSONResponse({'error': 'ROS node not ready'}, status_code=503)
    return node.spawn_or_move_obstacle(req.x, req.y, req.yaw)

@app.post('/api/obstacle/remove')
async def post_obstacle_remove():
    if not node:
        return JSONResponse({'error': 'ROS node not ready'}, status_code=503)
    return node.remove_obstacle()

@app.get('/api/obstacle/status')
async def get_obstacle_status():
    if not node:
        return JSONResponse({'error': 'ROS node not ready'}, status_code=503)
    return {'ok': True, 'obstacle': getattr(node, 'obstacle_pose', None)}

# -------- Mapping REST endpoints (Phase 1) --------

@app.get('/api/mapping/status')
async def get_mapping_status():
    has_map = node._has_live_map if node else False
    map_info = node.get_live_map_info() if node else {}
    return mapping_mgr.get_status(has_map, map_info, ros_node=node)

@app.post('/api/mapping/start')
async def post_mapping_start(req: StartMappingRequest = StartMappingRequest()):
    mapping_mgr.start(
        map_name=req.map_name,
        world=req.world,
        run_explore=req.run_explore,
        headless=req.headless,
        use_rviz=req.use_rviz
    )
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
    curr_active = node.active_map_name if node else 'warehouse_01'
    if maps_dir.exists():
        for yaml_file in sorted(maps_dir.glob("*.yaml")):
            name = yaml_file.stem
            pgm_file = maps_dir / f"{name}.pgm"
            png_file = maps_dir / f"{name}.png"
            graph_file = maps_dir / f"{name}_graph.json"
            places_file = maps_dir / f"{name}_places.json"
            meta = get_map_metadata_by_name(name)
            
            node_count = 0
            if graph_file.exists():
                try:
                    with open(graph_file, 'r', encoding='utf-8') as gf:
                        gd = json.load(gf)
                        node_count = len(gd.get('nodes', []))
                except Exception:
                    pass
            
            place_count = 0
            if places_file.exists():
                try:
                    with open(places_file, 'r', encoding='utf-8') as pf:
                        pd = json.load(pf)
                        place_count = len(pd) if isinstance(pd, list) else 0
                except Exception:
                    pass

            has_png = png_file.exists()
            maps.append({
                'name': name,
                'yaml': str(yaml_file.relative_to(_WORKSPACE_ROOT)),
                'has_pgm': pgm_file.exists(),
                'has_png': has_png,
                'has_graph': graph_file.exists(),
                'node_count': node_count,
                'place_count': place_count,
                'resolution': meta['resolution'],
                'dimensions': f"{meta['width']}×{meta['height']}",
                'size_kb': round(yaml_file.stat().st_size / 1024, 2),
                'is_active': (name == curr_active)
            })
    return {'maps': maps, 'active_map': curr_active}

@app.post('/api/mapping/graph/extract')
async def post_extract_graph(req: ExtractGraphRequest = ExtractGraphRequest()):
    """Phase 3: Extracts topological navigation roadmap (nodes & edges) from map YAML."""
    if graph_extractor is None:
        return JSONResponse({'error': 'Graph extractor module not available'}, status_code=500)

    target_name = (req.map_name or (node.active_map_name if node else None) or mapping_mgr.saved_map_name or mapping_mgr.map_name or 'warehouse_01').strip()
    maps_dir = _WORKSPACE_ROOT / "src" / "agv_description" / "maps"

    yaml_path = maps_dir / f"{target_name}.yaml"
    if not yaml_path.exists():
        if target_name in ('warehouse_map', 'warehouse'):
            yaml_path = maps_dir / "warehouse_map.yaml"
        elif target_name == 'warehouse_01':
            yaml_path = maps_dir / "warehouse_01.yaml"

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

    # Fallback to reading active map graph or warehouse_map graph
    fallback_graph = maps_dir / f"{getattr(node, 'active_map_name', 'warehouse_01')}_graph.json"
    if not fallback_graph.exists():
        fallback_graph = _GRAPH_JSON_PATH
    if fallback_graph.exists():
        try:
            with open(fallback_graph, 'r', encoding='utf-8') as f:
                raw = json.load(f)
            vis_b64 = None
            vis_candidates = [
                maps_dir / f"{fallback_graph.stem.replace('_graph', '')}_graph_vis.png",
                maps_dir / "warehouse_01_graph_vis.png",
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

@app.get('/api/mapping/graph/verify')
async def get_graph_verification(map_name: Optional[str] = None):
    """Phase 4: Performs topological, clearance, and routing audits on the graph roadmap."""
    maps_dir = _WORKSPACE_ROOT / "src" / "agv_description" / "maps"
    target_name = (map_name or (node.active_map_name if node else None) or mapping_mgr.saved_map_name or mapping_mgr.map_name or 'warehouse_01').strip()
    cand = maps_dir / f"{target_name}_graph.json"
    if not cand.exists():
        cand = maps_dir / f"{target_name}.json"
    if not cand.exists():
        return JSONResponse({'error': f'Graph roadmap "{target_name}_graph.json" not found to verify'}, status_code=404)
    target_json = cand

    try:
        with open(target_json, 'r', encoding='utf-8') as f:
            raw = json.load(f)

        nodes = raw.get('nodes', [])
        edges = raw.get('edges', [])
        num_nodes = len(nodes)
        num_edges = len(edges)

        if num_nodes == 0:
            return JSONResponse({'error': 'Graph contains 0 nodes'}, status_code=400)

        node_map = {n['id']: n for n in nodes}
        adj = {n['id']: [] for n in nodes}

        for e in edges:
            u = e['from']
            v = e['to']
            cost = float(e.get('cost', e.get('weight', 1.0)))
            if u in adj and v in adj:
                adj[u].append((v, cost))
                adj[v].append((u, cost))

        # Degree calculations
        degrees = {nid: len(nbrs) for nid, nbrs in adj.items()}
        isolated_nodes = [nid for nid, deg in degrees.items() if deg == 0]
        leaf_nodes = [nid for nid, deg in degrees.items() if deg == 1]
        avg_deg = round(sum(degrees.values()) / max(1, num_nodes), 2)
        min_deg = min(degrees.values()) if degrees else 0
        max_deg = max(degrees.values()) if degrees else 0

        # Connected components via BFS
        visited = set()
        components = []
        for n in adj:
            if n not in visited:
                comp = []
                q = deque([n])
                visited.add(n)
                while q:
                    curr = q.popleft()
                    comp.append(curr)
                    for nb, _ in adj[curr]:
                        if nb not in visited:
                            visited.add(nb)
                            q.append(nb)
                components.append(comp)

        components.sort(key=len, reverse=True)
        num_components = len(components)
        lcc_size = len(components[0]) if components else 0
        connectivity_pct = round((lcc_size / max(1, num_nodes)) * 100, 1)

        # Clearances
        clearances = [float(n.get('clearance', 0.0)) for n in nodes if 'clearance' in n]
        min_c = min(clearances) if clearances else 0.0
        avg_c = round(sum(clearances) / max(1, len(clearances)), 2) if clearances else 0.0
        low_clearance_nodes = [n['id'] for n in nodes if float(n.get('clearance', 0.0)) < 0.18]

        # Automated Navigability Probes (Dijkstra)
        total_probes = 0
        passed_probes = 0
        probe_routes = []

        if lcc_size >= 2:
            main_comp = components[0]
            step = max(1, len(main_comp) // 6)
            test_pairs = []
            for i in range(min(5, len(main_comp) // 2)):
                test_pairs.append((main_comp[i * step], main_comp[-(i * step + 1)]))

            for s, g_node in test_pairs:
                total_probes += 1
                dist = {nid: float('inf') for nid in adj}
                dist[s] = 0
                prev = {nid: None for nid in adj}
                pq = [(0, s)]
                found = False
                while pq:
                    d, u = heapq.heappop(pq)
                    if u == g_node:
                        found = True
                        break
                    if d > dist[u]:
                        continue
                    for v, w in adj[u]:
                        if d + w < dist[v]:
                            dist[v] = d + w
                            prev[v] = u
                            heapq.heappush(pq, (dist[v], v))
                if found:
                    passed_probes += 1
                    path = []
                    curr = g_node
                    while curr:
                        path.append(curr)
                        curr = prev[curr]
                    path.reverse()
                    probe_routes.append({
                        'start': s,
                        'goal': g_node,
                        'distance_m': round(dist[g_node], 2),
                        'hops': len(path) - 1,
                        'path': path[:8] + (['...'] if len(path) > 8 else [])
                    })

        navigability_pct = round((passed_probes / max(1, total_probes)) * 100, 1)

        checks = [
            {
                'name': 'Component Connectivity',
                'passed': num_components == 1,
                'status': 'PASS' if num_components == 1 else 'WARN',
                'detail': f'{lcc_size}/{num_nodes} nodes in main component ({connectivity_pct}%)'
            },
            {
                'name': 'Obstacle Clearance Safety',
                'passed': len(low_clearance_nodes) == 0,
                'status': 'PASS' if len(low_clearance_nodes) == 0 else 'WARN',
                'detail': f'Min clearance {min_c:.2f}m (Safe threshold: 0.18m)'
            },
            {
                'name': 'Dijkstra Navigability Probes',
                'passed': navigability_pct >= 95.0,
                'status': 'PASS' if navigability_pct >= 95.0 else 'WARN',
                'detail': f'{passed_probes}/{total_probes} sample mission routes verified ({navigability_pct}%)'
            },
            {
                'name': 'Network Density & LOS Quality',
                'passed': avg_deg >= 4.0,
                'status': 'PASS' if avg_deg >= 4.0 else 'WARN',
                'detail': f'Average {avg_deg} LOS edges per node (min: {min_deg}, max: {max_deg})'
            }
        ]

        is_verified = all(c['passed'] for c in checks)

        return {
            'ok': True,
            'status': 'VERIFIED' if is_verified else 'WARNING',
            'map_name': target_name,
            'json_file': str(target_json.relative_to(_WORKSPACE_ROOT)),
            'metrics': {
                'total_nodes': num_nodes,
                'total_edges': num_edges,
                'connected_components': num_components,
                'lcc_size': lcc_size,
                'connectivity_pct': connectivity_pct,
                'isolated_nodes_count': len(isolated_nodes),
                'leaf_nodes_count': len(leaf_nodes),
                'avg_degree': avg_deg,
                'min_degree': min_deg,
                'max_degree': max_deg,
                'min_clearance_m': min_c,
                'avg_clearance_m': avg_c,
                'navigability_pct': navigability_pct,
                'total_probes': total_probes,
                'passed_probes': passed_probes
            },
            'checks': checks,
            'probe_routes': probe_routes
        }
    except Exception as e:
        return JSONResponse({'error': f'Verification failed: {str(e)}'}, status_code=500)

@app.post('/api/mapping/graph/plan_route')
async def post_plan_route(req: PlanRouteRequest):
    """Phase 4: Computes shortest path and waypoint metrics between two roadmap nodes."""
    maps_dir = _WORKSPACE_ROOT / "src" / "agv_description" / "maps"
    target_name = (req.map_name or (node.active_map_name if node else None) or mapping_mgr.saved_map_name or mapping_mgr.map_name or 'warehouse_01').strip()
    cand = maps_dir / f"{target_name}_graph.json"
    if not cand.exists():
        cand = maps_dir / f"{target_name}.json"
    if not cand.exists():
        return JSONResponse({'error': f'Graph roadmap "{target_name}_graph.json" not found for route planning'}, status_code=404)
    target_json = cand

    try:
        with open(target_json, 'r', encoding='utf-8') as f:
            raw = json.load(f)

        nodes = raw.get('nodes', [])
        edges = raw.get('edges', [])
        node_map = {n['id']: n for n in nodes}

        if req.start_node not in node_map:
            return JSONResponse({'error': f'Start node "{req.start_node}" not found'}, status_code=400)
        if req.goal_node not in node_map:
            return JSONResponse({'error': f'Goal node "{req.goal_node}" not found'}, status_code=400)

        adj = {n['id']: [] for n in nodes}
        for e in edges:
            u = e['from']
            v = e['to']
            cost = float(e.get('cost', e.get('weight', 1.0)))
            if u in adj and v in adj:
                adj[u].append((v, cost))
                adj[v].append((u, cost))

        dist = {nid: float('inf') for nid in adj}
        dist[req.start_node] = 0
        prev = {nid: None for nid in adj}
        pq = [(0, req.start_node)]
        found = False

        while pq:
            d, u = heapq.heappop(pq)
            if u == req.goal_node:
                found = True
                break
            if d > dist[u]:
                continue
            for v, w in adj[u]:
                if d + w < dist[v]:
                    dist[v] = d + w
                    prev[v] = u
                    heapq.heappush(pq, (dist[v], v))

        if not found or dist[req.goal_node] == float('inf'):
            return JSONResponse({'error': f'No reachable route between {req.start_node} and {req.goal_node}'}, status_code=404)

        path = []
        curr = req.goal_node
        while curr:
            path.append(curr)
            curr = prev[curr]
        path.reverse()

        waypoints = []
        clearances = []
        for nid in path:
            n = node_map[nid]
            pt = n.get('point', [0, 0])
            px = pt[0] if isinstance(pt, (list, tuple)) and len(pt) >= 2 else n.get('px', 0)
            py = pt[1] if isinstance(pt, (list, tuple)) and len(pt) >= 2 else n.get('py', 0)
            c = float(n.get('clearance', 0.0))
            clearances.append(c)
            waypoints.append({
                'id': nid,
                'x': round(float(n.get('x', n.get('wx', 0.0))), 3),
                'y': round(float(n.get('y', n.get('wy', 0.0))), 3),
                'px': px,
                'py': py,
                'clearance': round(c, 3)
            })

        tot_dist = round(dist[req.goal_node], 2)
        return {
            'ok': True,
            'start_node': req.start_node,
            'goal_node': req.goal_node,
            'path': path,
            'waypoints': waypoints,
            'total_distance_m': tot_dist,
            'hop_count': len(path) - 1,
            'est_time_sec': round(tot_dist / 0.4, 1),
            'min_clearance_m': round(min(clearances), 3) if clearances else 0.0
        }
    except Exception as e:
        return JSONResponse({'error': f'Routing calculation failed: {str(e)}'}, status_code=500)

# -------- Phase 5: Name Places / Nodes Endpoints --------

@app.get('/api/mapping/places')
async def get_named_places(map_name: Optional[str] = None):
    """Phase 5: Retrieve all named places and stations for a roadmap."""
    maps_dir = _WORKSPACE_ROOT / "src" / "agv_description" / "maps"
    target_name = (map_name or (node.active_map_name if node else None) or mapping_mgr.saved_map_name or mapping_mgr.map_name or 'warehouse_01').strip()
    
    # Strictly isolated search candidates for target_name only:
    candidates = [
        maps_dir / f"{target_name}_places.json",
        maps_dir / f"{target_name}_graph.json",
    ]
    
    for cand in candidates:
        if cand.exists():
            try:
                with open(cand, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                if isinstance(data, list):
                    return {'ok': True, 'map_name': target_name, 'places': data, 'source_file': cand.name}
                elif isinstance(data, dict) and 'places' in data and isinstance(data['places'], list):
                    return {'ok': True, 'map_name': target_name, 'places': data['places'], 'source_file': cand.name}
            except Exception as e:
                logger.warning(f"Failed to read places from {cand}: {e}")

    return {'ok': True, 'map_name': target_name, 'places': [], 'source_file': None}


@app.post('/api/mapping/places/save')
async def post_save_named_places(req: SavePlacesRequest):
    """Phase 5: Persist named places into active graph and dedicated places files."""
    maps_dir = _WORKSPACE_ROOT / "src" / "agv_description" / "maps"
    active_now = node.active_map_name if node else 'warehouse_01'
    target_name = (req.map_name or active_now).strip()
    places_data = [p.dict() for p in req.places]
    
    saved_files = []
    
    # 1. Save dedicated <target_name>_places.json
    places_file = maps_dir / f"{target_name}_places.json"
    with open(places_file, 'w', encoding='utf-8') as f:
        json.dump(places_data, f, indent=2)
    saved_files.append(str(places_file.relative_to(_WORKSPACE_ROOT)))
    
    # 2. Inject places into corresponding <target_name>_graph.json ONLY
    target_graph = maps_dir / f"{target_name}_graph.json"
    if target_graph.exists():
        try:
            with open(target_graph, 'r', encoding='utf-8') as f:
                raw_graph = json.load(f)
            if isinstance(raw_graph, dict):
                raw_graph['places'] = places_data
                with open(target_graph, 'w', encoding='utf-8') as f:
                    json.dump(raw_graph, f, indent=2)
                saved_files.append(str(target_graph.relative_to(_WORKSPACE_ROOT)))
                if node and hasattr(node, '_cached_graphs') and target_name in node._cached_graphs:
                    node._cached_graphs[target_name]['places'] = places_data
                if node and hasattr(node, '_cached_graph') and node.active_map_name == target_name:
                    if isinstance(node._cached_graph, dict):
                        node._cached_graph['places'] = places_data
        except Exception as e:
            logger.error(f"Error injecting places into {target_graph}: {e}")

    return {
        'ok': True,
        'map_name': target_name,
        'count': len(places_data),
        'places': places_data,
        'saved_files': list(set(saved_files))
    }


@app.post('/api/mapping/places/default_templates')
async def post_generate_default_templates(map_name: Optional[str] = None):
    """Phase 5: Auto-generate intelligent default warehouse stations based on roadmap topology."""
    maps_dir = _WORKSPACE_ROOT / "src" / "agv_description" / "maps"
    target_name = (map_name or (node.active_map_name if node else None) or mapping_mgr.saved_map_name or mapping_mgr.map_name or 'warehouse_01').strip()
    
    # Load roadmap nodes strictly from target map
    target_graph = maps_dir / f"{target_name}_graph.json"
    if not target_graph.exists():
        return JSONResponse({'error': f'Graph roadmap "{target_name}_graph.json" not found to generate place templates'}, status_code=404)

    try:
        with open(target_graph, 'r', encoding='utf-8') as f:
            raw = json.load(f)
        nodes = raw.get('nodes', [])
        if not nodes:
            return JSONResponse({'error': 'Graph contains no nodes'}, status_code=400)
            
        total_n = len(nodes)
        
        n0 = nodes[0]
        idx_in = min(total_n - 1, max(1, total_n // 4))
        n_in = nodes[idx_in]
        idx_mid = min(total_n - 1, max(1, total_n // 2))
        n_mid = nodes[idx_mid]
        idx_out = min(total_n - 1, max(1, (3 * total_n) // 4))
        n_out = nodes[idx_out]
        idx_last = total_n - 1
        n_last = nodes[idx_last]
        
        templates = [
            {
                'id': 'place_charging_dock',
                'name': 'Home Charging Station',
                'node_id': n0['id'],
                'type': 'charging',
                'x': round(float(n0.get('x', n0.get('wx', 0.0))), 3),
                'y': round(float(n0.get('y', n0.get('wy', 0.0))), 3),
                'px': n0.get('px', 0),
                'py': n0.get('py', 0),
                'theta': 0.0,
                'icon': '⚡',
                'color': '#10b981',
                'description': 'Primary automated battery dock and fleet base'
            },
            {
                'id': 'place_inbound_pickup',
                'name': 'Inbound Intake Bay',
                'node_id': n_in['id'],
                'type': 'pickup',
                'x': round(float(n_in.get('x', n_in.get('wx', 0.0))), 3),
                'y': round(float(n_in.get('y', n_in.get('wy', 0.0))), 3),
                'px': n_in.get('px', 0),
                'py': n_in.get('py', 0),
                'theta': 1.57,
                'icon': '📦',
                'color': '#38bdf8',
                'description': 'Receiving dock for incoming materials'
            },
            {
                'id': 'place_assembly_staging',
                'name': 'Assembly WIP Staging',
                'node_id': n_mid['id'],
                'type': 'staging',
                'x': round(float(n_mid.get('x', n_mid.get('wx', 0.0))), 3),
                'y': round(float(n_mid.get('y', n_mid.get('wy', 0.0))), 3),
                'px': n_mid.get('px', 0),
                'py': n_mid.get('py', 0),
                'theta': 3.14,
                'icon': '🏢',
                'color': '#a855f7',
                'description': 'Intermediate buffer staging for sub-assemblies'
            },
            {
                'id': 'place_outbound_dispatch',
                'name': 'Outbound Dispatch Bay',
                'node_id': n_out['id'],
                'type': 'dropoff',
                'x': round(float(n_out.get('x', n_out.get('wx', 0.0))), 3),
                'y': round(float(n_out.get('y', n_out.get('wy', 0.0))), 3),
                'px': n_out.get('px', 0),
                'py': n_out.get('py', 0),
                'theta': 4.71,
                'icon': '📤',
                'color': '#f59e0b',
                'description': 'Outbound packaging and pallet dispatch area'
            },
            {
                'id': 'place_quality_waypoint',
                'name': 'Inspection Checkpoint',
                'node_id': n_last['id'],
                'type': 'waypoint',
                'x': round(float(n_last.get('x', n_last.get('wx', 0.0))), 3),
                'y': round(float(n_last.get('y', n_last.get('wy', 0.0))), 3),
                'px': n_last.get('px', 0),
                'py': n_last.get('py', 0),
                'theta': 0.0,
                'icon': '🎯',
                'color': '#ec4899',
                'description': 'Automated optical inspection and barcode scanning'
            }
        ]
        
        return {'ok': True, 'templates': templates, 'map_name': target_name, 'total_nodes': total_n}
    except Exception as e:
        return JSONResponse({'error': f'Failed to generate templates: {str(e)}'}, status_code=500)


@app.delete('/api/mapping/places/{place_id}')
async def delete_named_place(place_id: str, map_name: Optional[str] = None):
    """Phase 5: Remove a named place from the saved places list."""
    maps_dir = _WORKSPACE_ROOT / "src" / "agv_description" / "maps"
    target_name = (map_name or (node.active_map_name if node else None) or mapping_mgr.saved_map_name or mapping_mgr.map_name or 'warehouse_01').strip()
    
    places_file = maps_dir / f"{target_name}_places.json"
    if not places_file.exists():
        return JSONResponse({'error': f'No places file found for map "{target_name}"'}, status_code=404)
        
    try:
        with open(places_file, 'r', encoding='utf-8') as f:
            places = json.load(f)
            
        orig_len = len(places)
        places = [p for p in places if p.get('id') != place_id]
        
        if len(places) == orig_len:
            return JSONResponse({'error': f'Place ID "{place_id}" not found in map "{target_name}"'}, status_code=404)
            
        req = SavePlacesRequest(map_name=target_name, places=[NamedPlace(**p) for p in places])
        return await post_save_named_places(req)
    except Exception as e:
        return JSONResponse({'error': f'Delete failed: {str(e)}'}, status_code=500)


# ---------------------------------------------------------------------------
# Phase 6: Navigation Using Place Names REST Endpoints
# ---------------------------------------------------------------------------

@app.post('/api/mapping/navigation/dispatch_place')
async def post_dispatch_place(req: DispatchPlaceRequest):
    """Phase 6: Dispatch AMR to a named place using its topological node and world coordinates."""
    if node is None:
        return JSONResponse({'error': 'ROS node not ready'}, status_code=503)

    if req.map_name and req.map_name != node.active_map_name:
        try:
            node.set_active_map(req.map_name)
        except Exception as e:
            logger.warning(f"Could not switch active map to {req.map_name}: {e}")

    # 1. Publish to /goal_sequence with the assigned node_id
    node.publish_goal_sequence([req.node_id])

    # 2. Also publish to /goal_pose as standard Nav2 / AMCL pose fallback
    node.publish_goal(req.x, req.y, req.yaw or 0.0)

    # 3. Track active place navigation
    node.active_place_nav = {
        'target_place': req.place_name,
        'target_node': req.node_id,
        'target_x': float(req.x),
        'target_y': float(req.y),
        'status': 'NAVIGATING',
        'is_sequence': False,
        'dispatched_at': time.time(),
        'total_stops': 1,
        'current_stop': 1,
    }
    logger.info(f"Dispatched AMR to place '{req.place_name}' (node {req.node_id}) at ({req.x:.2f}, {req.y:.2f})")
    return {
        'ok': True,
        'target_place': req.place_name,
        'node_id': req.node_id,
        'x': req.x,
        'y': req.y,
        'status': 'NAVIGATING'
    }


@app.post('/api/mapping/navigation/dispatch_sequence')
async def post_dispatch_sequence(req: DispatchSequenceRequest):
    """Phase 6: Dispatch AMR to a multi-stop sequence of named places."""
    if node is None:
        return JSONResponse({'error': 'ROS node not ready'}, status_code=503)

    if not req.places:
        return JSONResponse({'error': 'Places sequence cannot be empty'}, status_code=400)

    if req.map_name and req.map_name != node.active_map_name:
        try:
            node.set_active_map(req.map_name)
        except Exception as e:
            logger.warning(f"Could not switch active map to {req.map_name}: {e}")

    node_ids = [p.node_id for p in req.places]
    node.publish_goal_sequence(node_ids)

    # First stop
    first = req.places[0]
    node.publish_goal(first.x, first.y, first.yaw or 0.0)

    node.active_place_nav = {
        'target_place': first.place_name,
        'target_node': first.node_id,
        'target_x': float(first.x),
        'target_y': float(first.y),
        'sequence': [p.dict() for p in req.places],
        'total_stops': len(req.places),
        'current_stop': 1,
        'status': 'NAVIGATING',
        'is_sequence': True,
        'dispatched_at': time.time(),
    }
    names = ' -> '.join(p.place_name for p in req.places)
    logger.info(f"Dispatched AMR multi-stop mission [{names}] ({len(req.places)} stops)")
    return {
        'ok': True,
        'total_stops': len(req.places),
        'first_target': first.place_name,
        'status': 'NAVIGATING'
    }


@app.post('/api/mapping/navigation/cancel')
async def post_cancel_navigation():
    """Phase 6: Cancel active navigation, halt robot motors, and clear mission queue."""
    if node:
        node.publish_cmd_vel(0.0, 0.0)
        node.publish_goal_sequence([])
        node.path_waypoints = []
        if hasattr(node, 'active_place_nav') and node.active_place_nav:
            node.active_place_nav['status'] = 'CANCELLED'
    return {'ok': True, 'status': 'CANCELLED'}


@app.get('/api/mapping/navigation/status')
async def get_navigation_status():
    """Phase 6: Return real-time navigation status, active place destination, and distance."""
    if node is None:
        return JSONResponse({'error': 'ROS node not ready'}, status_code=503)

    active = getattr(node, 'active_place_nav', None)
    dist = None
    if active and 'target_x' in active and 'target_y' in active:
        dist = math.hypot(node.pose['x'] - active['target_x'], node.pose['y'] - active['target_y'])

    return {
        'ok': True,
        'nav_state': node.nav_state,
        'pose': node.pose,
        'active_nav': active,
        'distance_to_target_m': round(dist, 2) if dist is not None else None,
        'mission': node.mission,
    }


# ---------------------------------------------------------------------------
# Navigation System Stack Master Control Endpoints
# ---------------------------------------------------------------------------

@app.post('/api/system/start')
async def post_system_start(req: SystemStartRequest):
    """Start full AMR navigation stack (navigation_launch + route_runner + bt_manager)."""
    curr_map = (req.map_name or (node.active_map_name if node else None) or 'warehouse_01').strip()
    if node:
        node.set_active_map(curr_map)
    return system_mgr.start(map_name=curr_map, world=req.world, headless=req.headless, use_rviz=req.use_rviz)


@app.post('/api/system/stop')
async def post_system_stop():
    """Stop the full AMR navigation stack cleanly."""
    return system_mgr.stop()


@app.get('/api/system/status')
async def get_system_status():
    """Get current lifecycle status of the navigation stack."""
    return system_mgr.get_status()


@app.post('/api/gui/launch')
async def post_gui_launch(req: GuiLaunchRequest):
    """Launch Gazebo GUI or RViz2 visualizer client in the current display."""
    target_app = req.app.lower().strip()
    env = os.environ.copy()
    env['DISPLAY'] = os.environ.get('DISPLAY', ':0')
    env['LIBGL_ALWAYS_SOFTWARE'] = '1'
    env['GALLIUM_DRIVER'] = 'llvmpipe'
    env['MESA_GL_VERSION_OVERRIDE'] = '4.5'

    if target_app in ('gazebo', 'gz'):
        script = _WORKSPACE_ROOT / "view_gazebo.sh"
        cmd = f"/bin/bash {script}"
    else:
        mode_arg = req.mode or ("explore" if mapping_mgr.state == 'MAPPING' else "nav")
        script = _WORKSPACE_ROOT / "view_rviz.sh"
        cmd = f"/bin/bash {script} {mode_arg}"

    logger.info(f"[bridge] Launching GUI: {cmd}")
    try:
        proc = subprocess.Popen(
            cmd,
            shell=True,
            executable='/bin/bash',
            cwd=str(_WORKSPACE_ROOT),
            env=env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid
        )
        return {'ok': True, 'pid': proc.pid, 'app': target_app}
    except Exception as e:
        logger.error(f"[bridge] Error launching GUI: {e}")
        return JSONResponse({'ok': False, 'error': str(e)}, status_code=500)


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
        os.environ['RCUTILS_LOGGING_SEVERITY_THRESHOLD'] = 'ERROR'
        if not rclpy.ok():
            rclpy.init(args=sys.argv)
        try:
            rclpy.logging.set_logger_level('', rclpy.logging.LoggingSeverity.ERROR)
        except Exception:
            pass
        node = AmrBridgeNode()
        logger.info("[bridge] AmrBridgeNode initialized and ready.")
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
