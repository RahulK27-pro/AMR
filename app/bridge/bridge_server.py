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

import argparse
import asyncio
import base64
import io
import json
import math
import os
import re
import shlex
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

_WS_EARLY = Path(os.environ.get("AMR_WS", Path.home() / "AMR" / "AMR-main"))


def _parse_runtime_args():
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--mode", default=os.environ.get("AMR_MODE", "sim"))
    parser.add_argument("--pi-ip", dest="pi_ip", default=os.environ.get("AMR_PI_IP", "10.244.248.208"))
    known, _ = parser.parse_known_args()
    mode = str(known.mode or "sim").strip().lower()
    if mode in ("hw", "hardware"):
        mode = "hardware"
    else:
        mode = "sim"
    pi_ip = str(known.pi_ip or "10.244.248.208").strip()
    return mode, pi_ip


def _detect_local_ip_for_peer(peer_ip: str) -> str:
    try:
        res = subprocess.run(
            ["ip", "route", "get", peer_ip],
            capture_output=True, text=True, timeout=1.5
        )
        if res.returncode == 0:
            tokens = res.stdout.split()
            for idx, tok in enumerate(tokens):
                if tok == "src" and idx + 1 < len(tokens):
                    return tokens[idx + 1]
    except Exception:
        pass
    try:
        res = subprocess.run(["hostname", "-I"], capture_output=True, text=True, timeout=1.5)
        if res.returncode == 0 and res.stdout.strip():
            return res.stdout.strip().split()[0]
    except Exception:
        pass
    return ""


def _write_cyclonedds_xml(ws_root: Path, pi_ip: str) -> Path:
    local_ip = _detect_local_ip_for_peer(pi_ip)
    net_iface_xml = f'<NetworkInterface address="{local_ip}"/>' if local_ip else '<NetworkInterface autodetermine="true"/>'
    xml_path = ws_root / "cyclonedds.xml"
    xml_body = (
        '<?xml version="1.0" encoding="UTF-8" ?>\n'
        '<CycloneDDS xmlns="https://cdds.io/config">\n'
        '  <Domain Id="7">\n'
        '    <General>\n'
        '      <Interfaces>\n'
        f'        {net_iface_xml}\n'
        '      </Interfaces>\n'
        '    </General>\n'
        '    <Discovery>\n'
        '      <Peers>\n'
        '        <Peer Address="127.0.0.1"/>\n'
        f'        <Peer Address="{pi_ip}"/>\n'
        '      </Peers>\n'
        '    </Discovery>\n'
        '  </Domain>\n'
        '</CycloneDDS>\n'
    )
    xml_path.write_text(xml_body, encoding="utf-8")
    return xml_path


def _configure_dds_environment(mode: str, pi_ip: str):
    os.environ["RCUTILS_LOGGING_SEVERITY_THRESHOLD"] = "ERROR"
    os.environ["AMR_MODE"] = mode
    os.environ["AMR_PI_IP"] = pi_ip
    os.environ["ROS_DOMAIN_ID"] = "7"
    os.environ["RMW_IMPLEMENTATION"] = os.environ.get("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
    if mode == "hardware":
        os.environ.pop("ROS_LOCALHOST_ONLY", None)
        try:
            xml_path = _write_cyclonedds_xml(_WS_EARLY, pi_ip)
            os.environ["CYCLONEDDS_URI"] = f"file://{xml_path}"
        except Exception as e:
            logger.warning(f"[bridge] Failed to write cyclonedds.xml: {e}")
    else:
        os.environ["ROS_LOCALHOST_ONLY"] = "1"
        os.environ.pop("CYCLONEDDS_URI", None)


AMR_MODE, AMR_PI_IP = _parse_runtime_args()
AMR_PI_USER = os.environ.get("AMR_PI_USER", "amr").strip() or "amr"
AMR_PI_DIR = os.environ.get("AMR_PI_DIR", "~/AMR").strip() or "~/AMR"
_configure_dds_environment(AMR_MODE, AMR_PI_IP)
_PENDING_REEXEC: Optional[Tuple[str, str]] = None
_UVICORN_SERVER: Optional[Any] = None
_SHUTDOWN_FLAG = threading.Event()


from PIL import Image, ImageDraw
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Odometry, Path as NavPath, OccupancyGrid
from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import String, Bool
try:
    from std_srvs.srv import Trigger
    _STD_SRVS_AVAILABLE = True
except ImportError:
    _STD_SRVS_AVAILABLE = False

try:
    from action_msgs.srv import CancelGoal
    _CANCEL_GOAL_AVAILABLE = True
except ImportError:
    _CANCEL_GOAL_AVAILABLE = False

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

from fastapi import Body, FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from fastapi.responses import JSONResponse, FileResponse
from pydantic import BaseModel
import uvicorn

# ---------------------------------------------------------------------------
# Config — unified map paths (simulation + hardware coexistence)
# ---------------------------------------------------------------------------
_WORKSPACE_ROOT = Path(os.environ.get("AMR_WS", Path.home() / "AMR" / "AMR-main"))
_SIM_MAPS_DIR   = _WORKSPACE_ROOT / "src" / "agv_description" / "maps"
_HW_MAPS_DIR    = _WORKSPACE_ROOT / "maps"
_APP_MAPS_DIR   = _WORKSPACE_ROOT / "app" / "src" / "agv_description" / "maps"
_DARK_CACHE_DIR = Path("/tmp/amr_map_dark_cache")

# Floorplan image and graph defaults
_MAP_PNG_PATH       = _SIM_MAPS_DIR / "warehouse_map.png"
_MAP_PNG_GRAPH_PATH = _SIM_MAPS_DIR / "warehouse_map_graph_vis.png"
_GRAPH_JSON_PATH    = _SIM_MAPS_DIR / "warehouse_map_graph.json"

# Calibrated Speed Profiles (matching teleop_wasd.py & hardware_navigation.launch.py)
SIM_TELEOP_LIMITS = {
    'max_linear': 0.80,
    'min_linear': 0.05,
    'default_linear': 0.64,
    'linear_step': 0.05,
    'max_angular': 1.80,
    'min_angular': 0.20,
    'default_angular': 1.44,
    'angular_step': 0.15,
}
HW_TELEOP_LIMITS = {
    'max_linear': 0.45,      # teleop_wasd.py MAX_LINEAR_SPEED
    'min_linear': 0.04,      # teleop_wasd.py MIN_LINEAR_SPEED
    'default_linear': 0.18,  # teleop_wasd.py DEFAULT_LINEAR
    'linear_step': 0.02,     # teleop_wasd.py LINEAR_STEP
    'max_angular': 2.00,     # teleop_wasd.py MAX_ANGULAR_SPEED
    'min_angular': 0.20,     # teleop_wasd.py MIN_ANGULAR_SPEED
    'default_angular': 0.80, # teleop_wasd.py DEFAULT_ANGULAR
    'angular_step': 0.10,    # teleop_wasd.py ANGULAR_STEP
    'presets': [
        {'label': 'Slow', 'linear': 0.10, 'angular': 0.50},
        {'label': 'Normal', 'linear': 0.20, 'angular': 0.80},
        {'label': 'Fast', 'linear': 0.35, 'angular': 1.40},
    ],
}
# Hardware SLAM mapping teleop limits calibrated for ~10 Hz YDLIDAR X2 + 35% PWM cap
HW_MAPPING_TELEOP_LIMITS = {
    'max_linear': 0.20,
    'max_reverse_linear': 0.12,
    'min_linear': 0.04,
    'default_linear': 0.13,
    'linear_step': 0.02,
    'max_angular': 0.50,
    'min_angular': 0.20,
    'default_angular': 0.35,
    'angular_step': 0.05,
    'presets': [
        {'label': 'Slow', 'linear': 0.08, 'angular': 0.25},
        {'label': 'Normal', 'linear': 0.13, 'angular': 0.35},
        {'label': 'Fast', 'linear': 0.18, 'angular': 0.50},
    ],
}
HW_NAV_LIMITS = {
    'max_linear_speed': 0.13,
    'cruise_linear_speed': 0.10,
    'max_angular_speed': 0.40,
    'lookahead_dist': 0.25,
    'use_bt': False,
}


def _active_teleop_limits() -> dict:
    """Return normal or mapping-restricted teleop limits for the current mode."""
    if AMR_MODE != 'hardware':
        return SIM_TELEOP_LIMITS
    mm = globals().get('mapping_mgr')
    is_hw_mapping = mm is not None and (
        getattr(mm, 'state', None) == 'MAPPING' or mm.is_external_mapping_running()
    )
    return HW_MAPPING_TELEOP_LIMITS if is_hw_mapping else HW_TELEOP_LIMITS


def _mode_map_dir() -> Path:
    return _HW_MAPS_DIR if AMR_MODE == 'hardware' else _SIM_MAPS_DIR


def _mode_sync_dirs() -> List[Path]:
    if AMR_MODE == 'hardware':
        return [_HW_MAPS_DIR]
    install_maps_dir = _WORKSPACE_ROOT / "install" / "agv_description" / "share" / "agv_description" / "maps"
    return [d for d in (_SIM_MAPS_DIR, install_maps_dir) if d.exists()]


def _filter_occupancy_speckles(raw_occ: np.ndarray) -> np.ndarray:
    """Remove isolated <4-cell occupied specks without turning unknown space into false free pockets."""
    out = raw_occ.copy()
    try:
        import cv2
        occ = (out > 50).astype(np.uint8)
        free_mask = (out == 0).astype(np.uint8)
        free_dilated = cv2.dilate(free_mask, np.ones((3, 3), dtype=np.uint8), iterations=1)
        n, labels, stats, _ = cv2.connectedComponentsWithStats(occ, connectivity=8)
        for i in range(1, n):
            if stats[i, cv2.CC_STAT_AREA] < 4:
                comp_mask = (labels == i)
                out[comp_mask] = 0 if np.any(free_dilated[comp_mask] > 0) else -1
    except Exception:
        pass
    return out


def _default_mapping_name_for_mode(mode: str) -> str:
    return 'hardware_map' if mode == 'hardware' else 'hospital_map'


def _get_map_search_dirs(mode: Optional[str] = None) -> List[Path]:
    active_mode = mode or AMR_MODE
    if active_mode == 'hardware':
        dirs = [_HW_MAPS_DIR, _SIM_MAPS_DIR, _APP_MAPS_DIR]
    else:
        dirs = [_SIM_MAPS_DIR, _HW_MAPS_DIR, _APP_MAPS_DIR]
    return [d for d in dirs if d.exists()]


def _find_map_file(map_name: str, suffixes: Tuple[str, ...], mode: Optional[str] = None) -> Optional[Path]:
    target = map_name.strip()
    for d in _get_map_search_dirs(mode):
        for suf in suffixes:
            cand = d / f"{target}{suf}"
            if cand.exists():
                return cand
    return None


def _get_primary_map_dir(map_name: str) -> Path:
    yaml_p = _find_map_file(map_name, ('.yaml',))
    if yaml_p is not None:
        return yaml_p.parent
    graph_p = _find_map_file(map_name, ('_graph.json', '.json'))
    if graph_p is not None:
        return graph_p.parent
    return _HW_MAPS_DIR if AMR_MODE == 'hardware' else _SIM_MAPS_DIR


def _default_map_for_mode(mode: str) -> str:
    if mode == 'hardware':
        for candidate in ('prakash_room', 'final_map', 'my_clean_warehouse11_map'):
            if _find_map_file(candidate, ('.yaml', '_graph.json'), mode='hardware'):
                return candidate
        if _HW_MAPS_DIR.exists():
            yamls = sorted(_HW_MAPS_DIR.glob('*.yaml'))
            if yamls:
                return yamls[0].stem
    return 'hospital_map'


# Import graph extractor engine
_MAPS_DIR = _SIM_MAPS_DIR
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
    target = (map_name or _default_map_for_mode(AMR_MODE)).strip()
    yaml_path = _find_map_file(target, ('.yaml',))
    if yaml_path is None:
        if target in ('warehouse_map', 'warehouse'):
            yaml_path = _find_map_file('warehouse_map', ('.yaml',))
        elif target == 'hospital_map':
            yaml_path = _find_map_file('hospital_map', ('.yaml',))
        else:
            for d in _get_map_search_dirs():
                matches = list(d.glob(f"*{target}*.yaml"))
                if matches:
                    yaml_path = matches[0]
                    break

    # Map-specific calibrated fallbacks
    if target in ('hospital', 'hospital_map'):
        res, ox, oy, width, height = 0.05, -8.5, -6.5, 340, 260
    else:
        res, ox, oy, width, height = 0.05, -7.397, -6.596, 329, 275

    if yaml_path and yaml_path.exists():
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
                    img_file = _find_map_file(target, ('.png', '.pgm')) or img_file
                if not img_file.exists():
                    img_file = _find_map_file(yaml_path.stem, ('.png', '.pgm')) or img_file

                if img_file.exists():
                    try:
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


def _synthesize_dark_map_png(src_img_path: Path, map_name: str) -> Optional[Path]:
    """Convert grayscale/light PGM-style PNGs (e.g. hardware maps) to dark-slate/cyan theme in cache."""
    try:
        _DARK_CACHE_DIR.mkdir(parents=True, exist_ok=True)
        out_path = _DARK_CACHE_DIR / f"{map_name}_dark.png"
        if out_path.exists() and out_path.stat().st_mtime >= src_img_path.stat().st_mtime:
            return out_path

        with Image.open(src_img_path) as im:
            rgb = np.array(im.convert('RGB'))

        # Skip conversion if image already has a dark floor (e.g., warehouse_map.png)
        dark_floor = np.mean((rgb[:, :, 0] < 45) & (rgb[:, :, 1] < 50) & (rgb[:, :, 2] < 70))
        if dark_floor > 0.35:
            return src_img_path

        out = np.zeros_like(rgb)
        gray = np.mean(rgb.astype(np.float32), axis=2)
        out[(gray >= 150) & (gray < 235)] = [26, 32, 46]     # unknown -> dark slate
        out[gray >= 235] = [12, 16, 28]                        # free -> deep navy
        out[gray < 150] = [0, 212, 255]                        # occupied -> cyan wall
        Image.fromarray(out).save(out_path, format='PNG')
        return out_path
    except Exception:
        return src_img_path


def _convert_pgm_to_cached_png(pgm_path: Path, map_name: str) -> Optional[Path]:
    """Convert a .pgm file to .png in _DARK_CACHE_DIR without writing into read-only map directories."""
    try:
        _DARK_CACHE_DIR.mkdir(parents=True, exist_ok=True)
        cached_png = _DARK_CACHE_DIR / f"{map_name}_raw.png"
        if not cached_png.exists() or cached_png.stat().st_mtime < pgm_path.stat().st_mtime:
            with Image.open(pgm_path) as pimg:
                pimg.save(cached_png, format='PNG')
        return cached_png
    except Exception:
        return None


def resolve_map_image_path(map_name: Optional[str] = None, theme: str = 'dark') -> Optional[Path]:
    """Find or convert the clean floorplan PNG across simulation & hardware map directories."""
    target = (map_name or _default_map_for_mode(AMR_MODE)).strip()
    search_dirs = _get_map_search_dirs()

    if theme == 'light':
        for d in search_dirs:
            lp = d / f"{target}_light.png"
            if lp.exists():
                return lp

    # 1. Direct png file
    for d in search_dirs:
        dp = d / f"{target}.png"
        if dp.exists():
            if theme == 'dark' and target != 'hospital_map':
                return _synthesize_dark_map_png(dp, target)
            return dp

    # 2. Check yaml
    for d in search_dirs:
        yaml_path = d / f"{target}.yaml"
        if yaml_path.exists():
            try:
                with open(yaml_path, 'r', encoding='utf-8') as f:
                    yd = yaml.safe_load(f)
                if isinstance(yd, dict) and 'image' in yd:
                    im_path = d / yd['image']
                    if im_path.suffix.lower() == '.pgm' and im_path.exists():
                        cached_png = _convert_pgm_to_cached_png(im_path, target)
                        if cached_png is not None:
                            return _synthesize_dark_map_png(cached_png, target) if (theme == 'dark' and target != 'hospital_map') else cached_png
                    if im_path.exists() and im_path.suffix.lower() == '.png':
                        return _synthesize_dark_map_png(im_path, target) if (theme == 'dark' and target != 'hospital_map') else im_path
            except Exception:
                pass

    # 3. If .pgm exists, convert to png in cache
    for d in search_dirs:
        pgm_path = d / f"{target}.pgm"
        if pgm_path.exists():
            cached_png = _convert_pgm_to_cached_png(pgm_path, target)
            if cached_png is not None:
                return _synthesize_dark_map_png(cached_png, target) if (theme == 'dark' and target != 'hospital_map') else cached_png

    return None


def world_to_pixel(wx: float, wy: float, map_name: Optional[str] = None):
    """Convert world (x,y) metres → (col, row) pixel in the map image."""
    meta = get_map_metadata_by_name(map_name or (node.active_map_name if 'node' in globals() and node else _default_map_for_mode(AMR_MODE)))
    col = int((wx - meta['origin_x']) / meta['resolution'])
    row = meta['height'] - int((wy - meta['origin_y']) / meta['resolution'])
    return col, row


# ---------------------------------------------------------------------------
# Mapping Process & State Manager (Phase 1)
# ---------------------------------------------------------------------------

class MappingManager:
    """Manages SLAM mapping sessions (Gazebo sim vs physical hardware) and verified map saving."""
    def __init__(self, workspace_root: Path):
        self.ws_root = workspace_root
        self.state = 'READY'  # 'READY' | 'MAPPING' | 'STOPPED' | 'SAVED' | 'ERROR'
        self.map_name = _default_mapping_name_for_mode(AMR_MODE)
        self.start_time: Optional[float] = None
        self.stop_time: Optional[float] = None
        self.proc: Optional[subprocess.Popen] = None
        self.explore_proc: Optional[subprocess.Popen] = None
        self.mapping_log = None
        self.explore_log = None
        self.active_log_path: Optional[str] = None
        self.target_world = 'hospital.world'
        self.run_explore = False
        self.is_saved: bool = False
        self.saved_map_name: Optional[str] = None
        self.saved_paths: Dict[str, str] = {}
        self.verification_report: Optional[dict] = None
        self.last_extracted_graph: Optional[dict] = None
        self.error_msg: Optional[str] = None
        self.stage_text: Optional[str] = None
        self._stop_pi_on_stop: bool = False
        self.lock = threading.Lock()

    @staticmethod
    def is_external_mapping_running() -> bool:
        try:
            res = subprocess.run(
                ['pgrep', '-f', r'[h]ardware_mapping\.launch\.py|[s]tart_hardware_mapping\.sh|[a]sync_slam_toolbox_node'],
                capture_output=True, text=True
            )
            if res.returncode == 0 and res.stdout.strip():
                return True
        except Exception:
            pass
        if AMR_MODE == 'hardware':
            n = globals().get('node')
            sm = globals().get('system_mgr')
            nav_inactive = sm is None or (getattr(sm, 'state', 'STOPPED') == 'STOPPED' and not sm._is_external_nav_running())
            if n is not None and nav_inactive and getattr(n, '_has_live_map', False):
                if (time.time() - getattr(n, '_last_map_time', 0.0)) < 5.0:
                    return True
        return False

    def _cleanup_processes(self):
        for proc in (self.explore_proc, self.proc):
            if proc and proc.poll() is None:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                    try:
                        proc.wait(timeout=2.5)
                    except subprocess.TimeoutExpired:
                        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                        try:
                            proc.wait(timeout=1.5)
                        except subprocess.TimeoutExpired:
                            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except Exception:
                    pass
        self.explore_proc = None
        self.proc = None

        for log_f in (self.explore_log, self.mapping_log):
            if log_f:
                try:
                    log_f.close()
                except Exception:
                    pass
        self.explore_log = None
        self.mapping_log = None

        patterns = [
            'hardware_mapping', 'mapping_session', 'async_slam_toolbox_node', 'slam_toolbox',
            'explore_node', 'explore_lite', 'controller_server', 'planner_server',
            'velocity_smoother', 'collision_monitor',
            'behavior_server', 'bt_navigator', 'waypoint_follower', 'lifecycle_manager',
            'rviz2_mapping',
        ]
        if AMR_MODE == 'sim':
            patterns.extend(['gz sim', 'parameter_bridge', 'ros_gz_bridge', 'ekf_node', 'robot_state_publisher', 'rviz2'])
        for pattern in patterns:
            try:
                subprocess.run(['pkill', '-9', '-f', pattern], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except Exception:
                pass

    def start(
        self,
        map_name: Optional[str] = None,
        world: Optional[str] = 'hospital.world',
        run_explore: bool = False,
        headless: bool = True,
        use_rviz: bool = False,
        auto_start_pi: bool = False,
        stop_pi_on_stop: bool = False,
    ) -> dict:
        with self.lock:
            sm = globals().get('system_mgr')
            if sm is not None and (sm.state in ('STARTING', 'RUNNING') or sm._is_external_nav_running()):
                return {
                    'ok': False,
                    'status_code': 409,
                    'error': 'Navigation stack is currently active. Stop navigation before starting SLAM mapping.'
                }
            if self.proc is None and self.is_external_mapping_running():
                return {
                    'ok': False,
                    'status_code': 409,
                    'error': 'An external hardware mapping session is already running. Use Save Map or stop the terminal session first.'
                }

            self._cleanup_processes()

            if 'node' in globals() and globals()['node'] is not None:
                try:
                    globals()['node'].reset_simulation_state()
                except Exception as e:
                    logger.warning(f"[MappingManager] Node reset warning: {e}")

            target_name = (map_name or self.map_name or _default_mapping_name_for_mode(AMR_MODE)).strip()
            self.map_name = target_name
            self.state = 'MAPPING'
            self.start_time = time.time()
            self.stop_time = None
            self.is_saved = False
            self.saved_map_name = None
            self.saved_paths = {}
            self.verification_report = None
            self.last_extracted_graph = None
            self.error_msg = None
            self.stage_text = None
            self.run_explore = bool(run_explore)
            self._stop_pi_on_stop = bool(stop_pi_on_stop) if AMR_MODE == 'hardware' else False

            if AMR_MODE == 'hardware':
                return self._start_hardware_mapping_locked(
                    run_explore=bool(run_explore),
                    use_rviz=bool(use_rviz),
                    auto_start_pi=bool(auto_start_pi),
                )

            world_val = (world or 'hospital.world').strip()
            self.target_world = world_val if (world_val.endswith('.world') or world_val.endswith('.sdf')) else f"{world_val}.world"
            run_sim_flag = 'true'
            run_explore_flag = 'true' if run_explore else 'false'
            headless_flag = 'true' if headless else 'false'
            rviz_flag = 'true' if use_rviz else 'false'
            display_env = os.environ.get('DISPLAY', ':0')
            domain_id = os.environ.get('ROS_DOMAIN_ID', '7')
            rmw_impl = os.environ.get('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')

            cmd = (
                f"export DISPLAY=\"{display_env}\" && "
                f"export QT_QPA_PLATFORM=xcb && "
                f"export ROS_DOMAIN_ID=\"{domain_id}\" && "
                f"export RMW_IMPLEMENTATION=\"{rmw_impl}\" && "
                f"export ROS_LOCALHOST_ONLY=1 && "
                f"export LIBGL_ALWAYS_SOFTWARE=1 && "
                f"export GALLIUM_DRIVER=llvmpipe && "
                f"export MESA_GL_VERSION_OVERRIDE=4.5 && "
                f"source /opt/ros/jazzy/setup.bash && "
                f"source {self.ws_root}/install/setup.bash && "
                f"ros2 launch agv_description mapping_session.launch.py "
                f"world:={self.target_world} run_sim:={run_sim_flag} run_explore:={run_explore_flag} "
                f"headless:={headless_flag} use_rviz:={rviz_flag}"
            )
            logger.info(f"[MappingManager] Starting simulation mapping_session: {cmd}")
            try:
                self.mapping_log = open('/tmp/amr_mapping.log', 'w')
                self.active_log_path = '/tmp/amr_mapping.log'
                sub_env = os.environ.copy()
                sub_env['DISPLAY'] = display_env
                sub_env['QT_QPA_PLATFORM'] = 'xcb'
                sub_env['ROS_DOMAIN_ID'] = domain_id
                sub_env['RMW_IMPLEMENTATION'] = rmw_impl
                sub_env['ROS_LOCALHOST_ONLY'] = '1'
                sub_env['LIBGL_ALWAYS_SOFTWARE'] = '1'
                sub_env['GALLIUM_DRIVER'] = 'llvmpipe'
                sub_env['MESA_GL_VERSION_OVERRIDE'] = '4.5'
                self.proc = subprocess.Popen(
                    cmd,
                    shell=True,
                    executable='/bin/bash',
                    cwd=str(self.ws_root),
                    preexec_fn=os.setsid,
                    stdout=self.mapping_log,
                    stderr=subprocess.STDOUT,
                    env=sub_env
                )
                return {'ok': True}
            except Exception as e:
                self.state = 'ERROR'
                self.error_msg = f"Failed to launch mapping_session: {e}"
                logger.error(f"[MappingManager] {self.error_msg}")
                return {'ok': False, 'status_code': 500, 'error': self.error_msg}

    def _start_hardware_mapping_locked(self, run_explore: bool, use_rviz: bool, auto_start_pi: bool) -> dict:
        """Launch hardware_mapping.launch.py on Domain 7 after ensuring Pi sensors & clock sync."""
        self.stage_text = f"Verifying Pi ({AMR_PI_IP}) sensors for hardware SLAM..."
        ok_sensors, err_msg = _ensure_pi_sensors_ready(
            auto_start_pi=auto_start_pi,
            status_cb=lambda msg: setattr(self, 'stage_text', msg),
        )
        if not ok_sensors:
            self.state = 'ERROR'
            self.error_msg = err_msg
            self.stage_text = err_msg
            return {'ok': False, 'status_code': 503, 'error': err_msg}

        # Clear any remote Pi slam_toolbox / static map->odom TF & sync clock
        pi_remote_mgr.clear_remote_slam_tf(AMR_PI_IP)
        pi_remote_mgr.sync_clock(AMR_PI_IP)
        for pat in ['amcl', 'map_server', 'lifecycle_manager_localization', 'static_transform_publisher.*map.*odom']:
            try:
                subprocess.run(['pkill', '-f', pat], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except Exception:
                pass

        _configure_dds_environment('hardware', AMR_PI_IP)
        dds_uri = os.environ.get('CYCLONEDDS_URI', '')
        log_dir = self.ws_root / "run_logs"
        log_dir.mkdir(parents=True, exist_ok=True)
        ts = time.strftime('%Y%m%d_%H%M%S')
        log_path = log_dir / f"hardware_mapping_{ts}.log"
        self.active_log_path = str(log_path)

        display_env = os.environ.get('DISPLAY', ':0')
        rviz_flag = 'true' if use_rviz else 'false'
        explore_flag = 'true' if run_explore else 'false'
        rviz_env_prefix = (
            f"export DISPLAY=\"{display_env}\" && "
            f"export QT_QPA_PLATFORM=xcb && "
            f"export LIBGL_ALWAYS_SOFTWARE=1 && "
            f"export GALLIUM_DRIVER=llvmpipe && "
            f"export MESA_GL_VERSION_OVERRIDE=4.5 && "
        ) if use_rviz else ""

        hw_cmd = (
            rviz_env_prefix
            + "unset ROS_LOCALHOST_ONLY && "
            + "export ROS_DOMAIN_ID=7 && "
            + "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && "
            + (f"export CYCLONEDDS_URI=\"{dds_uri}\" && " if dds_uri else "")
            + "source /opt/ros/jazzy/setup.bash && "
            + f"source {self.ws_root}/install/setup.bash && "
            + "ros2 launch agv_description hardware_mapping.launch.py "
            + f"use_sim_time:=false use_rviz:={rviz_flag} run_explore:={explore_flag}"
        )
        logger.info(f"[MappingManager] Starting hardware_mapping: {hw_cmd}")
        try:
            self.mapping_log = open(log_path, 'w')
            sub_env = os.environ.copy()
            sub_env.pop('ROS_LOCALHOST_ONLY', None)
            sub_env['ROS_DOMAIN_ID'] = '7'
            sub_env['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
            if dds_uri:
                sub_env['CYCLONEDDS_URI'] = dds_uri
            self.proc = subprocess.Popen(
                hw_cmd,
                shell=True,
                executable='/bin/bash',
                cwd=str(self.ws_root),
                preexec_fn=os.setsid,
                stdout=self.mapping_log,
                stderr=subprocess.STDOUT,
                env=sub_env,
            )
            time.sleep(1.2)
            if self.proc.poll() is not None:
                tail = SystemLaunchManager._tail_log(str(log_path))
                self.state = 'ERROR'
                self.error_msg = f"hardware_mapping.launch.py exited early: {tail}"
                self.stage_text = self.error_msg
                return {'ok': False, 'status_code': 500, 'error': self.error_msg}

            self.stage_text = (
                "Hardware SLAM + speed-capped Auto-Explore active"
                if run_explore
                else "Hardware SLAM Toolbox active (drive slowly via Teleop)"
            )
            return {'ok': True}
        except Exception as e:
            self.state = 'ERROR'
            self.error_msg = f"Failed to launch hardware_mapping: {e}"
            self.stage_text = self.error_msg
            return {'ok': False, 'status_code': 500, 'error': self.error_msg}

    def start_auto_explore(self, ros_node=None) -> dict:
        """Dynamically activate explore_lite (and speed-capped HW Nav2 stack if in hardware mode)."""
        if ros_node and (ros_node._estop_latched or ros_node.nav_state == 'ESTOP'):
            return {'ok': False, 'status_code': 409, 'error': 'E-STOP active. Clear E-STOP before starting exploration.'}

        with self.lock:
            if self.state != 'MAPPING':
                return {'ok': False, 'status_code': 409, 'error': 'Start SLAM mapping before activating Auto-Explore.'}

            self.run_explore = True
            is_running = False
            try:
                res = subprocess.run(['pgrep', '-f', '[e]xplore_node'], stdout=subprocess.PIPE)
                if res.returncode == 0 and res.stdout.strip():
                    is_running = True
            except Exception:
                pass

            if is_running:
                if ros_node:
                    ros_node.publish_explore_resume(True)
                return {'ok': True}

            if AMR_MODE == 'hardware':
                _configure_dds_environment('hardware', AMR_PI_IP)
                dds_uri = os.environ.get('CYCLONEDDS_URI', '')
                log_dir = self.ws_root / "run_logs"
                log_dir.mkdir(parents=True, exist_ok=True)
                hw_exp_log = log_dir / f"hardware_explore_{time.strftime('%Y%m%d_%H%M%S')}.log"
                cmd = (
                    "unset ROS_LOCALHOST_ONLY && "
                    "export ROS_DOMAIN_ID=7 && "
                    "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && "
                    + (f"export CYCLONEDDS_URI=\"{dds_uri}\" && " if dds_uri else "")
                    + "source /opt/ros/jazzy/setup.bash && "
                    + f"source {self.ws_root}/install/setup.bash && "
                    + "ros2 launch agv_description hardware_mapping.launch.py "
                    + "launch_slam:=false use_rviz:=false run_explore:=true use_sim_time:=false"
                )
                try:
                    self.explore_log = open(hw_exp_log, 'a')
                    sub_env = os.environ.copy()
                    sub_env.pop('ROS_LOCALHOST_ONLY', None)
                    sub_env['ROS_DOMAIN_ID'] = '7'
                    sub_env['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
                    if dds_uri:
                        sub_env['CYCLONEDDS_URI'] = dds_uri
                    self.explore_proc = subprocess.Popen(
                        cmd,
                        shell=True,
                        executable='/bin/bash',
                        cwd=str(self.ws_root),
                        preexec_fn=os.setsid,
                        stdout=self.explore_log,
                        stderr=subprocess.STDOUT,
                        env=sub_env,
                    )
                    logger.info("[MappingManager] Dynamically launched speed-capped HW Nav2 + Collision Monitor + explore_lite!")
                    return {'ok': True}
                except Exception as e:
                    self.run_explore = False
                    logger.error(f"[MappingManager] Failed to launch HW explore stack: {e}")
                    return {'ok': False, 'status_code': 500, 'error': str(e)}

            domain_id = os.environ.get('ROS_DOMAIN_ID', '7')
            rmw_impl = os.environ.get('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
            cmd = (
                f"export ROS_DOMAIN_ID=\"{domain_id}\" && "
                f"export RMW_IMPLEMENTATION=\"{rmw_impl}\" && "
                f"export ROS_LOCALHOST_ONLY=1 && "
                f"source /opt/ros/jazzy/setup.bash && "
                f"source {self.ws_root}/install/setup.bash && "
                f"ros2 run explore_lite explore --ros-args -r __node:=explore_node "
                f"--params-file {self.ws_root}/src/agv_description/config/nav2_params_explore.yaml "
                f"-p use_sim_time:=true"
            )
            try:
                self.explore_log = open('/tmp/amr_explore.log', 'a')
                self.explore_proc = subprocess.Popen(
                    cmd,
                    shell=True,
                    executable='/bin/bash',
                    cwd=str(self.ws_root),
                    preexec_fn=os.setsid,
                    stdout=self.explore_log,
                    stderr=subprocess.STDOUT
                )
                logger.info("[MappingManager] Spawning explore_node dynamically...")
                return {'ok': True}
            except Exception as e:
                logger.error(f"[MappingManager] Failed to spawn explore_node: {e}")
                return {'ok': False, 'status_code': 500, 'error': str(e)}

    def pause_auto_explore(self, ros_node=None) -> bool:
        """Pause exploration without killing the node, allowing manual driving."""
        with self.lock:
            self.run_explore = False
            if ros_node:
                ros_node.publish_explore_resume(False)
            return True

    def resume_auto_explore(self, ros_node=None) -> dict:
        """Resume exploration."""
        if ros_node and (ros_node._estop_latched or ros_node.nav_state == 'ESTOP'):
            return {'ok': False, 'status_code': 409, 'error': 'E-STOP active. Clear E-STOP before resuming exploration.'}
        with self.lock:
            self.run_explore = True
            if ros_node:
                ros_node.publish_explore_resume(True)
            return {'ok': True}

    def stop(self, stop_pi: Optional[bool] = None) -> dict:
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
            should_stop_pi = (
                AMR_MODE == 'hardware'
                and (stop_pi if stop_pi is not None else self._stop_pi_on_stop)
            )
            self._stop_pi_on_stop = False
            pi_stopped = False
            if should_stop_pi and 'pi_remote_mgr' in globals():
                pi_res = pi_remote_mgr.stop_hardware(AMR_PI_IP)
                pi_stopped = bool(pi_res and pi_res.get('ok'))
            logger.info("[MappingManager] Mapping session stopped cleanly.")
            return {'ok': True, 'pi_stopped': pi_stopped}

    def redo(self, stop_pi: Optional[bool] = None) -> dict:
        with self.lock:
            self.run_explore = False
            self._cleanup_processes()
            should_stop_pi = (
                AMR_MODE == 'hardware'
                and (stop_pi if stop_pi is not None else self._stop_pi_on_stop)
            )
            self._stop_pi_on_stop = False
            if should_stop_pi and 'pi_remote_mgr' in globals():
                pi_remote_mgr.stop_hardware(AMR_PI_IP)
            self.state = 'READY'
            self.start_time = None
            self.stop_time = None
            self.is_saved = False
            self.saved_map_name = None
            self.saved_paths = {}
            self.verification_report = None
            self.last_extracted_graph = None
            self.error_msg = None
            self.stage_text = None
            if 'node' in globals() and globals()['node'] is not None:
                try:
                    globals()['node'].reset_simulation_state()
                except Exception:
                    pass
            logger.info("[MappingManager] Mapping session reset (redo).")
            return {'ok': True}

    def save_map(self, map_name: str, ros_node=None) -> dict:
        """Save active SLAM map to the active mode's map dir with unknown-safe speckle filtering."""
        with self.lock:
            is_ext = self.is_external_mapping_running()
            if self.state not in ('MAPPING', 'STOPPED', 'SAVED') and not is_ext:
                return {
                    'ok': False,
                    'error': 'No active or stopped SLAM mapping session. Start mapping before saving.'
                }
            if not ros_node:
                return {'ok': False, 'error': 'ROS node not ready.'}

            with ros_node._map_lock:
                if not ros_node._has_live_map or ros_node._last_raw_occupancy is None:
                    return {
                        'ok': False,
                        'error': 'No active SLAM map data available to save. Run mapping before saving.'
                    }
                w = ros_node._live_map_width
                h = ros_node._live_map_height
                res = float(ros_node._live_map_resolution)
                ox = float(ros_node._live_map_origin_x)
                oy = float(ros_node._live_map_origin_y)
                raw_copy = ros_node._last_raw_occupancy.copy()

            raw_name = (map_name or self.map_name or _default_mapping_name_for_mode(AMR_MODE)).strip()
            clean_name = re.sub(r'[^a-zA-Z0-9_-]', '_', raw_name)
            if not clean_name:
                clean_name = _default_mapping_name_for_mode(AMR_MODE)

            category = 'hardware' if AMR_MODE == 'hardware' else 'simulation'
            maps_dir = _mode_map_dir()
            maps_dir.mkdir(parents=True, exist_ok=True)

            yaml_path = maps_dir / f"{clean_name}.yaml"
            pgm_path = maps_dir / f"{clean_name}.pgm"
            png_path = maps_dir / f"{clean_name}.png"

            # Apply unknown-safe <4-cell speckle filtering in hardware mode (matches save_hardware_map.sh)
            raw = _filter_occupancy_speckles(raw_copy) if AMR_MODE == 'hardware' else raw_copy

            # 1. Generate PGM data (ROS standard trinary values: free=254, occupied=0, unknown=205)
            pgm_arr = np.full((h, w), 205, dtype=np.uint8)
            pgm_arr[raw == 0] = 254
            pgm_arr[raw > 50] = 0
            pgm_flipped = np.flipud(pgm_arr)
            Image.fromarray(pgm_flipped, mode='L').save(str(pgm_path))

            # 2. Write YAML specification (matching Nav2 / map_server & save_hardware_map.sh)
            yaml_content = (
                f"image: {clean_name}.pgm\n"
                f"mode: trinary\n"
                f"resolution: {res:.4f}\n"
                f"origin: [{ox:.6f}, {oy:.6f}, 0]\n"
                f"negate: 0\n"
                f"occupied_thresh: 0.65\n"
                f"free_thresh: 0.25\n"
            )
            with open(yaml_path, 'w', encoding='utf-8') as f:
                f.write(yaml_content)

            # 3. Write Clean Dark Preview PNG for browser dashboard
            png_arr = np.zeros((h, w, 3), dtype=np.uint8)
            png_arr[raw == -1] = [26, 32, 46]     # unknown slate
            png_arr[raw == 0]  = [12, 16, 28]     # free space
            png_arr[raw > 50]  = [0, 212, 255]    # wall / occupied cyan
            Image.fromarray(np.flipud(png_arr)).save(str(png_path))

            # 4. On-disk Verification
            yaml_ok = yaml_path.is_file() and yaml_path.stat().st_size > 0
            pgm_ok = pgm_path.is_file() and pgm_path.stat().st_size > 0
            if not (yaml_ok and pgm_ok):
                return {
                    'ok': False,
                    'error': f"Failed to verify saved files on disk: yaml_ok={yaml_ok}, pgm_ok={pgm_ok}"
                }

            pi_synced = None
            pi_sync_error = None
            if AMR_MODE == 'hardware' and 'pi_remote_mgr' in globals():
                sync_res = pi_remote_mgr.sync_maps_to_pi([yaml_path, pgm_path, png_path], AMR_PI_IP)
                pi_synced = bool(sync_res.get('ok'))
                if not pi_synced:
                    pi_sync_error = sync_res.get('error')

            total_cells = w * h
            free_cells = int(np.sum(raw == 0))
            occupied_cells = int(np.sum(raw > 50))
            unknown_cells = int(np.sum(raw == -1))
            free_area = round(free_cells * (res ** 2), 2)
            occupied_area = round(occupied_cells * (res ** 2), 2)
            coverage_pct = round(((free_cells + occupied_cells) / max(1, total_cells)) * 100, 1)

            self.is_saved = True
            self.saved_map_name = clean_name
            self.map_name = clean_name
            self.saved_paths = {
                'yaml': str(yaml_path.relative_to(self.ws_root)),
                'pgm': str(pgm_path.relative_to(self.ws_root)),
                'png': str(png_path.relative_to(self.ws_root)),
            }
            self.state = 'SAVED'

            report = {
                'verified': True,
                'map_name': clean_name,
                'category': category,
                'status': 'VERIFIED & SAVED',
                'pi_synced': pi_synced,
                'pi_sync_error': pi_sync_error,
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
            if self.state == 'MAPPING' and self.proc and self.proc.poll() is not None:
                log_p = self.active_log_path or '/tmp/amr_mapping.log'
                tail = SystemLaunchManager._tail_log(log_p)
                self.state = 'ERROR'
                self.error_msg = f"Mapping process exited unexpectedly: {tail}"

            external_running = (self.proc is None) and self.is_external_mapping_running()

            elapsed = 0.0
            if self.state == 'MAPPING' and self.start_time:
                elapsed = time.time() - self.start_time
            elif (self.state in ('STOPPED', 'SAVED')) and self.start_time and self.stop_time:
                elapsed = self.stop_time - self.start_time

            mins = int(elapsed // 60)
            secs = int(elapsed % 60)
            elapsed_str = f"{mins:02d}:{secs:02d}"

            if external_running:
                map_status_text = "External hardware SLAM session active (terminal launch detected)."
            elif self.state == 'ERROR' and self.error_msg:
                map_status_text = self.error_msg
            elif has_map_data and self.state == 'MAPPING':
                map_status_text = "Receiving map data... (SLAM Toolbox active)"
            elif self.state == 'MAPPING':
                map_status_text = self.stage_text or "Mapping in progress... Initializing SLAM Toolbox..."
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
                'mode': AMR_MODE,
                'state': self.state,
                'external_running': external_running,
                'error': self.error_msg,
                'stage_text': self.stage_text,
                'map_name': self.map_name,
                'world': getattr(self, 'target_world', 'hospital.world'),
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


class PiRemoteManager:
    """Optional passwordless-SSH controller for Raspberry Pi ./start_hardware.sh bringup."""

    _PROC_PGREP_PATTERN = (
        r"[h]ardware_bringup\.launch\.py|"
        r"[a]mr_hardware_drive_node|"
        r"[a]mr_imu_node|"
        r"[a]mr_lidar_node"
    )
    _STOP_PGREP_PATTERN = (
        r"[h]ardware_bringup\.launch\.py|"
        r"[a]mr_hardware_drive_node|"
        r"[a]mr_imu_node|"
        r"[a]mr_lidar_node|"
        r"[e]kf_node|"
        r"[r]obot_state_publisher"
    )

    def __init__(self, pi_user: str = AMR_PI_USER, pi_dir: str = AMR_PI_DIR):
        self.pi_user = pi_user
        self.pi_dir = pi_dir
        self.lock = threading.Lock()
        self.state = 'STOPPED'  # 'STOPPED' | 'STARTING' | 'RUNNING' | 'STOPPING' | 'ERROR'
        self.ssh_ready = False
        self.reachable = False
        self.remote_running = False
        self.managed_pid: Optional[int] = None
        self.pids: List[str] = []
        self.last_error: Optional[str] = None
        self.last_check_time: float = 0.0
        self.start_time: Optional[float] = None
        self.clock_synced: Optional[bool] = None
        self.clock_sync_at: Optional[float] = None
        self._refresh_inflight = False

    def _remote_dir_expr(self) -> str:
        raw = self.pi_dir.strip()
        if raw == "~":
            return "$HOME"
        if raw.startswith("~/"):
            return f"$HOME/{shlex.quote(raw[2:])}"
        return shlex.quote(raw)

    def _run_ssh(self, pi_ip: str, remote_cmd: str, timeout: float = 5.0) -> subprocess.CompletedProcess:
        cmd = [
            'ssh',
            '-o', 'BatchMode=yes',
            '-o', 'ConnectTimeout=3',
            '-o', 'StrictHostKeyChecking=no',
            f'{self.pi_user}@{pi_ip}',
            remote_cmd,
        ]
        return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)

    @staticmethod
    def _classify_ssh_error(stderr: str, returncode: int) -> str:
        msg = (stderr or '').strip()
        low = msg.lower()
        if 'permission denied' in low or 'publickey' in low:
            return (
                f"Passwordless SSH key auth failed for {AMR_PI_USER}@{AMR_PI_IP}. "
                f"Run ssh-copy-id {AMR_PI_USER}@{AMR_PI_IP} first."
            )
        if 'timed out' in low or 'no route to host' in low or 'network is unreachable' in low:
            return f"Raspberry Pi ({AMR_PI_IP}) is unreachable on the network."
        if 'connection refused' in low:
            return f"SSH port refused on Raspberry Pi ({AMR_PI_IP})."
        return msg or f"SSH command failed (exit code {returncode})."

    def check_status(self, pi_ip: Optional[str] = None, force: bool = False) -> dict:
        target_ip = (pi_ip or AMR_PI_IP).strip()
        now = time.time()
        with self.lock:
            if not force and (now - self.last_check_time) < 3.5:
                return self._snapshot_locked(target_ip)

        dir_expr = self._remote_dir_expr()
        probe_script = (
            f'PI_DIR={dir_expr}; '
            f'PID_FILE="$PI_DIR/run_logs/pi_hardware.pid"; '
            f'MANAGED=""; '
            f'if [ -f "$PID_FILE" ]; then '
            f'  MP=$(cat "$PID_FILE" 2>/dev/null || true); '
            f'  if [ -n "$MP" ] && kill -0 "$MP" 2>/dev/null; then MANAGED="$MP"; fi; '
            f'fi; '
            f'PIDS=$(pgrep -f {shlex.quote(self._PROC_PGREP_PATTERN)} 2>/dev/null | tr "\\n" "," || true); '
            f'SCRIPT_OK=0; [ -x "$PI_DIR/start_hardware.sh" ] && SCRIPT_OK=1; '
            f'echo "SSH_OK=1|SCRIPT_OK=$SCRIPT_OK|MANAGED=$MANAGED|PIDS=$PIDS"'
        )
        try:
            res = self._run_ssh(target_ip, probe_script, timeout=4.5)
        except subprocess.TimeoutExpired:
            with self.lock:
                self.reachable = False
                self.ssh_ready = False
                self.remote_running = False
                self.pids = []
                self.managed_pid = None
                self.last_check_time = time.time()
                self.last_error = f"SSH connection to {self.pi_user}@{target_ip} timed out."
                return self._snapshot_locked(target_ip)
        except Exception as e:
            with self.lock:
                self.reachable = False
                self.ssh_ready = False
                self.remote_running = False
                self.last_check_time = time.time()
                self.last_error = f"SSH check error: {e}"
                return self._snapshot_locked(target_ip)

        with self.lock:
            self.last_check_time = time.time()
            if res.returncode != 0:
                err = self._classify_ssh_error(res.stderr, res.returncode)
                self.reachable = 'timed out' not in err.lower() and 'unreachable' not in err.lower()
                self.ssh_ready = False
                self.remote_running = False
                self.pids = []
                self.managed_pid = None
                self.last_error = err
                return self._snapshot_locked(target_ip)

            self.reachable = True
            self.ssh_ready = True
            line = (res.stdout or '').strip().splitlines()[-1] if (res.stdout or '').strip() else ''
            parsed: Dict[str, str] = {}
            for part in line.split('|'):
                if '=' in part:
                    k, v = part.split('=', 1)
                    parsed[k.strip()] = v.strip()

            pids = [p for p in parsed.get('PIDS', '').split(',') if p.strip().isdigit()]
            mp_str = parsed.get('MANAGED', '')
            self.pids = pids
            self.managed_pid = int(mp_str) if mp_str.isdigit() else None
            self.remote_running = bool(pids or self.managed_pid)
            if parsed.get('SCRIPT_OK') == '0':
                self.last_error = f"start_hardware.sh not found or not executable in {self.pi_dir} on {target_ip}."
            elif self.last_error and 'ssh' in self.last_error.lower():
                self.last_error = None

            if self.remote_running:
                if self.state != 'STARTING':
                    self.state = 'RUNNING'
            elif self.state in ('RUNNING', 'STOPPING'):
                self.state = 'STOPPED'
                self.start_time = None

            return self._snapshot_locked(target_ip)

    def _snapshot_locked(self, pi_ip: Optional[str] = None, sensors_ready: bool = False) -> dict:
        eff_state = self.state
        if sensors_ready and eff_state != 'STOPPING':
            eff_state = 'RUNNING'
        return {
            'ok': True,
            'pi_ip': (pi_ip or AMR_PI_IP).strip(),
            'pi_user': self.pi_user,
            'pi_dir': self.pi_dir,
            'reachable': self.reachable,
            'ssh_ready': self.ssh_ready,
            'remote_running': self.remote_running,
            'managed_pid': self.managed_pid,
            'pids': list(self.pids),
            'state': eff_state,
            'last_error': self.last_error,
            'clock_synced': self.clock_synced,
            'clock_sync_at': round(self.clock_sync_at, 2) if self.clock_sync_at else None,
            'checked_at': round(self.last_check_time, 2) if self.last_check_time else None,
        }

    def get_cached_status(self, sensors_ready: bool = False) -> dict:
        need_bg = False
        with self.lock:
            snap = self._snapshot_locked(AMR_PI_IP, sensors_ready=sensors_ready)
            if AMR_MODE == 'hardware' and not self._refresh_inflight and (time.time() - self.last_check_time) > 5.0:
                self._refresh_inflight = True
                need_bg = True

        if need_bg:
            def _bg():
                try:
                    self.check_status(AMR_PI_IP, force=True)
                finally:
                    with self.lock:
                        self._refresh_inflight = False
            threading.Thread(target=_bg, daemon=True).start()
        return snap

    def sync_clock(self, pi_ip: Optional[str] = None) -> bool:
        target_ip = (pi_ip or AMR_PI_IP).strip()
        try:
            epoch_str = f"{time.time():.6f}"
            res = self._run_ssh(target_ip, f"sudo -n date -s '@{epoch_str}'", timeout=3.5)
            ok = (res.returncode == 0)
            with self.lock:
                self.clock_synced = ok
                if ok:
                    self.clock_sync_at = time.time()
            return ok
        except Exception:
            with self.lock:
                self.clock_synced = False
            return False

    def clear_remote_slam_tf(self, pi_ip: Optional[str] = None) -> bool:
        """Stop any remote Pi slam_toolbox / static map->odom publisher so laptop SLAM owns map->odom."""
        target_ip = (pi_ip or AMR_PI_IP).strip()
        try:
            cmd = "pkill -INT -f '[s]lam_toolbox|[a]sync_slam_toolbox_node|[s]tatic_transform_publisher.*map.*odom' 2>/dev/null || true"
            res = self._run_ssh(target_ip, cmd, timeout=4.0)
            return res.returncode == 0
        except Exception:
            return False

    def sync_maps_to_pi(self, file_paths: List[Path], pi_ip: Optional[str] = None) -> dict:
        """Copy saved map/graph/place artifacts to amr@<PI_IP>:~/AMR/maps/ via SSH/SCP."""
        target_ip = (pi_ip or AMR_PI_IP).strip()
        existing = [str(p) for p in file_paths if p and Path(p).is_file()]
        if not existing:
            return {'ok': True, 'count': 0}
        dir_expr = self._remote_dir_expr()
        try:
            mk = self._run_ssh(target_ip, f'mkdir -p {dir_expr}/maps', timeout=4.0)
            if mk.returncode != 0:
                return {'ok': False, 'error': self._classify_ssh_error(mk.stderr, mk.returncode)}
            dest_dir = self.pi_dir.strip().rstrip('/') + '/maps/'
            cmd = [
                'scp', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=4', '-o', 'StrictHostKeyChecking=no',
                *existing, f'{self.pi_user}@{target_ip}:{dest_dir}'
            ]
            res = subprocess.run(cmd, capture_output=True, text=True, timeout=12.0)
            if res.returncode != 0:
                return {'ok': False, 'error': (res.stderr or '').strip() or f'scp failed ({res.returncode})'}
            return {'ok': True, 'count': len(existing)}
        except Exception as e:
            return {'ok': False, 'error': str(e)}

    def get_logs(self, pi_ip: Optional[str] = None, lines: int = 50) -> dict:
        target_ip = (pi_ip or AMR_PI_IP).strip()
        safe_lines = max(5, min(int(lines), 300))
        dir_expr = self._remote_dir_expr()
        cmd = (
            f'LOG_FILE={dir_expr}/run_logs/pi_hardware_latest.log; '
            f'if [ -f "$LOG_FILE" ]; then tail -n {safe_lines} "$LOG_FILE"; '
            f'else echo "(no pi_hardware_latest.log found on Pi yet)"; fi'
        )
        try:
            res = self._run_ssh(target_ip, cmd, timeout=4.5)
            if res.returncode != 0:
                return {'ok': False, 'error': self._classify_ssh_error(res.stderr, res.returncode), 'logs': ''}
            return {'ok': True, 'pi_ip': target_ip, 'logs': res.stdout}
        except Exception as e:
            return {'ok': False, 'error': str(e), 'logs': ''}

    def start_hardware(self, pi_ip: Optional[str] = None) -> dict:
        target_ip = (pi_ip or AMR_PI_IP).strip()
        status = self.check_status(target_ip, force=True)
        if not status.get('ssh_ready'):
            return {'ok': False, 'error': status.get('last_error') or f"SSH not ready for {target_ip}", **status}
        if status.get('remote_running'):
            with self.lock:
                self.state = 'RUNNING'
                self.last_error = None
            return {
                'ok': True,
                'already_running': True,
                'message': f"Pi hardware stack is already running on {target_ip}.",
                **self.check_status(target_ip, force=True),
            }

        laptop_ip = _detect_local_ip_for_peer(target_ip)
        arg_str = shlex.quote(laptop_ip) if laptop_ip else ""
        dir_expr = self._remote_dir_expr()
        launch_script = (
            f'PI_DIR={dir_expr}; '
            f'if [ ! -x "$PI_DIR/start_hardware.sh" ]; then '
            f'  echo "MISSING_SCRIPT:$PI_DIR/start_hardware.sh" >&2; exit 2; '
            f'fi; '
            f'mkdir -p "$PI_DIR/run_logs"; '
            f'rm -f "$PI_DIR/run_logs/pi_hardware.pid"; '
            f'cd "$PI_DIR"; '
            f'setsid nohup ./start_hardware.sh {arg_str} </dev/null > "$PI_DIR/run_logs/pi_hardware_latest.log" 2>&1 & '
            f'BG_PID=$!; '
            f'echo "$BG_PID" > "$PI_DIR/run_logs/pi_hardware.pid"; '
            f'sleep 0.9; '
            f'if kill -0 "$BG_PID" 2>/dev/null; then '
            f'  echo "STARTED_PID=$BG_PID"; exit 0; '
            f'else '
            f'  echo "EARLY_EXIT"; tail -n 15 "$PI_DIR/run_logs/pi_hardware_latest.log" 2>/dev/null || true; exit 3; '
            f'fi'
        )

        with self.lock:
            self.state = 'STARTING'
            self.start_time = time.time()
            self.last_error = None

        logger.info(f"[PiRemoteManager] Starting ./start_hardware.sh {laptop_ip} on {self.pi_user}@{target_ip}...")
        try:
            res = self._run_ssh(target_ip, launch_script, timeout=10.0)
        except Exception as e:
            with self.lock:
                self.state = 'ERROR'
                self.last_error = f"Failed launching start_hardware.sh via SSH: {e}"
                return {'ok': False, 'error': self.last_error, **self._snapshot_locked(target_ip)}

        if res.returncode != 0:
            detail = (res.stdout or res.stderr or '').strip()
            with self.lock:
                self.state = 'ERROR'
                self.last_error = f"Pi start_hardware.sh failed: {detail or f'exit {res.returncode}'}"
                return {'ok': False, 'error': self.last_error, **self._snapshot_locked(target_ip)}

        pid_match = re.search(r'STARTED_PID=(\d+)', res.stdout or '')
        with self.lock:
            self.managed_pid = int(pid_match.group(1)) if pid_match else None
            self.remote_running = True
            self.state = 'STARTING'
            self.last_check_time = time.time()
            return {
                'ok': True,
                'already_running': False,
                'laptop_ip': laptop_ip,
                'message': f"Launched ./start_hardware.sh {laptop_ip} on Pi ({target_ip}). Calibrating IMU & LiDAR...",
                **self._snapshot_locked(target_ip),
            }

    def stop_hardware(self, pi_ip: Optional[str] = None) -> dict:
        target_ip = (pi_ip or AMR_PI_IP).strip()
        with self.lock:
            self.state = 'STOPPING'

        # Zero /cmd_vel immediately from bridge side before tearing down Pi drive node
        n = globals().get('node')
        if n is not None:
            try:
                n.publish_cmd_vel(0.0, 0.0)
            except Exception:
                pass

        dir_expr = self._remote_dir_expr()
        stop_script = (
            f'PI_DIR={dir_expr}; '
            f'PID_FILE="$PI_DIR/run_logs/pi_hardware.pid"; '
            f'if [ -f "$PID_FILE" ]; then '
            f'  MP=$(cat "$PID_FILE" 2>/dev/null || true); '
            f'  if [ -n "$MP" ] && kill -0 "$MP" 2>/dev/null; then '
            f'    kill -INT -"$MP" 2>/dev/null || kill -INT "$MP" 2>/dev/null || true; '
            f'  fi; '
            f'fi; '
            f'pkill -INT -f {shlex.quote(self._STOP_PGREP_PATTERN)} 2>/dev/null || true; '
            f'for _ in 1 2 3 4 5; do '
            f'  if ! pgrep -f {shlex.quote(self._STOP_PGREP_PATTERN)} >/dev/null 2>&1; then break; fi; '
            f'  sleep 0.4; '
            f'done; '
            f'if pgrep -f {shlex.quote(self._STOP_PGREP_PATTERN)} >/dev/null 2>&1; then '
            f'  pkill -TERM -f {shlex.quote(self._STOP_PGREP_PATTERN)} 2>/dev/null || true; '
            f'  sleep 0.6; '
            f'fi; '
            f'if pgrep -f {shlex.quote(self._STOP_PGREP_PATTERN)} >/dev/null 2>&1; then '
            f'  pkill -KILL -f {shlex.quote(self._STOP_PGREP_PATTERN)} 2>/dev/null || true; '
            f'fi; '
            f'rm -f "$PID_FILE"; '
            f'echo "STOPPED_OK"'
        )
        try:
            res = self._run_ssh(target_ip, stop_script, timeout=10.0)
        except Exception as e:
            with self.lock:
                self.state = 'ERROR'
                self.last_error = f"SSH stop error: {e}"
                return {'ok': False, 'error': self.last_error, **self._snapshot_locked(target_ip)}

        with self.lock:
            if res.returncode != 0:
                self.state = 'ERROR'
                self.last_error = self._classify_ssh_error(res.stderr, res.returncode)
                return {'ok': False, 'error': self.last_error, **self._snapshot_locked(target_ip)}
            self.state = 'STOPPED'
            self.remote_running = False
            self.managed_pid = None
            self.pids = []
            self.start_time = None
            self.last_error = None
            self.last_check_time = time.time()
            logger.info(f"[PiRemoteManager] Stopped Pi hardware stack on {target_ip}.")
            return {
                'ok': True,
                'message': f"Stopped Pi hardware stack on {target_ip}.",
                **self._snapshot_locked(target_ip),
            }


pi_remote_mgr = PiRemoteManager()


def _ensure_pi_sensors_ready(
    auto_start_pi: bool = False,
    cancel_evt: Optional[threading.Event] = None,
    stop_check=None,
    status_cb=None,
) -> Tuple[bool, Optional[str]]:
    """Wait for Pi /scan & /odometry/filtered, optionally launching ./start_hardware.sh via SSH."""
    n = globals().get('node')

    def _aborted() -> bool:
        return bool((cancel_evt and cancel_evt.is_set()) or (stop_check and stop_check()))

    def _wait_loop(iterations: int, check_pi_health: bool) -> Tuple[bool, Optional[str]]:
        for idx in range(iterations):
            if _aborted():
                return False, 'Cancelled'
            now = time.time()
            if (
                n is not None
                and (now - getattr(n, '_last_scan_rx_time', 0.0)) < 3.0
                and (now - getattr(n, '_last_odom_rx_time', 0.0)) < 3.0
            ):
                return True, None
            if check_pi_health and idx > 0 and idx % 8 == 7:
                pi_st = pi_remote_mgr.check_status(AMR_PI_IP, force=True)
                if not pi_st.get('remote_running'):
                    logs = pi_remote_mgr.get_logs(AMR_PI_IP, lines=12).get('logs', '').strip()
                    tail = " | ".join(logs.splitlines()[-4:]) if logs else "no log output"
                    return False, f"Pi start_hardware.sh exited before sensors came online: {tail}"
            if cancel_evt is not None:
                if cancel_evt.wait(0.5):
                    return False, 'Cancelled'
            else:
                time.sleep(0.5)
        return False, None

    ok, err = _wait_loop(6, check_pi_health=False)
    if ok or err:
        return ok, err

    if not auto_start_pi:
        return (
            False,
            f"Pi sensors (/scan, /odometry/filtered) not detected from {AMR_PI_IP}. "
            f"Ensure './start_hardware.sh' is running on the Raspberry Pi 5 (or enable Auto-Start Pi via SSH)."
        )

    if status_cb:
        status_cb(
            f'Starting ./start_hardware.sh on Pi ({AMR_PI_IP}) via SSH — '
            f'keep robot stationary for IMU calibration...'
        )
    pi_res = pi_remote_mgr.start_hardware(AMR_PI_IP)
    if not pi_res.get('ok'):
        return False, pi_res.get('error') or f"Failed to start hardware on Pi {AMR_PI_IP}"

    ok, err = _wait_loop(56, check_pi_health=True)
    if ok:
        return True, None
    return False, err or (
        f"Pi sensors (/scan, /odometry/filtered) not detected from {AMR_PI_IP}. "
        f"IMU/LiDAR did not publish within timeout — check USB/I2C wiring and Pi logs."
    )


class SystemLaunchManager:
    """Manages the full navigation stack lifecycle for both Simulation and Physical Hardware:
       - sim mode: navigation_launch.py (Gazebo + AMCL) + route_runner + bt_manager + graph_visualizer
       - hardware mode: hardware_navigation.launch.py (map_server + AMCL + auto_localizer + route_runner + run_logger)
    """
    def __init__(self, workspace_root: Path):
        self.ws_root = workspace_root
        self.state = 'STOPPED'  # 'STOPPED' | 'STARTING' | 'RUNNING' | 'STOPPING' | 'ERROR'
        self.active_map = _default_map_for_mode(AMR_MODE)
        self.stage = 0  # 0: Stopped, 1..3: Starting, 4: Running
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
        self.active_log_path: Optional[str] = None
        self.error_msg: Optional[str] = None
        self._stop_pi_on_stop: bool = False
        self.lock = threading.Lock()
        self._cancel_event = threading.Event()
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

    @staticmethod
    def _is_external_nav_running() -> bool:
        try:
            res = subprocess.run(
                ['pgrep', '-f', r'[/]route_runner|[r]os2 launch agv_description (hardware_navigation|navigation_launch)'],
                capture_output=True, text=True
            )
            if res.returncode == 0 and res.stdout.strip():
                return True
        except Exception:
            pass
        if 'node' in globals() and globals()['node'] is not None:
            n = globals()['node']
            now = time.time()
            state_recent = (now - getattr(n, '_last_state_rx_time', 0.0)) < 3.0
            amcl_recent = (now - getattr(n, '_last_amcl_rx_time', 0.0)) < 5.0
            map_tf_locked = bool(getattr(n, '_using_map_tf', False) and (now - getattr(n, '_last_map_tf_time', 0.0)) < 2.5)
            if getattr(n, '_has_received_nav_state', False) and (state_recent or amcl_recent or map_tf_locked):
                return True
        return False

    def start(
        self,
        map_name: Optional[str] = None,
        world: Optional[str] = None,
        headless: bool = True,
        use_rviz: bool = False,
        auto_start_pi: bool = False,
        stop_pi_on_stop: bool = False,
    ) -> dict:
        with self.lock:
            if self.state in ('STARTING', 'RUNNING'):
                return {'ok': False, 'error': f'System is already {self.state.lower()}'}

            if self._is_external_nav_running():
                return {
                    'ok': False,
                    'error': 'An external navigation session is already running. Stop it before launching from the Web UI.'
                }

            if 'mapping_mgr' in globals() and globals()['mapping_mgr'].is_external_mapping_running():
                return {
                    'ok': False,
                    'error': 'An external hardware mapping session is running. Stop it before launching navigation.'
                }

            self._cancel_event.set()
            cancel_evt = threading.Event()
            self._cancel_event = cancel_evt

            target_map = (map_name or self.active_map or _default_map_for_mode(AMR_MODE)).strip()
            self.active_map = target_map
            self._stop_pi_on_stop = bool(stop_pi_on_stop) if AMR_MODE == 'hardware' else False
            self.state = 'STARTING'
            self.stage = 1
            if AMR_MODE == 'hardware':
                self.stage_text = f'1/3 Verifying Raspberry Pi sensors & launching hardware nav ({target_map})...'
            else:
                self.stage_text = f'1/3 Initializing simulation and map ({target_map})...'
            self.start_time = time.time()
            self.error_msg = None
            self.active_log_path = None

            # Clean any old stray processes first
            self._cleanup_processes()

            # Reset bridge node telemetry state & TF buffer for fresh run
            if 'node' in globals() and globals()['node'] is not None:
                try:
                    globals()['node'].reset_simulation_state()
                except Exception as e:
                    logger.warning(f"[SystemLaunchManager] Node reset warning: {e}")

            if AMR_MODE == 'hardware':
                self._worker_thread = threading.Thread(
                    target=self._start_hardware_sequence,
                    args=(cancel_evt, target_map, world, headless, use_rviz, bool(auto_start_pi)),
                    daemon=True
                )
            else:
                self._worker_thread = threading.Thread(
                    target=self._start_sequence,
                    args=(cancel_evt, target_map, world, headless, use_rviz),
                    daemon=True
                )
            self._worker_thread.start()

            return {'ok': True, 'state': self.state, 'map_name': target_map, 'mode': AMR_MODE}

    def _sync_graph_fallback(self, map_name: str):
        src_graph = _find_map_file(map_name, ('_graph.json', '.json'))
        if src_graph and src_graph.exists():
            for d in _mode_sync_dirs():
                if d.exists():
                    dest_graph = d / "warehouse_graph.json"
                    if src_graph.resolve() == dest_graph.resolve():
                        continue
                    try:
                        shutil.copy2(src_graph, dest_graph)
                    except Exception as e:
                        logger.warning(f"[SystemLaunchManager] Sync graph warning: {e}")

    def _start_hardware_sequence(
        self,
        cancel_evt: threading.Event,
        map_name: str,
        world: Optional[str],
        headless: bool = True,
        use_rviz: bool = False,
        auto_start_pi: bool = False,
    ):
        try:
            if cancel_evt.is_set():
                return

            if 'mapping_mgr' in globals() and globals()['mapping_mgr'] is not None:
                try:
                    if globals()['mapping_mgr'].state in ('MAPPING', 'ERROR'):
                        globals()['mapping_mgr'].stop()
                except Exception as e:
                    logger.warning(f"[SystemLaunchManager] Warning stopping mapping_mgr: {e}")

            if cancel_evt.is_set():
                return

            self._sync_graph_fallback(map_name)

            # 1. Verify Pi sensors (/scan & /odometry/filtered), optionally SSH-starting ./start_hardware.sh
            with self.lock:
                if cancel_evt.is_set() or self.state in ('STOPPING', 'STOPPED'):
                    return
                self.stage = 1
                self.stage_text = f'1/3 Checking Pi ({AMR_PI_IP}) /scan & /odometry/filtered...'

            n = globals().get('node')
            sensors_ok, sensor_err = _ensure_pi_sensors_ready(
                auto_start_pi=auto_start_pi,
                cancel_evt=cancel_evt,
                stop_check=lambda: self.state in ('STOPPING', 'STOPPED'),
                status_cb=lambda msg: setattr(self, 'stage_text', f'1/3 {msg}'),
            )
            if cancel_evt.is_set() or self.state in ('STOPPING', 'STOPPED'):
                return
            if not sensors_ok:
                raise RuntimeError(sensor_err or f"Pi sensors not ready on {AMR_PI_IP}")

            if cancel_evt.is_set():
                return

            # 2. Clear any remote Pi SLAM & sync clock before launching hardware navigation
            with self.lock:
                if cancel_evt.is_set() or self.state in ('STOPPING', 'STOPPED'):
                    return
                self.stage = 2
                self.stage_text = f'2/3 Syncing Pi clock & launching hardware navigation ({map_name})...'

            pi_remote_mgr.clear_remote_slam_tf(AMR_PI_IP)
            pi_remote_mgr.sync_clock(AMR_PI_IP)

            if cancel_evt.is_set() or self.state in ('STOPPING', 'STOPPED'):
                return

            # Kill lingering SLAM/static map->odom TF so AMCL owns map->odom
            for pat in ['slam_toolbox', 'static_transform_publisher.*map.*odom']:
                try:
                    subprocess.run(['pkill', '-f', pat], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                except Exception:
                    pass

            # Prepare run_logs file
            log_dir = self.ws_root / "run_logs"
            log_dir.mkdir(parents=True, exist_ok=True)
            ts = time.strftime('%Y%m%d_%H%M%S')
            launch_log_path = log_dir / f"hardware_nav_{ts}.log"
            latest_link = log_dir / "latest_launch.log"
            try:
                if latest_link.is_symlink() or latest_link.exists():
                    latest_link.unlink()
                latest_link.symlink_to(launch_log_path)
            except Exception:
                pass
            self.active_log_path = str(launch_log_path)

            # Resolve map YAML or map stem
            yaml_p = _find_map_file(map_name, ('.yaml',), mode='hardware')
            map_arg = str(yaml_p) if yaml_p else map_name

            _configure_dds_environment('hardware', AMR_PI_IP)
            dds_uri = os.environ.get('CYCLONEDDS_URI', '')
            rviz_flag = 'true' if use_rviz else 'false'
            display_env = os.environ.get('DISPLAY', ':0')
            rviz_env_prefix = (
                f"export DISPLAY=\"{display_env}\" && "
                f"export QT_QPA_PLATFORM=xcb && "
                f"export LIBGL_ALWAYS_SOFTWARE=1 && "
                f"export GALLIUM_DRIVER=llvmpipe && "
                f"export MESA_GL_VERSION_OVERRIDE=4.5 && "
            ) if use_rviz else ""

            # Keep exact hardware navigation speeds & use_bt:=false
            hw_cmd = (
                rviz_env_prefix
                + f"unset ROS_LOCALHOST_ONLY && "
                f"export ROS_DOMAIN_ID=7 && "
                f"export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && "
                + (f"export CYCLONEDDS_URI=\"{dds_uri}\" && " if dds_uri else "")
                + f"source /opt/ros/jazzy/setup.bash && "
                f"source {self.ws_root}/install/setup.bash && "
                f"ros2 launch agv_description hardware_navigation.launch.py "
                f"map:=\"{map_arg}\" "
                f"max_linear_speed:={HW_NAV_LIMITS['max_linear_speed']} "
                f"cruise_linear_speed:={HW_NAV_LIMITS['cruise_linear_speed']} "
                f"max_angular_speed:={HW_NAV_LIMITS['max_angular_speed']} "
                f"lookahead_dist:={HW_NAV_LIMITS['lookahead_dist']} "
                f"use_bt:=false "
                f"use_rviz:={rviz_flag}"
            )
            if cancel_evt.is_set() or self.state in ('STOPPING', 'STOPPED'):
                return

            logger.info(f"[SystemLaunchManager] Starting hardware_navigation: {hw_cmd}")
            with self.lock:
                if cancel_evt.is_set() or self.state in ('STOPPING', 'STOPPED'):
                    return
                self.nav_log = open(launch_log_path, 'w')
                self.nav_proc = subprocess.Popen(
                    hw_cmd,
                    shell=True,
                    executable='/bin/bash',
                    cwd=str(self.ws_root),
                    preexec_fn=os.setsid,
                    stdout=self.nav_log,
                    stderr=subprocess.STDOUT
                )

            # Wait for hardware_navigation nodes (map_server, amcl, auto_localizer, route_runner) to come online
            with self.lock:
                if cancel_evt.is_set() or self.state in ('STOPPING', 'STOPPED'):
                    return
                self.stage = 3
                self.stage_text = f'3/3 Waiting for AMCL, Auto-Localizer & Route Runner ({map_name})...'

            for _ in range(10):
                if cancel_evt.wait(0.8) or cancel_evt.is_set() or self.state in ('STOPPING', 'STOPPED'):
                    return
                if self.nav_proc.poll() is not None:
                    tail = self._tail_log(str(launch_log_path))
                    raise RuntimeError(f"hardware_navigation.launch.py exited early: {tail}")
                if n is not None and (time.time() - getattr(n, '_last_state_rx_time', 0.0)) < 2.0:
                    break

            with self.lock:
                if not cancel_evt.is_set() and self.state not in ('STOPPING', 'STOPPED'):
                    self.state = 'RUNNING'
                    self.stage = 4
                    self.stage_text = f'Hardware Nav Online ({map_name})'
                    logger.info(f"[SystemLaunchManager] Hardware navigation stack is RUNNING on {map_name}!")
        except Exception as e:
            with self.lock:
                if cancel_evt.is_set() or self.state in ('STOPPING', 'STOPPED'):
                    self._cleanup_processes()
                    return
                self.state = 'ERROR'
                self.error_msg = str(e)
                self.stage_text = f'Error: {e}'
                logger.error(f"[SystemLaunchManager] Hardware startup failed: {e}")
                self._cleanup_processes()

    def _start_sequence(self, cancel_evt: threading.Event, map_name: str, world: Optional[str], headless: bool = True, use_rviz: bool = False):
        try:
            if cancel_evt.is_set():
                return

            # Stop any active mapping session first
            if 'mapping_mgr' in globals() and globals()['mapping_mgr'] is not None:
                try:
                    globals()['mapping_mgr'].stop()
                except Exception as e:
                    logger.warning(f"[SystemLaunchManager] Warning stopping mapping_mgr: {e}")

            if cancel_evt.is_set():
                return

            self._sync_graph_fallback(map_name)

            # 1. Launch navigation_launch.py
            world_arg = f"world:={world}" if world else ""
            headless_arg = f"headless:={'true' if headless else 'false'}"
            rviz_arg = f"use_rviz:={'true' if use_rviz else 'false'}"
            nav_cmd = (
                f"export ROS_LOCALHOST_ONLY=1 && "
                f"export LIBGL_ALWAYS_SOFTWARE=1 && "
                f"export MESA_GL_VERSION_OVERRIDE=3.3 && "
                f"source /opt/ros/jazzy/setup.bash && "
                f"source {self.ws_root}/install/setup.bash && "
                f"ros2 launch agv_description navigation_launch.py map:={map_name} {world_arg} {headless_arg} {rviz_arg}"
            )
            logger.info(f"[SystemLaunchManager] Starting navigation_launch: {nav_cmd}")
            with self.lock:
                if cancel_evt.is_set() or self.state in ('STOPPING', 'STOPPED'):
                    return
                self.nav_log = open('/tmp/amr_navigation.log', 'w')
                self.active_log_path = '/tmp/amr_navigation.log'
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
                if cancel_evt.wait(1.0) or cancel_evt.is_set() or self.state in ('STOPPING', 'STOPPED'):
                    return
                if self.nav_proc.poll() is not None:
                    tail = self._tail_log('/tmp/amr_navigation.log')
                    raise RuntimeError(f"navigation_launch.py terminated unexpectedly: {tail}")

            # 2. Launch route_runner
            with self.lock:
                if cancel_evt.is_set() or self.state in ('STOPPING', 'STOPPED'):
                    return
                self.stage = 2
                self.stage_text = '2/3 Starting MPPI Route Runner...'

            runner_cmd = (
                f"export ROS_LOCALHOST_ONLY=1 && "
                f"source /opt/ros/jazzy/setup.bash && "
                f"source {self.ws_root}/install/setup.bash && "
                f"ros2 run agv_navigation route_runner --ros-args -p map_name:={map_name} -p use_sim_time:=true"
            )
            logger.info("[SystemLaunchManager] Starting route_runner...")
            with self.lock:
                if cancel_evt.is_set() or self.state in ('STOPPING', 'STOPPED'):
                    return
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

            if cancel_evt.wait(2.0) or cancel_evt.is_set() or self.state in ('STOPPING', 'STOPPED'):
                return
            if self.runner_proc.poll() is not None:
                tail = self._tail_log('/tmp/amr_route_runner.log')
                raise RuntimeError(f"route_runner terminated unexpectedly: {tail}")

            # 3. Launch bt_manager
            with self.lock:
                if cancel_evt.is_set() or self.state in ('STOPPING', 'STOPPED'):
                    return
                self.stage = 3
                self.stage_text = '3/3 Starting Behavior Tree Manager...'

            bt_cmd = (
                f"export ROS_LOCALHOST_ONLY=1 && "
                f"source /opt/ros/jazzy/setup.bash && "
                f"source {self.ws_root}/install/setup.bash && "
                f"ros2 run agv_navigation bt_manager --ros-args -p map_name:={map_name} -p use_sim_time:=true"
            )
            logger.info("[SystemLaunchManager] Starting bt_manager...")
            with self.lock:
                if cancel_evt.is_set() or self.state in ('STOPPING', 'STOPPED'):
                    return
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
                f"export ROS_LOCALHOST_ONLY=1 && "
                f"source /opt/ros/jazzy/setup.bash && "
                f"source {self.ws_root}/install/setup.bash && "
                f"ros2 run agv_navigation graph_visualizer --ros-args -p map_name:={map_name} -p use_sim_time:=true"
            )
            logger.info("[SystemLaunchManager] Starting graph_visualizer...")
            with self.lock:
                if cancel_evt.is_set() or self.state in ('STOPPING', 'STOPPED'):
                    return
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

            if cancel_evt.wait(1.0) or cancel_evt.is_set():
                return
            with self.lock:
                if cancel_evt.is_set() or self.state in ('STOPPING', 'STOPPED'):
                    return
                self.state = 'RUNNING'
                self.stage = 4
                self.stage_text = f'Running ({map_name})'
                logger.info(f"[SystemLaunchManager] Navigation stack is fully RUNNING on {map_name}!")

            # Auto-spawn dynamic obstacle into Gazebo if one was chosen
            if not cancel_evt.is_set() and 'node' in globals() and globals()['node'] is not None:
                n = globals()['node']
                if getattr(n, 'obstacle_pose', None) and n.obstacle_pose.get('active'):
                    try:
                        n.spawn_or_move_obstacle(n.obstacle_pose['x'], n.obstacle_pose['y'], n.obstacle_pose.get('yaw', 0.0))
                    except Exception as err:
                        logger.warning(f"[SystemLaunchManager] Auto-spawn obstacle error: {err}")
        except Exception as e:
            with self.lock:
                if cancel_evt.is_set() or self.state in ('STOPPING', 'STOPPED'):
                    self._cleanup_processes()
                    return
                self.state = 'ERROR'
                self.error_msg = str(e)
                self.stage_text = f'Error: {e}'
                logger.error(f"[SystemLaunchManager] Startup failed: {e}")
                self._cleanup_processes()

    def stop(self, stop_pi: Optional[bool] = None) -> dict:
        self._cancel_event.set()
        with self.lock:
            self.state = 'STOPPING'
            self.stage_text = 'Stopping navigation stack...'
            self._cleanup_processes()
            should_stop_pi = (
                AMR_MODE == 'hardware'
                and (stop_pi if stop_pi is not None else self._stop_pi_on_stop)
            )
            self._stop_pi_on_stop = False
            pi_stop_res = None
            if should_stop_pi:
                self.stage_text = 'Stopping Raspberry Pi hardware stack via SSH...'
                pi_stop_res = pi_remote_mgr.stop_hardware(AMR_PI_IP)
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
            return {'ok': True, 'state': self.state, 'pi_stopped': bool(pi_stop_res and pi_stop_res.get('ok'))}

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

        patterns = [
            'graph_visualizer', 'bt_manager', 'route_runner',
            'navigation_launch', 'hardware_navigation',
            'auto_localizer', 'run_logger',
            'map_server', 'amcl', 'lifecycle_manager',
        ]
        if AMR_MODE == 'sim':
            patterns.extend(['gz sim', 'parameter_bridge', 'ros_gz_bridge', 'ekf_node', 'robot_state_publisher'])
        for pattern in patterns:
            try:
                subprocess.run(['pkill', '-9', '-f', pattern], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except Exception:
                pass

    def get_status(self) -> dict:
        with self.lock:
            if self.state == 'RUNNING':
                if self.nav_proc and self.nav_proc.poll() is not None:
                    self.state = 'ERROR'
                    log_path = self.active_log_path or '/tmp/amr_navigation.log'
                    tail = self._tail_log(log_path)
                    self.error_msg = f'navigation launch terminated: {tail}'
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

            external_running = False
            if self.state == 'STOPPED':
                external_running = self._is_external_nav_running()

            uptime = round(time.time() - self.start_time, 1) if (self.start_time and self.state == 'RUNNING') else 0.0

            return {
                'ok': True,
                'mode': AMR_MODE,
                'state': self.state,
                'external_running': external_running,
                'stage': self.stage,
                'stage_text': 'Active (External Session)' if external_running else self.stage_text,
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
    def __init__(self, use_sim_time: Optional[bool] = None):
        is_sim = (AMR_MODE == 'sim') if use_sim_time is None else bool(use_sim_time)
        super().__init__(
            'amr_web_bridge',
            parameter_overrides=[
                rclpy.parameter.Parameter('use_sim_time', rclpy.parameter.Parameter.Type.BOOL, is_sim)
            ]
        )
        self.use_sim_time = is_sim
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
        self.auto_localize_client = self.create_client(Trigger, '/trigger_auto_relocalization') if _STD_SRVS_AVAILABLE else None
        self._nav_cancel_client = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal') if _CANCEL_GOAL_AVAILABLE else None

        latched_map_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, reliability=QoSReliabilityPolicy.RELIABLE)
        explore_status_qos = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, reliability=QoSReliabilityPolicy.RELIABLE)

        # --- Subscribers ---
        self.create_subscription(Odometry,      '/dynamic_obstacle/odom', self._obstacle_odom_cb, 10)
        self.create_subscription(Odometry,      '/odometry/filtered', self._odom_cb,    10)
        self.create_subscription(LaserScan,     '/scan',              self._scan_cb,    qos_profile_sensor_data)
        self.create_subscription(String,        '/agv_state',         self._state_cb,   10)
        self.create_subscription(NavPath,       '/agv_dense_path',    self._path_cb,    10)
        self.create_subscription(String,        '/obstacle_alert',    self._alert_cb,   10)
        self.create_subscription(Imu,           '/imu/data',          self._imu_cb,     qos_profile_sensor_data)
        self.create_subscription(String,        '/mission_progress',  self._mission_cb, 10)
        # AMCL pose — authoritative map-frame localization (VOLATILE matches both sim & hw AMCL)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, 10
        )
        # Live /map subscription from SLAM Toolbox
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, latched_map_qos)
        # Latched map-match & localization verification status from auto_localizer
        self.create_subscription(String, '/localization_status', self._loc_status_cb, latched_map_qos)

        # Exploration status & frontiers subscriptions
        if _EXPLORE_MSGS_AVAILABLE:
            self.create_subscription(
                ExploreStatus, '/explore/status', self._explore_status_cb, explore_status_qos
            )
        if _MARKER_MSGS_AVAILABLE:
            self.create_subscription(
                MarkerArray, '/explore/frontiers', self._explore_frontiers_cb, 10
            )

        # TF2 buffer for map->base_link lookups (real-time localization in map frame)
        self._amcl_pose_received = False
        self._using_map_tf = False
        self._last_map_tf_time = 0.0
        self._last_sim_time = 0.0
        self._last_scan_rx_time = 0.0
        self._last_odom_rx_time = 0.0
        self._last_amcl_rx_time = 0.0
        self._last_state_rx_time = 0.0
        self._has_received_nav_state = False
        self._estop_latched = False
        self.localization_status: Optional[dict] = None
        self._loc_status_rx_time = 0.0
        self._tf_buffer   = None
        self._tf_listener = None
        if _TF2_AVAILABLE:
            try:
                self._tf_buffer   = tf2_ros.Buffer()
                self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
                # Timer: TF lookup at 10 Hz
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
        self.active_map_name: str = _default_map_for_mode(AMR_MODE)
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
        self._map_lock = threading.Lock()
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
        """Reset all telemetry state, live SLAM buffers, waypoints, and TF buffer for a clean run."""
        logger.info("[bridge] Resetting telemetry state and clearing buffers...")
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
        self._using_map_tf = False
        self._last_map_tf_time = 0.0
        self._last_sim_time = 0.0
        self._last_scan_rx_time = 0.0
        self._last_odom_rx_time = 0.0
        self._last_amcl_rx_time = 0.0
        self._last_state_rx_time = 0.0
        self._has_received_nav_state = False
        self._estop_latched = False
        self.localization_status: Optional[dict] = None
        self._loc_status_rx_time = 0.0
        self.explore_raw_status = 'idle'
        self.explore_status_label = 'Not started'
        self.is_exploring = False
        self.is_explore_paused = False
        self.frontiers = []
        self.frontiers_count = 0
        with self._map_lock:
            self._has_live_map = False
            self._last_map_time = 0.0
            self._map_msg_count = 0
            self._live_map_width = 0
            self._live_map_height = 0
            self._live_map_b64 = None
            self._clean_map_b64 = None
            self._last_raw_occupancy = None
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
        if active == 'hospital_map':
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
        self._last_odom_rx_time = time.time()
        # Use odom only for velocity; pose comes from AMCL / TF (map frame)
        self.velocity['linear']  = msg.twist.twist.linear.x
        self.velocity['angular'] = msg.twist.twist.angular.z

        # Detect simulation restart (time jump backward > 2.0s) only in sim mode
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.use_sim_time and hasattr(self, '_last_sim_time') and stamp_sec > 0 and self._last_sim_time > 0:
            if stamp_sec < (self._last_sim_time - 2.0):
                logger.warning(f"[bridge] Detected simulation time jump backward ({self._last_sim_time:.2f}s -> {stamp_sec:.2f}s)! Resetting simulation state.")
                self.reset_simulation_state()
        self._last_sim_time = stamp_sec

        # Update velocity always from odom
        self.velocity['linear']  = round(float(msg.twist.twist.linear.x), 3)
        self.velocity['angular'] = round(float(msg.twist.twist.angular.z), 3)

        # Fallback: update pose from odom ONLY if map TF has not been received recently (<2s) and no AMCL
        has_recent_map_tf = (time.time() - getattr(self, '_last_map_tf_time', 0.0)) < 2.0
        if not has_recent_map_tf and not self._amcl_pose_received:
            self.pose['x']   = round(float(msg.pose.pose.position.x), 3)
            self.pose['y']   = round(float(msg.pose.pose.position.y), 3)
            q = msg.pose.pose.orientation
            self.pose['yaw'] = self._quat_to_yaw(q)

    def _amcl_cb(self, msg):
        """AMCL pose — map-frame, highest-confidence source for robot position."""
        self._amcl_pose_received = True
        self._last_amcl_rx_time = time.time()
        self.pose['x']   = round(float(msg.pose.pose.position.x), 3)
        self.pose['y']   = round(float(msg.pose.pose.position.y), 3)
        self.pose['yaw'] = self._quat_to_yaw(msg.pose.pose.orientation)

    def _tf_pose_cb(self):
        """10 Hz TF lookup: map->base_link as authoritative map-frame source."""
        if not _TF2_AVAILABLE or self._tf_buffer is None:
            return
        try:
            tf = self._tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05)
            )
            t = tf.transform.translation
            self.pose['x']   = round(float(t.x), 3)
            self.pose['y']   = round(float(t.y), 3)
            self.pose['yaw'] = self._quat_to_yaw(tf.transform.rotation)
            self._using_map_tf = True
            self._last_map_tf_time = time.time()
        except Exception:
            self._using_map_tf = False

    def _scan_cb(self, msg):
        self._last_scan_rx_time = time.time()
        self.scan_ranges    = list(msg.ranges)
        self.scan_angle_min = msg.angle_min
        self.scan_angle_inc = msg.angle_increment
        valid = [r for r in msg.ranges if not math.isnan(r) and not math.isinf(r) and r > 0]
        self.min_obstacle_dist = min(valid) if valid else float('inf')

    def _state_cb(self, msg):
        self._last_state_rx_time = time.time()
        self._has_received_nav_state = True
        self.nav_state = msg.data
        if msg.data == 'ESTOP':
            self._estop_latched = True
        elif self._estop_latched and msg.data in ('IDLE', 'PLANNING', 'NAVIGATING'):
            self._estop_latched = False
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

    def _loc_status_cb(self, msg):
        try:
            data = json.loads(msg.data)
            self.localization_status = data
            self._loc_status_rx_time = time.time()
        except Exception:
            pass

    def get_dispatch_block_error(self) -> Optional[str]:
        if self._estop_latched or self.nav_state == 'ESTOP':
            return 'E-STOP active. Clear E-STOP before dispatching navigation.'
        loc = self.localization_status
        if loc and not loc.get('verified', True):
            st = loc.get('status', '')
            if st in ('MAP_MISMATCH', 'MAP_MISSING', 'SCAN_MISSING'):
                return loc.get('message') or f'Navigation blocked: localization status is {st}.'
        return None

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
            w = msg.info.width
            h = msg.info.height
            res = msg.info.resolution
            ox = msg.info.origin.position.x
            oy = msg.info.origin.position.y
            raw_copy = None
            clean_b64 = None

            if w > 0 and h > 0 and len(msg.data) == w * h:
                raw_data = np.array(msg.data, dtype=np.int8).reshape((h, w))
                raw_copy = raw_data.copy()

                # -1 = unknown (dark slate), 0 = free (deep blue/black), >50 = occupied (cyan)
                rgb = np.zeros((h, w, 3), dtype=np.uint8)
                rgb[raw_data == -1] = [26, 32, 46]     # unknown
                rgb[raw_data == 0]  = [12, 16, 28]     # free space
                rgb[raw_data > 50]  = [0, 212, 255]    # wall / occupied
                rgb = np.flipud(rgb)
                pil_img = Image.fromarray(rgb)

                clean_buf = io.BytesIO()
                pil_img.save(clean_buf, format='PNG')
                clean_b64 = base64.b64encode(clean_buf.getvalue()).decode()

            with self._map_lock:
                self._live_map_width = w
                self._live_map_height = h
                self._live_map_resolution = res
                self._live_map_origin_x = ox
                self._live_map_origin_y = oy
                self._has_live_map = True
                self._last_map_time = time.time()
                self._map_msg_count += 1
                if raw_copy is not None and clean_b64 is not None:
                    self._last_raw_occupancy = raw_copy
                    self._clean_map_b64 = clean_b64
                    self._live_map_b64 = clean_b64
        except Exception as e:
            self.get_logger().error(f"Error processing /map: {e}")

    # -------- Publisher helpers --------

    def trigger_auto_relocalization(self, timeout_sec: float = 6.0) -> dict:
        if not _STD_SRVS_AVAILABLE or self.auto_localize_client is None:
            return {'ok': False, 'error': 'std_srvs Trigger not available in bridge'}
        if not self.auto_localize_client.wait_for_service(timeout_sec=1.5):
            return {
                'ok': False,
                'error': 'Auto-localizer service (/trigger_auto_relocalization) is not active yet.'
            }
        fut = self.auto_localize_client.call_async(Trigger.Request())
        done_evt = threading.Event()
        fut.add_done_callback(lambda _: done_evt.set())
        if not done_evt.wait(timeout=timeout_sec):
            return {'ok': False, 'error': 'Auto-relocalization request timed out.'}
        try:
            res = fut.result()
            return {'ok': bool(res.success), 'message': res.message}
        except Exception as e:
            return {'ok': False, 'error': str(e)}

    def publish_cmd_vel(self, linear: float, angular: float):
        if (self._estop_latched or self.nav_state == 'ESTOP') and (abs(linear) > 1e-4 or abs(angular) > 1e-4):
            return False
        lin = float(linear)
        ang = float(angular)
        if AMR_MODE == 'hardware':
            limits = _active_teleop_limits()
            min_lin = -limits.get('max_reverse_linear', limits['max_linear'])
            lin = max(min_lin, min(limits['max_linear'], lin))
            ang = max(-limits['max_angular'], min(limits['max_angular'], ang))
        t = Twist()
        t.linear.x  = lin
        t.angular.z = ang
        self.cmd_pub.publish(t)
        return True

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
        self._estop_latched = bool(active)
        b = Bool()
        b.data = active
        self.estop_pub.publish(b)
        if active:
            self.nav_state = 'ESTOP'
            try:
                self.cmd_pub.publish(Twist())
            except Exception:
                pass
            mm = globals().get('mapping_mgr')
            if mm is not None and (getattr(mm, 'run_explore', False) or getattr(mm, 'state', None) == 'MAPPING'):
                mm.pause_auto_explore(ros_node=self)
            try:
                self.publish_goal_sequence([])
            except Exception:
                pass
            if _CANCEL_GOAL_AVAILABLE and getattr(self, '_nav_cancel_client', None) is not None:
                try:
                    if self._nav_cancel_client.service_is_ready():
                        self._nav_cancel_client.call_async(CancelGoal.Request())
                except Exception:
                    pass
        elif self.nav_state == 'ESTOP':
            self.nav_state = 'IDLE'

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
        found_asset = _find_map_file(target_name, ('.yaml', '_graph.json', '.png', '.pgm'))
        if not found_asset:
            raise FileNotFoundError(f"Map '{target_name}' not found on disk.")

        self.active_map_name = target_name
        # Pre-cache graph for active map
        self.get_graph_data(target_name)

        # Synchronize active graph to mode-appropriate directory for ROS2 route_runner fallback
        src_graph = _find_map_file(target_name, ('_graph.json', '.json'))
        if src_graph and src_graph.exists():
            for d in _mode_sync_dirs():
                if d.exists():
                    dest_graph = d / "warehouse_graph.json"
                    if src_graph.resolve() == dest_graph.resolve():
                        continue
                    try:
                        shutil.copy2(src_graph, dest_graph)
                    except Exception as e:
                        self.get_logger().warning(f"Could not sync graph to {dest_graph}: {e}")

    def get_graph_data(self, map_name: Optional[str] = None) -> dict:
        target_name = (map_name or self.active_map_name or _default_map_for_mode(AMR_MODE)).strip()
        if target_name in self._cached_graphs:
            return self._cached_graphs[target_name]

        graph_path = _find_map_file(target_name, ('_graph.json', '.json'))
        if (graph_path is None or not graph_path.exists()) and target_name == 'warehouse':
            graph_path = _find_map_file('warehouse', ('_graph.json', '.json'))

        if graph_path and graph_path.exists():
            try:
                with open(graph_path, 'r', encoding='utf-8') as f:
                    raw = json.load(f)
                nodes = raw.get('nodes', [])
                edges = raw.get('edges', [])
                places = raw.get('places', [])
                vis_b64 = None
                vis_path = _find_map_file(target_name, ('_graph_vis.png', '_vis.png'))
                if vis_path and vis_path.exists():
                    try:
                        with open(vis_path, 'rb') as img_f:
                            vis_b64 = base64.b64encode(img_f.read()).decode('utf-8')
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
        with self._map_lock:
            return {
                'width': self._live_map_width,
                'height': self._live_map_height,
                'resolution': float(self._live_map_resolution),
                'origin_x': float(self._live_map_origin_x),
                'origin_y': float(self._live_map_origin_y),
                'msg_count': self._map_msg_count,
                'last_update': round(self._last_map_time, 2)
            }

    def get_live_map_image_b64(self, clean: bool = False) -> Optional[str]:
        with self._map_lock:
            if clean and self._clean_map_b64:
                return self._clean_map_b64
            return self._live_map_b64

    def get_map_verification_data(self) -> dict:
        with self._map_lock:
            if not self._has_live_map or self._last_raw_occupancy is None:
                return {'has_map': False, 'error': 'No map data available yet'}
            w = self._live_map_width
            h = self._live_map_height
            res = float(self._live_map_resolution)
            ox = float(self._live_map_origin_x)
            oy = float(self._live_map_origin_y)
            raw = self._last_raw_occupancy.copy()
            clean_b64 = self._clean_map_b64
            live_b64 = self._live_map_b64
            msg_count = self._map_msg_count

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
            'origin_x': round(ox, 3),
            'origin_y': round(oy, 3),
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
            'clean_image_b64': clean_b64,
            'overlay_image_b64': live_b64,
            'map_msg_count': msg_count
        }

    # -------- Snapshot --------

    def telemetry_snapshot(self) -> dict:
        obs_dist = self.min_obstacle_dist if not math.isinf(self.min_obstacle_dist) else None
        # Downsample scan to ~72 points (every ~5°) for the app & scale angle_inc by step
        scan_ds: List[float] = []
        scan_step = 1
        if self.scan_ranges:
            scan_step = max(1, len(self.scan_ranges) // 72)
            for i in range(0, len(self.scan_ranges), scan_step):
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

        now = time.time()
        scan_alive = (now - getattr(self, '_last_scan_rx_time', 0.0)) < 2.5
        odom_alive = (now - getattr(self, '_last_odom_rx_time', 0.0)) < 2.5
        map_tf_locked = bool(self._using_map_tf and (now - getattr(self, '_last_map_tf_time', 0.0)) < 2.5)
        amcl_alive = bool(self._amcl_pose_received and (now - getattr(self, '_last_amcl_rx_time', 0.0)) < 5.0)
        external_nav_active = bool(
            getattr(self, '_has_received_nav_state', False)
            and (scan_alive or map_tf_locked or (now - getattr(self, '_last_amcl_rx_time', 0.0)) < 5.0)
        )
        teleop_limits = HW_TELEOP_LIMITS if AMR_MODE == 'hardware' else SIM_TELEOP_LIMITS
        mapping_limits = HW_MAPPING_TELEOP_LIMITS if AMR_MODE == 'hardware' else SIM_TELEOP_LIMITS
        pi_ssh_snap = (
            pi_remote_mgr.get_cached_status(sensors_ready=(scan_alive and odom_alive))
            if (AMR_MODE == 'hardware' and 'pi_remote_mgr' in globals())
            else None
        )

        return {
            'mode':            AMR_MODE,
            'pi_ip':           AMR_PI_IP,
            'use_sim_time':    self.use_sim_time,
            'estop_active':    bool(self._estop_latched or self.nav_state == 'ESTOP'),
            'limits':          teleop_limits,
            'mapping_limits':  mapping_limits,
            'nav_limits':      HW_NAV_LIMITS if AMR_MODE == 'hardware' else None,
            'hardware_link': {
                'pi_ip':               AMR_PI_IP,
                'scan_alive':          scan_alive,
                'odom_alive':          odom_alive,
                'sensors_ready':       scan_alive and odom_alive,
                'map_tf_locked':       map_tf_locked,
                'amcl_localized':      amcl_alive or map_tf_locked,
                'external_nav_active': external_nav_active,
                'loc_verified':        bool(self.localization_status.get('verified', True)) if self.localization_status else (amcl_alive or map_tf_locked),
                'clock_synced':        pi_ssh_snap.get('clock_synced') if pi_ssh_snap else None,
                'clock_sync_at':       pi_ssh_snap.get('clock_sync_at') if pi_ssh_snap else None,
                'pi_ssh':              pi_ssh_snap,
            },
            'localization_status': self.localization_status,
            'pose':            self.pose,
            'velocity':        self.velocity,
            'min_obstacle_dist': round(obs_dist, 3) if obs_dist is not None else None,
            'nav_state':       self.nav_state,
            'path':            self.path_waypoints,
            'obstacle_alert':  self.obstacle_alert,
            'imu':             self.imu,
            'mission':         self.mission,
            'active_map':      getattr(self, 'active_map_name', _default_map_for_mode(AMR_MODE)),
            'active_place_nav': getattr(self, 'active_place_nav', None),
            'dynamic_obstacle': dyn_obs if AMR_MODE == 'sim' else None,
            'scan':            scan_ds,
            'scan_angle_min':  round(self.scan_angle_min, 4),
            'scan_angle_inc':  round(self.scan_angle_inc * scan_step, 5),
            'mapping':         mapping_mgr.get_status(self._has_live_map, self.get_live_map_info(), ros_node=self),
            'system_status':   system_mgr.get_status() if 'system_mgr' in globals() else {},
            'ts':              round(now, 3),
        }

    # -------- Map PNG with robot overlay --------

    def get_map_image_b64(self, map_name: Optional[str] = None, clean: bool = False) -> Optional[str]:
        """Return base64-encoded JPEG of map PNG (clean or with robot dot overlaid)."""
        target = (map_name or self.active_map_name or 'hospital_map').strip()
        img_path = resolve_map_image_path(target)
        if not img_path or not img_path.exists():
            return None

        try:
            from PIL import Image, ImageDraw
            img = Image.open(img_path).convert('RGBA')
            if not clean:
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
            img.convert('RGB').save(buf, format='JPEG', quality=85)
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
    map_name: Optional[str] = None
    world: Optional[str] = None
    headless: bool = True
    use_rviz: bool = False
    auto_start_pi: bool = False
    stop_pi_on_stop: bool = False


class SystemStopRequest(BaseModel):
    stop_pi: Optional[bool] = None

class StartMappingRequest(BaseModel):
    map_name: Optional[str] = None
    world: Optional[str] = 'hospital.world'
    run_explore: bool = False
    headless: bool = True
    use_rviz: bool = False
    auto_start_pi: bool = False
    stop_pi_on_stop: bool = False


class MappingStopRequest(BaseModel):
    stop_pi: Optional[bool] = None

class GuiLaunchRequest(BaseModel):
    app: str = "rviz"  # "rviz" | "gazebo"
    mode: Optional[str] = None  # "nav" | "explore"

class SaveMapRequest(BaseModel):
    map_name: Optional[str] = None

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


class ModeSwitchRequest(BaseModel):
    mode: str  # 'sim' | 'hardware'
    pi_ip: Optional[str] = None


# -------- REST endpoints --------

@app.get('/api/mode')
async def get_mode():
    return {
        'ok': True,
        'mode': AMR_MODE,
        'pi_ip': AMR_PI_IP,
        'use_sim_time': (AMR_MODE == 'sim'),
        'teleop_limits': HW_TELEOP_LIMITS if AMR_MODE == 'hardware' else SIM_TELEOP_LIMITS,
        'mapping_limits': HW_MAPPING_TELEOP_LIMITS if AMR_MODE == 'hardware' else SIM_TELEOP_LIMITS,
        'nav_limits': HW_NAV_LIMITS if AMR_MODE == 'hardware' else None,
    }


@app.post('/api/mode')
async def post_mode(req: ModeSwitchRequest):
    """Switch between 'sim' and 'hardware' cleanly by re-execing the bridge process with fresh DDS env."""
    global _PENDING_REEXEC
    target_mode = (req.mode or '').strip().lower()
    if target_mode in ('hw', 'hardware'):
        target_mode = 'hardware'
    elif target_mode in ('sim', 'simulation'):
        target_mode = 'sim'
    else:
        return JSONResponse({'ok': False, 'error': f"Invalid mode '{req.mode}'. Expected 'sim' or 'hardware'."}, status_code=400)

    target_pi_ip = (req.pi_ip or AMR_PI_IP or '10.244.248.208').strip()
    if target_mode == 'hardware' and not re.match(r'^\d{1,3}(\.\d{1,3}){3}$', target_pi_ip):
        return JSONResponse({'ok': False, 'error': f"Invalid Pi IP address '{target_pi_ip}'."}, status_code=400)

    if target_mode == AMR_MODE and target_pi_ip == AMR_PI_IP:
        return {'ok': True, 'mode': AMR_MODE, 'pi_ip': AMR_PI_IP, 'restarting': False}

    _PENDING_REEXEC = (target_mode, target_pi_ip)
    _SHUTDOWN_FLAG.set()

    def _trigger_reexec():
        time.sleep(0.15)
        try:
            system_mgr.stop()
        except Exception:
            pass
        try:
            mapping_mgr.stop()
        except Exception:
            pass
        if _UVICORN_SERVER is not None:
            _UVICORN_SERVER.should_exit = True

    threading.Thread(target=_trigger_reexec, daemon=True).start()
    return {
        'ok': True,
        'mode': target_mode,
        'pi_ip': target_pi_ip,
        'restarting': True,
        'message': f"Switching bridge to {target_mode.upper()} mode..."
    }

@app.get('/api/status')
async def get_status():
    return node.telemetry_snapshot() if node else {'error': 'ROS node not ready'}

@app.get('/api/map/active')
async def get_active_map():
    curr = node.active_map_name if node else 'hospital_map'
    return {'ok': True, 'active_map': curr}

@app.post('/api/map/active')
async def post_active_map(req: SetActiveMapRequest):
    if not node:
        return JSONResponse({'ok': False, 'error': 'ROS node not ready'}, status_code=503)
    try:
        node.set_active_map(req.map_name)
        return {'ok': True, 'active_map': node.active_map_name}
    except Exception as e:
        return JSONResponse({'ok': False, 'error': str(e)}, status_code=400)

@app.get('/api/map')
async def get_map(map_name: Optional[str] = None, clean: bool = False):
    if node is None:
        return JSONResponse({'error': 'ROS node not ready'}, status_code=503)
    target = (map_name or node.active_map_name or 'hospital_map').strip()
    data = node.get_map_image_b64(target, clean=clean)
    if data is None:
        return JSONResponse({'error': f'Map image for {target} not found'}, status_code=404)
    return {'image': data, 'encoding': 'jpeg/base64', 'map_name': target}

@app.post('/api/cmd_vel')
async def post_cmd_vel(req: CmdVelRequest):
    if node:
        if node._estop_latched and (req.linear != 0.0 or req.angular != 0.0):
            return JSONResponse({'ok': False, 'error': 'E-STOP active. Clear E-STOP before driving.'}, status_code=409)
        # Seamless manual takeover: If user drives manually with non-zero command, pause auto-explore so Nav2 does not fight the user
        if (req.linear != 0.0 or req.angular != 0.0) and mapping_mgr.run_explore:
            mapping_mgr.pause_auto_explore(ros_node=node)
        node.publish_cmd_vel(req.linear, req.angular)
    return {'ok': True}

@app.post('/api/localization/auto')
async def post_auto_localize():
    """Trigger zero-prior global scan-matching relocalization on hardware auto_localizer."""
    if node is None:
        return JSONResponse({'ok': False, 'error': 'ROS node not ready'}, status_code=503)
    res = await asyncio.get_event_loop().run_in_executor(None, node.trigger_auto_relocalization)
    if not res.get('ok'):
        return JSONResponse(res, status_code=400)
    return res

@app.post('/api/goal')
async def post_goal(req: GoalRequest):
    if node:
        block_err = node.get_dispatch_block_error()
        if block_err:
            return JSONResponse({'ok': False, 'error': block_err}, status_code=409)
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
        block_err = node.get_dispatch_block_error()
        if block_err:
            return JSONResponse({'ok': False, 'error': block_err}, status_code=409)
        node.publish_goal_sequence(req.nodes)
    return {'ok': True}

@app.get('/api/map/metadata')
async def get_map_metadata(map_name: Optional[str] = None):
    target = (map_name or (node.active_map_name if node else None) or 'hospital_map').strip()
    return get_map_metadata_by_name(target)

@app.get('/api/map/raw')
async def get_map_raw(map_name: Optional[str] = None, theme: str = 'dark'):
    """Serve the clean architectural floorplan (no graph overlay) for dark or light theme."""
    target = (map_name or (node.active_map_name if node else None) or 'hospital_map').strip()
    img_path = resolve_map_image_path(target, theme=theme)
    if not img_path or not img_path.exists():
        return JSONResponse({'error': f'Map image for {target} not found'}, status_code=404)
    return FileResponse(img_path, media_type='image/png', headers={
        'Cache-Control': 'no-cache, no-store, must-revalidate',
        'Pragma': 'no-cache',
        'Expires': '0',
    })

@app.get('/api/map/clean')
async def get_map_clean(map_name: Optional[str] = None, theme: str = 'dark'):
    """Alias for /api/map/raw — explicitly returns the clean floorplan."""
    return await get_map_raw(map_name, theme=theme)

@app.get('/api/graph')
async def get_graph(map_name: Optional[str] = None):
    if node is None:
        return JSONResponse({'error': 'ROS node not ready'}, status_code=503)
    target = (map_name or node.active_map_name or 'hospital_map').strip()
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
    target_name = (req.map_name or mapping_mgr.map_name or _default_mapping_name_for_mode(AMR_MODE)).strip()
    res = mapping_mgr.start(
        map_name=target_name,
        world=req.world,
        run_explore=req.run_explore,
        headless=req.headless,
        use_rviz=req.use_rviz,
        auto_start_pi=req.auto_start_pi,
        stop_pi_on_stop=req.stop_pi_on_stop,
    )
    has_map = node._has_live_map if node else False
    map_info = node.get_live_map_info() if node else {}
    status_payload = mapping_mgr.get_status(has_map, map_info, ros_node=node)
    if not res.get('ok'):
        return JSONResponse(
            {'ok': False, 'error': res.get('error', 'Failed to start mapping'), 'mapping': status_payload},
            status_code=int(res.get('status_code', 500)),
        )
    if node:
        node.active_map_name = target_name
    return {'ok': True, 'mapping': status_payload}

@app.post('/api/mapping/stop')
async def post_mapping_stop(req: Optional[MappingStopRequest] = Body(default=None)):
    stop_pi = req.stop_pi if req is not None else None
    stop_res = mapping_mgr.stop(stop_pi=stop_pi)
    has_map = node._has_live_map if node else False
    map_info = node.get_live_map_info() if node else {}
    return {'ok': True, 'pi_stopped': bool(stop_res.get('pi_stopped')), 'mapping': mapping_mgr.get_status(has_map, map_info, ros_node=node)}

@app.post('/api/mapping/redo')
async def post_mapping_redo(req: Optional[MappingStopRequest] = Body(default=None)):
    stop_pi = req.stop_pi if req is not None else None
    mapping_mgr.redo(stop_pi=stop_pi)
    has_map = node._has_live_map if node else False
    map_info = node.get_live_map_info() if node else {}
    return {'ok': True, 'mapping': mapping_mgr.get_status(has_map, map_info, ros_node=node)}

@app.post('/api/mapping/explore/start')
async def post_explore_start():
    if node is None:
        return JSONResponse({'ok': False, 'error': 'ROS node not ready'}, status_code=503)
    res = mapping_mgr.start_auto_explore(ros_node=node)
    has_map = node._has_live_map if node else False
    map_info = node.get_live_map_info() if node else {}
    status_payload = mapping_mgr.get_status(has_map, map_info, ros_node=node)
    if not res.get('ok'):
        return JSONResponse(
            {'ok': False, 'error': res.get('error', 'Failed to start auto-explore'), 'mapping': status_payload},
            status_code=int(res.get('status_code', 400)),
        )
    return {'ok': True, 'mapping': status_payload}

@app.post('/api/mapping/explore/pause')
async def post_explore_pause():
    if node is None:
        return JSONResponse({'ok': False, 'error': 'ROS node not ready'}, status_code=503)
    ok = mapping_mgr.pause_auto_explore(ros_node=node)
    has_map = node._has_live_map if node else False
    map_info = node.get_live_map_info() if node else {}
    return {'ok': ok, 'mapping': mapping_mgr.get_status(has_map, map_info, ros_node=node)}

@app.post('/api/mapping/explore/resume')
async def post_explore_resume():
    if node is None:
        return JSONResponse({'ok': False, 'error': 'ROS node not ready'}, status_code=503)
    res = mapping_mgr.resume_auto_explore(ros_node=node)
    has_map = node._has_live_map if node else False
    map_info = node.get_live_map_info() if node else {}
    status_payload = mapping_mgr.get_status(has_map, map_info, ros_node=node)
    if not res.get('ok'):
        return JSONResponse(
            {'ok': False, 'error': res.get('error', 'Failed to resume auto-explore'), 'mapping': status_payload},
            status_code=int(res.get('status_code', 409)),
        )
    return {'ok': True, 'mapping': status_payload}

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
    target_name = (
        req.map_name
        or (mapping_mgr.map_name if mapping_mgr.state == 'MAPPING' else None)
        or (node.active_map_name if node else None)
        or mapping_mgr.map_name
        or _default_map_for_mode(AMR_MODE)
    ).strip()
    mapping_mgr.map_name = target_name
    res = mapping_mgr.save_map(target_name, ros_node=node)
    if res.get('ok') and node:
        node.active_map_name = target_name
    return res

@app.get('/api/mapping/saved_maps')
async def get_saved_maps():
    maps = []
    seen_names = set()
    curr_active = node.active_map_name if node else _default_map_for_mode(AMR_MODE)
    search_dirs = [
        (_HW_MAPS_DIR, 'hardware'),
        (_SIM_MAPS_DIR, 'simulation'),
    ]
    if AMR_MODE == 'sim':
        search_dirs = [
            (_SIM_MAPS_DIR, 'simulation'),
            (_HW_MAPS_DIR, 'hardware'),
        ]

    for maps_dir, category in search_dirs:
        if not maps_dir.exists():
            continue
        for yaml_file in sorted(maps_dir.glob("*.yaml")):
            name = yaml_file.stem
            # Deduplicate only if exact same name already added in active mode's primary list,
            # or allow distinct (category, name) entries; keep unique name within each category
            if (category, name) in seen_names:
                continue
            seen_names.add((category, name))

            pgm_file = maps_dir / f"{name}.pgm"
            png_file = maps_dir / f"{name}.png"
            graph_file = maps_dir / f"{name}_graph.json"
            places_file = maps_dir / f"{name}_places.json"
            meta = get_map_metadata_by_name(name)

            node_count = 0
            graph_places_count = 0
            if graph_file.exists():
                try:
                    with open(graph_file, 'r', encoding='utf-8') as gf:
                        gd = json.load(gf)
                        node_count = len(gd.get('nodes', []))
                        if isinstance(gd.get('places'), list):
                            graph_places_count = len(gd['places'])
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
            if place_count == 0 and graph_places_count > 0:
                place_count = graph_places_count

            maps.append({
                'name': name,
                'category': category,
                'yaml': str(yaml_file.relative_to(_WORKSPACE_ROOT)),
                'has_pgm': pgm_file.exists(),
                'has_png': png_file.exists() or pgm_file.exists(),
                'has_graph': graph_file.exists(),
                'node_count': node_count,
                'place_count': place_count,
                'resolution': meta['resolution'],
                'dimensions': f"{meta['width']}×{meta['height']}",
                'size_kb': round(yaml_file.stat().st_size / 1024, 2),
                'is_active': (name == curr_active)
            })
    return {'maps': maps, 'active_map': curr_active, 'mode': AMR_MODE}

@app.post('/api/mapping/graph/extract')
async def post_extract_graph(req: ExtractGraphRequest = ExtractGraphRequest()):
    """Phase 3: Extracts topological navigation roadmap (nodes & edges) from map YAML."""
    if graph_extractor is None:
        return JSONResponse({'error': 'Graph extractor module not available'}, status_code=500)

    target_name = (req.map_name or (node.active_map_name if node else None) or mapping_mgr.saved_map_name or mapping_mgr.map_name or _default_map_for_mode(AMR_MODE)).strip()
    yaml_path = _find_map_file(target_name, ('.yaml',))
    if yaml_path is None:
        if target_name in ('warehouse_map', 'warehouse'):
            yaml_path = _find_map_file('warehouse_map', ('.yaml',))
        elif target_name == 'hospital_map':
            yaml_path = _find_map_file('hospital_map', ('.yaml',))

    if not yaml_path or not yaml_path.exists():
        return JSONResponse({'error': f'Map YAML file not found for "{target_name}"'}, status_code=404)

    try:
        report = graph_extractor.extract_graph(
            str(yaml_path),
            robot_radius=req.robot_radius,
            safety_margin=req.safety_margin,
            global_search_radius=req.search_radius,
            step_corridor=req.step_corridor,
            step_medium=req.step_medium,
            step_open=req.step_open,
            sync_default=False,
        )

        # Update mode-appropriate directories so downstream ROS nodes see the extracted graph without cross-polluting sim/hw
        json_p = Path(report['json_path'])
        vis_p = Path(report['vis_path']) if report.get('vis_path') else None
        for sync_dir in _mode_sync_dirs():
            if sync_dir.exists() and json_p.exists():
                dest_named = sync_dir / json_p.name
                if json_p.resolve() != dest_named.resolve():
                    try:
                        shutil.copy2(json_p, dest_named)
                    except Exception as e:
                        print(f"[bridge] Note: Could not copy {json_p.name} to {sync_dir}: {e}")
                dest_wh = sync_dir / "warehouse_graph.json"
                if json_p.resolve() != dest_wh.resolve():
                    try:
                        shutil.copy2(json_p, dest_wh)
                    except Exception as e:
                        print(f"[bridge] Note: Could not copy warehouse_graph.json to {sync_dir}: {e}")

        if AMR_MODE == 'hardware' and 'pi_remote_mgr' in globals():
            sync_files = [p for p in (json_p, vis_p) if p and p.exists()]
            sync_res = pi_remote_mgr.sync_maps_to_pi(sync_files, AMR_PI_IP)
            report['pi_synced'] = bool(sync_res.get('ok'))
            if not sync_res.get('ok'):
                report['pi_sync_error'] = sync_res.get('error')

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
    if map_name:
        cand_json = _find_map_file(map_name, ('_graph.json', '.json'))
        if cand_json and cand_json.exists():
            try:
                with open(cand_json, 'r', encoding='utf-8') as f:
                    raw = json.load(f)
                vis_b64 = None
                cand_vis = _find_map_file(map_name, ('_graph_vis.png', '_vis.png'))
                if cand_vis and cand_vis.exists():
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

    # Fallback to reading active map graph or default graph
    active_name = getattr(node, 'active_map_name', _default_map_for_mode(AMR_MODE))
    fallback_graph = _find_map_file(active_name, ('_graph.json', '.json')) or _GRAPH_JSON_PATH
    if fallback_graph and fallback_graph.exists():
        try:
            with open(fallback_graph, 'r', encoding='utf-8') as f:
                raw = json.load(f)
            vis_b64 = None
            stem = fallback_graph.stem.replace('_graph', '')
            vc = _find_map_file(stem, ('_graph_vis.png', '_vis.png'))
            if vc and vc.exists():
                try:
                    with open(vc, 'rb') as img_f:
                        vis_b64 = base64.b64encode(img_f.read()).decode('utf-8')
                except Exception:
                    pass
            return {
                'ok': True,
                'graph': {
                    'map_name': stem,
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
    target_name = (map_name or (node.active_map_name if node else None) or mapping_mgr.saved_map_name or mapping_mgr.map_name or _default_map_for_mode(AMR_MODE)).strip()
    cand = _find_map_file(target_name, ('_graph.json', '.json'))
    if not cand or not cand.exists():
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
    target_name = (req.map_name or (node.active_map_name if node else None) or mapping_mgr.saved_map_name or mapping_mgr.map_name or _default_map_for_mode(AMR_MODE)).strip()
    cand = _find_map_file(target_name, ('_graph.json', '.json'))
    if not cand or not cand.exists():
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
        cruise_v = HW_NAV_LIMITS['cruise_linear_speed'] if AMR_MODE == 'hardware' else 0.4
        return {
            'ok': True,
            'start_node': req.start_node,
            'goal_node': req.goal_node,
            'path': path,
            'waypoints': waypoints,
            'total_distance_m': tot_dist,
            'hop_count': len(path) - 1,
            'est_time_sec': round(tot_dist / max(0.05, cruise_v), 1),
            'min_clearance_m': round(min(clearances), 3) if clearances else 0.0
        }
    except Exception as e:
        return JSONResponse({'error': f'Routing calculation failed: {str(e)}'}, status_code=500)

# -------- Phase 5: Name Places / Nodes Endpoints --------

@app.get('/api/mapping/places')
async def get_named_places(map_name: Optional[str] = None):
    """Phase 5: Retrieve all named places and stations for a roadmap."""
    target_name = (map_name or (node.active_map_name if node else None) or mapping_mgr.saved_map_name or mapping_mgr.map_name or _default_map_for_mode(AMR_MODE)).strip()

    candidates = []
    for d in _get_map_search_dirs():
        candidates.append(d / f"{target_name}_places.json")
    for d in _get_map_search_dirs():
        candidates.append(d / f"{target_name}_graph.json")

    for cand in candidates:
        if cand.exists():
            try:
                with open(cand, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                if isinstance(data, list) and len(data) > 0:
                    return {'ok': True, 'map_name': target_name, 'places': data, 'source_file': cand.name}
                elif isinstance(data, dict) and 'places' in data and isinstance(data['places'], list) and len(data['places']) > 0:
                    return {'ok': True, 'map_name': target_name, 'places': data['places'], 'source_file': cand.name}
            except Exception as e:
                logger.warning(f"Failed to read places from {cand}: {e}")

    return {'ok': True, 'map_name': target_name, 'places': [], 'source_file': None}


@app.post('/api/mapping/places/save')
async def post_save_named_places(req: SavePlacesRequest):
    """Phase 5: Persist named places into active graph and dedicated places files."""
    active_now = node.active_map_name if node else _default_map_for_mode(AMR_MODE)
    target_name = (req.map_name or active_now).strip()
    maps_dir = _get_primary_map_dir(target_name)
    maps_dir.mkdir(parents=True, exist_ok=True)
    places_data = [p.dict() for p in req.places]

    saved_files = []

    # 1. Save dedicated <target_name>_places.json in primary map dir
    places_file = maps_dir / f"{target_name}_places.json"
    with open(places_file, 'w', encoding='utf-8') as f:
        json.dump(places_data, f, indent=2)
    saved_files.append(str(places_file.relative_to(_WORKSPACE_ROOT)))

    # 2. Inject places into corresponding <target_name>_graph.json in mode-appropriate dirs
    sync_paths: List[Path] = [places_file]
    for d in _mode_sync_dirs():
        target_graph = d / f"{target_name}_graph.json"
        if target_graph.exists():
            try:
                with open(target_graph, 'r', encoding='utf-8') as f:
                    raw_graph = json.load(f)
                if isinstance(raw_graph, dict):
                    raw_graph['places'] = places_data
                    with open(target_graph, 'w', encoding='utf-8') as f:
                        json.dump(raw_graph, f, indent=2)
                    saved_files.append(str(target_graph.relative_to(_WORKSPACE_ROOT)))
                    sync_paths.append(target_graph)
            except Exception as e:
                logger.error(f"Error injecting places into {target_graph}: {e}")

    pi_synced = None
    if AMR_MODE == 'hardware' and 'pi_remote_mgr' in globals():
        pi_res = pi_remote_mgr.sync_maps_to_pi(sync_paths, AMR_PI_IP)
        pi_synced = bool(pi_res.get('ok'))

    if node and hasattr(node, '_cached_graphs') and target_name in node._cached_graphs:
        node._cached_graphs[target_name]['places'] = places_data
    if node and hasattr(node, '_cached_graph') and node.active_map_name == target_name:
        if isinstance(node._cached_graph, dict):
            node._cached_graph['places'] = places_data

    return {
        'ok': True,
        'map_name': target_name,
        'count': len(places_data),
        'places': places_data,
        'pi_synced': pi_synced,
        'saved_files': list(set(saved_files))
    }


@app.post('/api/mapping/places/default_templates')
async def post_generate_default_templates(map_name: Optional[str] = None):
    """Phase 5: Auto-generate intelligent default warehouse stations based on roadmap topology."""
    target_name = (map_name or (node.active_map_name if node else None) or mapping_mgr.saved_map_name or mapping_mgr.map_name or _default_map_for_mode(AMR_MODE)).strip()

    # Load roadmap nodes strictly from target map
    target_graph = _find_map_file(target_name, ('_graph.json',))
    if not target_graph or not target_graph.exists():
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
    target_name = (map_name or (node.active_map_name if node else None) or mapping_mgr.saved_map_name or mapping_mgr.map_name or _default_map_for_mode(AMR_MODE)).strip()

    places = None
    places_file = _find_map_file(target_name, ('_places.json',))
    if places_file and places_file.exists():
        try:
            with open(places_file, 'r', encoding='utf-8') as f:
                data = json.load(f)
            if isinstance(data, list) and len(data) > 0:
                places = data
        except Exception as e:
            logger.warning(f"Failed reading {places_file}: {e}")

    if places is None:
        graph_file = _find_map_file(target_name, ('_graph.json',))
        if graph_file and graph_file.exists():
            try:
                with open(graph_file, 'r', encoding='utf-8') as f:
                    gdata = json.load(f)
                if isinstance(gdata, dict) and isinstance(gdata.get('places'), list) and len(gdata['places']) > 0:
                    places = gdata['places']
            except Exception as e:
                logger.warning(f"Failed reading places from {graph_file}: {e}")

    if places is None:
        return JSONResponse({'error': f'No places found for map "{target_name}"'}, status_code=404)

    try:
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

    block_err = node.get_dispatch_block_error()
    if block_err:
        return JSONResponse({'ok': False, 'error': block_err}, status_code=409)

    if req.map_name and req.map_name != node.active_map_name:
        try:
            node.set_active_map(req.map_name)
        except Exception as e:
            logger.warning(f"Could not switch active map to {req.map_name}: {e}")

    # Publish to /goal_sequence (route_runner handles single-stop and multi-stop sequences cleanly)
    if req.node_id:
        node.publish_goal_sequence([req.node_id])
    else:
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

    block_err = node.get_dispatch_block_error()
    if block_err:
        return JSONResponse({'ok': False, 'error': block_err}, status_code=409)

    if not req.places:
        return JSONResponse({'error': 'Places sequence cannot be empty'}, status_code=400)

    if req.map_name and req.map_name != node.active_map_name:
        try:
            node.set_active_map(req.map_name)
        except Exception as e:
            logger.warning(f"Could not switch active map to {req.map_name}: {e}")

    node_ids = [p.node_id for p in req.places]
    node.publish_goal_sequence(node_ids)

    first = req.places[0]

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
async def post_system_start(req: SystemStartRequest = SystemStartRequest()):
    """Start full AMR navigation stack (navigation_launch + route_runner + bt_manager)."""
    curr_map = (req.map_name or (node.active_map_name if node else None) or _default_map_for_mode(AMR_MODE)).strip()
    if node and not _find_map_file(curr_map, ('.yaml', '_graph.json', '.png', '.pgm')):
        return JSONResponse({'ok': False, 'error': f"Map '{curr_map}' not found on disk."}, status_code=404)
    res = system_mgr.start(
        map_name=curr_map,
        world=req.world,
        headless=req.headless,
        use_rviz=req.use_rviz,
        auto_start_pi=req.auto_start_pi,
        stop_pi_on_stop=req.stop_pi_on_stop,
    )
    if res.get('ok') and node:
        try:
            node.set_active_map(curr_map)
        except Exception as e:
            logger.warning(f"[post_system_start] set_active_map warning for {curr_map}: {e}")
    return res


@app.post('/api/system/stop')
async def post_system_stop(req: Optional[SystemStopRequest] = Body(default=None)):
    """Stop the full AMR navigation stack cleanly (optionally stops Pi hardware stack via SSH)."""
    stop_pi = req.stop_pi if req is not None else None
    return system_mgr.stop(stop_pi=stop_pi)


@app.get('/api/system/status')
async def get_system_status():
    """Get current lifecycle status of the navigation stack."""
    return system_mgr.get_status()


@app.get('/api/hardware/pi/status')
async def get_pi_hardware_status(force: bool = True):
    """Check Raspberry Pi SSH reachability and ./start_hardware.sh process state."""
    return pi_remote_mgr.check_status(AMR_PI_IP, force=force)


@app.post('/api/hardware/pi/start')
async def post_pi_hardware_start():
    """Optionally start ./start_hardware.sh on the Raspberry Pi via passwordless SSH."""
    if AMR_MODE != 'hardware':
        return JSONResponse({'ok': False, 'error': 'Switch to HARDWARE mode before starting Pi hardware.'}, status_code=400)
    res = pi_remote_mgr.start_hardware(AMR_PI_IP)
    status_code = 200 if res.get('ok') else 500
    return JSONResponse(res, status_code=status_code)


@app.post('/api/hardware/pi/stop')
async def post_pi_hardware_stop():
    """Stop ./start_hardware.sh and its hardware ROS 2 nodes cleanly on the Raspberry Pi via SSH."""
    if AMR_MODE != 'hardware':
        return JSONResponse({'ok': False, 'error': 'Switch to HARDWARE mode before stopping Pi hardware.'}, status_code=400)
    res = pi_remote_mgr.stop_hardware(AMR_PI_IP)
    status_code = 200 if res.get('ok') else 500
    return JSONResponse(res, status_code=status_code)


@app.get('/api/hardware/pi/logs')
async def get_pi_hardware_logs(lines: int = 60):
    """Fetch recent lines of ~/AMR/run_logs/pi_hardware_latest.log from the Raspberry Pi."""
    return pi_remote_mgr.get_logs(AMR_PI_IP, lines=lines)


@app.post('/api/gui/launch')
async def post_gui_launch(req: GuiLaunchRequest):
    """Launch Gazebo GUI or RViz2 visualizer client in the current display."""
    target_app = req.app.lower().strip()
    if AMR_MODE == 'hardware' and target_app in ('gazebo', 'gz'):
        return JSONResponse({'ok': False, 'error': 'Gazebo 3D is disabled in Real Hardware mode.'}, status_code=400)

    env = os.environ.copy()
    env['DISPLAY'] = os.environ.get('DISPLAY', ':0')
    env['QT_QPA_PLATFORM'] = 'xcb'
    env['ROS_DOMAIN_ID'] = os.environ.get('ROS_DOMAIN_ID', '7')
    env['RMW_IMPLEMENTATION'] = os.environ.get('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    env['AMR_MODE'] = AMR_MODE
    env['AMR_PI_IP'] = AMR_PI_IP
    if AMR_MODE == 'sim':
        env['ROS_LOCALHOST_ONLY'] = '1'
    else:
        env.pop('ROS_LOCALHOST_ONLY', None)
    env['LIBGL_ALWAYS_SOFTWARE'] = '1'
    env['GALLIUM_DRIVER'] = 'llvmpipe'
    env['MESA_GL_VERSION_OVERRIDE'] = '4.5'

    if target_app in ('gazebo', 'gz'):
        script = _WORKSPACE_ROOT / "view_gazebo.sh"
        cmd = f"/bin/bash {script}"
    else:
        is_mapping_now = (mapping_mgr.state == 'MAPPING' or mapping_mgr.is_external_mapping_running())
        if AMR_MODE == 'hardware':
            mode_arg = req.mode or ("hw_explore" if is_mapping_now else "hw_nav")
        else:
            mode_arg = req.mode or ("explore" if is_mapping_now else "nav")
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
    if _SHUTDOWN_FLAG.is_set():
        await websocket.close(code=1012)
        return
    await websocket.accept()
    active_ws.append(websocket)
    try:
        while not _SHUTDOWN_FLAG.is_set():
            try:
                await asyncio.wait_for(websocket.receive_text(), timeout=0.5)
            except asyncio.TimeoutError:
                continue
        try:
            await websocket.close(code=1012)
        except Exception:
            pass
    except WebSocketDisconnect:
        pass
    except Exception:
        pass
    finally:
        if websocket in active_ws:
            active_ws.remove(websocket)


async def _broadcast_loop():
    """Push telemetry to all connected WebSocket clients at 5 Hz."""
    while not _SHUTDOWN_FLAG.is_set():
        await asyncio.sleep(0.2)
        if _SHUTDOWN_FLAG.is_set():
            break
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
    executor = None
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
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        try:
            while rclpy.ok() and not _SHUTDOWN_FLAG.is_set():
                executor.spin_once(timeout_sec=0.1)
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
                if executor is not None:
                    executor.shutdown(timeout_sec=1.0)
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
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    t = threading.Thread(target=_ros_thread, daemon=True)
    t.start()
    time.sleep(1.0)  # Give ROS a moment to initialise
    config = uvicorn.Config(
        app,
        host='0.0.0.0',
        port=8000,
        log_level='info',
        timeout_graceful_shutdown=1.5,
    )
    _UVICORN_SERVER = uvicorn.Server(config)
    _UVICORN_SERVER.run()
    _SHUTDOWN_FLAG.set()
    t.join(timeout=2.0)
    if _PENDING_REEXEC is not None:
        next_mode, next_pi_ip = _PENDING_REEXEC
        _configure_dds_environment(next_mode, next_pi_ip)
        script_path = str(Path(__file__).resolve())
        logger.info(f"[bridge] Re-execing bridge server in '{next_mode}' mode (pi_ip={next_pi_ip})...")
        os.execv(sys.executable, [sys.executable, script_path, '--mode', next_mode, '--pi-ip', next_pi_ip])
