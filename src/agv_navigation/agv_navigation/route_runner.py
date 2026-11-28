import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from geometry_msgs.msg import Twist, PoseStamped, Point
from std_msgs.msg import String, Bool
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
from tf2_ros import Buffer, TransformListener, TransformException
import math
import json
import os
import time
import heapq
import yaml

import numpy as np
from scipy.spatial import KDTree
from scipy.ndimage import binary_dilation
from PIL import Image

from agv_navigation.ml_perception import (
    CLASS_CLUTTER,
    CLASS_NAMES,
    CLASS_PERSON,
    CLASS_WALL,
    PerceptionPipeline,
)


class RouteRunner(Node):
    def __init__(self):
        super().__init__('route_runner')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/agv_dense_path', 10)  # Publish dense path for RViz
        # Mission progress publisher: JSON string of {current, total, goal_node, state}
        self.mission_pub = self.create_publisher(String, '/mission_progress', 10)
        # App bridge: publish nav state so the Flutter / web dashboard can display it
        self.agv_state_pub = self.create_publisher(String, '/agv_state', 10)
        # ML obstacle visualization, telemetry, and bridge alert publishers
        self.obs_marker_pub = self.create_publisher(MarkerArray, '/agv_obstacle_markers', 10)
        self.obs_alert_pub = self.create_publisher(String, '/obstacle_alert', 10)
        self.ml_obs_pub = self.create_publisher(String, '/agv_obstacles_ml', 10)

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.goal_sub_alt = self.create_subscription(PoseStamped, '/goal', self.goal_callback, 10)
        self.goal_sub_mb = self.create_subscription(PoseStamped, '/move_base_simple/goal', self.goal_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        # Multi-goal sequence subscriber: expects JSON array of node IDs e.g. '["N5","N12","N40"]'
        self.seq_sub = self.create_subscription(String, '/goal_sequence', self.sequence_callback, 10)
        # App bridge: emergency-stop command from Flutter / web dashboard
        self.estop_sub = self.create_subscription(
            Bool, '/agv_estop', self._estop_callback, 10)
        # Live ML model hot-reload hook (triggered after dataset_trainer_agent exports better weights)
        self.reload_ml_sub = self.create_subscription(
            String, '/ml_pipeline/reload_models', self._reload_ml_models_cb, 10)

        # TF Buffer and Listener for AMCL / SLAM localization (map -> base_link)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.using_map_tf = False

        # Load the graph — check active map, warehouse_graph, or map candidates
        self.declare_parameter('map_name', '')
        map_name_param = self.get_parameter('map_name').get_parameter_value().string_value.strip()

        # Static map occupancy grid for filtering ghost dynamic obstacles
        self.static_map_data = None
        self._static_occ_grid = None
        self.static_map_resolution = 0.05
        self.static_map_origin_x = 0.0
        self.static_map_origin_y = 0.0
        self.static_map_width = 0
        self.static_map_height = 0
        self.static_wall_mask = None
        self._load_static_map_file(map_name_param)

        latched_map_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, latched_map_qos)
        self.loc_status_sub = self.create_subscription(String, '/localization_status', self._localization_status_callback, latched_map_qos)

        candidates = []
        if map_name_param and os.path.isabs(map_name_param) and os.path.exists(map_name_param):
            candidates.append(map_name_param)

        ws_root = os.path.join(os.environ.get('HOME', ''), 'AMR', 'AMR-main')
        ws_root_maps = os.path.join(ws_root, 'maps')
        ws_maps = os.path.join(ws_root, 'src', 'agv_description', 'maps')

        if map_name_param:
            candidates.append(os.path.join(ws_root_maps, f'{map_name_param}_graph.json'))
            candidates.append(os.path.join(ws_root_maps, f'{map_name_param}.json'))
            candidates.append(os.path.join(ws_maps, f'{map_name_param}_graph.json'))

        try:
            pkg_share = get_package_share_directory('agv_description')
            if map_name_param:
                candidates.append(os.path.join(pkg_share, 'maps', f'{map_name_param}_graph.json'))
            candidates.append(os.path.join(pkg_share, 'maps', 'warehouse_graph.json'))
            candidates.append(os.path.join(pkg_share, 'maps', 'warehouse_map_graph.json'))
            candidates.append(os.path.join(pkg_share, 'maps', 'warehouse_01_graph.json'))
        except Exception:
            pass

        candidates.append(os.path.join(ws_maps, 'warehouse_graph.json'))
        candidates.append(os.path.join(ws_maps, 'warehouse_map_graph.json'))
        candidates.append(os.path.join(ws_maps, 'warehouse_01_graph.json'))

        self.map_path = None
        for c in candidates:
            if os.path.exists(c):
                self.map_path = c
                break

        if not self.map_path:
            raise FileNotFoundError(f"Could not find valid graph file in {candidates}")

        with open(self.map_path, 'r') as f:
            raw_data = json.load(f)

        self.map_data = {}
        for node in raw_data.get("nodes", []):
            self.map_data[node["id"]] = {
                "x": node["x"],
                "y": node["y"],
                "edges": []
            }

        for edge in raw_data.get("edges", []):
            from_node = edge["from"]
            to_node = edge["to"]
            cost = edge.get("cost", 1.0)
            if from_node in self.map_data:
                self.map_data[from_node]["edges"].append({
                    "to_node": to_node,
                    "distance_m": cost
                })

        # Fix Graph Connectivity: Ensure all edges are bidirectional!
        for node_id, data in list(self.map_data.items()):
            for edge in list(data["edges"]):
                target_id = edge["to_node"]
                weight = edge["distance_m"]
                if target_id in self.map_data:
                    target_edges = self.map_data[target_id]["edges"]
                    if not any(e["to_node"] == node_id for e in target_edges):
                        target_edges.append({
                            "to_node": node_id,
                            "distance_m": weight
                        })

        # Build KD-Tree for O(log N) fast node snapping
        self.node_ids = list(self.map_data.keys())
        self.node_coords = np.array([[self.map_data[nid]["x"], self.map_data[nid]["y"]] for nid in self.node_ids])
        self.node_kdtree = KDTree(self.node_coords) if len(self.node_coords) > 0 else None

        # Navigation state machine: IDLE, PLANNING, NAVIGATING, YIELDING, RE_ROUTING
        self.state = "IDLE"
        self._estop_active = False
        self.actual_w = 0.0
        self._is_pivoting = False
        self.last_pub_w = 0.0

        # Diagnostic rate-limiters & tracking
        self._last_block_log_time = 0.0
        self._last_turn_log_time = 0.0
        self._last_dyn_log_time = 0.0
        self._last_mppi_evade_log_time = 0.0
        self._last_pose_log_time = 0.0
        self.last_snapped_node = None

        # Path storage
        self.path_plan = []            # Dense (x, y, theta) waypoints
        self.current_target_index = 0
        self.node_path = []            # Dijkstra node sequence
        self.target_node = None        # Final goal node

        # Multi-goal mission queue
        self.goal_queue = []           # List of (goal_x, goal_y) tuples to visit in order
        self.mission_total = 0         # Total goals in the current mission
        self.mission_current = 0       # Index of the goal currently being executed (1-based)

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_ready = False

        # Obstacles & ML Perception State
        self.obstacles = np.array([])  # Static and dynamic points in map frame
        self.prev_obstacle_clusters = []
        self.dynamic_obstacles = []    # List of dicts for PERSON / moving obstacles with pred_traj & pred_std
        self.clutter_obstacles = []    # List of dicts for stationary CLUTTER obstacles
        self.ml_obstacles = []         # Full TrackedObstacle list from PerceptionPipeline
        self.last_scan_time = None
        self._last_ml_infer_ms = 0.0
        self._last_obs_alert_text = ""
        self._last_obs_alert_pub_time = 0.0
        self._obs_age_map = {}
        self._obs_start_pos_map = {}
        self._obs_vel_map = {}

        # Traffic & Re-routing parameters
        self.yield_timeout = 4.0       # Seconds to wait for transient dynamic obstacles before re-routing
        self.yield_timeout_static = 1.0  # Fast reroute timeout (1.0s) for stationary unmapped blockages
        self.yield_timeout_dynamic = 4.0 # Timeout (4.0s) for dynamic moving obstacles
        self.yield_start_time = None
        self.blocked_edges = {}        # {(u, v): (original_cost, restore_timestamp)}

        # Physical dimensions & Safe navigation clearances (22cm robot diameter)
        self.robot_radius = 0.11       # Cylinder radius 0.11m (diameter 0.22m)
        self.collision_radius = 0.15   # Hard safety clearance (0.11m + 0.04m physical margin)
        self.min_passable_gap = 0.35   # Minimum passage width for 22cm robot (0.22m + 0.13m corridor margin)

        # Declare configurable navigation parameters (auto-adapts for Physical Hardware vs Simulation)
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)
        self.use_sim_time = self.get_parameter('use_sim_time').value

        # Physical Hardware Defaults (calibrated smooth slow speeds) vs Simulation (fast test)
        default_v_max = 0.75 if self.use_sim_time else 0.13
        default_cruise_v = 0.45 if self.use_sim_time else 0.10
        default_min_v = 0.08 if self.use_sim_time else 0.04
        default_w_max = 1.80 if self.use_sim_time else 0.40
        default_lookahead = 0.75 if self.use_sim_time else 0.25
        default_accel_v = 0.60 if self.use_sim_time else 0.18
        default_decel_v = 0.90 if self.use_sim_time else 0.30
        default_accel_w = 1.60 if self.use_sim_time else 0.60
        default_kp_h = 1.40 if self.use_sim_time else 0.90
        default_standoff = 0.50 if self.use_sim_time else 0.30
        default_crit_prox = 0.20 if self.use_sim_time else 0.10
        default_noise_v = 0.30 if self.use_sim_time else 0.04
        default_noise_w = 0.50 if self.use_sim_time else 0.08
        default_dyn_rep = 0.65 if self.use_sim_time else 0.42
        default_evasion_r = 1.20 if self.use_sim_time else 0.80
        default_swerve_str = 0.35 if self.use_sim_time else 0.15

        self.declare_parameter('max_linear_speed', default_v_max)
        self.declare_parameter('cruise_linear_speed', default_cruise_v)
        self.declare_parameter('min_linear_speed', default_min_v)
        self.declare_parameter('max_angular_speed', default_w_max)
        self.declare_parameter('lookahead_dist', default_lookahead)
        self.declare_parameter('accel_lim_v', default_accel_v)
        self.declare_parameter('decel_lim_v', default_decel_v)
        self.declare_parameter('accel_lim_w', default_accel_w)
        self.declare_parameter('kp_heading', default_kp_h)
        self.declare_parameter('safe_standoff', default_standoff)
        self.declare_parameter('critical_proximity', default_crit_prox)
        self.declare_parameter('goal_arrival_tolerance', 0.08)

        self.v_max = float(self.get_parameter('max_linear_speed').value)
        self.cruise_linear_speed = float(self.get_parameter('cruise_linear_speed').value)
        self.min_linear_speed = float(self.get_parameter('min_linear_speed').value)
        self.v_min = 0.0
        self.w_max = float(self.get_parameter('max_angular_speed').value)
        self.lookahead_dist = float(self.get_parameter('lookahead_dist').value)
        self.accel_lim_v = float(self.get_parameter('accel_lim_v').value)
        self.decel_lim_v = float(self.get_parameter('decel_lim_v').value)
        self.accel_lim_w = float(self.get_parameter('accel_lim_w').value)
        self.kp_heading = float(self.get_parameter('kp_heading').value)
        self.safe_standoff = float(self.get_parameter('safe_standoff').value)
        self.critical_proximity = float(self.get_parameter('critical_proximity').value)
        self.goal_arrival_tolerance = float(self.get_parameter('goal_arrival_tolerance').value)

        # Final Goal Coordinates for precise lead-out navigation
        self.final_goal_x = None
        self.final_goal_y = None

        # Smooth Early Detour (Circumnavigation) state
        self.active_detour = None      # Stores {rejoin_idx, created_time, side} when circumnavigating
        self._detour_cooldown_time = 0.0
        self._reroute_cooldown_time = 0.0
        self._last_detour_log_time = 0.0

        # Stuck detector & Active Reverse Recovery
        self._stuck_counter = 0
        self._last_stuck_check_pos = None
        self._prev_heading_err = None  # Tracks heading error delta for kinematic stuck detection
        self._stuck_threshold_ticks = 45  # ~4.5s allows calm in-place pivot without false triggers
        self._recovery_phase = None        # None, 'BACKUP', or 'REROUTE'
        self._recovery_start_time = None
        self._recovery_backup_dur = 2.2   # Max seconds to reverse if standoff not yet achieved
        self._last_recovery_log_time = 0.0
        self.hazard_zones = []         # Persistent anti-loop hazard memory: [{'pos': (x,y), 'radius': r, 'expire_time': timestamp}]

        # MPPI Parameters
        self.dt = 0.1                  # 10 Hz
        default_horizon = 20 if self.use_sim_time else 15
        self.declare_parameter('enable_ml_perception', True)
        self.declare_parameter('prediction_horizon_steps', default_horizon)
        self.declare_parameter('classifier_model_path', '')
        self.declare_parameter('predictor_model_path', '')
        self.declare_parameter('person_social_margin', 0.16 if self.use_sim_time else 0.12)

        self.enable_ml_perception = bool(self.get_parameter('enable_ml_perception').value)
        self.horizon = max(10, int(self.get_parameter('prediction_horizon_steps').value))
        self.person_social_margin = float(self.get_parameter('person_social_margin').value)
        self.num_samples = 80

        clf_path = self._resolve_ml_model_path('classifier_model_path', 'obstacle_classifier.npz')
        pred_path = self._resolve_ml_model_path('predictor_model_path', 'trajectory_predictor.npz')
        self.perception = PerceptionPipeline(
            classifier_path=clf_path,
            predictor_path=pred_path,
            prediction_steps=self.horizon,
            dt=self.dt,
            enable_ml=self.enable_ml_perception,
        )
        if self._static_occ_grid is not None:
            self.perception.update_map(
                self._static_occ_grid,
                self.static_map_resolution,
                self.static_map_origin_x,
                self.static_map_origin_y,
            )
        self.get_logger().info(
            f"[ML_PERCEPTION] Enabled={self.enable_ml_perception} | Horizon={self.horizon * self.dt:.1f}s ({self.horizon} steps) | "
            f"Classifier={'LOADED' if self.perception.classifier.loaded else 'FALLBACK'} | "
            f"Predictor={'LOADED' if self.perception.predictor.loaded else 'KF_FALLBACK'}"
        )

        # MPPI Noise covariance
        self.noise_v = default_noise_v
        self.noise_w = default_noise_w
        self.lambda_weight = 0.5

        # MPPI Cost Weights
        self.w_dist = 4.0
        self.w_heading = 4.5
        self.w_collision = 5000.0
        
        # Repulsion fields (tuned for physical indoor environments and 22cm robot)
        self.dynamic_repulsive_dist = default_dyn_rep  # Proactive evasion bubble for moving obstacles (m)
        self.dynamic_w_repulsive = 45.0     # Smooth proactive repulsion from dynamic obstacles
        self.static_repulsive_dist = 0.22   # Wall repulsion activation distance (m) (tuned for narrow passages)
        self.static_w_repulsive = 50.0      # Quadratic wall repulsion (smooth gradient in narrow passages)

        self.w_cross_track_nominal = 8.0
        self.w_cross_track_evasion = 2.0   # Soften centerline tracking during active evasion to glide smoothly past obstacles
        self.w_cross_track = self.w_cross_track_nominal

        # Distance-gated evasion: only relax w_cross_track for obstacles within this range
        self.evasion_range = default_evasion_r       # metres

        # Swerve side commitment: commit to one side briefly to avoid L/R oscillation
        self._swerve_bias = 0.0           # Current angular bias added to MPPI w_seq mean (rad/s)
        self._swerve_lock_ticks = 0       # Ticks remaining on current committed swerve direction
        self.swerve_lock_duration = 8     # Ticks (~0.8s at 10 Hz) to hold swerve commitment
        self.swerve_bias_strength = default_swerve_str  # Angular bias magnitude (rad/s)

        self.last_cmd_v = 0.0          # Last published linear velocity
        self.last_cmd_w = 0.0          # Last published angular velocity
        self.last_target_v = 0.0       # EMA smoothed target v
        self.last_target_w = 0.0       # EMA smoothed target w
        self.ema_alpha_v = 0.70        # EMA smoothing factor for linear velocity
        self.ema_alpha_w = 0.65        # EMA smoothing factor for angular velocity

        # Persistent Dynamic Obstacle Tracking state
        self._tracked_obstacles = {}   # track_id -> dict
        self._next_track_id = 1
        self._visited_plan_nodes = set()
        self._last_dev_alert_time = 0.0

        # Localization Verification & LiDAR-vs-Odom Wheel-Slip Safety Guards
        self._loc_status = None
        self._odom_pose = None
        self._latest_scan_local = None
        self._slip_anchor_odom = None
        self._slip_anchor_scan = None
        self._slip_anchor_time = None
        self._slip_strikes = 0
        self._map_mismatch_strikes = 0
        self._last_map_check_time = 0.0

        # Control timer
        self.control_timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info(f"Route Runner Active with KD-Tree ({len(self.node_ids)} nodes), Proactive Detour & Active Reverse Escape.")

    # ------------------------------------------------------------------
    # Static Map Mask, ML Model Path & App Bridge helpers
    # ------------------------------------------------------------------

    def _resolve_ml_model_path(self, param_name: str, default_filename: str):
        """Resolve ML model .npz path from ROS parameter, package share, or workspace."""
        param_val = self.get_parameter(param_name).get_parameter_value().string_value.strip()
        if param_val and os.path.exists(param_val):
            return param_val
        candidates = []
        try:
            candidates.append(os.path.join(get_package_share_directory('agv_navigation'), 'models', default_filename))
        except Exception:
            pass
        ws_root = os.path.join(os.environ.get('HOME', ''), 'AMR', 'AMR-main')
        candidates.append(os.path.join(ws_root, 'src', 'agv_navigation', 'models', default_filename))
        candidates.append(os.path.join(os.path.dirname(os.path.dirname(__file__)), 'models', default_filename))
        for c in candidates:
            if os.path.exists(c):
                return c
        return None

    def _reload_ml_models_cb(self, msg: String):
        """Hot-reload newly trained ML classifier & trajectory predictor weights without restarting node."""
        try:
            clf_path = self._resolve_ml_model_path('classifier_model_path', 'obstacle_classifier.npz')
            pred_path = self._resolve_ml_model_path('predictor_model_path', 'trajectory_predictor.npz')
            if clf_path and self.enable_ml_perception:
                self.perception.classifier.load(clf_path)
            if pred_path and self.enable_ml_perception:
                self.perception.predictor.load(pred_path)
            self.get_logger().info(
                f"[ML_PERCEPTION] Hot-reloaded models ({msg.data}) | "
                f"Classifier={self.perception.classifier.model_type if self.perception.classifier.loaded else 'FALLBACK'} | "
                f"Predictor={'LOADED' if self.perception.predictor.loaded else 'KF_FALLBACK'}"
            )
        except Exception as e:
            self.get_logger().warn(f"[ML_PERCEPTION] Hot-reload failed: {e}")

    def _compute_lateral_corridor_clearances(self, target_heading: float):
        """Estimate open left vs right lateral clearance (m) around the robot using live obstacles."""
        left_clear = 2.0
        right_clear = 2.0
        if len(self.obstacles) == 0:
            return left_clear, right_clear
        dx = self.obstacles[:, 0] - self.current_x
        dy = self.obstacles[:, 1] - self.current_y
        ch = math.cos(target_heading)
        sh = math.sin(target_heading)
        fwd = dx * ch + dy * sh
        lat = -dx * sh + dy * ch
        corridor = (fwd > -0.15) & (fwd < 1.80)
        if np.any(corridor):
            l_pts = lat[corridor & (lat > 0.12)]
            r_pts = -lat[corridor & (lat < -0.12)]
            if len(l_pts) > 0:
                left_clear = float(np.min(l_pts))
            if len(r_pts) > 0:
                right_clear = float(np.min(r_pts))
        return left_clear, right_clear

    def _load_static_map_file(self, map_name_param):
        """Loads static map YAML and PGM file to construct a binary wall mask."""
        candidates = []
        if map_name_param and os.path.isabs(map_name_param) and os.path.exists(map_name_param):
            candidates.append(map_name_param)

        ws_root = os.path.join(os.environ.get('HOME', ''), 'AMR', 'AMR-main')
        ws_root_maps = os.path.join(ws_root, 'maps')
        ws_maps = os.path.join(ws_root, 'src', 'agv_description', 'maps')

        if map_name_param:
            candidates.append(os.path.join(ws_root_maps, f'{map_name_param}.yaml'))
            candidates.append(os.path.join(ws_maps, f'{map_name_param}.yaml'))

        try:
            pkg_share = get_package_share_directory('agv_description')
            if map_name_param:
                candidates.append(os.path.join(pkg_share, 'maps', f'{map_name_param}.yaml'))
            candidates.append(os.path.join(pkg_share, 'maps', 'hospital_map.yaml'))
            candidates.append(os.path.join(pkg_share, 'maps', 'warehouse_map.yaml'))
        except Exception:
            pass

        candidates.append(os.path.join(ws_maps, 'hospital_map.yaml'))
        candidates.append(os.path.join(ws_maps, 'warehouse_map.yaml'))

        yaml_path = None
        for c in candidates:
            if os.path.exists(c):
                yaml_path = c
                break

        if not yaml_path:
            self.get_logger().warn("[MAP_MASK] No static map YAML found, waiting for /map topic...")
            return

        try:
            with open(yaml_path, 'r') as f:
                meta = yaml.safe_load(f)
            pgm_file = meta.get('image', '')
            if not os.path.isabs(pgm_file):
                pgm_file = os.path.join(os.path.dirname(yaml_path), pgm_file)

            if not os.path.exists(pgm_file):
                self.get_logger().warn(f"[MAP_MASK] PGM file not found at {pgm_file}")
                return

            img = Image.open(pgm_file)
            arr = np.array(img)
            # Flip vertically because ROS map origin is bottom-left (y_min)
            arr = np.flipud(arr)
            # In ROS map standard: pixel value < 50 indicates occupied cell
            occ = (arr < 50)
            self._static_occ_grid = occ
            # Dilate occupied walls by ~0.28m to absorb normal sensor spread
            iterations = max(2, int(0.28 / meta.get('resolution', 0.05)))
            self.static_wall_mask = binary_dilation(occ, iterations=iterations)
            self.static_map_resolution = float(meta.get('resolution', 0.05))
            orig = meta.get('origin', [0.0, 0.0, 0.0])
            self.static_map_origin_x = float(orig[0])
            self.static_map_origin_y = float(orig[1])
            self.static_map_height, self.static_map_width = arr.shape
            if getattr(self, 'perception', None) is not None:
                self.perception.update_map(
                    occ,
                    self.static_map_resolution,
                    self.static_map_origin_x,
                    self.static_map_origin_y,
                )
            self.get_logger().info(
                f"[MAP_MASK] Loaded static environment mask from {os.path.basename(pgm_file)} "
                f"({self.static_map_width}x{self.static_map_height}, res={self.static_map_resolution}m, origin=({self.static_map_origin_x:.2f}, {self.static_map_origin_y:.2f}))."
            )
        except Exception as e:
            self.get_logger().warn(f"[MAP_MASK] Failed to load static map image: {e}")

    def map_callback(self, msg: OccupancyGrid):
        """Dynamic map subscription update from map_server / SLAM."""
        try:
            w = msg.info.width
            h = msg.info.height
            grid = np.array(msg.data, dtype=np.int8).reshape((h, w))
            occ = (grid > 50)
            self._static_occ_grid = occ
            res = msg.info.resolution
            iterations = max(2, int(0.28 / res))
            self.static_wall_mask = binary_dilation(occ, iterations=iterations)
            self.static_map_resolution = float(res)
            self.static_map_origin_x = float(msg.info.origin.position.x)
            self.static_map_origin_y = float(msg.info.origin.position.y)
            self.static_map_width = w
            self.static_map_height = h
            if getattr(self, 'perception', None) is not None:
                self.perception.update_map(
                    occ,
                    self.static_map_resolution,
                    self.static_map_origin_x,
                    self.static_map_origin_y,
                )
            self.get_logger().info(f"[MAP_MASK] Updated static wall mask from /map topic ({w}x{h}).")
        except Exception as e:
            self.get_logger().warn(f"[MAP_MASK] Error parsing /map: {e}")

    def is_near_static_wall(self, x, y):
        """Returns True if world coordinate (x, y) lies on or within ~0.22m of static environment geometry."""
        if self.static_wall_mask is None:
            return False
        gx = int((x - self.static_map_origin_x) / self.static_map_resolution)
        gy = int((y - self.static_map_origin_y) / self.static_map_resolution)
        if 0 <= gx < self.static_map_width and 0 <= gy < self.static_map_height:
            return bool(self.static_wall_mask[gy, gx])
        return True  # Outside map bounds is treated as non-traversable

    def _is_on_mapped_wall(self, x, y):
        """Strict check: True only if (x, y) is inside map bounds AND near a mapped wall cell."""
        if self.static_wall_mask is None:
            return False
        gx = int((x - self.static_map_origin_x) / self.static_map_resolution)
        gy = int((y - self.static_map_origin_y) / self.static_map_resolution)
        if 0 <= gx < self.static_map_width and 0 <= gy < self.static_map_height:
            return bool(self.static_wall_mask[gy, gx])
        return False

    def _localization_status_callback(self, msg: String):
        """Receives latched verification status from auto_localizer (/localization_status)."""
        try:
            data = json.loads(msg.data)
            self._loc_status = data
            st = data.get('status')
            if st == 'VERIFIED':
                self._map_mismatch_strikes = 0
            elif st == 'MAP_MISMATCH' and self.state in ('PLANNING', 'NAVIGATING', 'YIELDING'):
                self._halt_with_fault('MAP_MISMATCH', data.get('message', 'Map mismatch detected by auto_localizer.'))
        except Exception:
            pass

    def _is_localization_blocked(self):
        """Returns (blocked: bool, reason: str) if hardware localization is explicitly rejected."""
        if not self._loc_status:
            return False, ''
        st = self._loc_status.get('status', '')
        if not self._loc_status.get('verified', True) and st in ('MAP_MISMATCH', 'MAP_MISSING', 'SCAN_MISSING'):
            return True, self._loc_status.get('message', f'Localization status is {st}')
        return False, ''

    def _halt_with_fault(self, fault_state: str, reason: str):
        """Immediately halts robot, clears active mission/path, and broadcasts fault state."""
        self.goal_queue = []
        self.path_plan = []
        self.node_path = []
        self.active_detour = None
        self._recovery_phase = None
        self._slip_anchor_odom = None
        self._slip_anchor_scan = None
        self._slip_strikes = 0
        self.state = "IDLE"
        self._stop_robot()
        self.publish_active_path()
        self._publish_agv_state(fault_state)
        self._publish_mission_progress(fault_state)
        self.get_logger().error(f"[SAFETY: {fault_state}] {reason}")

    def _publish_agv_state(self, state_str: str):
        """Publish the current nav state string to /agv_state for the app bridge."""
        msg = String()
        msg.data = state_str
        self.agv_state_pub.publish(msg)

    def _estop_callback(self, msg):
        """Handle emergency-stop command from Flutter / web dashboard."""
        if msg.data:
            self._estop_active = True
            self.state = "IDLE"
            self.path_plan = []
            self.goal_queue = []
            self._stop_robot()
            self._publish_agv_state("ESTOP")
            self.get_logger().warn("[APP_BRIDGE] E-STOP received — navigation halted.")
        else:
            self._estop_active = False
            self._publish_agv_state("IDLE")
            self.get_logger().info("[APP_BRIDGE] E-STOP cleared.")

    def _stop_robot(self):
        """Crisp immediate stop resetting command slew rates."""
        self.last_cmd_v = 0.0
        self.last_cmd_w = 0.0
        self.last_pub_w = 0.0
        self.last_target_v = 0.0
        self.last_target_w = 0.0
        self._is_pivoting = False
        self.cmd_pub.publish(Twist())

    def destroy_node(self):
        try:
            self._stop_robot()
        except Exception:
            pass
        super().destroy_node()

    def _now_sec(self):
        """Return current ROS time as float seconds (sim-time aware)."""
        return self.get_clock().now().nanoseconds / 1e9

    def get_yaw_from_quaternion(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def find_closest_node(self, x, y):
        if self.node_kdtree is None or len(self.node_ids) == 0:
            return None
        _, idx = self.node_kdtree.query([x, y])
        return self.node_ids[idx]

    def calculate_dijkstra(self, start_node, target_node):
        distances = {node: float('infinity') for node in self.map_data}
        distances[start_node] = 0
        previous_nodes = {node: None for node in self.map_data}

        pq = [(0, start_node)]

        while pq:
            current_distance, current_node = heapq.heappop(pq)
            if current_node == target_node:
                break
            if current_distance > distances[current_node]:
                continue
            for edge in self.map_data[current_node]["edges"]:
                neighbor = edge["to_node"]
                weight = edge["distance_m"]
                distance = current_distance + weight
                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    previous_nodes[neighbor] = current_node
                    heapq.heappush(pq, (distance, neighbor))

        path = []
        current = target_node
        while current is not None:
            path.append(current)
            current = previous_nodes[current]

        path.reverse()
        if not path or path[0] != start_node:
            self.get_logger().error(f"NO VALID PATH FOUND FROM {start_node} TO {target_node}!")
            return []
        return path

    def update_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.08)
            )
            self.current_x = t.transform.translation.x
            self.current_y = t.transform.translation.y
            self.current_yaw = self.get_yaw_from_quaternion(t.transform.rotation)
            if not self.using_map_tf:
                self.get_logger().info("Localization Active! Received map -> base_link TF transform.")
                self.using_map_tf = True
            self.odom_ready = True
            return True
        except TransformException:
            # If map TF was already acquired, retain the last known valid map pose.
            # NEVER fall back to raw odom coordinates which have an arbitrary origin!
            return self.odom_ready

    def odom_callback(self, msg):
        # Always capture physical angular velocity for rotation-gated scan filtering
        self.actual_w = float(msg.twist.twist.angular.z)
        self._odom_pose = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            self.get_yaw_from_quaternion(msg.pose.pose.orientation),
        )
        # On physical hardware with a static map, robot pose MUST strictly come from map -> base_link TF.
        # Raw /odometry/filtered is in the 'odom' frame and must NEVER overwrite map coordinates!
        if not self.using_map_tf and self.use_sim_time:
            self.current_x = msg.pose.pose.position.x
            self.current_y = msg.pose.pose.position.y
            self.current_yaw = self.get_yaw_from_quaternion(msg.pose.pose.orientation)
            self.odom_ready = True


    def scan_callback(self, msg):
        self.update_robot_pose()
        now = self._now_sec()
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        angles = (msg.angle_min + np.arange(len(ranges)) * msg.angle_increment).astype(np.float32)

        # Basic check before perception
        valid_any = np.any((ranges > max(0.08, msg.range_min)) & (ranges < msg.range_max) & np.isfinite(ranges))
        if not valid_any:
            self.obstacles = np.array([])
            self.dynamic_obstacles = []
            self.clutter_obstacles = []
            self.ml_obstacles = []
            self._latest_scan_local = None
            return

        # Use laser scan timestamp for map transformation to prevent rotational skew artifacts
        try:
            t = self.tf_buffer.lookup_transform(
                'map',
                msg.header.frame_id if msg.header.frame_id else 'base_link',
                msg.header.stamp
            )
            pose_x = t.transform.translation.x
            pose_y = t.transform.translation.y
            pose_yaw = self.get_yaw_from_quaternion(t.transform.rotation)
        except TransformException:
            pose_x = self.current_x
            pose_y = self.current_y
            pose_yaw = self.current_yaw

        dt_scan = max(0.04, min(0.35, now - self.last_scan_time)) if self.last_scan_time is not None else self.dt

        # Full-360-beam polar segmentation, Kalman tracking, ML classification & 1-2s GRU trajectory prediction
        t_ml0 = time.perf_counter()
        tracks, ext_local, ext_map = self.perception.process_scan(
            ranges=ranges,
            angles=angles,
            pose_x=pose_x,
            pose_y=pose_y,
            pose_yaw=pose_yaw,
            dt=dt_scan,
            range_min=float(msg.range_min),
            range_max=float(msg.range_max),
            robot_ang_vel=self.actual_w,
            prediction_steps=self.horizon,
        )
        self._last_ml_infer_ms = (time.perf_counter() - t_ml0) * 1000.0

        if len(ext_local) == 0:
            self.obstacles = np.array([])
            self.dynamic_obstacles = []
            self.clutter_obstacles = []
            self.ml_obstacles = []
            self._latest_scan_local = None
            return

        # Keep bounded local scan points (0.18m..6.0m) for wheel-slip / stationary-chassis cross-checks
        r_ext = np.hypot(ext_local[:, 0], ext_local[:, 1])
        valid_local = (r_ext > 0.18) & (r_ext < 6.0)
        self._latest_scan_local = ext_local[valid_local] if np.any(valid_local) else None

        # Keep high-density external points for MPPI static cost (downsample only if > 220 points)
        self.obstacles = ext_map[::2] if len(ext_map) > 220 else ext_map

        # Separate ML-classified tracks by semantic role
        new_dynamic_obs = []
        new_clutter_obs = []
        for tr in tracks:
            if tr.age < 2:
                continue
            if tr.class_id == CLASS_PERSON and (tr.speed >= 0.08 or float(tr.class_probs[CLASS_PERSON]) >= 0.58):
                new_dynamic_obs.append({
                    'id': tr.track_id,
                    'pos': tr.pos,
                    'vel': tr.vel,
                    'speed': tr.speed,
                    'radius': max(0.22, min(0.42, tr.radius)),
                    'age': tr.age,
                    'class_name': tr.class_name,
                    'class_probs': tr.class_probs,
                    'pred_traj': tr.pred_traj,
                    'pred_std': tr.pred_std,
                    'is_leg_pair': tr.is_leg_pair,
                })
            elif tr.class_id == CLASS_CLUTTER:
                new_clutter_obs.append({
                    'id': tr.track_id,
                    'pos': tr.pos,
                    'radius': max(0.18, min(0.45, tr.radius)),
                    'age': tr.age,
                    'class_name': tr.class_name,
                    'class_probs': tr.class_probs,
                })

        self.ml_obstacles = tracks
        self.dynamic_obstacles = new_dynamic_obs
        self.clutter_obstacles = new_clutter_obs
        self.last_scan_time = now

        # Publish RViz visualization markers, bridge alerts, and ML telemetry
        self._publish_ml_obstacle_outputs(now)

        # Real-time LiDAR dynamic obstacle detection logging (rate-limited to 1 Hz)
        if len(self.dynamic_obstacles) > 0 and (now - self._last_dyn_log_time) > 1.0:
            self._last_dyn_log_time = now
            for i, dob in enumerate(self.dynamic_obstacles):
                d_robot = math.hypot(dob['pos'][0] - self.current_x, dob['pos'][1] - self.current_y)
                p_person = float(dob['class_probs'][CLASS_PERSON])
                self.get_logger().info(
                    f"[LIDAR_SCAN: DYNAMIC_OBSTACLE] Tracked Obs #{dob.get('id', i+1)} [{dob['class_name']} p={p_person:.2f}] "
                    f"at map({dob['pos'][0]:.2f}, {dob['pos'][1]:.2f}) | "
                    f"Vel=({dob['vel'][0]:.2f}, {dob['vel'][1]:.2f}) Speed={dob['speed']:.2f}m/s | "
                    f"DistToRobot={d_robot:.2f}m | Infer={self._last_ml_infer_ms:.2f}ms | RobotPose=({self.current_x:.2f}, {self.current_y:.2f})"
                )

    def _publish_ml_obstacle_outputs(self, now: float):
        """Publish RViz MarkerArray, bridge /obstacle_alert, and /agv_obstacles_ml JSON."""
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()

        clear_m = Marker()
        clear_m.header.frame_id = "map"
        clear_m.header.stamp = stamp
        clear_m.action = Marker.DELETEALL
        markers.markers.append(clear_m)

        counts = {"WALL": 0, "CLUTTER": 0, "PERSON": 0}
        nearest_alert = None
        nearest_alert_dist = 2.4

        for tr in self.ml_obstacles:
            if tr.age < 2:
                continue
            counts[tr.class_name] = counts.get(tr.class_name, 0) + 1
            d_robot = math.hypot(float(tr.pos[0]) - self.current_x, float(tr.pos[1]) - self.current_y)

            if tr.class_id in (CLASS_PERSON, CLASS_CLUTTER) and d_robot < nearest_alert_dist:
                nearest_alert_dist = d_robot
                nearest_alert = tr

            # Bounding cylinder marker
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = stamp
            m.ns = "ml_obstacles"
            m.id = int(tr.track_id)
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = float(tr.pos[0])
            m.pose.position.y = float(tr.pos[1])
            m.pose.position.z = 0.45 if tr.class_id == CLASS_PERSON else 0.25
            m.pose.orientation.w = 1.0
            diam = float(max(0.20, 2.0 * tr.radius))
            m.scale.x = diam
            m.scale.y = diam
            m.scale.z = 0.90 if tr.class_id == CLASS_PERSON else 0.45
            if tr.class_id == CLASS_PERSON:
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.22, 0.12, 0.72
            elif tr.class_id == CLASS_CLUTTER:
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.78, 0.10, 0.68
            else:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.55, 0.62, 0.70, 0.35
            markers.markers.append(m)

            # Text label for PERSON and CLUTTER
            if tr.class_id in (CLASS_PERSON, CLASS_CLUTTER):
                txt = Marker()
                txt.header.frame_id = "map"
                txt.header.stamp = stamp
                txt.ns = "ml_labels"
                txt.id = 10000 + int(tr.track_id)
                txt.type = Marker.TEXT_VIEW_FACING
                txt.action = Marker.ADD
                txt.pose.position.x = float(tr.pos[0])
                txt.pose.position.y = float(tr.pos[1])
                txt.pose.position.z = 1.05 if tr.class_id == CLASS_PERSON else 0.65
                txt.scale.z = 0.18
                txt.color.r, txt.color.g, txt.color.b, txt.color.a = 1.0, 1.0, 1.0, 0.95
                p_cls = float(tr.class_probs[tr.class_id])
                txt.text = f"#{tr.track_id} {tr.class_name} ({p_cls:.2f}) {tr.speed:.2f}m/s"
                markers.markers.append(txt)

            # Predicted 1-2s trajectory ribbon & uncertainty ellipses for PERSON
            if tr.class_id == CLASS_PERSON and tr.pred_traj is not None and len(tr.pred_traj) > 0:
                rib = Marker()
                rib.header.frame_id = "map"
                rib.header.stamp = stamp
                rib.ns = "ml_predictions"
                rib.id = 20000 + int(tr.track_id)
                rib.type = Marker.LINE_STRIP
                rib.action = Marker.ADD
                rib.scale.x = 0.045
                rib.color.r, rib.color.g, rib.color.b, rib.color.a = 0.0, 0.92, 1.0, 0.90
                p0 = Point(x=float(tr.pos[0]), y=float(tr.pos[1]), z=0.12)
                rib.points.append(p0)
                for pt in tr.pred_traj:
                    rib.points.append(Point(x=float(pt[0]), y=float(pt[1]), z=0.12))
                markers.markers.append(rib)

                # Uncertainty ellipses at 1.0s (idx 9) and horizon end
                if tr.pred_std is not None:
                    for k_idx in (min(9, len(tr.pred_traj) - 1), len(tr.pred_traj) - 1):
                        unc = Marker()
                        unc.header.frame_id = "map"
                        unc.header.stamp = stamp
                        unc.ns = "ml_uncertainty"
                        unc.id = 30000 + int(tr.track_id) * 10 + k_idx
                        unc.type = Marker.SPHERE
                        unc.action = Marker.ADD
                        unc.pose.position.x = float(tr.pred_traj[k_idx, 0])
                        unc.pose.position.y = float(tr.pred_traj[k_idx, 1])
                        unc.pose.position.z = 0.06
                        unc.pose.orientation.w = 1.0
                        unc.scale.x = float(max(0.15, 2.0 * tr.pred_std[k_idx, 0]))
                        unc.scale.y = float(max(0.15, 2.0 * tr.pred_std[k_idx, 1]))
                        unc.scale.z = 0.05
                        unc.color.r, unc.color.g, unc.color.b, unc.color.a = 0.1, 0.85, 1.0, 0.28
                        markers.markers.append(unc)

        self.obs_marker_pub.publish(markers)

        # Publish /obstacle_alert for bridge_server.py (Flutter & Web UI)
        if nearest_alert is not None:
            alert_text = f"{nearest_alert.class_name} detected at {nearest_alert_dist:.2f}m ({nearest_alert.speed:.2f} m/s)"
        else:
            alert_text = ""

        if alert_text != self._last_obs_alert_text or (now - self._last_obs_alert_pub_time) > 1.0:
            self._last_obs_alert_text = alert_text
            self._last_obs_alert_pub_time = now
            amsg = String()
            amsg.data = alert_text
            self.obs_alert_pub.publish(amsg)

        # Publish structured summary for run_logger & diagnostics
        summary = String()
        summary.data = json.dumps({
            "counts": counts,
            "infer_ms": round(self._last_ml_infer_ms, 3),
            "dynamic_count": len(self.dynamic_obstacles),
            "clutter_count": len(self.clutter_obstacles),
        })
        self.ml_obs_pub.publish(summary)

    def densify_path(self, node_path, step_m=0.30):
        """Interpolate dense (x, y, theta) waypoints every step_m meters."""
        dense = []
        for i in range(len(node_path)):
            nx = self.map_data[node_path[i]]["x"]
            ny = self.map_data[node_path[i]]["y"]
            if i == 0:
                if len(node_path) > 1:
                    nx2 = self.map_data[node_path[1]]["x"]
                    ny2 = self.map_data[node_path[1]]["y"]
                    theta0 = math.atan2(ny2 - ny, nx2 - nx)
                else:
                    theta0 = self.current_yaw
                dense.append((nx, ny, theta0))
                continue
            px = self.map_data[node_path[i-1]]["x"]
            py = self.map_data[node_path[i-1]]["y"]
            seg_theta = math.atan2(ny - py, nx - px)
            seg_len = math.sqrt((nx - px)**2 + (ny - py)**2)
            num_steps = max(1, int(seg_len / step_m))
            for k in range(1, num_steps + 1):
                t = k / num_steps
                ix = px + t * (nx - px)
                iy = py + t * (ny - py)
                dense.append((ix, iy, seg_theta))

        # Smooth theta: point each waypoint toward the next dense point
        for i in range(len(dense) - 1):
            x1, y1, _ = dense[i]
            x2, y2, _ = dense[i + 1]
            dense[i] = (x1, y1, math.atan2(y2 - y1, x2 - x1))
        return dense

    def sequence_callback(self, msg):
        """Receives a JSON array of node IDs and builds a goal queue.

        Example message payload: '["N5", "N12", "N40"]'
        Each node ID is snapped to that graph node's (x, y) world coordinates.
        An empty array '[]' cancels the active mission and halts the robot.
        """
        if self._estop_active:
            self._stop_robot()
            self._publish_agv_state("ESTOP")
            self.get_logger().warn("[APP_BRIDGE] Ignoring goal_sequence — E-STOP is latched.")
            return

        try:
            node_ids = json.loads(msg.data)
            if not isinstance(node_ids, list):
                self.get_logger().error("goal_sequence: expected a JSON array of node IDs.")
                return

            if len(node_ids) == 0:
                self.goal_queue = []
                self.path_plan = []
                self.node_path = []
                self.active_detour = None
                self.state = "IDLE"
                self._stop_robot()
                self._publish_agv_state("IDLE")
                self._publish_mission_progress("CANCELLED")
                self.mission_total = 0
                self.mission_current = 0
                self.publish_active_path()
                self.get_logger().info("[APP_BRIDGE] Mission cancelled — robot halted and queue cleared.")
                return

            loc_blocked, loc_reason = self._is_localization_blocked()
            if loc_blocked:
                self._halt_with_fault('MAP_MISMATCH', f"Refusing goal_sequence: {loc_reason}")
                return

            # Validate every node exists in the graph
            unknown = [n for n in node_ids if n not in self.map_data]
            if unknown:
                self.get_logger().error(f"goal_sequence: unknown node IDs {unknown}")
                return

            # Build queue from node world positions
            self.goal_queue = [
                (self.map_data[n]["x"], self.map_data[n]["y"]) for n in node_ids
            ]
            self.mission_total = len(self.goal_queue)
            self.mission_current = 0
            self.get_logger().info(
                f"Mission loaded: {self.mission_total} goals -> {node_ids}"
            )
            # Kick off first goal immediately
            self._dispatch_next_goal()
        except json.JSONDecodeError as e:
            self.get_logger().error(f"goal_sequence: JSON parse error — {e}")

    def _dispatch_next_goal(self):
        """Dequeue the next goal and begin planning toward it."""
        if not self.goal_queue:
            self.get_logger().info("All mission goals completed! Robot idle.")
            self._publish_mission_progress("MISSION_COMPLETE")
            return

        goal_x, goal_y = self.goal_queue.pop(0)
        self.mission_current += 1
        self.get_logger().info(
            f"Mission goal {self.mission_current}/{self.mission_total}: "
            f"({goal_x:.2f}, {goal_y:.2f})"
        )
        # Reuse existing planning logic via a synthetic PoseStamped
        synthetic = PoseStamped()
        synthetic.pose.position.x = goal_x
        synthetic.pose.position.y = goal_y
        self.goal_callback(synthetic)

    def _publish_mission_progress(self, status_override=None):
        """Publish a JSON mission progress string to /mission_progress."""
        status = status_override if status_override else self.state
        payload = {
            "current": self.mission_current,
            "total": self.mission_total,
            "goal_node": self.target_node,
            "state": status,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.mission_pub.publish(msg)

    def _build_lead_in(self, start_node, next_node=None):
        """Builds smooth linear lead-in waypoints from current robot position to start node."""
        if start_node not in self.map_data:
            return []
        sx = self.map_data[start_node]["x"]
        sy = self.map_data[start_node]["y"]
        if next_node and next_node in self.map_data:
            nx = self.map_data[next_node]["x"]
            ny = self.map_data[next_node]["y"]
            dot = (sx - self.current_x) * (nx - self.current_x) + (sy - self.current_y) * (ny - self.current_y)
            seg_d = self._dist_point_to_segment(self.current_x, self.current_y, sx, sy, nx, ny)
            if dot < -0.01 and seg_d < 0.35:
                return []
        dist_to_start = math.hypot(sx - self.current_x, sy - self.current_y)
        lead_in = []
        if dist_to_start > 0.08:
            num_lead = max(2, int(dist_to_start / 0.10))
            lead_th = math.atan2(sy - self.current_y, sx - self.current_x)
            for k in range(num_lead):
                t = k / float(num_lead)
                lx = self.current_x + t * (sx - self.current_x)
                ly = self.current_y + t * (sy - self.current_y)
                lead_in.append((lx, ly, lead_th))
        return lead_in

    def _build_lead_out(self, target_node, goal_x, goal_y):
        """Builds smooth linear lead-out waypoints from target node to exact user-clicked goal."""
        if target_node not in self.map_data or goal_x is None or goal_y is None:
            return []
        tx = self.map_data[target_node]["x"]
        ty = self.map_data[target_node]["y"]
        dist_to_goal = math.hypot(goal_x - tx, goal_y - ty)
        lead_out = []
        if dist_to_goal > 0.005:
            num_lead_out = max(1, int(math.ceil(dist_to_goal / 0.10)))
            lead_out_th = math.atan2(goal_y - ty, goal_x - tx)
            for k in range(1, num_lead_out + 1):
                t = k / float(num_lead_out)
                lx = tx + t * (goal_x - tx)
                ly = ty + t * (goal_y - ty)
                lead_out.append((lx, ly, lead_out_th))
        return lead_out

    def _set_node_path_plan(self, node_path, start_node, goal_x, goal_y):
        """Assemble densified trajectory with lead-in/lead-out and reset tracking indices."""
        self.node_path = node_path
        base_plan = self.densify_path(self.node_path, step_m=0.15)
        next_node = node_path[1] if len(node_path) >= 2 else None
        lead_in = self._build_lead_in(start_node, next_node=next_node)
        end_node = node_path[-1] if node_path else self.target_node
        lead_out = self._build_lead_out(end_node, goal_x, goal_y) if goal_x is not None else []
        self.path_plan = lead_in + base_plan + lead_out

        if len(lead_in) == 0 and len(self.path_plan) > 1:
            search_end = min(len(self.path_plan), 10)
            dists = [math.hypot(self.path_plan[k][0] - self.current_x, self.path_plan[k][1] - self.current_y) for k in range(search_end)]
            self.current_target_index = int(np.argmin(dists))
        else:
            self.current_target_index = 0
        self._visited_plan_nodes = set()
        self._is_pivoting = False

    def goal_callback(self, msg):
        if self._estop_active:
            self._stop_robot()
            self._publish_agv_state("ESTOP")
            self.get_logger().warn("[APP_BRIDGE] Ignoring goal — E-STOP is latched.")
            return

        loc_blocked, loc_reason = self._is_localization_blocked()
        if loc_blocked:
            self._halt_with_fault('MAP_MISMATCH', f"Refusing goal: {loc_reason}")
            return

        self.update_robot_pose()
        if not self.odom_ready:
            self.get_logger().warn("Waiting for localization/odometry before accepting goals.")
            return

        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        self.final_goal_x = goal_x
        self.final_goal_y = goal_y

        self.state = "PLANNING"
        self._publish_agv_state("PLANNING")
        self.get_logger().info(f"Received Goal: ({goal_x:.2f}, {goal_y:.2f})")

        # Reset active recovery, safety monitors, and dynamic blockage penalties on new user goal
        self._recovery_phase = None
        self._stuck_counter = 0
        self._prev_heading_err = None
        self._is_pivoting = False
        self._slip_anchor_odom = None
        self._slip_anchor_scan = None
        self._slip_strikes = 0
        self._map_mismatch_strikes = 0
        self.blocked_edges.clear()
        self.hazard_zones.clear()

        start_node = self.find_closest_node(self.current_x, self.current_y)
        self.target_node = self.find_closest_node(goal_x, goal_y)

        self.get_logger().info(f"Snapping to nodes: Start={start_node}, Target={self.target_node}")

        if start_node == self.target_node:
            dist_direct = math.hypot(goal_x - self.current_x, goal_y - self.current_y)
            if dist_direct < self.goal_arrival_tolerance:
                self.get_logger().info(f"[BRAIN] Already at target destination ({dist_direct:.3f}m < {self.goal_arrival_tolerance:.2f}m). Arrival complete.")
                self.state = "IDLE"
                self._stop_robot()
                self._publish_agv_state("ARRIVED")
                return
            num_pts = max(3, int(math.ceil(dist_direct / 0.08)))
            direct_th = math.atan2(goal_y - self.current_y, goal_x - self.current_x)
            self.node_path = [start_node]
            self.path_plan = [
                (self.current_x + (k / float(num_pts)) * (goal_x - self.current_x),
                 self.current_y + (k / float(num_pts)) * (goal_y - self.current_y),
                 direct_th)
                for k in range(1, num_pts + 1)
            ]
            self.current_target_index = 0
            self._visited_plan_nodes = set()
        else:
            node_path = self.calculate_dijkstra(start_node, self.target_node)

            if not node_path:
                self.state = "IDLE"
                self._publish_agv_state("IDLE")
                return

            self._set_node_path_plan(node_path, start_node, goal_x, goal_y)

        self.get_logger().info(
            f"Active Plan: {self.node_path} ({len(self.path_plan)} waypoints, "
            f"ending at destination ({self.path_plan[-1][0]:.2f}, {self.path_plan[-1][1]:.2f}))"
        )

        self.last_cmd_v = 0.0
        self.last_cmd_w = 0.0
        self.last_pub_w = 0.0
        self.last_target_v = 0.0
        self.last_target_w = 0.0
        self.state = "NAVIGATING"
        self._publish_agv_state("NAVIGATING")
        self.publish_active_path()
        self._publish_mission_progress()

    def publish_active_path(self):
        ros_path = Path()
        ros_path.header.frame_id = "map"
        ros_path.header.stamp = self.get_clock().now().to_msg()
        for (px, py, ptheta) in self.path_plan:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = px
            pose.pose.position.y = py
            pose.pose.orientation.z = math.sin(ptheta / 2.0)
            pose.pose.orientation.w = math.cos(ptheta / 2.0)
            ros_path.poses.append(pose)
        self.path_pub.publish(ros_path)

    def get_lookahead_target(self, lookahead_dist=1.2):
        """Finds target point along path arc-length to prevent corner cutting."""
        min_dist = float('inf')
        closest_idx = self.current_target_index
        # Advance monotonically along the active path (prevents in-place rotation jitter regressions)
        start_search = max(0, min(self.current_target_index, len(self.path_plan) - 1))
        search_window = min(len(self.path_plan), start_search + 8)
        for i in range(start_search, search_window):
            px, py, _ = self.path_plan[i]
            dist = math.sqrt((px - self.current_x)**2 + (py - self.current_y)**2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        self.current_target_index = closest_idx

        # Accumulate arc-length along the path curve
        arc_length = 0.0
        target_idx = closest_idx
        for i in range(closest_idx, len(self.path_plan) - 1):
            x1, y1, _ = self.path_plan[i]
            x2, y2, _ = self.path_plan[i + 1]
            arc_length += math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            if arc_length >= lookahead_dist:
                target_idx = i + 1
                break
        else:
            target_idx = len(self.path_plan) - 1

        return self.path_plan[target_idx], target_idx

    def check_proximity_and_clearance(self):
        """Calculates minimum obstacle distance in front (120 deg) and rear (120 deg) body arcs."""
        if len(self.obstacles) == 0:
            return 99.0, 99.0

        c_yaw = math.cos(self.current_yaw)
        s_yaw = math.sin(self.current_yaw)

        dx = self.obstacles[:, 0] - self.current_x
        dy = self.obstacles[:, 1] - self.current_y

        x_rel = dx * c_yaw + dy * s_yaw
        y_rel = -dx * s_yaw + dy * c_yaw
        dist = np.sqrt(x_rel**2 + y_rel**2)

        # Front swept body corridor: check obstacles in the forward travel path
        # Chassis footprint radius is ~0.155m; ignore internal self-reflections (x_rel <= 0.16m)
        front_lat_limit = np.maximum(0.16, x_rel * 0.466)
        front_mask = (x_rel > 0.16) & (np.abs(y_rel) < front_lat_limit) & (dist < 6.0)
        d_front = float(np.min(dist[front_mask])) if np.any(front_mask) else 99.0

        # Rear swept body corridor: check obstacles directly behind chassis
        rear_lat_limit = np.maximum(0.16, -x_rel * 0.466)
        rear_mask = (x_rel < -0.16) & (np.abs(y_rel) < rear_lat_limit) & (dist < 6.0)
        d_rear = float(np.min(dist[rear_mask])) if np.any(rear_mask) else 99.0

        return d_front, d_rear

    def get_corridor_obstacle_analysis(self, lookahead_horizon=2.8):
        """Proactively scans up to lookahead_horizon meters ahead along the planned path.
        Detects unmapped stationary and dynamic obstacles, computes aisle passability for
        a 22cm robot, and identifies the best bypass direction (LEFT/RIGHT).
        """
        if len(self.obstacles) == 0 or len(self.path_plan) < 5:
            return None

        # Only consider unmapped obstacles so known static room walls don't cause start-node reroute loops
        if self.static_wall_mask is not None:
            unmapped_mask = np.array([not self.is_near_static_wall(p[0], p[1]) for p in self.obstacles], dtype=bool)
            cand_obstacles = self.obstacles[unmapped_mask]
        else:
            cand_obstacles = self.obstacles

        if len(cand_obstacles) == 0:
            return None

        arc_dist = 0.0
        start_idx = self.current_target_index
        upcoming_indices = []

        for i in range(start_idx, len(self.path_plan) - 1):
            p1 = self.path_plan[i]
            p2 = self.path_plan[i + 1]
            seg_len = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
            arc_dist += seg_len
            # Ignore points < 0.25m along path so corners/walls behind the robot don't trigger false detours
            if arc_dist >= 0.25:
                upcoming_indices.append(i + 1)
            if arc_dist >= lookahead_horizon:
                break

        if len(upcoming_indices) < 2:
            return None

        path_pts = np.array([[self.path_plan[i][0], self.path_plan[i][1]] for i in upcoming_indices])

        # Distance from each unmapped obstacle point to upcoming path waypoints
        dists_matrix = np.linalg.norm(cand_obstacles[:, np.newaxis, :] - path_pts[np.newaxis, :, :], axis=-1)

        # Swept corridor width for 22cm robot: points within 0.22m of path centerline
        min_dists_to_path = np.min(dists_matrix, axis=0)
        blocked_wpts = np.where(min_dists_to_path < 0.22)[0]
        if len(blocked_wpts) == 0:
            return None

        earliest_local_idx = int(blocked_wpts[0])
        block_idx = upcoming_indices[earliest_local_idx]
        block_pt = self.path_plan[block_idx]

        # Ignore obstacle points in the immediate arrival zone (< 0.40m) of the final goal destination
        final_x, final_y, _ = self.path_plan[-1]
        if math.hypot(block_pt[0] - final_x, block_pt[1] - final_y) < 0.40:
            return None

        # Calculate distance along path from robot to the obstacle
        d_along = 0.0
        for i in range(start_idx, block_idx):
            d_along += math.hypot(self.path_plan[i+1][0] - self.path_plan[i][0],
                                  self.path_plan[i+1][1] - self.path_plan[i][1])

        # Cluster unmapped points around the obstacle location
        cluster_mask = np.linalg.norm(cand_obstacles - np.array([block_pt[0], block_pt[1]]), axis=-1) < 0.60
        cluster_pts = cand_obstacles[cluster_mask]
        # Require at least 4 contiguous LiDAR points to confirm a real physical obstacle (rejecting sensor noise / dust)
        if len(cluster_pts) < 4:
            return None

        ox = float(np.mean(cluster_pts[:, 0]))
        oy = float(np.mean(cluster_pts[:, 1]))
        min_dist_to_robot = math.hypot(ox - self.current_x, oy - self.current_y)

        # Estimate obstacle physical radius from cluster spread (clamped between 0.18m and 0.35m)
        obs_radii = np.linalg.norm(cluster_pts - np.array([ox, oy]), axis=-1)
        r_obs = float(np.percentile(obs_radii, 90)) if len(obs_radii) >= 3 else 0.25
        r_obs = max(0.18, min(0.35, r_obs))

        # Circular Safe Zone: obstacle radius + robot radius (0.11m) + clearance margin (0.10m)
        r_safe = r_obs + self.robot_radius + 0.10

        # Check if obstacle is ML-classified dynamic (PERSON) or static clutter (CLUTTER)
        is_dynamic = False
        obs_vel = (0.0, 0.0)
        obs_speed = 0.0
        obs_class = 'CLUTTER'
        for dob in self.dynamic_obstacles:
            if math.hypot(float(dob['pos'][0]) - ox, float(dob['pos'][1]) - oy) < 0.75:
                is_dynamic = True
                obs_vel = (float(dob['vel'][0]), float(dob['vel'][1]))
                obs_speed = float(dob['speed'])
                obs_class = dob.get('class_name', 'PERSON')
                r_obs = max(r_obs, float(dob.get('radius', r_obs)))
                r_safe = r_obs + self.robot_radius + 0.10
                break
        if not is_dynamic:
            for cob in self.clutter_obstacles:
                if math.hypot(float(cob['pos'][0]) - ox, float(cob['pos'][1]) - oy) < 0.65:
                    obs_class = cob.get('class_name', 'CLUTTER')
                    r_obs = max(r_obs, float(cob.get('radius', r_obs)))
                    r_safe = r_obs + self.robot_radius + 0.10
                    break

        # Coordinate frame along the path at obstacle
        theta_path = block_pt[2]
        nx_left = -math.sin(theta_path)
        ny_left = math.cos(theta_path)

        # Obstacle center's lateral offset relative to path centerline
        d_obs_lat = (ox - block_pt[0]) * nx_left + (oy - block_pt[1]) * ny_left

        # Environmental clearance calculation:
        # Exclude points belonging to the obstacle itself (points within r_obs + 0.05m of (ox, oy))
        dists_to_obs = np.linalg.norm(self.obstacles - np.array([ox, oy]), axis=-1)
        env_mask = (dists_to_obs >= (r_obs + 0.05)) & (dists_to_obs < 2.2)
        env_pts = self.obstacles[env_mask]

        clearance_left = 1.2
        clearance_right = 1.2

        if len(env_pts) > 0:
            dx_env = env_pts[:, 0] - ox
            dy_env = env_pts[:, 1] - oy
            lat_disp = dx_env * nx_left + dy_env * ny_left

            # Left environment points (outside the obstacle circle on the left)
            left_env = lat_disp[lat_disp > (r_obs + 0.05)]
            # Right environment points (outside the obstacle circle on the right)
            right_env = -lat_disp[lat_disp < -(r_obs + 0.05)]

            # Clearance between the circular safe zone apex and any walls
            if len(left_env) > 0:
                dist_wall_left = float(np.min(left_env))
                clearance_left = max(0.0, dist_wall_left - r_safe)
            if len(right_env) > 0:
                dist_wall_right = float(np.min(right_env))
                clearance_right = max(0.0, dist_wall_right - r_safe)

        # Minimum opening needed: robot radius (0.11m) + 0.05m wall margin = 0.16m
        min_clearance_needed = self.robot_radius + 0.05
        passable_left = clearance_left >= min_clearance_needed
        passable_right = clearance_right >= min_clearance_needed

        # Deterministic decision with hysteresis (prevent left/right oscillation)
        best_side = None
        passable = False

        if self.active_detour is not None and self.active_detour.get('side') in ['LEFT', 'RIGHT']:
            locked_side = self.active_detour['side']
            if locked_side == 'LEFT' and passable_left:
                best_side = 'LEFT'
                passable = True
            elif locked_side == 'RIGHT' and passable_right:
                best_side = 'RIGHT'
                passable = True

        if best_side is None:
            if passable_left and passable_right:
                # Both open: prefer side opposite to obstacle center offset
                if d_obs_lat < -0.04:
                    best_side = 'LEFT'
                elif d_obs_lat > 0.04:
                    best_side = 'RIGHT'
                else:
                    best_side = 'LEFT' if clearance_left >= clearance_right else 'RIGHT'
                passable = True
            elif passable_left:
                best_side = 'LEFT'
                passable = True
            elif passable_right:
                best_side = 'RIGHT'
                passable = True

        # Calculate lateral offset from nominal path centerline to circular safe zone apex
        lateral_offset = 0.0
        if best_side == 'LEFT':
            lateral_offset = min(0.65, max(0.28, d_obs_lat + r_safe))
        elif best_side == 'RIGHT':
            lateral_offset = -min(0.65, max(0.28, -d_obs_lat + r_safe))

        block_start_idx = max(start_idx, block_idx - 5)
        block_rejoin_idx = min(len(self.path_plan) - 1, block_idx + 5)

        return {
            'obstacle_pos': (ox, oy),
            'obstacle_radius': r_obs,
            'safe_radius': r_safe,
            'dist_along_path': d_along,
            'min_dist_to_robot': min_dist_to_robot,
            'is_dynamic': is_dynamic,
            'vel': obs_vel,
            'speed': obs_speed,
            'passable': passable,
            'best_side': best_side,
            'clearance_left': clearance_left,
            'clearance_right': clearance_right,
            'lateral_offset': lateral_offset,
            'block_idx': block_idx,
            'block_start_idx': block_start_idx,
            'block_rejoin_idx': block_rejoin_idx,
        }

    @staticmethod
    def _dist_point_to_segment(px, py, x1, y1, x2, y2):
        """Calculates perpendicular Euclidean distance from point (px, py) to segment (x1, y1)-(x2, y2)."""
        dx = x2 - x1
        dy = y2 - y1
        l2 = dx * dx + dy * dy
        if l2 == 0.0:
            return math.hypot(px - x1, py - y1)
        t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / l2))
        proj_x = x1 + t * dx
        proj_y = y1 + t * dy
        return math.hypot(px - proj_x, py - proj_y)

    def _get_edge_cost(self, u, v):
        """Gets original edge distance between u and v."""
        if u in self.map_data:
            for edge in self.map_data[u]["edges"]:
                if edge["to_node"] == v:
                    return edge.get("distance_m", 1.0)
        return 1.0

    def find_best_reroute_start_node(self, rx, ry, ryaw, blocked_nodes):
        """Finds the best unblocked candidate start node near the robot,
        preferring nodes ahead of or alongside the robot heading to avoid 180 deg turnaround."""
        candidates = []
        for nid, data in self.map_data.items():
            if nid in blocked_nodes:
                continue
            dist = math.hypot(data["x"] - rx, data["y"] - ry)
            if dist < 1.3:
                # Reject start nodes whose direct line-of-sight from robot crosses any active hazard zone
                if any(self._dist_point_to_segment(h['pos'][0], h['pos'][1], rx, ry, data["x"], data["y"]) < h['radius'] for h in self.hazard_zones):
                    continue
                # Forward projection along current heading vector
                fwd_proj = (data["x"] - rx) * math.cos(ryaw) + (data["y"] - ry) * math.sin(ryaw)
                candidates.append((dist, fwd_proj, nid))

        if not candidates:
            # Fallback to closest unblocked node in whole map
            best_nid = None
            min_dist = float('inf')
            for nid, data in self.map_data.items():
                if nid in blocked_nodes:
                    continue
                dist = math.hypot(data["x"] - rx, data["y"] - ry)
                if dist < min_dist:
                    min_dist = dist
                    best_nid = nid
            return best_nid

        # Sort: prefer nodes with forward_proj >= -0.15m (ahead or alongside), then by distance
        candidates.sort(key=lambda c: (0 if c[1] >= -0.15 else 1, c[0]))
        return candidates[0][2]

    def calculate_dijkstra_avoiding(self, start_node, target_node, blocked_nodes, blocked_edges):
        """Calculates shortest topological node path avoiding blocked nodes and edges."""
        if start_node == target_node:
            return [start_node]

        distances = {node: float('infinity') for node in self.map_data}
        distances[start_node] = 0.0
        previous_nodes = {node: None for node in self.map_data}
        pq = [(0.0, start_node)]

        while pq:
            current_distance, current_node = heapq.heappop(pq)
            if current_node == target_node:
                break
            if current_distance > distances[current_node]:
                continue
            for edge in self.map_data[current_node]["edges"]:
                neighbor = edge["to_node"]
                if neighbor in blocked_nodes and neighbor != target_node:
                    continue
                e_pair = (min(current_node, neighbor), max(current_node, neighbor))
                if e_pair in blocked_edges:
                    continue
                weight = edge["distance_m"]
                distance = current_distance + weight
                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    previous_nodes[neighbor] = current_node
                    heapq.heappush(pq, (distance, neighbor))

        path = []
        current = target_node
        while current is not None:
            path.append(current)
            current = previous_nodes[current]
        path.reverse()
        if not path or path[0] != start_node:
            return []
        return path

    def replan_around_obstacle(self, ox, oy, r_safe=0.42):
        """Computes a connected topological node path from the warehouse graph around the obstacle."""
        now = self._now_sec()

        # Clean expired hazard memory
        self.hazard_zones = [h for h in self.hazard_zones if now < h.get('expire_time', 0.0)]

        # Try with requested r_safe, fall back to tighter safe radius if graph is narrow
        for safe_radius in [max(0.35, min(0.45, r_safe)), 0.35]:
            # 1. Identify all nodes within safe radius (current obstacle + active hazard zones)
            blocked_nodes = set()
            for nid, data in self.map_data.items():
                if math.hypot(data["x"] - ox, data["y"] - oy) < safe_radius:
                    blocked_nodes.add(nid)
                for h in self.hazard_zones:
                    if math.hypot(data["x"] - h['pos'][0], data["y"] - h['pos'][1]) < h['radius']:
                        blocked_nodes.add(nid)

            # 2. Identify all edges intersecting safe zone
            blocked_edges = set()
            check_centers = [(ox, oy, safe_radius)] + [(h['pos'][0], h['pos'][1], h['radius']) for h in self.hazard_zones]
            for nid, data in self.map_data.items():
                for (cx, cy, cr) in check_centers:
                    if math.hypot(data["x"] - cx, data["y"] - cy) < 2.5:
                        x1, y1 = data["x"], data["y"]
                        for edge in data["edges"]:
                            v = edge["to_node"]
                            x2, y2 = self.map_data[v]["x"], self.map_data[v]["y"]
                            if self._dist_point_to_segment(cx, cy, x1, y1, x2, y2) < cr:
                                blocked_edges.add((min(nid, v), max(nid, v)))

            # 3. Find candidate start nodes near the robot
            candidates = []
            for nid, data in self.map_data.items():
                if nid in blocked_nodes:
                    continue
                dist = math.hypot(data["x"] - self.current_x, data["y"] - self.current_y)
                if dist < 1.3:
                    # Critical Line-of-Sight Check: direct segment from robot to candidate start node
                    # must NOT cross the obstacle or any hazard zone!
                    d_obs_seg = self._dist_point_to_segment(ox, oy, self.current_x, self.current_y, data["x"], data["y"])
                    if d_obs_seg < safe_radius:
                        continue
                    if any(self._dist_point_to_segment(h['pos'][0], h['pos'][1], self.current_x, self.current_y, data["x"], data["y"]) < h['radius'] for h in self.hazard_zones):
                        continue

                    fwd_proj = (data["x"] - self.current_x) * math.cos(self.current_yaw) + (data["y"] - self.current_y) * math.sin(self.current_yaw)
                    candidates.append((dist, fwd_proj, nid))

            candidates.sort(key=lambda c: (0 if c[1] >= -0.15 else 1, c[0]))

            # Try planning from the best candidate start node to target_node
            new_path = []
            chosen_start = None
            for dist, fwd_proj, s_node in candidates:
                path = self.calculate_dijkstra_avoiding(s_node, self.target_node, blocked_nodes, blocked_edges)
                if path and len(path) > 1:
                    new_path = path
                    chosen_start = s_node
                    break

            if not new_path:
                fallback_start = self.find_best_reroute_start_node(self.current_x, self.current_y, self.current_yaw, blocked_nodes)
                if fallback_start:
                    path = self.calculate_dijkstra_avoiding(fallback_start, self.target_node, blocked_nodes, blocked_edges)
                    if path and len(path) > 1:
                        new_path = path
                        chosen_start = fallback_start

            if new_path and len(new_path) > 1:
                # Temporarily penalize blocked edges in self.blocked_edges so other routines respect them
                for (u, v) in blocked_edges:
                    if (u, v) not in self.blocked_edges:
                        self.blocked_edges[(u, v)] = (self._get_edge_cost(u, v), now + 25.0)

                self._set_node_path_plan(new_path, chosen_start, self.final_goal_x, self.final_goal_y)
                self._reroute_cooldown_time = now + 3.5  # Lock path for 3.5s to prevent flip-flopping
                self.active_detour = None
                self._swerve_bias = 0.0
                self._swerve_lock_ticks = 0
                self.publish_active_path()
                self.state = "NAVIGATING"
                self.yield_start_time = None
                self.get_logger().info(
                    f"[BRAIN: TOPOLOGICAL_AVOIDANCE] Successfully planned CONNECTED NODE path around obstacle at "
                    f"({ox:.2f}, {oy:.2f}) [R_safe={safe_radius:.2f}m]: {self.node_path} "
                    f"({len(self.path_plan)} waypoints, ending at ({self.path_plan[-1][0]:.2f}, {self.path_plan[-1][1]:.2f})). Resuming navigation."
                )
                return True

        self.get_logger().warn(
            f"[BRAIN: TOPOLOGICAL_AVOIDANCE] No clear connected node path found around obstacle at ({ox:.2f}, {oy:.2f}) "
            f"to {self.target_node}!"
        )
        return False

    def trigger_reroute(self):
        """Dynamically penalizes the blockage and computes an alternate connected node path."""
        now = self._now_sec()
        self.active_detour = None

        # Clean expired hazard memory
        self.hazard_zones = [h for h in self.hazard_zones if now < h.get('expire_time', 0.0)]

        # 1. Prefer obstacle from proactive corridor scan if available
        corridor_obs = self.get_corridor_obstacle_analysis(lookahead_horizon=2.8)
        if corridor_obs is not None:
            ox, oy = corridor_obs['obstacle_pos']
            r_safe = max(0.35, corridor_obs.get('safe_radius', 0.45))
            self.get_logger().warn(
                f"[BRAIN: DIJKSTRA_ROUTER] Triggering connected node avoidance from corridor scan around ({ox:.2f}, {oy:.2f})..."
            )
            if self.replan_around_obstacle(ox, oy, r_safe=r_safe):
                return

        # 2. Otherwise locate closest clustered obstacle in front of the robot (filter out walls)
        ox, oy = None, None
        if len(self.obstacles) > 0:
            c_yaw = math.cos(self.current_yaw)
            s_yaw = math.sin(self.current_yaw)
            dx = self.obstacles[:, 0] - self.current_x
            dy = self.obstacles[:, 1] - self.current_y
            x_rel = dx * c_yaw + dy * s_yaw
            y_rel = -dx * s_yaw + dy * c_yaw
            # Focus on front travel corridor: x in [0.05, 2.0], |y| < 0.40
            front_mask = (x_rel > 0.05) & (x_rel < 2.0) & (np.abs(y_rel) < 0.40)
            if np.any(front_mask):
                front_pts = self.obstacles[front_mask]
                dists = np.sqrt(x_rel[front_mask]**2 + y_rel[front_mask]**2)
                closest_idx = np.argmin(dists)
                ox = float(front_pts[closest_idx, 0])
                oy = float(front_pts[closest_idx, 1])

        if ox is not None and oy is not None:
            self.get_logger().warn(
                f"[BRAIN: DIJKSTRA_ROUTER] Triggering connected node avoidance around obstacle at ({ox:.2f}, {oy:.2f})..."
            )
            if self.replan_around_obstacle(ox, oy, r_safe=0.45):
                return

        # Fallback if no specific obstacle cluster was found or replan_around_obstacle failed:
        best_idx = 0
        min_d = float('inf')
        for i, nid in enumerate(self.node_path):
            nx = self.map_data[nid]["x"]
            ny = self.map_data[nid]["y"]
            d = math.hypot(nx - self.current_x, ny - self.current_y)
            if d < min_d:
                min_d = d
                best_idx = i

        u, v = None, None
        if best_idx + 1 < len(self.node_path):
            u = self.node_path[best_idx]
            v = self.node_path[best_idx + 1]
        elif best_idx > 0:
            u = self.node_path[best_idx - 1]
            v = self.node_path[best_idx]

        if u and v:
            orig_cost = self._get_edge_cost(u, v)
            for edge in self.map_data[u]["edges"]:
                if edge["to_node"] == v:
                    edge["distance_m"] = 999.0
                    self.blocked_edges[(u, v)] = (orig_cost, now + 25.0)

            for edge in self.map_data[v]["edges"]:
                if edge["to_node"] == u:
                    orig_cost = edge["distance_m"]
                    edge["distance_m"] = 999.0
                    self.blocked_edges[(v, u)] = (orig_cost, now + 25.0)

            self.get_logger().warn(
                f"[BRAIN: DIJKSTRA_ROUTER] Commanded ROUTE CHANGE to avoid blockage! "
                f"Blocked edge ({u} <-> {v}) temporarily penalized (+999.0 for 25s)."
            )

        blocked_nodes = set()
        for h in self.hazard_zones:
            for nid, data in self.map_data.items():
                if math.hypot(data["x"] - h['pos'][0], data["y"] - h['pos'][1]) < h['radius']:
                    blocked_nodes.add(nid)

        start_node = self.find_best_reroute_start_node(self.current_x, self.current_y, self.current_yaw, blocked_nodes)
        if not start_node:
            start_node = self.find_closest_node(self.current_x, self.current_y)

        new_path = self.calculate_dijkstra(start_node, self.target_node)
        if new_path and len(new_path) > 1:
            self._set_node_path_plan(new_path, start_node, self.final_goal_x, self.final_goal_y)
            self._reroute_cooldown_time = now + 3.5
            self._swerve_bias = 0.0
            self._swerve_lock_ticks = 0
            self.publish_active_path()
            self.state = "NAVIGATING"
            self.yield_start_time = None
            self.get_logger().info(
                f"[BRAIN: DIJKSTRA_ROUTER] DYNAMIC REROUTE SUCCESSFUL! "
                f"New route: {self.node_path} ({len(self.path_plan)} waypoints, ending at ({self.path_plan[-1][0]:.2f}, {self.path_plan[-1][1]:.2f})). Resuming navigation."
            )
        else:
            self.get_logger().warn(
                f"[BRAIN: DIJKSTRA_ROUTER] No alternate route available around blockage from {start_node} to {self.target_node}. "
                f"Robot remains YIELDING."
            )
            self.state = "YIELDING"
            if self.yield_start_time is None:
                self.yield_start_time = now

    def restore_unblocked_edges(self):
        """Restores temporarily blocked edges once their cooldown has elapsed."""
        now = self._now_sec()
        restored = []
        for (u, v), (orig_cost, restore_time) in list(self.blocked_edges.items()):
            if now >= restore_time:
                if u in self.map_data:
                    for edge in self.map_data[u]["edges"]:
                        if edge["to_node"] == v:
                            edge["distance_m"] = orig_cost
                restored.append((u, v))
                del self.blocked_edges[(u, v)]

        if restored:
            self.get_logger().info(f"Restored graph edges: {restored}")

    def _execute_recovery(self):
        """Active reverse escape: checks rear clearance from 360 LiDAR, reverses smoothly
        until reaching safe standoff (0.75m) or timeout, then initiates Dijkstra reroute."""
        now = self._now_sec()
        self._swerve_bias = 0.0
        self._swerve_lock_ticks = 0

        d_front, d_rear = self.check_proximity_and_clearance()

        if self._recovery_phase is None:
            self._recovery_phase = "BACKUP"
            self._recovery_start_time = now
            self._stuck_counter = 0
            self._last_stuck_check_pos = None

            # Pinpoint the hazard location that triggered this recovery
            hazard_pt = None
            if len(self.obstacles) > 0:
                c_yaw = math.cos(self.current_yaw)
                s_yaw = math.sin(self.current_yaw)
                dx = self.obstacles[:, 0] - self.current_x
                dy = self.obstacles[:, 1] - self.current_y
                x_rel = dx * c_yaw + dy * s_yaw
                y_rel = -dx * s_yaw + dy * c_yaw
                front_mask = (x_rel > 0.05) & (x_rel < 1.8) & (np.abs(y_rel) < 0.40)
                if np.any(front_mask):
                    front_pts = self.obstacles[front_mask]
                    dists = np.sqrt(x_rel[front_mask]**2 + y_rel[front_mask]**2)
                    closest_idx = np.argmin(dists)
                    hazard_pt = (float(front_pts[closest_idx, 0]), float(front_pts[closest_idx, 1]))

            if hazard_pt is not None:
                self.hazard_zones.append({
                    'pos': hazard_pt,
                    'radius': 0.45,
                    'expire_time': now + 30.0
                })
                self.get_logger().warn(
                    f"[BRAIN: HAZARD_ZONE] Registered hazard memory at ({hazard_pt[0]:.2f}, {hazard_pt[1]:.2f}) "
                    f"for 30s to prevent circular re-collision."
                )

            self.get_logger().warn(
                f"[BRAIN: RECOVERY] Active Reverse Escape initiated at ({self.current_x:.2f}, {self.current_y:.2f}). "
                f"Front proximity={d_front:.2f}m, Rear clearance={d_rear:.2f}m. "
                f"Reversing to safe standoff distance ({self.safe_standoff:.2f}m)..."
            )

        if self._recovery_phase == "BACKUP":
            elapsed = now - self._recovery_start_time

            # Safety check: if rear obstacle is too close (< 0.20m), abort reverse to prevent collision
            if d_rear < 0.20:
                self.get_logger().warn(
                    f"[BRAIN: RECOVERY] Rear boundary detected ({d_rear:.2f}m < 0.28m)! "
                    f"Halting reverse immediately for safety."
                )
                self._stop_robot()
                self._recovery_phase = "REROUTE"
            # Standoff check: if front distance is now >= safe_standoff (0.75m), or max duration reached
            elif d_front >= self.safe_standoff or elapsed >= self._recovery_backup_dur:
                self.get_logger().info(
                    f"[BRAIN: RECOVERY] Standoff distance achieved (front={d_front:.2f}m, elapsed={elapsed:.1f}s). "
                    f"Halting reverse."
                )
                self._stop_robot()
                self._recovery_phase = "REROUTE"
            else:
                # Command smooth, controlled minimum reverse
                rev_speed = -max(self.min_linear_speed, min(0.08, self.cruise_linear_speed * 0.7))
                self.last_cmd_v = rev_speed
                self.last_cmd_w = 0.0
                cmd = Twist()
                cmd.linear.x = rev_speed
                cmd.angular.z = 0.0
                self.cmd_pub.publish(cmd)
                return

        if self._recovery_phase == "REROUTE":
            self.get_logger().warn(
                f"[BRAIN: RECOVERY] Re-orienting & REROUTING via Dijkstra from "
                f"({self.current_x:.2f}, {self.current_y:.2f}) with safe standoff {d_front:.2f}m."
            )
            self._recovery_phase = None
            self._stuck_counter = 0
            self._last_stuck_check_pos = None
            self.active_detour = None
            self.trigger_reroute()

    def _check_wheel_slip(self) -> bool:
        """Cross-check wheel/EKF odometry motion against physical LiDAR scan motion.

        Detects wheel spin on an elevated stand or severe floor slip where /odometry/filtered
        advances while consecutive LiDAR scans remain physically stationary.
        """
        is_actively_driving = (
            self.state == "NAVIGATING"
            and self._recovery_phase is None
            and (abs(self.last_cmd_v) >= 0.02 or abs(self.last_pub_w) >= 0.12)
        )
        if not is_actively_driving or self._odom_pose is None or self._latest_scan_local is None or len(self._latest_scan_local) < 25:
            if self.state != "NAVIGATING" or self._recovery_phase is not None:
                self._slip_anchor_odom = None
                self._slip_anchor_scan = None
                self._slip_strikes = 0
            return False

        if self._slip_anchor_odom is None or self._slip_anchor_scan is None:
            self._slip_anchor_odom = self._odom_pose
            self._slip_anchor_scan = self._latest_scan_local.copy()
            return False

        anc_ox, anc_oy, anc_oyaw = self._slip_anchor_odom
        cur_ox, cur_oy, cur_oyaw = self._odom_pose
        dx_w = cur_ox - anc_ox
        dy_w = cur_oy - anc_oy
        ds_odom = math.hypot(dx_w, dy_w)
        dyaw_odom = math.atan2(math.sin(cur_oyaw - anc_oyaw), math.cos(cur_oyaw - anc_oyaw))

        if ds_odom < 0.25 and abs(dyaw_odom) < 0.50:
            return False

        anc_pts = self._slip_anchor_scan
        cur_pts = self._latest_scan_local
        kd = KDTree(anc_pts)

        # 1. Static hypothesis: physical robot did NOT move
        static_dists, _ = kd.query(cur_pts)
        err_static = float(np.median(static_dists))

        # 2. Odom hypothesis: transform cur_pts into anchor frame using claimed EKF delta
        cos_a = math.cos(anc_oyaw)
        sin_a = math.sin(anc_oyaw)
        dx_loc = dx_w * cos_a + dy_w * sin_a
        dy_loc = -dx_w * sin_a + dy_w * cos_a
        c_d = math.cos(dyaw_odom)
        s_d = math.sin(dyaw_odom)
        pred_pts = np.column_stack((
            dx_loc + cur_pts[:, 0] * c_d - cur_pts[:, 1] * s_d,
            dy_loc + cur_pts[:, 0] * s_d + cur_pts[:, 1] * c_d,
        ))
        pred_dists, _ = kd.query(pred_pts)
        err_pred = float(np.median(pred_dists))

        if err_static < 0.045 and err_pred > max(0.11, err_static * 2.2):
            self._slip_strikes += 1
            self.get_logger().warn(
                f"[SAFETY: WHEEL_SLIP] Window strike {self._slip_strikes}/2: "
                f"odom delta=(ds={ds_odom:.2f}m, dyaw={math.degrees(dyaw_odom):.1f}°), "
                f"scan err_static={err_static:.3f}m vs err_pred={err_pred:.3f}m."
            )
            if self._slip_strikes >= 2 or (ds_odom >= 0.45 and err_static < 0.040):
                self._halt_with_fault(
                    'WHEEL_SLIP',
                    f"Wheels spun (odom ds={ds_odom:.2f}m, dyaw={math.degrees(dyaw_odom):.1f}°) "
                    f"with stationary LiDAR scan (err_static={err_static:.3f}m vs err_pred={err_pred:.3f}m). "
                    f"Check floor contact / wheel traction."
                )
                return True
        else:
            self._slip_strikes = 0

        # Re-anchor window for the next displacement interval
        self._slip_anchor_odom = self._odom_pose
        self._slip_anchor_scan = cur_pts.copy()
        return False

    def _check_runtime_map_consistency(self, now: float) -> bool:
        """Continuously checks whether live scan points in map frame agree with static map walls."""
        if (now - self._last_map_check_time) < 0.5:
            return False
        self._last_map_check_time = now

        if (
            self.state != "NAVIGATING"
            or not self.using_map_tf
            or self.static_wall_mask is None
            or len(self.obstacles) < 30
        ):
            self._map_mismatch_strikes = 0
            return False

        d_sq = (self.obstacles[:, 0] - self.current_x) ** 2 + (self.obstacles[:, 1] - self.current_y) ** 2
        bounded = self.obstacles[d_sq <= 36.0]
        if len(bounded) < 25:
            return False

        hits = sum(1 for p in bounded if self._is_on_mapped_wall(p[0], p[1]))
        hit_frac = hits / float(len(bounded))
        if hit_frac < 0.22:
            self._map_mismatch_strikes += 1
            if self._map_mismatch_strikes >= 8:
                self._halt_with_fault(
                    'MAP_MISMATCH',
                    f"Runtime scan-to-map agreement dropped to {hit_frac:.2f} (<0.22 for 4.0s). Wrong map or localization lost."
                )
                return True
        else:
            self._map_mismatch_strikes = 0
        return False

    def control_loop(self):
        if self._estop_active:
            self._stop_robot()
            return
        self.update_robot_pose()
        if self.state not in ["NAVIGATING", "YIELDING"]:
            self._slip_anchor_odom = None
            self._slip_anchor_scan = None
            self._slip_strikes = 0
            return

        now = self._now_sec()
        if self._check_wheel_slip() or self._check_runtime_map_consistency(now):
            return
        self.restore_unblocked_edges()

        snapped_node = self.find_closest_node(self.current_x, self.current_y)
        self.last_snapped_node = snapped_node

        # Handle active recovery maneuver directly before standard control flow
        if self._recovery_phase is not None:
            self._stuck_counter = 0
            self._last_stuck_check_pos = None
            self._execute_recovery()
            return

        # Check critical proximity immediately (before path following)
        d_front, d_rear = self.check_proximity_and_clearance()
        # When following an active detour around the obstacle, the robot intentionally skirts close to R_safe;
        # only engage emergency reverse if distance is critically inside the physical chassis contact zone (< 0.18m).
        proximity_limit = 0.16 if (self.active_detour is not None) else self.critical_proximity
        if d_front < proximity_limit:
            self.get_logger().warn(
                f"[BRAIN: PROXIMITY_ESCAPE] Critical front proximity ({d_front:.2f}m < {proximity_limit:.2f}m)! "
                f"Halting forward motion and engaging Active Reverse Escape..."
            )
            self.active_detour = None
            self._execute_recovery()
            return

        # Check final destination
        final_x, final_y, _ = self.path_plan[-1]
        dist_to_final = math.sqrt((final_x - self.current_x)**2 + (final_y - self.current_y)**2)
        if dist_to_final < self.goal_arrival_tolerance:
            self._stop_robot()
            self.state = "IDLE"
            self.active_detour = None
            self._publish_agv_state("ARRIVED")
            if self.goal_queue:
                # More goals remain — dequeue and navigate to next waypoint
                self.get_logger().info(
                    f"[BRAIN: MISSION_EXEC] Reached Waypoint {self.mission_current}/{self.mission_total} "
                    f"at ({self.current_x:.2f}, {self.current_y:.2f}). "
                    f"Proceeding to next goal ({len(self.goal_queue)} remaining)."
                )
                self._dispatch_next_goal()
            else:
                # Final goal reached — mission complete
                total = self.mission_total if self.mission_total > 0 else 1
                self.get_logger().info(
                    f"[BRAIN: GOAL_ARRIVAL] FINAL DESTINATION REACHED at ({self.current_x:.2f}, {self.current_y:.2f})! "
                    f"Target was ({final_x:.2f}, {final_y:.2f}) [Arrival Error: {dist_to_final:.3f}m]. "
                    f"Brain commanded full stop (v=0, w=0). Mission complete ({total} goal(s) visited)."
                )
                self._publish_mission_progress("MISSION_COMPLETE")
                self.mission_total = 0
                self.mission_current = 0
            return

        # Detour status management
        if self.active_detour is not None:
            if self.current_target_index >= self.active_detour['rejoin_idx'] or (now - self.active_detour['created_time'] > 12.0):
                self.get_logger().info("[BRAIN: DETOUR] Circumnavigation complete. Rejoined global roadmap.")
                self.active_detour = None

        # Proactive ahead-of-path corridor scan (scaled for room scale navigation)
        corridor_obs = self.get_corridor_obstacle_analysis(lookahead_horizon=1.2)
        if corridor_obs is not None and dist_to_final > 0.8:
            # Proactively re-route via connected graph nodes around the obstacle ahead of reaching its close zone
            if now > self._reroute_cooldown_time:
                ox, oy = corridor_obs['obstacle_pos']
                r_safe = max(0.35, corridor_obs.get('safe_radius', 0.42))
                if not corridor_obs.get('is_dynamic', False):
                    self.get_logger().info(
                        f"[BRAIN: CORRIDOR_SCAN] Detected stationary obstacle in path corridor at ({ox:.2f}, {oy:.2f}) "
                        f"[dist_along={corridor_obs['dist_along_path']:.2f}m, dist_robot={corridor_obs['min_dist_to_robot']:.2f}m]. "
                        f"Re-planning route via connected roadmap nodes..."
                    )
                    self.replan_around_obstacle(ox, oy, r_safe=r_safe)

        # Get lookahead target
        (target_x, target_y, target_theta), target_idx = self.get_lookahead_target(lookahead_dist=self.lookahead_dist)

        # Continuous Path Deviation Monitoring & Milestone Node Tracking
        if len(self.path_plan) > 0:
            s_min = max(0, self.current_target_index - 5)
            s_max = min(len(self.path_plan), self.current_target_index + 12)
            local_pts = np.array([[self.path_plan[k][0], self.path_plan[k][1]] for k in range(s_min, s_max)])
            if len(local_pts) > 0:
                dists_sq = (local_pts[:, 0] - self.current_x)**2 + (local_pts[:, 1] - self.current_y)**2
                cross_track_err = math.sqrt(float(np.min(dists_sq)))
                if cross_track_err > 0.45 and (now - self._last_dev_alert_time) > 2.5:
                    self._last_dev_alert_time = now
                    self.get_logger().warn(
                        f"[DEVIATION ALERT] Robot off-path: cross-track error = {cross_track_err:.2f}m (> 0.45m) at "
                        f"({self.current_x:.2f}, {self.current_y:.2f})."
                    )

        # Milestone Node Progress Tracking
        if self.node_path:
            for ni, nid in enumerate(self.node_path):
                if nid not in self._visited_plan_nodes:
                    ndist = math.hypot(self.map_data[nid]["x"] - self.current_x,
                                      self.map_data[nid]["y"] - self.current_y)
                    if ndist < 0.35:
                        self._visited_plan_nodes.add(nid)
                        yaw_deg = math.degrees(self.current_yaw)
                        self.get_logger().info(
                            f"[NODE TRACK] Reached Node {nid} ({ni + 1}/{len(self.node_path)} in plan) "
                            f"at ({self.current_x:.2f}, {self.current_y:.2f}), yaw={yaw_deg:.1f}°"
                        )
                        break

        # Check if an obstacle is directly in front on the upcoming path
        upcoming_slice = self.path_plan[self.current_target_index : min(len(self.path_plan), target_idx + 3)]
        obstacle_blocking_path = False
        _blocking_diag = None  # filled below for diagnostics
        if len(self.obstacles) > 0 and len(upcoming_slice) > 0:
            path_pts = np.array([(p[0], p[1]) for p in upcoming_slice])
            # Distance from each LiDAR point to nearest upcoming path point
            dists_to_path = np.min(np.linalg.norm(self.obstacles[:, np.newaxis, :] - path_pts[np.newaxis, :, :], axis=-1), axis=1)
            # Distance from each LiDAR point to robot
            dists_to_robot = np.linalg.norm(self.obstacles - np.array([self.current_x, self.current_y]), axis=-1)
            # Ignore distant mapped static walls (>0.45m from chassis) so room boundaries don't weaken path tracking
            if self.static_wall_mask is not None:
                is_unmapped = np.array([not self.is_near_static_wall(p[0], p[1]) for p in self.obstacles], dtype=bool)
            else:
                is_unmapped = np.ones(len(self.obstacles), dtype=bool)
            block_mask = (dists_to_path < 0.18) & ((is_unmapped & (dists_to_robot < 1.2)) | (dists_to_robot < 0.45))
            obstacle_blocking_path = bool(np.any(block_mask))
            if obstacle_blocking_path:
                # Collect diagnostic: which point is closest to the path
                worst_idx = int(np.argmin(np.where(block_mask, dists_to_path, np.inf)))
                _blocking_diag = (
                    float(self.obstacles[worst_idx, 0]),
                    float(self.obstacles[worst_idx, 1]),
                    float(dists_to_path[worst_idx]),
                    float(dists_to_robot[worst_idx])
                )

        # Dynamically soften cross-track error to allow swerving around obstacles
        # Fix 2 — Distance-gated evasion: only relax w_cross_track for obstacles within evasion_range
        is_evading = False
        evasion_causes = []
        close_dyn = [
            d for d in self.dynamic_obstacles
            if math.hypot(d['pos'][0] - self.current_x, d['pos'][1] - self.current_y) < self.evasion_range
        ]
        if len(close_dyn) > 0:
            is_evading = True
            evasion_causes.append(f"{len(close_dyn)} dynamic obstacle(s) within {self.evasion_range:.1f}m")
        if obstacle_blocking_path:
            is_evading = True
            evasion_causes.append("static obstacle near path")

        if is_evading:
            self.w_cross_track = self.w_cross_track_evasion
            if (now - self._last_mppi_evade_log_time) > 1.0:
                self._last_mppi_evade_log_time = now
                self.get_logger().info(
                    f"[BRAIN: MPPI_CONTROLLER] Tactical evasion active ({', '.join(evasion_causes)}). "
                    f"Brain tweaked controller: relaxed centerline weight w_cross_track {self.w_cross_track_nominal} -> {self.w_cross_track_evasion}. "
                    f"Robot pose=({self.current_x:.2f}, {self.current_y:.2f}, {math.degrees(self.current_yaw):.1f}°)."
                )
        else:
            self.w_cross_track = self.w_cross_track_nominal

        # Calculate vector heading from current robot pose to lookahead target
        target_heading = math.atan2(target_y - self.current_y, target_x - self.current_x)
        heading_error = math.atan2(math.sin(target_heading - self.current_yaw), math.cos(target_heading - self.current_yaw))

        # --- MPPI Controller with Predictive Dynamic Rollout ---
        # 1. Sample control sequences
        # Goal approach velocity scaling: smooth deceleration as robot nears exact target coordinate
        if dist_to_final < 0.35:
            approach_scale = max(0.35, dist_to_final / 0.35)
        else:
            approach_scale = 1.0

        # Predictive dynamic obstacle slowdown: only decelerate when predicted trajectory actually crosses near our path
        dyn_slowdown = 1.0
        for dob in self.dynamic_obstacles:
            d_obs = math.hypot(float(dob['pos'][0]) - self.current_x, float(dob['pos'][1]) - self.current_y)
            if d_obs < 1.5:
                pred_traj = dob.get('pred_traj')
                if pred_traj is not None and len(pred_traj) > 0:
                    check_n = min(12, len(pred_traj))
                    t_vec = np.arange(1, check_n + 1, dtype=np.float32) * self.dt
                    nom_v = max(self.min_linear_speed, self.cruise_linear_speed * 0.7)
                    ego_x = self.current_x + nom_v * math.cos(self.current_yaw) * t_vec
                    ego_y = self.current_y + nom_v * math.sin(self.current_yaw) * t_vec
                    min_cpa = float(np.min(np.hypot(pred_traj[:check_n, 0] - ego_x, pred_traj[:check_n, 1] - ego_y)))
                    if min_cpa < 0.65:
                        dyn_slowdown = min(dyn_slowdown, max(0.35, min_cpa / 0.65))
                else:
                    angle_to_obs = math.atan2(float(dob['pos'][1]) - self.current_y, float(dob['pos'][0]) - self.current_x)
                    angle_diff = abs(math.atan2(math.sin(angle_to_obs - self.current_yaw), math.cos(angle_to_obs - self.current_yaw)))
                    if angle_diff < math.radians(55):
                        dyn_slowdown = min(dyn_slowdown, max(0.35, d_obs / 1.5))

        # Heading-Velocity Coupling with Hysteresis Pivot Alignment:
        # Align heading cleanly before accelerating so robot tracks path centerline & hits exact goal coordinates
        abs_h_err = abs(heading_error)
        heading_alignment = max(0.0, math.cos(heading_error))

        pivot_enter = math.radians(25.0 if dist_to_final < 0.40 else 42.0)
        pivot_exit = math.radians(12.0 if dist_to_final < 0.40 else 18.0)
        if self._is_pivoting:
            if abs_h_err <= pivot_exit:
                self._is_pivoting = False
        elif abs_h_err >= pivot_enter:
            self._is_pivoting = True

        if self._is_pivoting:
            corner_scale = 0.0  # Pure in-place alignment pivot
        else:
            frac = min(1.0, abs_h_err / math.radians(42.0))
            corner_scale = 0.45 + 0.55 * 0.5 * (1.0 + math.cos(math.pi * frac))

        base_v = self.cruise_linear_speed * (heading_alignment ** 1.5) * corner_scale * approach_scale * dyn_slowdown
        v_mean = 0.0 if corner_scale == 0.0 else max(self.min_linear_speed, base_v)

        if self._is_pivoting:
            v_seq = np.zeros((self.num_samples, self.horizon))
        else:
            v_seq = np.random.normal(v_mean, self.noise_v, (self.num_samples, self.horizon))

        # Swerve side commitment: guides MPPI sampling prior using predicted CPA position
        if len(close_dyn) > 0 and is_evading:
            # If already deviating (> 45 deg) from target heading, cancel swerve bias to prioritize path realignment
            if abs_h_err > math.radians(45):
                self._swerve_bias = 0.0
                self._swerve_lock_ticks = 0
            elif self._swerve_lock_ticks <= 0:
                nearest = min(close_dyn, key=lambda d: math.hypot(
                    float(d['pos'][0]) - self.current_x, float(d['pos'][1]) - self.current_y))
                dx_t = target_x - self.current_x
                dy_t = target_y - self.current_y

                # Evaluate predicted Closest Point of Approach (CPA) along GRU rollout so we pass BEHIND crossing pedestrians
                obs_ref_x = float(nearest['pos'][0])
                obs_ref_y = float(nearest['pos'][1])
                pred_traj = nearest.get('pred_traj')
                if pred_traj is not None and len(pred_traj) > 0:
                    check_n = min(10, len(pred_traj))
                    t_vec = (np.arange(1, check_n + 1, dtype=np.float32) * self.dt)
                    ego_x = self.current_x + v_mean * math.cos(self.current_yaw) * t_vec
                    ego_y = self.current_y + v_mean * math.sin(self.current_yaw) * t_vec
                    cpa_dists = np.hypot(pred_traj[:check_n, 0] - ego_x, pred_traj[:check_n, 1] - ego_y)
                    cpa_idx = int(np.argmin(cpa_dists))
                    obs_ref_x = float(pred_traj[cpa_idx, 0])
                    obs_ref_y = float(pred_traj[cpa_idx, 1])

                dx_o = obs_ref_x - self.current_x
                dy_o = obs_ref_y - self.current_y
                cross = dx_t * dy_o - dy_t * dx_o  # >0 = predicted obstacle on left -> swerve right
                obs_dist = math.hypot(float(nearest['pos'][0]) - self.current_x, float(nearest['pos'][1]) - self.current_y)
                proposed_bias_sign = -1.0 if cross > 0 else 1.0

                # Respect physical wall/furniture clearance so robot never swerves into a wall or parked trolley
                left_clear, right_clear = self._compute_lateral_corridor_clearances(target_heading)
                min_side_clear = self.robot_radius + 0.14
                if proposed_bias_sign < 0 and right_clear < min_side_clear and left_clear > right_clear + 0.12:
                    proposed_bias_sign = 1.0
                elif proposed_bias_sign > 0 and left_clear < min_side_clear and right_clear > left_clear + 0.12:
                    proposed_bias_sign = -1.0
                side_str = "RIGHT" if proposed_bias_sign < 0 else "LEFT"

                raw_bias = self.swerve_bias_strength * (self.evasion_range / max(0.25, obs_dist))
                max_bias = 0.65 if self.use_sim_time else 0.35
                bias_mag = min(max_bias, max(0.18, raw_bias))
                self._swerve_bias = proposed_bias_sign * bias_mag
                self._swerve_lock_ticks = self.swerve_lock_duration
                self.get_logger().info(
                    f"[BRAIN: SWERVE] Committed predictive evasion swerve {side_str} for {self.swerve_lock_duration} ticks | "
                    f"bias={self._swerve_bias:+.2f} rad/s (dist={obs_dist:.2f}m, L={left_clear:.2f}m, R={right_clear:.2f}m)"
                )
            else:
                self._swerve_lock_ticks -= 1
        else:
            if self._swerve_lock_ticks > 0:
                self._swerve_lock_ticks -= 1
            else:
                self._swerve_bias = 0.0

        # MPPI Angular Sampling Prior: blend proportional heading guidance with evasion bias
        kp_heading = self.kp_heading
        w_guidance = kp_heading * heading_error

        # In-place pivot torque boost: smooth, gentle angular velocity that preserves LiDAR scan fidelity and AMCL stability
        if corner_scale == 0.0 and abs(heading_error) > math.radians(5):
            min_pivot_w = 0.35  # Rad/s (~20.0 deg/s) prevents 10Hz LiDAR motion blur & AMCL cloud divergence
            if abs(w_guidance) < min_pivot_w:
                w_guidance = math.copysign(min_pivot_w, heading_error)

        w_mean = float(np.clip(w_guidance + self._swerve_bias, -self.w_max, self.w_max))
        eff_noise_w = self.noise_w * (1.35 if is_evading else 1.0)
        w_seq = np.random.normal(w_mean, eff_noise_w, (self.num_samples, self.horizon))
        v_seq = np.clip(v_seq, self.v_min, self.v_max)
        w_seq = np.clip(w_seq, -self.w_max, self.w_max)

        # 2. Rollout AGV Trajectories
        x_rollout = np.full((self.num_samples, self.horizon), self.current_x)
        y_rollout = np.full((self.num_samples, self.horizon), self.current_y)
        yaw_rollout = np.full((self.num_samples, self.horizon), self.current_yaw)

        for t in range(1, self.horizon):
            yaw_rollout[:, t] = yaw_rollout[:, t-1] + w_seq[:, t-1] * self.dt
            x_rollout[:, t] = x_rollout[:, t-1] + v_seq[:, t-1] * np.cos(yaw_rollout[:, t-1]) * self.dt
            y_rollout[:, t] = y_rollout[:, t-1] + v_seq[:, t-1] * np.sin(yaw_rollout[:, t-1]) * self.dt

        # 3. Evaluate Costs
        costs = np.zeros(self.num_samples)

        # Terminal distance cost
        terminal_dists = np.sqrt((x_rollout[:, -1] - target_x)**2 + (y_rollout[:, -1] - target_y)**2)
        costs += self.w_dist * terminal_dists

        # Horizon-wide heading cost (evaluate alignment to target_heading)
        for t in range(self.horizon):
            h_err = np.arctan2(
                np.sin(target_heading - yaw_rollout[:, t]),
                np.cos(target_heading - yaw_rollout[:, t])
            )
            costs += (self.w_heading / self.horizon) * np.abs(h_err)

        # Cross-track error cost
        local_path_full = self.path_plan[self.current_target_index : target_idx + 2]
        local_path = np.array([(p[0], p[1]) for p in local_path_full])
        if len(local_path) > 0:
            rollout_pts = np.stack((x_rollout, y_rollout), axis=-1)[:, :, np.newaxis, :]
            path_pts = local_path[np.newaxis, np.newaxis, :, :]
            dists = np.linalg.norm(rollout_pts - path_pts, axis=-1)
            min_dists = np.min(dists, axis=-1)
            costs += self.w_cross_track * np.sum(min_dists, axis=-1)

        # Collision & Uncertainty-Aware Repulsion from Dynamic Moving Obstacles (Kalman-NN 1-2s Rollout)
        if len(self.dynamic_obstacles) > 0:
            rollout_xy = np.stack([x_rollout, y_rollout], axis=-1)  # (K, T, 2)
            t_vec = (np.arange(1, self.horizon + 1, dtype=np.float32) * self.dt)[:, None]
            for dyn_obs in self.dynamic_obstacles:
                obs_pos = np.asarray(dyn_obs['pos'], dtype=np.float32)
                obs_vel = np.asarray(dyn_obs['vel'], dtype=np.float32)
                r_obs = float(dyn_obs.get('radius', 0.28))
                pred_traj = dyn_obs.get('pred_traj')
                pred_std = dyn_obs.get('pred_std')

                if pred_traj is not None and len(pred_traj) >= self.horizon:
                    traj_t = pred_traj[:self.horizon]
                    std_t = pred_std[:self.horizon] if (pred_std is not None and len(pred_std) >= self.horizon) else np.full((self.horizon, 2), 0.10, dtype=np.float32)
                else:
                    traj_t = obs_pos[None, :] + obs_vel[None, :] * t_vec
                    std_t = np.full((self.horizon, 2), 0.08, dtype=np.float32) + 0.12 * t_vec

                diff = rollout_xy - traj_t[None, :, :]  # (K, T, 2)
                dists = np.linalg.norm(diff, axis=-1)   # (K, T)

                # Effective collision clearance accounts for robot radius + physical obstacle radius + bounded uncertainty pad
                sigma_mag = np.hypot(std_t[:, 0], std_t[:, 1])
                sigma_pad = np.minimum(0.16, 0.40 * sigma_mag)
                eff_coll_r = (self.robot_radius + 0.04 + r_obs + sigma_pad)[None, :]
                col_any = np.any(dists < eff_coll_r, axis=1)
                costs[col_any] += self.w_collision

                # Smooth uncertainty-expanded social repulsion + anisotropic Gaussian risk
                eff_bubble = np.maximum(
                    self.dynamic_repulsive_dist,
                    eff_coll_r + self.person_social_margin + 0.20 + np.mean(std_t, axis=-1)[None, :],
                )
                rep_mask = dists < eff_bubble
                if np.any(rep_mask):
                    pen = np.where(rep_mask, eff_bubble - dists, 0.0)
                    costs += self.dynamic_w_repulsive * np.sum(pen + 0.75 * pen * pen, axis=1)

                # Mild probabilistic Mahalanobis risk over predicted covariance envelope
                sx2 = (std_t[:, 0] + r_obs + self.robot_radius) ** 2 + 1e-4
                sy2 = (std_t[:, 1] + r_obs + self.robot_radius) ** 2 + 1e-4
                maha2 = (diff[:, :, 0] ** 2) / sx2[None, :] + (diff[:, :, 1] ** 2) / sy2[None, :]
                costs += 18.0 * np.sum(np.exp(-0.5 * np.minimum(maha2, 20.0)), axis=1)

        # Collision & Repulsion from Static Obstacles (Walls, Shelves)
        if len(self.obstacles) > 0:
            # Spatial pre-filter: keep only obstacles within 3.5m radius of the robot
            obs_dists_sq = (self.obstacles[:, 0] - self.current_x)**2 + (self.obstacles[:, 1] - self.current_y)**2
            near_mask = obs_dists_sq < 12.25  # 3.5^2
            near_obs = self.obstacles[near_mask]

            # Exclude points belonging to active moving PERSON obstacles at t >= 2 so they aren't frozen as walls
            static_only_obs = near_obs
            if len(near_obs) > 0 and len(self.dynamic_obstacles) > 0:
                keep_static = np.ones(len(near_obs), dtype=bool)
                for dob in self.dynamic_obstacles:
                    dp = np.asarray(dob['pos'], dtype=np.float32)
                    dr = float(dob.get('radius', 0.30)) + 0.14
                    keep_static &= (np.linalg.norm(near_obs - dp[None, :], axis=-1) > dr)
                static_only_obs = near_obs[keep_static]

            if len(near_obs) > 0:
                obs_imm = near_obs[np.newaxis, :, :]
                obs_fut = static_only_obs[np.newaxis, :, :] if len(static_only_obs) > 0 else None
                for t in range(self.horizon):
                    obs_t = obs_imm if t < 2 else obs_fut
                    if obs_t is None:
                        continue
                    pts = np.stack([x_rollout[:, t], y_rollout[:, t]], axis=-1)[:, np.newaxis, :]
                    min_dists = np.min(np.linalg.norm(pts - obs_t, axis=-1), axis=1)

                    # Hard collision
                    col_mask = min_dists < self.collision_radius
                    costs[col_mask] += self.w_collision

                    # Graduated repulsion field
                    rep_mask = min_dists < self.static_repulsive_dist
                    costs[rep_mask] += self.static_w_repulsive * ((self.static_repulsive_dist - min_dists[rep_mask]) ** 2)

        # --- Traffic & Yielding Decision ---
        best_cost = np.min(costs)
        all_colliding = best_cost >= self.w_collision

        # Determine dynamic vs stationary yield timeout
        is_dynamic_block = False
        if corridor_obs is not None:
            is_dynamic_block = corridor_obs['is_dynamic']
        elif len(close_dyn) > 0:
            is_dynamic_block = True

        current_yield_timeout = self.yield_timeout_dynamic if is_dynamic_block else self.yield_timeout_static

        if all_colliding and obstacle_blocking_path:
            # If an active detour is underway, allow MPPI to continue tracking along the detour arc
            if self.active_detour is not None and self.current_target_index < self.active_detour['rejoin_idx']:
                pass
            elif self.state == "NAVIGATING":
                self.state = "YIELDING"
                self.yield_start_time = now
                ox, oy, dp, dr = _blocking_diag if _blocking_diag else (0.0, 0.0, 0.0, 0.0)
                obs_type_str = "Dynamic obstacle" if is_dynamic_block else "Stationary unmapped obstacle"
                self.get_logger().warn(
                    f"[BRAIN: STATE_MACHINE] Commanded YIELD (v=0.0). "
                    f"Cause: {obs_type_str} at map({ox:.3f}, {oy:.3f}), "
                    f"dist_to_path={dp:.3f}m, dist_to_robot={dr:.3f}m, MPPI_best_cost={best_cost:.1f}. "
                    f"Holding position up to {current_yield_timeout:.1f}s before re-routing..."
                )
                self._last_block_log_time = now
                self._stop_robot()
                return
            elif self.state == "YIELDING":
                elapsed = now - self.yield_start_time
                if elapsed >= current_yield_timeout:
                    obs_type_str = "dynamic" if is_dynamic_block else "stationary unmapped"
                    self.get_logger().warn(
                        f"[BRAIN: STATE_MACHINE] Yield timeout ({elapsed:.1f}s >= {current_yield_timeout:.1f}s) exceeded for {obs_type_str} blockage! "
                        f"Delegating to Dijkstra Brain for dynamic re-route."
                    )
                    self.trigger_reroute()
                    return
                else:
                    # Rate-limited diagnostic while waiting
                    if (now - self._last_block_log_time) > 1.0:
                        ox, oy, dp, dr = _blocking_diag if _blocking_diag else (0.0, 0.0, 0.0, 0.0)
                        self.get_logger().warn(
                            f"[BRAIN: STATE_MACHINE] Holding YIELD ({elapsed:.1f}s/{current_yield_timeout:.1f}s): "
                            f"Obstacle at map({ox:.3f}, {oy:.3f}), dist_path={dp:.3f}m, dist_robot={dr:.3f}m."
                        )
                        self._last_block_log_time = now
                    self._stop_robot()
                    return

        # Path is clear — exit YIELDING only when BOTH MPPI and path-blocker agree
        # (Previously only checked all_colliding, causing oscillation when one MPPI
        # sample sneaked through while a wall was still <0.25 m from the path.)
        if self.state == "YIELDING" and (not all_colliding) and (not obstacle_blocking_path):
            self.get_logger().info(
                f"[BRAIN: STATE_MACHINE] Corridor cleared! Resuming NAVIGATING mode. "
                f"Robot at ({self.current_x:.2f}, {self.current_y:.2f})."
            )
            self.state = "NAVIGATING"
            self.yield_start_time = None

        # 4. Softmax Weighting and Command Execution
        beta = np.min(costs)
        weights = np.exp(-1.0 / self.lambda_weight * (costs - beta))
        weights = weights / np.sum(weights)

        optimal_v = float(np.sum(weights * v_seq[:, 0]))
        optimal_w = float(np.sum(weights * w_seq[:, 0]))

        # Periodic High-Level Telemetry Logger (every 1.5s)
        if (now - self._last_pose_log_time) > 1.5:
            self._last_pose_log_time = now
            min_obs_d = float(np.min(np.linalg.norm(self.obstacles - np.array([self.current_x, self.current_y]), axis=-1))) if len(self.obstacles) > 0 else 99.0
            num_dyn = len(self.dynamic_obstacles)
            self.get_logger().info(
                f"[TELEMETRY] Pose=({self.current_x:.2f}, {self.current_y:.2f}, {math.degrees(self.current_yaw):.1f}°) | "
                f"Node={snapped_node} | GoalDist={dist_to_final:.2f}m | "
                f"Cmd=(v={optimal_v:.2f}m/s, w={optimal_w:.2f}rad/s) | MinObs={min_obs_d:.2f}m | DynObs={num_dyn} | State={self.state}"
            )

        # Turn Area & Sharp Steering Telemetry Logger
        abs_h_err_deg = math.degrees(abs(heading_error))
        if (abs_h_err_deg > 15.0 or abs(optimal_w) > 0.3) and (now - self._last_turn_log_time) > 0.8:
            self._last_turn_log_time = now
            min_obs_d = float(np.min(np.linalg.norm(self.obstacles - np.array([self.current_x, self.current_y]), axis=-1))) if len(self.obstacles) > 0 else 99.0
            self.get_logger().info(
                f"[TURN TELEMETRY] Node={snapped_node} | Pose=({self.current_x:.2f}, {self.current_y:.2f}, {math.degrees(self.current_yaw):.1f}°) | "
                f"Wpt={target_idx}/{len(self.path_plan)} ({target_x:.2f}, {target_y:.2f}) | "
                f"HeadingErr={abs_h_err_deg:.1f}° | Cmd=(v={optimal_v:.2f}, w={optimal_w:.2f}) | MinObs={min_obs_d:.2f}m"
            )

        # Stuck detector: count ticks where robot is not translating (halted or spinning in place without progress)
        # Excluded during intentional YIELDING (where robot is waiting for traffic to pass up to 4.5s)
        if self._recovery_phase is None and self.state != "YIELDING":
            if self._last_stuck_check_pos is not None:
                moved = math.hypot(
                    self.current_x - self._last_stuck_check_pos[0],
                    self.current_y - self._last_stuck_check_pos[1]
                )
                is_spinning = abs(optimal_w) > 0.20
                halted_thresh = max(0.025, self.min_linear_speed * 0.65)
                is_halted = optimal_v < halted_thresh

                # Kinematic-Aware Turn Progress Guard:
                # Differentiate intentional in-place turn toward target from physical stall
                is_turning_in_place = is_spinning and is_halted
                heading_err_abs = abs(heading_error)
                heading_progress = False
                if self._prev_heading_err is not None:
                    # Heading error is actively reducing (~0.45 deg per tick)
                    heading_progress = (heading_err_abs < self._prev_heading_err - 0.008)
                self._prev_heading_err = heading_err_abs

                if is_turning_in_place and heading_progress:
                    # Healthy in-place pivot toward waypoint: reset stuck counter!
                    self._stuck_counter = 0
                elif dist_to_final < max(0.16, self.goal_arrival_tolerance * 2.0) and optimal_v >= halted_thresh:
                    self._stuck_counter = 0
                    self._last_stuck_check_pos = (self.current_x, self.current_y)
                elif moved < 0.05 and (is_halted or is_spinning):
                    self._stuck_counter += 1
                else:
                    self._stuck_counter = 0
                    self._last_stuck_check_pos = (self.current_x, self.current_y)
            else:
                self._last_stuck_check_pos = (self.current_x, self.current_y)
                self._prev_heading_err = abs(heading_error)

            if self._stuck_counter >= self._stuck_threshold_ticks:
                self.get_logger().warn(
                    f"[BRAIN: STUCK_DETECTOR] Stuck for {self._stuck_counter} ticks ({self._stuck_counter*self.dt:.1f}s) at "
                    f"({self.current_x:.2f}, {self.current_y:.2f}). Triggering recovery."
                )
                self._stuck_counter = 0
                self._last_stuck_check_pos = None
                self._prev_heading_err = None
                self._swerve_bias = 0.0
                self._swerve_lock_ticks = 0
                self._execute_recovery()
                return

        # 1. EMA Low-Pass Filter on raw MPPI Monte Carlo outputs to eliminate tick-to-tick flutter
        target_v = self.ema_alpha_v * optimal_v + (1.0 - self.ema_alpha_v) * self.last_target_v
        target_w = self.ema_alpha_w * optimal_w + (1.0 - self.ema_alpha_w) * self.last_target_w
        self.last_target_v = target_v
        self.last_target_w = target_w

        # 2. Acceleration / Deceleration Slew-Rate Limiting for smooth physical motion
        dv = target_v - self.last_cmd_v
        if dv > 0.0:
            max_dv = self.accel_lim_v * self.dt
            cmd_v = self.last_cmd_v + min(dv, max_dv)
        else:
            max_dv = self.decel_lim_v * self.dt
            cmd_v = self.last_cmd_v + max(dv, -max_dv)

        dw = target_w - self.last_cmd_w
        max_dw = self.accel_lim_w * self.dt
        cmd_w = self.last_cmd_w + float(np.clip(dw, -max_dw, max_dw))

        # Clamp continuous slew-limited commands within physical limits
        slew_v = float(np.clip(cmd_v, self.v_min, self.v_max))
        slew_w = float(np.clip(cmd_w, -self.w_max, self.w_max))

        # Store un-clamped slew state so minimum-pivot deadband never ratchets sign reversals
        self.last_cmd_v = slew_v
        self.last_cmd_w = slew_w

        # Motor Deadband & Breakaway Protection applied only to the published command:
        pub_v = slew_v
        pub_w = slew_w
        if self._is_pivoting or (abs(pub_w) > 0.06 and pub_v < 0.03):
            pub_v = 0.0
            if abs(target_w) > 0.04 and (target_w * pub_w) < 0.0:
                # Decelerating through zero to reverse turn direction: pause briefly instead of pushing wrong way
                pub_w = 0.0
            elif abs(pub_w) > 0.04 and abs(pub_w) < 0.22:
                pub_w = math.copysign(0.22, pub_w)
        elif pub_v > 0.015 and pub_v < self.min_linear_speed:
            # Forward motion: snap to minimum reliable moving speed
            pub_v = self.min_linear_speed

        self.last_pub_w = pub_w

        twist = Twist()
        twist.linear.x = pub_v
        twist.angular.z = pub_w
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = RouteRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()