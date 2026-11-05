import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String, Bool
from sensor_msgs.msg import LaserScan
import tf2_ros
from tf2_ros import Buffer, TransformListener, TransformException
import math
import json
import os
import heapq
import yaml

import numpy as np
from scipy.spatial import KDTree
from scipy.ndimage import binary_dilation
from PIL import Image


class RouteRunner(Node):
    def __init__(self):
        super().__init__('route_runner')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/agv_dense_path', 10)  # Publish dense path for RViz
        # Mission progress publisher: JSON string of {current, total, goal_node, state}
        self.mission_pub = self.create_publisher(String, '/mission_progress', 10)
        # App bridge: publish nav state so the Flutter / web dashboard can display it
        self.agv_state_pub = self.create_publisher(String, '/agv_state', 10)

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

        # TF Buffer and Listener for AMCL / SLAM localization (map -> base_link)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.using_map_tf = False

        # Load the graph — check active map, warehouse_graph, or map candidates
        self.declare_parameter('map_name', '')
        map_name_param = self.get_parameter('map_name').get_parameter_value().string_value.strip()

        # Static map occupancy grid for filtering ghost dynamic obstacles
        self.static_map_data = None
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

        candidates = []
        try:
            pkg_share = get_package_share_directory('agv_description')
            if map_name_param:
                candidates.append(os.path.join(pkg_share, 'maps', f'{map_name_param}_graph.json'))
            candidates.append(os.path.join(pkg_share, 'maps', 'warehouse_graph.json'))
            candidates.append(os.path.join(pkg_share, 'maps', 'warehouse_map_graph.json'))
            candidates.append(os.path.join(pkg_share, 'maps', 'warehouse_01_graph.json'))
        except Exception:
            pass

        ws_maps = os.path.join(os.environ.get('HOME', ''), 'AMR', 'AMR-main', 'src', 'agv_description', 'maps')
        if map_name_param:
            candidates.append(os.path.join(ws_maps, f'{map_name_param}_graph.json'))
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

        # Obstacles
        self.obstacles = np.array([])  # Static and dynamic points in map frame
        self.prev_obstacle_clusters = []
        self.dynamic_obstacles = []    # List of dicts: {'pos': (x,y), 'vel': (vx,vy), 'speed': s, 'radius': r, 'age': int}
        self.last_scan_time = None
        # Fix 1 — LiDAR age filter: track how many consecutive scans each cluster has been seen
        self._obs_age_map = {}         # Maps prev_cluster index -> age (consecutive scan count)
        self._obs_start_pos_map = {}   # Maps prev_cluster index -> initial (x, y) centroid position
        self._obs_vel_map = {}         # Maps prev_cluster index -> smoothed velocity vector

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
        self.critical_proximity = 0.20 # Distance threshold (m) triggering active reverse escape (chassis radius is 0.11m)
        self.safe_standoff = 0.50      # Target front standoff distance (m) to achieve during reverse escape

        # Smooth Early Detour (Circumnavigation) state
        self.active_detour = None      # Stores {rejoin_idx, created_time, side} when circumnavigating
        self._detour_cooldown_time = 0.0
        self._reroute_cooldown_time = 0.0
        self._last_detour_log_time = 0.0

        # Stuck detector & Active Reverse Recovery
        self._stuck_counter = 0
        self._last_stuck_check_pos = None
        self._prev_heading_err = None  # Tracks heading error delta for kinematic stuck detection
        self._stuck_threshold_ticks = 20  # ~2.0s of zero translation / spin triggers recovery
        self._recovery_phase = None        # None, 'BACKUP', or 'REROUTE'
        self._recovery_start_time = None
        self._recovery_backup_dur = 2.2   # Max seconds to reverse if standoff not yet achieved
        self._last_recovery_log_time = 0.0
        self.hazard_zones = []         # Persistent anti-loop hazard memory: [{'pos': (x,y), 'radius': r, 'expire_time': timestamp}]

        # MPPI Parameters
        self.v_max = 0.75
        self.v_min = 0.0               # Forward-only in normal MPPI rollouts; reverse handled deterministically
        self.w_max = 1.8               # High turning authority for obstacle evasion
        self.dt = 0.1                  # 10 Hz
        self.horizon = 15              # 1.5s preview
        self.num_samples = 80

        # MPPI Noise covariance
        self.noise_v = 0.3
        self.noise_w = 0.5
        self.lambda_weight = 0.5

        # MPPI Cost Weights
        self.w_dist = 4.0
        self.w_heading = 3.0
        self.w_collision = 5000.0
        
        # Repulsion fields (tuned for 1.2m - 1.5m warehouse aisles and 22cm robot)
        self.dynamic_repulsive_dist = 0.65  # Proactive evasion bubble for moving obstacles (m)
        self.dynamic_w_repulsive = 45.0     # Smooth proactive repulsion from dynamic obstacles
        self.static_repulsive_dist = 0.26   # Wall repulsion activation distance (m) (tuned for narrow corridors)
        self.static_w_repulsive = 50.0      # Quadratic wall repulsion (smooth gradient in narrow passages)

        self.w_cross_track_nominal = 6.0
        self.w_cross_track_evasion = 2.0   # Soften centerline tracking during evasion while preventing wandering
        self.w_cross_track = self.w_cross_track_nominal

        # Distance-gated evasion: only relax w_cross_track for obstacles within this range
        self.evasion_range = 1.2       # metres

        # Swerve side commitment: commit to one side briefly to avoid L/R oscillation
        self._swerve_bias = 0.0           # Current angular bias added to MPPI w_seq mean (rad/s)
        self._swerve_lock_ticks = 0       # Ticks remaining on current committed swerve direction
        self.swerve_lock_duration = 8     # Ticks (~0.8s at 10 Hz) to hold swerve commitment
        self.swerve_bias_strength = 0.35  # Angular bias magnitude (rad/s)

        # Motion Smoothness & Command Slew-Rate Limiter Parameters
        self.accel_lim_v = 0.60         # Max forward acceleration: 0.60 m/s^2 (0.060 m/s per 100ms tick)
        self.decel_lim_v = 0.90         # Max forward deceleration: 0.90 m/s^2 (0.090 m/s per 100ms tick)
        self.accel_lim_w = 1.60         # Max angular acceleration: 1.60 rad/s^2 (0.160 rad/s per 100ms tick)

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

        # Control timer
        self.control_timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info(f"Route Runner Active with KD-Tree ({len(self.node_ids)} nodes), Proactive Detour & Active Reverse Escape.")

    # ------------------------------------------------------------------
    # Static Map Mask & App Bridge helpers
    # ------------------------------------------------------------------

    def _load_static_map_file(self, map_name_param):
        """Loads static map YAML and PGM file to construct a binary wall mask."""
        candidates = []
        try:
            pkg_share = get_package_share_directory('agv_description')
            if map_name_param:
                candidates.append(os.path.join(pkg_share, 'maps', f'{map_name_param}.yaml'))
            candidates.append(os.path.join(pkg_share, 'maps', 'hospital_map.yaml'))
            candidates.append(os.path.join(pkg_share, 'maps', 'warehouse_map.yaml'))
        except Exception:
            pass

        ws_maps = os.path.join(os.environ.get('HOME', ''), 'AMR', 'AMR-main', 'src', 'agv_description', 'maps')
        if map_name_param:
            candidates.append(os.path.join(ws_maps, f'{map_name_param}.yaml'))
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
            # Dilate occupied walls by ~0.22m (approx 4-5 cells at 0.05m resolution)
            iterations = max(2, int(0.22 / meta.get('resolution', 0.05)))
            self.static_wall_mask = binary_dilation(occ, iterations=iterations)
            self.static_map_resolution = float(meta.get('resolution', 0.05))
            orig = meta.get('origin', [0.0, 0.0, 0.0])
            self.static_map_origin_x = float(orig[0])
            self.static_map_origin_y = float(orig[1])
            self.static_map_height, self.static_map_width = arr.shape
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
            res = msg.info.resolution
            iterations = max(2, int(0.22 / res))
            self.static_wall_mask = binary_dilation(occ, iterations=iterations)
            self.static_map_resolution = float(res)
            self.static_map_origin_x = float(msg.info.origin.position.x)
            self.static_map_origin_y = float(msg.info.origin.position.y)
            self.static_map_width = w
            self.static_map_height = h
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
        self.last_target_v = 0.0
        self.last_target_w = 0.0
        self.cmd_pub.publish(Twist())

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
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            self.current_x = t.transform.translation.x
            self.current_y = t.transform.translation.y
            self.current_yaw = self.get_yaw_from_quaternion(t.transform.rotation)
            if not self.using_map_tf:
                self.get_logger().info("Localization Active! Received map -> base_link TF transform.")
                self.using_map_tf = True
            self.odom_ready = True
            return True
        except TransformException:
            if self.using_map_tf:
                self.get_logger().warn("AMCL TF lost! Falling back to /odometry/filtered.")
                self.using_map_tf = False
            return self.odom_ready

    def odom_callback(self, msg):
        if not self.using_map_tf:
            self.current_x = msg.pose.pose.position.x
            self.current_y = msg.pose.pose.position.y
            self.current_yaw = self.get_yaw_from_quaternion(msg.pose.pose.orientation)
            self.odom_ready = True


    def scan_callback(self, msg):
        self.update_robot_pose()
        now = self._now_sec()
        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        # Basic range validity filter
        valid = (ranges > max(0.08, msg.range_min)) & (ranges < msg.range_max) & np.isfinite(ranges)
        ranges = ranges[valid]
        angles = angles[valid]

        if len(ranges) == 0:
            self.obstacles = np.array([])
            self.dynamic_obstacles = []
            return

        # Downsample LiDAR to every 3rd point
        ranges = ranges[::3]
        angles = angles[::3]

        # Convert to robot local coordinate frame
        ox_local = ranges * np.cos(angles)
        oy_local = ranges * np.sin(angles)
        r_local = np.sqrt(ox_local**2 + oy_local**2)

        # Precise robot footprint self-masking:
        # - Chassis is a cylinder of radius 0.11m; caster is inside; wheels are at y = +/-0.13m, width 0.026m
        # Mask points strictly inside physical chassis cylinder: r < 0.112m
        is_chassis_self = r_local < 0.112
        # Mask points strictly on the side wheel protrusions: |y| in [0.112, 0.148] and |x| <= 0.045
        is_wheel_self = (np.abs(oy_local) >= 0.112) & (np.abs(oy_local) <= 0.148) & (np.abs(ox_local) <= 0.045)

        external_mask = (~is_chassis_self) & (~is_wheel_self)
        ranges = ranges[external_mask]
        angles = angles[external_mask]
        ox_local = ox_local[external_mask]
        oy_local = oy_local[external_mask]

        if len(ranges) == 0:
            self.obstacles = np.array([])
            self.dynamic_obstacles = []
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

        # Convert to global map frame
        cos_yaw = np.cos(pose_yaw)
        sin_yaw = np.sin(pose_yaw)

        ox_global = pose_x + ox_local * cos_yaw - oy_local * sin_yaw
        oy_global = pose_y + ox_local * sin_yaw + oy_local * cos_yaw

        self.obstacles = np.column_stack((ox_global, oy_global))

        # --- Dynamic Obstacle Velocity Estimation ---
        # 1. Spatial pre-filter: Keep obstacle points within 4.5m of the robot
        dists_to_robot = np.sqrt(ox_local**2 + oy_local**2)
        near_mask = dists_to_robot < 4.5
        near_pts = self.obstacles[near_mask]

        # 2. Static Map Background Rejection:
        # Filter out LiDAR points that fall onto known static walls, partitions, and perimeter geometry
        if len(near_pts) > 0 and self.static_wall_mask is not None:
            non_wall_mask = np.array([not self.is_near_static_wall(p[0], p[1]) for p in near_pts], dtype=bool)
            dyn_candidate_pts = near_pts[non_wall_mask]
        else:
            dyn_candidate_pts = near_pts

        current_clusters = []
        if len(dyn_candidate_pts) > 0:
            # Cluster candidate points (0.35m grid bins)
            bins = {}
            for p in dyn_candidate_pts:
                key = (round(p[0] / 0.35), round(p[1] / 0.35))
                bins.setdefault(key, []).append(p)

            for key, pts in bins.items():
                if len(pts) >= 3:  # Require at least 3 points to reject noise specks
                    centroid = np.mean(pts, axis=0)
                    current_clusters.append(centroid)

        # 3. Persistent Object Tracking with Association:
        new_dynamic_obs = []
        if self.last_scan_time is not None:
            dt = max(0.05, min(0.30, now - self.last_scan_time))
            matched_track_ids = set()

            for curr_c in current_clusters:
                best_track_id = None
                min_match_dist = 0.35  # Realistic max displacement in 0.1s (< 3.5 m/s)

                for tid, track in self._tracked_obstacles.items():
                    if tid in matched_track_ids:
                        continue
                    d = float(np.linalg.norm(curr_c - track['pos']))
                    if d < min_match_dist:
                        min_match_dist = d
                        best_track_id = tid

                if best_track_id is not None:
                    track = self._tracked_obstacles[best_track_id]
                    raw_vel = (curr_c - track['pos']) / dt
                    # Smooth velocity: 0.5 current, 0.5 historical
                    smooth_vel = 0.5 * raw_vel + 0.5 * track['vel']
                    speed = float(np.linalg.norm(smooth_vel))

                    track['pos'] = curr_c
                    track['vel'] = smooth_vel
                    track['speed'] = speed
                    track['age'] += 1
                    track['misses'] = 0
                    matched_track_ids.add(best_track_id)

                    disp = float(np.linalg.norm(curr_c - track['start_pos']))

                    # True dynamic obstacle confirmation:
                    # - Tracked for at least 3 consecutive scans (>= 0.3s)
                    # - Speed between 0.22 m/s and 2.0 m/s
                    # - Net displacement >= 0.20m from origin spot
                    # - Not located on a static wall
                    if (0.22 <= speed <= 2.0 and track['age'] >= 3 and disp >= 0.20 and
                            not self.is_near_static_wall(curr_c[0], curr_c[1])):
                        new_dynamic_obs.append({
                            'id': best_track_id,
                            'pos': curr_c,
                            'vel': smooth_vel,
                            'speed': speed,
                            'radius': 0.35,
                            'age': track['age']
                        })
                else:
                    # New candidate track (suppress instantiation if robot is rotating fast)
                    if abs(self.last_cmd_w) < 0.60:
                        new_tid = self._next_track_id
                        self._next_track_id += 1
                        self._tracked_obstacles[new_tid] = {
                            'pos': curr_c,
                            'vel': np.zeros(2),
                            'speed': 0.0,
                            'start_pos': curr_c,
                            'age': 1,
                            'misses': 0
                        }

            # Age out unmatched tracks
            to_delete = []
            for tid, track in self._tracked_obstacles.items():
                if tid not in matched_track_ids:
                    track['misses'] += 1
                    if track['misses'] > 2:
                        to_delete.append(tid)
            for tid in to_delete:
                del self._tracked_obstacles[tid]

        self.dynamic_obstacles = new_dynamic_obs
        self.last_scan_time = now

        # Real-time LiDAR dynamic obstacle detection logging (rate-limited to 1 Hz)
        if len(self.dynamic_obstacles) > 0 and (now - self._last_dyn_log_time) > 1.0:
            self._last_dyn_log_time = now
            for i, dob in enumerate(self.dynamic_obstacles):
                d_robot = math.hypot(dob['pos'][0] - self.current_x, dob['pos'][1] - self.current_y)
                self.get_logger().info(
                    f"[LIDAR_SCAN: DYNAMIC_OBSTACLE] Tracked Obs #{dob.get('id', i+1)} at map({dob['pos'][0]:.2f}, {dob['pos'][1]:.2f}) | "
                    f"Vel=({dob['vel'][0]:.2f}, {dob['vel'][1]:.2f}) Speed={dob['speed']:.2f}m/s | "
                    f"DistToRobot={d_robot:.2f}m | RobotPose=({self.current_x:.2f}, {self.current_y:.2f})"
                )

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
        """
        try:
            node_ids = json.loads(msg.data)
            if not isinstance(node_ids, list) or len(node_ids) == 0:
                self.get_logger().error("goal_sequence: expected a non-empty JSON array of node IDs.")
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

    def goal_callback(self, msg):
        self.update_robot_pose()
        if not self.odom_ready:
            self.get_logger().warn("Waiting for localization/odometry before accepting goals.")
            return

        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y

        self.state = "PLANNING"
        self._publish_agv_state("PLANNING")
        self.get_logger().info(f"Received Goal: ({goal_x:.2f}, {goal_y:.2f})")

        # Reset active recovery and dynamic blockage penalties on new user goal
        self._recovery_phase = None
        self._stuck_counter = 0
        self._prev_heading_err = None
        self.blocked_edges.clear()
        self.hazard_zones.clear()

        start_node = self.find_closest_node(self.current_x, self.current_y)
        self.target_node = self.find_closest_node(goal_x, goal_y)

        self.get_logger().info(f"Snapping to nodes: Start={start_node}, Target={self.target_node}")

        self.node_path = self.calculate_dijkstra(start_node, self.target_node)

        if not self.node_path:
            self.state = "IDLE"
            self._publish_agv_state("IDLE")
            return

        base_plan = self.densify_path(self.node_path, step_m=0.30)
        sx = self.map_data[start_node]["x"]
        sy = self.map_data[start_node]["y"]
        dist_to_start = math.hypot(sx - self.current_x, sy - self.current_y)
        lead_in = []
        if dist_to_start > 0.10:
            num_lead = max(2, int(dist_to_start / 0.15))
            lead_th = math.atan2(sy - self.current_y, sx - self.current_x)
            for k in range(num_lead):
                t = k / float(num_lead)
                lx = self.current_x + t * (sx - self.current_x)
                ly = self.current_y + t * (sy - self.current_y)
                lead_in.append((lx, ly, lead_th))

        self.path_plan = lead_in + base_plan
        self.get_logger().info(f"Dijkstra Path: {self.node_path} ({len(self.path_plan)} waypoints with {len(lead_in)} lead-in pts)")

        self.current_target_index = 0
        self._visited_plan_nodes = set()
        self.last_cmd_v = 0.0
        self.last_cmd_w = 0.0
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
        # Search window from slightly behind current index to max 8 waypoints ahead
        start_search = max(0, self.current_target_index - 2)
        search_window = min(len(self.path_plan), self.current_target_index + 8)
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
        # Chassis half-width is 0.11m; swept half-width is 0.16m.
        # Angular expansion accounts for forward trajectory up to +/-25 deg.
        front_lat_limit = np.maximum(0.16, x_rel * 0.466)
        front_mask = (x_rel > 0.05) & (np.abs(y_rel) < front_lat_limit) & (dist < 6.0)
        d_front = float(np.min(dist[front_mask])) if np.any(front_mask) else 99.0

        # Rear swept body corridor: check obstacles directly behind chassis
        rear_lat_limit = np.maximum(0.16, -x_rel * 0.466)
        rear_mask = (x_rel < -0.05) & (np.abs(y_rel) < rear_lat_limit) & (dist < 6.0)
        d_rear = float(np.min(dist[rear_mask])) if np.any(rear_mask) else 99.0

        return d_front, d_rear

    def get_corridor_obstacle_analysis(self, lookahead_horizon=2.8):
        """Proactively scans up to lookahead_horizon meters ahead along the planned path.
        Detects unmapped stationary and dynamic obstacles, computes aisle passability for
        a 22cm robot, and identifies the best bypass direction (LEFT/RIGHT).
        """
        if len(self.obstacles) == 0 or len(self.path_plan) < 5:
            return None

        arc_dist = 0.0
        start_idx = self.current_target_index
        upcoming_indices = [start_idx]

        for i in range(start_idx, len(self.path_plan) - 1):
            p1 = self.path_plan[i]
            p2 = self.path_plan[i + 1]
            seg_len = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
            arc_dist += seg_len
            upcoming_indices.append(i + 1)
            if arc_dist >= lookahead_horizon:
                break

        if len(upcoming_indices) < 3:
            return None

        path_pts = np.array([[self.path_plan[i][0], self.path_plan[i][1]] for i in upcoming_indices])

        # Distance from each obstacle point to upcoming path waypoints
        dists_matrix = np.linalg.norm(self.obstacles[:, np.newaxis, :] - path_pts[np.newaxis, :, :], axis=-1)

        # Swept corridor width for 22cm robot: points within 0.22m of path centerline
        min_dists_to_path = np.min(dists_matrix, axis=0)
        blocked_wpts = np.where(min_dists_to_path < 0.22)[0]
        if len(blocked_wpts) == 0:
            return None

        earliest_local_idx = int(blocked_wpts[0])
        block_idx = upcoming_indices[earliest_local_idx]
        block_pt = self.path_plan[block_idx]

        # Calculate distance along path from robot to the obstacle
        d_along = 0.0
        for i in range(start_idx, block_idx):
            d_along += math.hypot(self.path_plan[i+1][0] - self.path_plan[i][0],
                                  self.path_plan[i+1][1] - self.path_plan[i][1])

        # Cluster points around the obstacle location
        cluster_mask = np.linalg.norm(self.obstacles - np.array([block_pt[0], block_pt[1]]), axis=-1) < 0.60
        cluster_pts = self.obstacles[cluster_mask]
        if len(cluster_pts) == 0:
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

        # Check if obstacle is dynamic (moving) or static unmapped
        is_dynamic = False
        obs_vel = (0.0, 0.0)
        obs_speed = 0.0
        for dob in self.dynamic_obstacles:
            if math.hypot(dob['pos'][0] - ox, dob['pos'][1] - oy) < 0.70 and dob['speed'] >= 0.25:
                is_dynamic = True
                obs_vel = tuple(dob['vel'])
                obs_speed = dob['speed']
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

                self.node_path = new_path
                base_plan = self.densify_path(self.node_path, step_m=0.30)

                # Smooth Lead-In spline from current robot pose to chosen_start node
                sx = self.map_data[chosen_start]["x"]
                sy = self.map_data[chosen_start]["y"]
                dist_to_start = math.hypot(sx - self.current_x, sy - self.current_y)
                lead_in = []
                if dist_to_start > 0.10:
                    num_lead = max(2, int(dist_to_start / 0.15))
                    lead_th = math.atan2(sy - self.current_y, sx - self.current_x)
                    for k in range(num_lead):
                        t = k / float(num_lead)
                        lx = self.current_x + t * (sx - self.current_x)
                        ly = self.current_y + t * (sy - self.current_y)
                        lead_in.append((lx, ly, lead_th))

                self.path_plan = lead_in + base_plan
                self.current_target_index = 0
                self._visited_plan_nodes = set()
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
                    f"({len(self.path_plan)} waypoints with {len(lead_in)} lead-in pts starting from {chosen_start}). Resuming navigation."
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
            self.node_path = new_path
            base_plan = self.densify_path(self.node_path, step_m=0.30)

            # Smooth Lead-In from robot pose
            sx = self.map_data[start_node]["x"]
            sy = self.map_data[start_node]["y"]
            dist_to_start = math.hypot(sx - self.current_x, sy - self.current_y)
            lead_in = []
            if dist_to_start > 0.10:
                num_lead = max(2, int(dist_to_start / 0.15))
                lead_th = math.atan2(sy - self.current_y, sx - self.current_x)
                for k in range(num_lead):
                    t = k / float(num_lead)
                    lx = self.current_x + t * (sx - self.current_x)
                    ly = self.current_y + t * (sy - self.current_y)
                    lead_in.append((lx, ly, lead_th))

            self.path_plan = lead_in + base_plan
            self.current_target_index = 0
            self._visited_plan_nodes = set()
            self._reroute_cooldown_time = now + 3.5
            self._swerve_bias = 0.0
            self._swerve_lock_ticks = 0
            self.publish_active_path()
            self.state = "NAVIGATING"
            self.yield_start_time = None
            self.get_logger().info(
                f"[BRAIN: DIJKSTRA_ROUTER] DYNAMIC REROUTE SUCCESSFUL! "
                f"New route: {self.node_path} ({len(self.path_plan)} waypoints with {len(lead_in)} lead-in pts). Resuming navigation."
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
                # Command smooth reverse
                self.last_cmd_v = -0.15
                self.last_cmd_w = 0.0
                cmd = Twist()
                cmd.linear.x = -0.15  # 0.15 m/s reverse
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

    def control_loop(self):
        self.update_robot_pose()
        if self.state not in ["NAVIGATING", "YIELDING"]:
            return

        now = self._now_sec()
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
        if dist_to_final < 0.30:
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

        # Proactive ahead-of-path corridor scan (up to 2.8m ahead on path)
        corridor_obs = self.get_corridor_obstacle_analysis(lookahead_horizon=2.8)
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
        (target_x, target_y, target_theta), target_idx = self.get_lookahead_target(lookahead_dist=0.75)

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
            # Only count points that are <0.18 m from path centreline AND <1.2 m from robot (0.18m = robot body width clearance)
            block_mask = (dists_to_path < 0.18) & (dists_to_robot < 1.2)
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
        # Goal approach velocity scaling: smooth quadratic deceleration as robot nears destination
        if dist_to_final < 1.2:
            approach_scale = max(0.28, min(1.0, dist_to_final / 1.0))
        else:
            approach_scale = 1.0

        # Dynamic obstacle proximity deceleration: if dynamic obstacle is ahead within 1.5m, scale v_mean down
        dyn_slowdown = 1.0
        for dob in self.dynamic_obstacles:
            d_obs = math.hypot(dob['pos'][0] - self.current_x, dob['pos'][1] - self.current_y)
            if d_obs < 1.5:
                angle_to_obs = math.atan2(dob['pos'][1] - self.current_y, dob['pos'][0] - self.current_x)
                angle_diff = abs(math.atan2(math.sin(angle_to_obs - self.current_yaw), math.cos(angle_to_obs - self.current_yaw)))
                if angle_diff < math.radians(60):
                    dyn_slowdown = min(dyn_slowdown, max(0.20, d_obs / 1.5))

        # Heading-Velocity Coupling:
        # Continuous C1-smooth cornering velocity profile without hard step cliff
        abs_h_err = abs(heading_error)
        heading_alignment = max(0.0, math.cos(heading_error))

        if abs_h_err <= math.radians(45):
            corner_scale = 1.0
        elif abs_h_err <= math.radians(90):
            frac = (abs_h_err - math.radians(45)) / math.radians(45)
            corner_scale = 0.15 + 0.85 * 0.5 * (1.0 + math.cos(math.pi * frac))
        else:
            corner_scale = 0.0  # In-place pivot for acute / reverse turns

        base_v = 0.45 * (heading_alignment ** 1.5) * corner_scale * approach_scale * dyn_slowdown
        v_mean = max(0.0, base_v) if corner_scale == 0.0 else max(0.08, base_v)

        v_seq = np.random.normal(v_mean, self.noise_v, (self.num_samples, self.horizon))

        # Swerve side commitment: guides MPPI sampling prior only for unexpected close dynamic obstacles
        if len(close_dyn) > 0 and is_evading:
            # If already deviating (> 45 deg) from target heading, cancel swerve bias to prioritize path realignment
            if abs_h_err > math.radians(45):
                self._swerve_bias = 0.0
                self._swerve_lock_ticks = 0
            elif self._swerve_lock_ticks <= 0:
                nearest = min(close_dyn, key=lambda d: math.hypot(
                    d['pos'][0] - self.current_x, d['pos'][1] - self.current_y))
                dx_t = target_x - self.current_x
                dy_t = target_y - self.current_y
                dx_o = nearest['pos'][0] - self.current_x
                dy_o = nearest['pos'][1] - self.current_y
                cross = dx_t * dy_o - dy_t * dx_o  # >0 = obstacle on left -> swerve right
                obs_dist = math.hypot(dx_o, dy_o)
                proposed_bias_sign = -1.0 if cross > 0 else 1.0
                side_str = "RIGHT (obs on LEFT)" if cross > 0 else "LEFT (obs on RIGHT)"

                raw_bias = self.swerve_bias_strength * (self.evasion_range / max(0.20, obs_dist))
                bias_mag = min(0.35, max(0.15, raw_bias))
                self._swerve_bias = proposed_bias_sign * bias_mag
                self._swerve_lock_ticks = self.swerve_lock_duration
                self.get_logger().info(
                    f"[BRAIN: SWERVE] Committed dynamic evasion swerve {side_str} for {self.swerve_lock_duration} ticks | "
                    f"bias={self._swerve_bias:+.2f} rad/s (dist={obs_dist:.2f}m)"
                )
            else:
                self._swerve_lock_ticks -= 1
        else:
            if self._swerve_lock_ticks > 0:
                self._swerve_lock_ticks -= 1
            else:
                self._swerve_bias = 0.0

        # MPPI Angular Sampling Prior: blend proportional heading guidance with evasion bias
        kp_heading = 1.4
        w_guidance = kp_heading * heading_error
        w_mean = float(np.clip(w_guidance + self._swerve_bias, -self.w_max, self.w_max))
        w_seq = np.random.normal(w_mean, self.noise_w, (self.num_samples, self.horizon))
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

        # Collision & Repulsion from Dynamic Moving Obstacles (Predictive Rollout)
        for dyn_obs in self.dynamic_obstacles:
            obs_pos = dyn_obs['pos']
            obs_vel = dyn_obs['vel']
            for t in range(self.horizon):
                # Predicted dynamic obstacle position at horizon step t
                pred_obs_t = obs_pos + obs_vel * (t * self.dt)
                pts_t = np.stack([x_rollout[:, t], y_rollout[:, t]], axis=-1)
                dist_to_dyn = np.linalg.norm(pts_t - pred_obs_t, axis=-1)

                # Hard safety collision
                col_mask = dist_to_dyn < self.collision_radius
                costs[col_mask] += self.w_collision

                # Smooth proactive repulsion
                rep_mask = dist_to_dyn < self.dynamic_repulsive_dist
                costs[rep_mask] += self.dynamic_w_repulsive * (self.dynamic_repulsive_dist - dist_to_dyn[rep_mask])

        # Collision & Repulsion from Static Obstacles (Walls, Shelves)
        if len(self.obstacles) > 0:
            # Spatial pre-filter: keep only obstacles within 3.5m radius of the robot
            obs_dists_sq = (self.obstacles[:, 0] - self.current_x)**2 + (self.obstacles[:, 1] - self.current_y)**2
            near_mask = obs_dists_sq < 12.25  # 3.5^2
            near_obs = self.obstacles[near_mask]

            if len(near_obs) > 0:
                obs = near_obs[np.newaxis, :, :]
                for t in range(self.horizon):
                    pts = np.stack([x_rollout[:, t], y_rollout[:, t]], axis=-1)[:, np.newaxis, :]
                    min_dists = np.min(np.linalg.norm(pts - obs, axis=-1), axis=1)

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
                is_halted = optimal_v < 0.06

                # Kinematic-Aware Turn Progress Guard:
                # Differentiate intentional in-place turn toward target from physical stall
                is_turning_in_place = is_spinning and is_halted
                heading_err_abs = abs(heading_error)
                heading_progress = False
                if self._prev_heading_err is not None:
                    # Heading error is actively reducing (>= 0.015 rad / ~0.85 deg per tick)
                    heading_progress = (heading_err_abs < self._prev_heading_err - 0.015)
                self._prev_heading_err = heading_err_abs

                if is_turning_in_place and heading_progress:
                    # Healthy in-place pivot toward waypoint: reset stuck counter!
                    self._stuck_counter = 0
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

        # Clamp within global physical limits
        cmd_v = float(np.clip(cmd_v, self.v_min, self.v_max))
        cmd_w = float(np.clip(cmd_w, -self.w_max, self.w_max))

        self.last_cmd_v = cmd_v
        self.last_cmd_w = cmd_w

        twist = Twist()
        twist.linear.x = cmd_v
        twist.angular.z = cmd_w
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