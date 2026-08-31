import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import tf2_ros
from tf2_ros import Buffer, TransformListener, TransformException
import math
import json
import os
import heapq

import numpy as np
from scipy.spatial import KDTree


class RouteRunner(Node):
    def __init__(self):
        super().__init__('route_runner')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/agv_dense_path', 10)  # Publish dense path for RViz
        # Mission progress publisher: JSON string of {current, total, goal_node, state}
        self.mission_pub = self.create_publisher(String, '/mission_progress', 10)

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.goal_sub_alt = self.create_subscription(PoseStamped, '/goal', self.goal_callback, 10)
        self.goal_sub_mb = self.create_subscription(PoseStamped, '/move_base_simple/goal', self.goal_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        # Multi-goal sequence subscriber: expects JSON array of node IDs e.g. '["N5","N12","N40"]'
        self.seq_sub = self.create_subscription(String, '/goal_sequence', self.sequence_callback, 10)

        # TF Buffer and Listener for AMCL / SLAM localization (map -> base_link)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.using_map_tf = False

        # Load the graph — use installed share path (portable)
        try:
            pkg_share = get_package_share_directory('agv_description')
            self.map_path = os.path.join(pkg_share, 'maps', 'warehouse_graph.json')
        except Exception:
            # Fallback for dev workspace (not installed)
            self.map_path = os.path.join(os.environ['HOME'], 'AMR', 'AMR-main', 'src', 'agv_description', 'maps', 'warehouse_graph.json')

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

        # Diagnostic rate-limiter: only log blocking details once per second
        self._last_block_log_time = 0.0

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
        self.dynamic_obstacles = []    # List of dicts: {'pos': (x,y), 'vel': (vx,vy), 'radius': r}
        self.last_scan_time = None


        # Traffic & Re-routing parameters
        self.yield_timeout = 4.5       # Seconds to wait for transient obstacles before re-routing
        self.yield_start_time = None
        self.blocked_edges = {}        # {(u, v): (original_cost, restore_timestamp)}

        # MPPI Parameters
        self.v_max = 0.8
        self.v_min = 0.0               # Forward-only to avoid backward jitter
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
        self.collision_radius = 0.22   # Safety hard clearance (m) — robot body radius 0.15 + 0.07 margin
        self.w_repulsive = 35.0        # Smooth proactive repulsion from obstacles
        self.repulsive_dist = 0.55     # Distance at which repulsion activates (m) — tuned for narrow corridors

        self.w_cross_track_nominal = 6.0
        self.w_cross_track_evasion = 2.5   # Soften centerline tracking to steer around obstacles
        self.w_cross_track = self.w_cross_track_nominal

        # Control timer
        self.control_timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info(f"Route Runner Active with KD-Tree ({len(self.node_ids)} nodes) & Dynamic Obstacle Avoidance.")

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

        valid = (ranges > msg.range_min) & (ranges < msg.range_max)
        ranges = ranges[valid]
        angles = angles[valid]

        if len(ranges) == 0:
            self.obstacles = np.array([])
            self.dynamic_obstacles = []
            return

        # Downsample LiDAR to every 3rd point
        ranges = ranges[::3]
        angles = angles[::3]

        # Convert to global map frame
        ox_local = ranges * np.cos(angles)
        oy_local = ranges * np.sin(angles)

        cos_yaw = np.cos(self.current_yaw)
        sin_yaw = np.sin(self.current_yaw)

        ox_global = self.current_x + ox_local * cos_yaw - oy_local * sin_yaw
        oy_global = self.current_y + ox_local * sin_yaw + oy_local * cos_yaw

        self.obstacles = np.column_stack((ox_global, oy_global))

        # --- Dynamic Obstacle Velocity Estimation ---
        # Cluster nearby obstacle points (within 5m) to find moving objects
        dists_to_robot = np.sqrt(ox_local**2 + oy_local**2)
        near_mask = dists_to_robot < 5.0
        near_pts = self.obstacles[near_mask]

        current_clusters = []
        if len(near_pts) > 0:
            # Simple spatial grid clustering (0.4m grid bins)
            bins = {}
            for p in near_pts:
                key = (round(p[0] / 0.4), round(p[1] / 0.4))
                bins.setdefault(key, []).append(p)

            for key, pts in bins.items():
                if len(pts) >= 2:  # Filter noise specks
                    centroid = np.mean(pts, axis=0)
                    current_clusters.append(centroid)

        # Estimate velocity by matching clusters with previous frame
        new_dynamic_obs = []
        if self.last_scan_time is not None and len(self.prev_obstacle_clusters) > 0:
            dt = max(0.05, min(0.3, now - self.last_scan_time))
            for curr_c in current_clusters:
                best_prev = None
                min_match_dist = 0.8  # Max jump between scans (equiv. to 8 m/s max)
                for prev_c in self.prev_obstacle_clusters:
                    d = np.linalg.norm(curr_c - prev_c)
                    if d < min_match_dist:
                        min_match_dist = d
                        best_prev = prev_c

                if best_prev is not None:
                    vel = (curr_c - best_prev) / dt
                    speed = np.linalg.norm(vel)
                    # Classify as dynamic if moving between 0.1 m/s and 2.5 m/s
                    if 0.10 <= speed <= 2.5:
                        new_dynamic_obs.append({
                            'pos': curr_c,
                            'vel': vel,
                            'radius': 0.35
                        })

        self.dynamic_obstacles = new_dynamic_obs
        self.prev_obstacle_clusters = current_clusters
        self.last_scan_time = now

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
        self.get_logger().info(f"Received Goal: ({goal_x:.2f}, {goal_y:.2f})")

        start_node = self.find_closest_node(self.current_x, self.current_y)
        self.target_node = self.find_closest_node(goal_x, goal_y)

        self.get_logger().info(f"Snapping to nodes: Start={start_node}, Target={self.target_node}")

        self.node_path = self.calculate_dijkstra(start_node, self.target_node)

        if not self.node_path:
            self.state = "IDLE"
            return

        self.path_plan = self.densify_path(self.node_path, step_m=0.30)
        self.get_logger().info(f"Dijkstra Path: {self.node_path} ({len(self.path_plan)} waypoints)")

        self.current_target_index = 1 if len(self.path_plan) > 1 else 0
        self.state = "NAVIGATING"
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
        search_window = min(len(self.path_plan), self.current_target_index + 40)
        for i in range(self.current_target_index, search_window):
            px, py, _ = self.path_plan[i]
            dist = math.sqrt((px - self.current_x)**2 + (py - self.current_y)**2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        self.current_target_index = max(self.current_target_index, closest_idx)

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

    def trigger_reroute(self):
        """Dynamically penalizes the blocked edge and computes an alternate path."""
        now = self._now_sec()
        start_node = self.find_closest_node(self.current_x, self.current_y)

        # Identify upcoming edge in current plan
        if len(self.node_path) >= 2:
            try:
                curr_idx = self.node_path.index(start_node)
                if curr_idx + 1 < len(self.node_path):
                    u = self.node_path[curr_idx]
                    v = self.node_path[curr_idx + 1]

                    # Penalize this blocked edge heavily
                    for edge in self.map_data[u]["edges"]:
                        if edge["to_node"] == v:
                            orig_cost = edge["distance_m"]
                            edge["distance_m"] = 999.0
                            self.blocked_edges[(u, v)] = (orig_cost, now + 25.0)

                    for edge in self.map_data[v]["edges"]:
                        if edge["to_node"] == u:
                            orig_cost = edge["distance_m"]
                            edge["distance_m"] = 999.0
                            self.blocked_edges[(v, u)] = (orig_cost, now + 25.0)

                    self.get_logger().warn(f"Temporarily closed blocked corridor edge: {u} <-> {v}")
            except ValueError:
                pass

        # Re-compute Dijkstra on updated graph
        new_path = self.calculate_dijkstra(start_node, self.target_node)
        if new_path and len(new_path) > 1:
            self.node_path = new_path
            self.path_plan = self.densify_path(self.node_path, step_m=0.30)
            self.current_target_index = 0
            self.publish_active_path()
            self.state = "NAVIGATING"
            self.yield_start_time = None
            self.get_logger().info(f"DYNAMIC REROUTE SUCCESSFUL! New path: {self.node_path}")
        else:
            self.get_logger().warn("No alternate route available around blockage. Will continue waiting...")
            self.state = "YIELDING"
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

    def control_loop(self):
        self.update_robot_pose()
        if self.state not in ["NAVIGATING", "YIELDING"]:
            return

        now = self._now_sec()
        self.restore_unblocked_edges()

        # Check final destination
        final_x, final_y, _ = self.path_plan[-1]
        dist_to_final = math.sqrt((final_x - self.current_x)**2 + (final_y - self.current_y)**2)
        if dist_to_final < 0.30:
            self.cmd_pub.publish(Twist())
            self.state = "IDLE"
            if self.goal_queue:
                # More goals remain — dequeue and navigate to next waypoint
                self.get_logger().info(
                    f"Waypoint {self.mission_current}/{self.mission_total} reached. "
                    f"Proceeding to next goal ({len(self.goal_queue)} remaining)."
                )
                self._dispatch_next_goal()
            else:
                # Final goal reached — mission complete
                total = self.mission_total if self.mission_total > 0 else 1
                self.get_logger().info(
                    f"FINAL DESTINATION REACHED! "
                    f"Mission complete ({total} goal(s) visited)."
                )
                self._publish_mission_progress("MISSION_COMPLETE")
                self.mission_total = 0
                self.mission_current = 0
            return


        # Get lookahead target
        (target_x, target_y, target_theta), target_idx = self.get_lookahead_target(lookahead_dist=1.2)

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
            # Only count points that are <0.25 m from the path centreline AND <1.2 m from robot
            block_mask = (dists_to_path < 0.25) & (dists_to_robot < 1.2)
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
        if obstacle_blocking_path or len(self.dynamic_obstacles) > 0:
            self.w_cross_track = self.w_cross_track_evasion
        else:
            self.w_cross_track = self.w_cross_track_nominal

        # --- MPPI Controller with Predictive Dynamic Rollout ---
        # 1. Sample control sequences
        v_seq = np.random.normal(0.25, self.noise_v, (self.num_samples, self.horizon))
        w_seq = np.random.normal(0.0, self.noise_w, (self.num_samples, self.horizon))
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

        # Horizon-wide heading cost
        for t in range(self.horizon):
            h_err = np.arctan2(
                np.sin(target_theta - yaw_rollout[:, t]),
                np.cos(target_theta - yaw_rollout[:, t])
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
                rep_mask = dist_to_dyn < self.repulsive_dist
                costs[rep_mask] += self.w_repulsive * (self.repulsive_dist - dist_to_dyn[rep_mask])

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
                    rep_mask = min_dists < 0.45  # Tightened from 0.65 to prevent corridor-wide repulsion
                    costs[rep_mask] += 25.0 * (0.45 - min_dists[rep_mask])

        # --- Traffic & Yielding Decision ---
        best_cost = np.min(costs)
        all_colliding = best_cost >= self.w_collision

        if all_colliding and obstacle_blocking_path:
            # All sampled trajectories are blocked AND an obstacle is on the path
            if self.state == "NAVIGATING":
                self.state = "YIELDING"
                self.yield_start_time = now
                # Diagnostic: log which obstacle point triggered the block (rate-limited to 1 Hz)
                if _blocking_diag and (now - self._last_block_log_time) > 1.0:
                    ox, oy, dp, dr = _blocking_diag
                    self.get_logger().warn(
                        f"Narrow corridor blocked! "
                        f"Obstacle at map({ox:.3f}, {oy:.3f}), "
                        f"dist_to_path={dp:.3f}m, dist_to_robot={dr:.3f}m, "
                        f"MPPI_best_cost={best_cost:.1f}"
                    )
                    self._last_block_log_time = now
                self.cmd_pub.publish(Twist())
                return
            elif self.state == "YIELDING":
                elapsed = now - self.yield_start_time
                if elapsed >= self.yield_timeout:
                    self.get_logger().warn(f"Obstacle remained blocked for {elapsed:.1f}s. Triggering DYNAMIC REROUTE!")
                    self.trigger_reroute()
                    return
                else:
                    # Rate-limited diagnostic while waiting
                    if _blocking_diag and (now - self._last_block_log_time) > 1.0:
                        ox, oy, dp, dr = _blocking_diag
                        self.get_logger().warn(
                            f"Still yielding ({elapsed:.1f}s): obstacle at map({ox:.3f}, {oy:.3f}), "
                            f"dist_to_path={dp:.3f}m, dist_to_robot={dr:.3f}m"
                        )
                        self._last_block_log_time = now
                    self.cmd_pub.publish(Twist())
                    return

        # Path is clear — exit YIELDING only when BOTH MPPI and path-blocker agree
        # (Previously only checked all_colliding, causing oscillation when one MPPI
        # sample sneaked through while a wall was still <0.25 m from the path.)
        if self.state == "YIELDING" and (not all_colliding) and (not obstacle_blocking_path):
            self.get_logger().info("Corridor cleared! Resuming navigation.")
            self.state = "NAVIGATING"
            self.yield_start_time = None

        # 4. Softmax Weighting and Command Execution
        beta = np.min(costs)
        weights = np.exp(-1.0 / self.lambda_weight * (costs - beta))
        weights = weights / np.sum(weights)

        optimal_v = float(np.sum(weights * v_seq[:, 0]))
        optimal_w = float(np.sum(weights * w_seq[:, 0]))

        twist = Twist()
        twist.linear.x = optimal_v
        twist.angular.z = optimal_w
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