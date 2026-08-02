import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
import tf2_ros
from tf2_ros import Buffer, TransformListener, TransformException
import math
import json
import os
import heapq
import numpy as np

class RouteRunner(Node):
    def __init__(self):
        super().__init__('route_runner')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/agv_dense_path', 10)  # Publish dense path for RViz
        # Use EKF-fused odometry (/odometry/filtered) to match Nav2 stack — consistent with AMCL
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.goal_sub_alt = self.create_subscription(PoseStamped, '/goal', self.goal_callback, 10)
        self.goal_sub_mb = self.create_subscription(PoseStamped, '/move_base_simple/goal', self.goal_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # TF Buffer and Listener for AMCL / SLAM localization (map -> base_link)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.using_map_tf = False
        
        # Load the map
        # Path updated to point to the new warehouse map in agv_description
        self.map_path = os.path.join(os.environ['HOME'], 'AMR', 'AMR-main', 'src', 'agv_description', 'maps', 'warehouse_graph.json')
        
        # Fallback for the older path if the above fails
        if not os.path.exists(self.map_path):
            self.map_path = os.path.join(os.environ['HOME'], 'agv_ws', 'src', 'agv_description', 'maps', 'warehouse_graph.json')
            
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
            

        self.state = "IDLE" # IDLE, PLANNING, NAVIGATING
        
        # path_plan stores dense (x, y) waypoints interpolated between Dijkstra nodes
        self.path_plan = []           
        self.current_target_index = 0
        self.node_path = []  # The raw Dijkstra node sequence for logging
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_ready = False
        
        # Obstacles
        self.obstacles = np.array([]) # Shape (N, 2)
        
        # MPPI Parameters
        self.v_max = 0.8
        self.v_min = 0.0  # Forward-only: prevents backward oscillation near waypoints
        self.w_max = 1.8  # INCREASED turning authority to quickly correct lateral drift
        self.dt = 0.1 # 10 Hz
        self.horizon = 15  # Reduced from 20 to cut computation time
        self.num_samples = 80  # Reduced from 150 to stay within 100ms control deadline
        
        # MPPI Noise covariance
        self.noise_v = 0.3
        self.noise_w = 0.5
        # lambda_weight: lower = sharper (collision-aware), higher = smoother but ignores obstacles
        # 0.5 is a safe middle ground: smooth enough to avoid jitter, sharp enough to respect walls
        self.lambda_weight = 0.5
        
        # MPPI Cost Weights
        self.w_dist = 5.0
        self.w_heading = 1.0
        self.w_collision = 5000.0  # Raised: make collision cost dominate trajectory selection
        self.collision_radius = 0.30  # Slightly conservative clearance
        
        # Control timer
        self.control_timer = self.create_timer(self.dt, self.control_loop)
        
        self.get_logger().info("Route Runner Active. Waiting for Goal in RViz (2D Goal Pose)...")

    def get_yaw_from_quaternion(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def find_closest_node(self, x, y):
        closest_node = None
        min_dist = float('inf')
        for node_id, data in self.map_data.items():
            dist = math.sqrt((x - data["x"])**2 + (y - data["y"])**2)
            if dist < min_dist:
                min_dist = dist
                closest_node = node_id
        return closest_node

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
            # FIX: Reset using_map_tf so odom_callback resumes updating the pose
            # This prevents the frozen-pose bug when AMCL TF temporarily drops
            if self.using_map_tf:
                self.get_logger().warn("AMCL TF lost! Falling back to /odometry/filtered.")
                self.using_map_tf = False
            return self.odom_ready

    def odom_callback(self, msg):
        # Update pose from /odometry/filtered (EKF-fused) when map TF is not active
        # This matches the odometry source used by the rest of the Nav2 stack
        if not self.using_map_tf:
            self.current_x = msg.pose.pose.position.x
            self.current_y = msg.pose.pose.position.y
            self.current_yaw = self.get_yaw_from_quaternion(msg.pose.pose.orientation)
            self.odom_ready = True
        
    def scan_callback(self, msg):
        self.update_robot_pose()
        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        
        # Filter out inf and out of range
        valid = (ranges > msg.range_min) & (ranges < msg.range_max)
        ranges = ranges[valid]
        angles = angles[valid]
        
        if len(ranges) == 0:
            self.obstacles = np.array([])
            return

        # Downsample LiDAR to every 3rd point — balances computation speed vs wall detection
        # Every 5th was too sparse and caused walls to be missed entirely
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

    def densify_path(self, node_path, step_m=0.30):
        """Interpolate dense (x, y, theta) waypoints every step_m meters.
        theta is the path tangent direction — the direction the robot should
        be facing when it passes through this point."""
        dense = []
        for i in range(len(node_path)):
            nx = self.map_data[node_path[i]]["x"]
            ny = self.map_data[node_path[i]]["y"]
            if i == 0:
                # For the first node, theta points toward the second node
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
            # Segment heading = direction from prev node to this node
            seg_theta = math.atan2(ny - py, nx - px)
            seg_len = math.sqrt((nx - px)**2 + (ny - py)**2)
            num_steps = max(1, int(seg_len / step_m))
            for k in range(1, num_steps + 1):
                t = k / num_steps
                ix = px + t * (nx - px)
                iy = py + t * (ny - py)
                dense.append((ix, iy, seg_theta))
        return dense

    def goal_callback(self, msg):
        self.update_robot_pose()
        if not self.odom_ready:
            self.get_logger().warn("Waiting for localization/odometry before accepting goals.")
            return
            
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        
        self.state = "PLANNING"
        self.get_logger().info(f"Received RViz Goal: ({goal_x:.2f}, {goal_y:.2f})")
        
        start_node = self.find_closest_node(self.current_x, self.current_y)
        target_node = self.find_closest_node(goal_x, goal_y)
        
        self.get_logger().info(f"Snapping to nodes: Start={start_node}, Target={target_node}")
        
        self.node_path = self.calculate_dijkstra(start_node, target_node)
        
        if not self.node_path:
            self.state = "IDLE"
            return
        
        # Densify: create a waypoint every 0.3m along the full node path
        self.path_plan = self.densify_path(self.node_path, step_m=0.30)
        
        self.get_logger().info(
            f"DIJKSTRA PATH: {self.node_path} → Densified to {len(self.path_plan)} waypoints"
        )
        self.current_target_index = 1 if len(self.path_plan) > 1 else 0
        self.state = "NAVIGATING"

        # Publish dense path so graph_visualizer can show it in RViz as a magenta line
        ros_path = Path()
        ros_path.header.frame_id = "map"
        ros_path.header.stamp = self.get_clock().now().to_msg()
        for (px, py, ptheta) in self.path_plan:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = px
            pose.pose.position.y = py
            # Convert theta to quaternion (rotation around Z)
            pose.pose.orientation.z = math.sin(ptheta / 2.0)
            pose.pose.orientation.w = math.cos(ptheta / 2.0)
            ros_path.poses.append(pose)
        self.path_pub.publish(ros_path)
        
    def get_lookahead_target(self, lookahead_dist=1.0):
        """Finds a target point on the path that is lookahead_dist away from the robot."""
        # Find closest point on path starting from current_target_index
        min_dist = float('inf')
        closest_idx = self.current_target_index
        # Only search a local window to avoid snapping to a different part of a looping path
        search_window = min(len(self.path_plan), self.current_target_index + 20)
        for i in range(self.current_target_index, search_window):
            px, py, _ = self.path_plan[i]
            dist = math.sqrt((px - self.current_x)**2 + (py - self.current_y)**2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
                
        self.current_target_index = closest_idx
        
        # Search forward to find the point lookahead_dist away
        target_idx = closest_idx
        for i in range(closest_idx, len(self.path_plan)):
            px, py, _ = self.path_plan[i]
            dist = math.sqrt((px - self.current_x)**2 + (py - self.current_y)**2)
            if dist >= lookahead_dist:
                target_idx = i
                break
        else:
            target_idx = len(self.path_plan) - 1
            
        return self.path_plan[target_idx], target_idx

    def control_loop(self):
        self.update_robot_pose()
        if self.state != "NAVIGATING":
            return
        
        # Check if we have reached the FINAL destination
        final_x, final_y, _ = self.path_plan[-1]
        dist_to_final = math.sqrt((final_x - self.current_x)**2 + (final_y - self.current_y)**2)
        
        if dist_to_final < 0.30:
            self.get_logger().info("FINAL DESTINATION REACHED! Mission Complete.")
            self.cmd_pub.publish(Twist())
            self.state = "IDLE"
            return
            
        # Get dynamic lookahead target (1.2 meters ahead) so MPPI horizon doesn't overshoot it
        (target_x, target_y, target_theta), target_idx = self.get_lookahead_target(lookahead_dist=1.2)
            
        # --- MPPI Controller ---
        # 1. Sample control sequences
        # Baseline nominal control (slight forward momentum, but allows reverse)
        v_seq = np.random.normal(0.2, self.noise_v, (self.num_samples, self.horizon))
        w_seq = np.random.normal(0.0, self.noise_w, (self.num_samples, self.horizon))
        
        # Clip to kinematic limits
        v_seq = np.clip(v_seq, self.v_min, self.v_max)
        w_seq = np.clip(w_seq, -self.w_max, self.w_max)
        
        # 2. Rollout
        # Initialize state arrays for all samples
        x_rollout = np.full((self.num_samples, self.horizon), self.current_x)
        y_rollout = np.full((self.num_samples, self.horizon), self.current_y)
        yaw_rollout = np.full((self.num_samples, self.horizon), self.current_yaw)
        
        for t in range(1, self.horizon):
            yaw_rollout[:, t] = yaw_rollout[:, t-1] + w_seq[:, t-1] * self.dt
            x_rollout[:, t] = x_rollout[:, t-1] + v_seq[:, t-1] * np.cos(yaw_rollout[:, t-1]) * self.dt
            y_rollout[:, t] = y_rollout[:, t-1] + v_seq[:, t-1] * np.sin(yaw_rollout[:, t-1]) * self.dt
            
        # 3. Evaluate Cost
        costs = np.zeros(self.num_samples)
        
        # Terminal cost: Distance to target node
        terminal_dists = np.sqrt((x_rollout[:, -1] - target_x)**2 + (y_rollout[:, -1] - target_y)**2)
        costs += self.w_dist * terminal_dists
        
        # Heading cost towards target: use path tangent theta stored in waypoint, not point-to-point angle
        angle_errors = target_theta - yaw_rollout[:, -1]
        angle_errors = np.arctan2(np.sin(angle_errors), np.cos(angle_errors))
        costs += self.w_heading * np.abs(angle_errors)
        
        # Cross-Track Error Penalty (Strict Path Tracking)
        # 1. Get local path segment (from current index to target index + 1)
        local_path_full = self.path_plan[self.current_target_index : target_idx + 2]
        local_path = np.array([(p[0], p[1]) for p in local_path_full])
        if len(local_path) > 0:
            # 2. Shape rollout points for broadcasting: (num_samples, horizon, 1, 2)
            rollout_pts = np.stack((x_rollout, y_rollout), axis=-1)[:, :, np.newaxis, :]
            
            # 3. Shape path points: (1, 1, N, 2)
            path_pts = local_path[np.newaxis, np.newaxis, :, :]
            
            # 4. Calculate distances to all path points: (num_samples, horizon, N)
            dists = np.linalg.norm(rollout_pts - path_pts, axis=-1)
            
            # 5. Find distance to the closest path point for each rollout step: (num_samples, horizon)
            min_dists = np.min(dists, axis=-1)
            
            # 6. Sum the errors over the horizon and add to total cost
            w_cross_track = 15.0  # Strong penalty for lateral deviation
            costs += w_cross_track * np.sum(min_dists, axis=-1)
        
        # Collision cost
        if len(self.obstacles) > 0:
            for t in range(self.horizon):
                # Broadcasting: (num_samples, 1, 2) - (1, num_obs, 2)
                pts = np.stack([x_rollout[:, t], y_rollout[:, t]], axis=-1)[:, np.newaxis, :]
                obs = self.obstacles[np.newaxis, :, :]
                
                # Minimum distance to any obstacle for each sample at time t
                min_dists = np.min(np.linalg.norm(pts - obs, axis=-1), axis=1)
                
                collision_mask = min_dists < self.collision_radius
                costs[collision_mask] += self.w_collision
                
        # 4. Update and generate optimal command
        beta = np.min(costs)
        weights = np.exp(-1.0 / self.lambda_weight * (costs - beta))
        
        weights = weights / np.sum(weights)
        optimal_v = np.sum(weights * v_seq[:, 0])
        optimal_w = np.sum(weights * w_seq[:, 0])
            
        # Execute
        twist = Twist()
        twist.linear.x = float(optimal_v)
        twist.angular.z = float(optimal_w)
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = RouteRunner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()