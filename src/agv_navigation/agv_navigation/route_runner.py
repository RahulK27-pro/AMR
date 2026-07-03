import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
import math
import json
import os
import heapq
import numpy as np

class RouteRunner(Node):
    def __init__(self):
        super().__init__('route_runner')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
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
        
        self.path_plan = []           
        self.current_target_index = 0
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_ready = False
        
        # Obstacles
        self.obstacles = np.array([]) # Shape (N, 2)
        
        # MPPI Parameters
        self.v_max = 1.0
        self.v_min = 0.0
        self.w_max = 1.5
        self.dt = 0.1 # 10 Hz
        self.horizon = 20 # 2.0s / 0.1s
        self.num_samples = 150
        
        # MPPI Noise covariance
        self.noise_v = 0.3
        self.noise_w = 0.5
        self.lambda_weight = 0.1 # Temperature (lowered to allow sharper turns)
        
        # MPPI Cost Weights
        self.w_dist = 5.0
        self.w_heading = 1.0
        self.w_collision = 1000.0
        self.collision_radius = 0.4 # meters
        
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

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = self.get_yaw_from_quaternion(msg.pose.pose.orientation)
        self.odom_ready = True
        
    def scan_callback(self, msg):
        # Convert LaserScan to 2D Cartesian points in base_link frame, then to odom frame
        # For simplicity, assuming scan is in base_link frame
        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        
        # Filter out inf and out of range
        valid = (ranges > msg.range_min) & (ranges < msg.range_max)
        ranges = ranges[valid]
        angles = angles[valid]
        
        if len(ranges) == 0:
            self.obstacles = np.array([])
            return
            
        # Convert to global frame
        ox_local = ranges * np.cos(angles)
        oy_local = ranges * np.sin(angles)
        
        # Rotate and translate to global frame
        cos_yaw = np.cos(self.current_yaw)
        sin_yaw = np.sin(self.current_yaw)
        
        ox_global = self.current_x + ox_local * cos_yaw - oy_local * sin_yaw
        oy_global = self.current_y + ox_local * sin_yaw + oy_local * cos_yaw
        
        self.obstacles = np.column_stack((ox_global, oy_global))

    def goal_callback(self, msg):
        if not self.odom_ready:
            self.get_logger().warn("Waiting for Odometry before accepting goals.")
            return
            
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        
        self.state = "PLANNING"
        self.get_logger().info(f"Received RViz Goal: ({goal_x:.2f}, {goal_y:.2f})")
        
        start_node = self.find_closest_node(self.current_x, self.current_y)
        target_node = self.find_closest_node(goal_x, goal_y)
        
        self.get_logger().info(f"Snapping to nodes: Start={start_node}, Target={target_node}")
        
        self.path_plan = self.calculate_dijkstra(start_node, target_node)
        
        if not self.path_plan:
            self.state = "IDLE"
            return
            
        self.get_logger().info(f"DIJKSTRA PATH FOUND: {self.path_plan}")
        self.current_target_index = 1 if len(self.path_plan) > 1 else 0
        self.state = "NAVIGATING"
        
    def control_loop(self):
        if self.state != "NAVIGATING":
            return
            
        active_node_id = self.path_plan[self.current_target_index]
        target_x = self.map_data[active_node_id]["x"]
        target_y = self.map_data[active_node_id]["y"]
        
        distance_to_target = math.sqrt((target_x - self.current_x)**2 + (target_y - self.current_y)**2)
        
        # Check if reached node
        if distance_to_target < 0.4:
            self.get_logger().info(f"Reached Waypoint: Node {active_node_id}")
            if self.current_target_index < len(self.path_plan) - 1:
                self.current_target_index += 1
            else:
                self.get_logger().info("FINAL DESTINATION REACHED! Mission Complete.")
                twist = Twist()
                self.cmd_pub.publish(twist)
                self.state = "IDLE"
            return
            
        # --- MPPI Controller ---
        # 1. Sample control sequences
        # Baseline nominal control (forward momentum)
        v_seq = np.random.normal(0.5, self.noise_v, (self.num_samples, self.horizon))
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
        
        # Heading cost towards target
        target_angles = np.arctan2(target_y - y_rollout[:, -1], target_x - x_rollout[:, -1])
        angle_errors = target_angles - yaw_rollout[:, -1]
        angle_errors = np.arctan2(np.sin(angle_errors), np.cos(angle_errors))
        costs += self.w_heading * np.abs(angle_errors)
        
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
        
        # If the BEST path still incurs a collision penalty, we are blocked!
        if beta >= self.w_collision:
            self.get_logger().warn("MPPI: All paths lead to collision! Stopping.")
            optimal_v = 0.0
            optimal_w = 0.0
        else:
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