import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import json
import os
import heapq  # Required for Dijkstra's Priority Queue

class RouteRunner(Node):
    def __init__(self):
        super().__init__('route_runner')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Load the map
        self.map_path = os.path.join(os.environ['HOME'], 'agv_ws', 'src', 'agv_navigation', 'maps', 'test_map.json')
        with open(self.map_path, 'r') as f:
            self.map_data = json.load(f)["graph"]
            
        # ==========================================
        # NEW: Autonomous Routing Variables
        # ==========================================
        self.final_destination = "6"  # CHANGE THIS TO YOUR DESIRED END NODE!
        self.path_plan = []           # Will be filled by Dijkstra
        self.path_calculated = False  # Flag to wait for first GPS/Odom ping
        self.current_target_index = 0
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.mission_complete = False

        self.get_logger().info(f"Route Runner Active. Waiting for initial odometry to calculate route to Node {self.final_destination}...")

    def get_yaw_from_quaternion(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    # ----------------------------------------------------
    # NEW: Find closest node to robot's current position
    # ----------------------------------------------------
    def find_closest_node(self, x, y):
        closest_node = None
        min_dist = float('inf')
        for node_id, data in self.map_data.items():
            dist = math.sqrt((x - data["x"])**2 + (y - data["y"])**2)
            if dist < min_dist:
                min_dist = dist
                closest_node = node_id
        return closest_node

    # ----------------------------------------------------
    # NEW: Dijkstra's Shortest Path Algorithm
    # ----------------------------------------------------
    def calculate_dijkstra(self, start_node, target_node):
        distances = {node: float('infinity') for node in self.map_data}
        distances[start_node] = 0
        previous_nodes = {node: None for node in self.map_data}
        
        # Priority queue stores tuples: (distance, node_id)
        pq = [(0, start_node)]
        
        while pq:
            current_distance, current_node = heapq.heappop(pq)
            
            # If we reached the target, stop searching
            if current_node == target_node:
                break
                
            # If we found a longer path than what's already recorded, ignore it
            if current_distance > distances[current_node]:
                continue
                
            # Check all connected neighboring nodes
            for edge in self.map_data[current_node]["edges"]:
                neighbor = edge["to_node"]
                weight = edge["distance_m"]
                distance = current_distance + weight
                
                # If we found a shorter path to the neighbor, update it!
                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    previous_nodes[neighbor] = current_node
                    heapq.heappush(pq, (distance, neighbor))
                    
        # Reconstruct the path by walking backward from the target
        path = []
        current = target_node
        while current is not None:
            path.append(current)
            current = previous_nodes[current]
            
        path.reverse() # Flip it from [End -> Start] to [Start -> End]
        
        # Safety check if path is impossible
        if path[0] != start_node:
            self.get_logger().error(f"NO VALID PATH FOUND FROM {start_node} TO {target_node}!")
            return []
            
        return path

    def odom_callback(self, msg):
        if self.mission_complete:
            return

        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = self.get_yaw_from_quaternion(msg.pose.pose.orientation)

        # ==========================================
        # NEW: Calculate Path on First Ping
        # ==========================================
        if not self.path_calculated:
            start_node = self.find_closest_node(self.current_x, self.current_y)
            self.get_logger().info(f"Auto-Localized at Node {start_node}. Calculating shortest path to {self.final_destination}...")
            
            self.path_plan = self.calculate_dijkstra(start_node, self.final_destination)
            
            if not self.path_plan:
                self.mission_complete = True # Abort if no path
                return
                
            self.get_logger().info(f"DIJKSTRA PATH FOUND: {self.path_plan}")
            self.path_calculated = True
            
            # If the robot is already AT the final destination
            if len(self.path_plan) == 1 and self.path_plan[0] == self.final_destination:
                self.get_logger().info("Already at destination!")
                self.mission_complete = True
            return

        # --- Standard P-Controller Execution ---
        active_node_id = self.path_plan[self.current_target_index]
        target_x = self.map_data[active_node_id]["x"]
        target_y = self.map_data[active_node_id]["y"]

        distance_error = math.sqrt((target_x - self.current_x)**2 + (target_y - self.current_y)**2)
        target_angle = math.atan2(target_y - self.current_y, target_x - self.current_x)
        angle_error = target_angle - self.current_yaw
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        twist = Twist()

        if distance_error < 0.15:
            self.get_logger().info(f"Reached Waypoint: Node {active_node_id}")
            
            if self.current_target_index < len(self.path_plan) - 1:
                self.current_target_index += 1
            else:
                self.get_logger().info("FINAL DESTINATION REACHED! Mission Complete.")
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
                self.mission_complete = True
            return

        if abs(angle_error) > 0.17: 
            twist.linear.x = 0.0
            twist.angular.z = 1.0 * angle_error 
        else:
            twist.linear.x = 0.5 * distance_error 
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = RouteRunner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()