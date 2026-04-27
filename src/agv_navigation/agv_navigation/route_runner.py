import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
import math
import json
import os
import heapq

class RouteRunner(Node):
    def __init__(self):
        super().__init__('route_runner')
        
        # Publishers and Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # NEW: Vision system subscriber for obstacle avoidance
        self.vision_sub = self.create_subscription(Point, '/obstacle_alert', self.obstacle_callback, 10)
        self.obstacle_offset_x = 0.0
        self.obstacle_proximity = 0.0
        
        # Load the topological map
        self.map_path = os.path.join(os.environ['HOME'], 'agv_ws', 'src', 'agv_navigation', 'maps', 'test_map.json')
        with open(self.map_path, 'r') as f:
            self.map_data = json.load(f)["graph"]
            
        # Autonomous Routing Variables
        self.final_destination = "4"  # Set to your desired end node
        self.path_plan = []           
        self.path_calculated = False  
        self.current_target_index = 0
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.mission_complete = False

        self.get_logger().info(f"Hybrid Route Runner Active. Waiting for Odom to calculate route to Node {self.final_destination}...")

    def get_yaw_from_quaternion(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def obstacle_callback(self, msg):
        # msg.x = Horizontal offset (-1.0 to 1.0)
        # msg.y = Proximity (Bounding box area in pixels)
        self.obstacle_offset_x = msg.x
        self.obstacle_proximity = msg.y

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

        # 1. Dijkstra Path Calculation (Runs Once)
        if not self.path_calculated:
            start_node = self.find_closest_node(self.current_x, self.current_y)
            self.get_logger().info(f"Auto-Localized at Node {start_node}. Calculating shortest path...")
            
            self.path_plan = self.calculate_dijkstra(start_node, self.final_destination)
            
            if not self.path_plan:
                self.mission_complete = True 
                return
                
            self.get_logger().info(f"DIJKSTRA PATH: {self.path_plan}")
            self.path_calculated = True
            
            if len(self.path_plan) == 1 and self.path_plan[0] == self.final_destination:
                self.get_logger().info("Already at destination!")
                self.mission_complete = True
            return

        # 2. Get current target waypoint
        active_node_id = self.path_plan[self.current_target_index]
        target_x = self.map_data[active_node_id]["x"]
        target_y = self.map_data[active_node_id]["y"]

        # 3. Calculate Global Tracking Errors
        distance_error = math.sqrt((target_x - self.current_x)**2 + (target_y - self.current_y)**2)
        target_angle = math.atan2(target_y - self.current_y, target_x - self.current_x)
        angle_error = target_angle - self.current_yaw
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        twist = Twist()

        # 4. Waypoint Arrival Check
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

        # ==========================================
        # 5. HYBRID P-CONTROLLER (APF + WAYPOINT)
        # ==========================================
        
        # Attractive Force (Pull to target node)
        steering_force = 1.0 * angle_error 
        speed = 0.5 * distance_error 
        
        # Repulsive Force (Push away from Obstacle)
        # Bounding box > 5000 pixels means the obstacle is directly in our path and close
        if self.obstacle_proximity > 5000:
            self.get_logger().warn(f"EVASIVE MANEUVER! Swerving around obstacle!")
            
            # Failsafe: If the obstacle is dead-center (offset near 0), force a left swerve
            # Otherwise, the repulsive force is 0 and we crash.
            effective_offset = self.obstacle_offset_x
            if abs(effective_offset) < 0.1:
                effective_offset = 0.5  # Simulate the box being slightly to the right to force a left turn
                
            # Calculate the push force. Multiplier dictates how aggressively it swerves.
            repulsive_force = -2.5 * effective_offset 
            
            # Combine the global tracking with the local swerve
            steering_force += repulsive_force
            
            # Cap the speed while maneuvering to keep physics stable
            speed = 0.2 
            
        elif abs(angle_error) > 0.17: 
            # Normal driving: If the angle is off by a lot, stop moving forward and just spin to fix heading.
            speed = 0.0

        # Safety clamps to prevent the robot from flipping in Gazebo
        twist.linear.x = max(min(speed, 0.5), 0.0) 
        twist.angular.z = max(min(steering_force, 1.5), -1.5)

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = RouteRunner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()