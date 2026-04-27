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
        
        # Vision system subscriber for obstacle avoidance
        self.vision_sub = self.create_subscription(Point, '/obstacle_alert', self.obstacle_callback, 10)
        self.obstacle_offset_x = 0.0
        self.obstacle_proximity = 0.0
        
        # ==========================================
        # NEW: Evasion State Memory Variables
        # ==========================================
        self.is_evading = False
        self.evasion_cooldown = 0
        self.evasion_steer_cmd = 0.0
        
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
        # 5. STATE-MACHINE HYBRID CONTROLLER
        # ==========================================
        
        # Default: Global Attractive Force (Pull to target node)
        steering_force = 1.0 * angle_error 
        speed = 0.5 * distance_error 

        # --- TRIGGER CHECK ---
        # If we clearly see an obstacle and aren't already evading, trigger the evasion state
        if self.obstacle_proximity > 2000 and not self.is_evading:
            self.is_evading = True
            self.evasion_cooldown = 30 # Lock the turn for 30 loop cycles to clear the box
            
            # Decide which way to turn ONCE, and lock it in.
            effective_offset = self.obstacle_offset_x
            if abs(effective_offset) < 0.2:
                self.evasion_steer_cmd = 1.5  # Dead center? Hard Left.
            else:
                # Steer away from the side it's on
                self.evasion_steer_cmd = -4.0 * effective_offset 

        # --- EXECUTE EVASION ---
        if self.is_evading:
            self.get_logger().info(f"COMMITTED EVASION! Holding turn. Cooldown: {self.evasion_cooldown}")
            steering_force = self.evasion_steer_cmd
            speed = 0.15 # Go slow and steady around the corner
            
            # Decrease cooldown every loop cycle, even if the camera goes blind!
            self.evasion_cooldown -= 1
            if self.evasion_cooldown <= 0:
                self.get_logger().info("Evasion complete. Snapping back to global path.")
                self.is_evading = False # Resume normal driving toward the node!
                    
        # --- NORMAL DRIVING ---
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