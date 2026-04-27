import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
import json
import os

class MapBuilder(Node):
    def __init__(self):
        super().__init__('map_builder')
        self.sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.graph = {}
        self.node_counter = 1
        
        self.last_node_id = None
        self.last_x = None
        self.last_y = None
        
        # Snap Radius: If we are within 0.5m of an existing node, link to it instead of making a new one
        self.snap_radius = 0.5 
        
        self.map_path = os.path.join(os.environ['HOME'], 'agv_ws', 'src', 'agv_navigation', 'maps', 'test_map.json')
        os.makedirs(os.path.dirname(self.map_path), exist_ok=True)
        
        self.get_logger().info("Smart Map Builder Active. Bidirectional Edges & Branching Enabled.")

    def get_yaw_from_quaternion(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def add_bidirectional_edge(self, node_a, node_b):
        # Prevent a node from connecting to itself
        if node_a == node_b: return

        # Get coordinates
        xa, ya = self.graph[node_a]["x"], self.graph[node_a]["y"]
        xb, yb = self.graph[node_b]["x"], self.graph[node_b]["y"]

        distance = round(math.sqrt((xb - xa)**2 + (yb - ya)**2), 2)

        # Forward Direction (A -> B)
        dir_ab = round(math.atan2(yb - ya, xb - xa), 2)
        # Check if edge already exists to prevent duplicates
        if not any(e['to_node'] == node_b for e in self.graph[node_a]["edges"]):
            self.graph[node_a]["edges"].append({"to_node": node_b, "distance_m": distance, "direction_rad": dir_ab})

        # Reverse Direction (B -> A)
        dir_ba = round(math.atan2(ya - yb, xa - xb), 2)
        if not any(e['to_node'] == node_a for e in self.graph[node_b]["edges"]):
            self.graph[node_b]["edges"].append({"to_node": node_a, "distance_m": distance, "direction_rad": dir_ba})

    def odom_callback(self, msg):
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        current_yaw = self.get_yaw_from_quaternion(msg.pose.pose.orientation)

        # 1. Initialize the very first node
        if self.last_node_id is None:
            new_id = str(self.node_counter)
            self.graph[new_id] = {"x": round(current_x, 2), "y": round(current_y, 2), "yaw": round(current_yaw, 2), "edges": []}
            self.last_node_id = new_id
            self.last_x, self.last_y = current_x, current_y
            self.node_counter += 1
            self.save_graph()
            self.get_logger().info(f"Dropped Start Node {new_id}")
            return

        # 2. Check distance from the last node dropped
        distance_from_last = math.sqrt((current_x - self.last_x)**2 + (current_y - self.last_y)**2)

        # 3. If we have traveled 1 meter, it is time to drop or snap!
        if distance_from_last >= 1.0:
            
            # Phase A: INTERSECTION DETECTION (Branching)
            # Look at all existing nodes. Are we near an old one?
            closest_existing_node = None
            for n_id, data in self.graph.items():
                dist_to_existing = math.sqrt((current_x - data["x"])**2 + (current_y - data["y"])**2)
                if dist_to_existing < self.snap_radius and n_id != self.last_node_id:
                    closest_existing_node = n_id
                    break

            # Phase B: SNAP OR CREATE
            if closest_existing_node:
                # We found an intersection! Link the last node to this existing node.
                self.add_bidirectional_edge(self.last_node_id, closest_existing_node)
                self.get_logger().info(f"Loop Closure! Linked {self.last_node_id} to existing Node {closest_existing_node}")
                self.last_node_id = closest_existing_node
                self.last_x, self.last_y = self.graph[closest_existing_node]["x"], self.graph[closest_existing_node]["y"]
            
            else:
                # We are in unexplored territory. Drop a new node.
                new_id = str(self.node_counter)
                self.graph[new_id] = {"x": round(current_x, 2), "y": round(current_y, 2), "yaw": round(current_yaw, 2), "edges": []}
                self.add_bidirectional_edge(self.last_node_id, new_id)
                self.get_logger().info(f"Dropped Node {new_id} at X:{current_x:.2f}, Y:{current_y:.2f}")
                self.last_node_id = new_id
                self.last_x, self.last_y = current_x, current_y
                self.node_counter += 1
            
            self.save_graph()

    def save_graph(self):
        with open(self.map_path, 'w') as f:
            json.dump({"graph": self.graph}, f, indent=4)

def main(args=None):
    rclpy.init(args=args)
    node = MapBuilder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()