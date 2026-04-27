import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import json
import os

class GraphVisualizer(Node):
    def __init__(self):
        super().__init__('graph_visualizer')
        self.marker_pub = self.create_publisher(MarkerArray, '/agv_graph_markers', 10)
        
        # Load the map
        self.map_path = os.path.join(os.environ['HOME'], 'agv_ws', 'src', 'agv_navigation', 'maps', 'test_map.json')
        with open(self.map_path, 'r') as f:
            self.map_data = json.load(f)["graph"]
            
        self.timer = self.create_timer(1.0, self.publish_markers) # Publish every 1 second
        self.get_logger().info("Graph Visualizer Active. Open RViz to see the map.")

    def publish_markers(self):
        marker_array = MarkerArray()
        marker_id = 0

        for node_id, data in self.map_data.items():
            # 1. Draw the Node (A glowing yellow sphere)
            node_marker = Marker()
            node_marker.header.frame_id = "odom"
            node_marker.header.stamp = self.get_clock().now().to_msg()
            node_marker.ns = "nodes"
            node_marker.id = marker_id
            node_marker.type = Marker.SPHERE
            node_marker.action = Marker.ADD
            node_marker.pose.position.x = data["x"]
            node_marker.pose.position.y = data["y"]
            node_marker.pose.position.z = 0.05 # Slightly above ground
            node_marker.scale.x = 0.15
            node_marker.scale.y = 0.15
            node_marker.scale.z = 0.15
            node_marker.color.r = 1.0
            node_marker.color.g = 1.0
            node_marker.color.b = 0.0
            node_marker.color.a = 1.0
            marker_array.markers.append(node_marker)
            marker_id += 1

            # 2. Draw the ID text above the sphere
            text_marker = Marker()
            text_marker.header.frame_id = "odom"
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "labels"
            text_marker.id = marker_id
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = data["x"]
            text_marker.pose.position.y = data["y"]
            text_marker.pose.position.z = 0.3 # Float above the sphere
            text_marker.scale.z = 0.2 # Text size
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = f"Node {node_id}"
            marker_array.markers.append(text_marker)
            marker_id += 1

            # 3. Draw the Edges (Lines connecting nodes)
            for edge in data["edges"]:
                target_id = edge["to_node"]
                if target_id in self.map_data:
                    line_marker = Marker()
                    line_marker.header.frame_id = "odom"
                    line_marker.header.stamp = self.get_clock().now().to_msg()
                    line_marker.ns = "edges"
                    line_marker.id = marker_id
                    line_marker.type = Marker.LINE_STRIP
                    line_marker.action = Marker.ADD
                    line_marker.scale.x = 0.02 # Line thickness
                    line_marker.color.r = 0.0
                    line_marker.color.g = 1.0
                    line_marker.color.b = 1.0 # Cyan lines
                    line_marker.color.a = 0.8
                    
                    # Start point
                    p_start = Point()
                    p_start.x = data["x"]
                    p_start.y = data["y"]
                    p_start.z = 0.05
                    
                    # End point
                    p_end = Point()
                    p_end.x = self.map_data[target_id]["x"]
                    p_end.y = self.map_data[target_id]["y"]
                    p_end.z = 0.05
                    
                    line_marker.points = [p_start, p_end]
                    marker_array.markers.append(line_marker)
                    marker_id += 1

        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = GraphVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()