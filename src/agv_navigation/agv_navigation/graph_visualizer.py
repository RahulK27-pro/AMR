import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
import json
import os
import math

class GraphVisualizer(Node):
    def __init__(self):
        super().__init__('graph_visualizer')
        self.marker_pub = self.create_publisher(MarkerArray, '/agv_graph_markers', 10)

        # Subscribe to active dense path published by route_runner
        self.path_sub = self.create_subscription(Path, '/agv_dense_path', self.path_callback, 10)
        self.active_path_points = []  # list of (x, y)

        # Load the graph — use installed share path (portable)
        try:
            pkg_share = get_package_share_directory('agv_description')
            self.map_path = os.path.join(pkg_share, 'maps', 'warehouse_graph.json')
        except Exception:
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
            if from_node in self.map_data:
                self.map_data[from_node]["edges"].append({"to_node": to_node})

        self.prev_marker_count = 0  # Track marker count for cleanup
        self.timer = self.create_timer(0.5, self.publish_markers)
        self.get_logger().info("Graph Visualizer Active. Open RViz and subscribe to /agv_graph_markers.")

    def path_callback(self, msg):
        """Update the active dense path to display in RViz."""
        self.active_path_points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def publish_markers(self):
        marker_array = MarkerArray()
        marker_id = 0
        stamp = self.get_clock().now().to_msg()

        # Delete all stale markers from previous publish to prevent ghosts
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        marker_array.markers.append(delete_all)
        marker_id += 1

        for node_id, data in self.map_data.items():
            # 1. Node sphere (yellow)
            node_marker = Marker()
            node_marker.header.frame_id = "map"
            node_marker.header.stamp = stamp
            node_marker.ns = "nodes"
            node_marker.id = marker_id
            node_marker.type = Marker.SPHERE
            node_marker.action = Marker.ADD
            node_marker.pose.position.x = data["x"]
            node_marker.pose.position.y = data["y"]
            node_marker.pose.position.z = 0.05
            node_marker.scale.x = 0.15
            node_marker.scale.y = 0.15
            node_marker.scale.z = 0.15
            node_marker.color.r = 1.0
            node_marker.color.g = 1.0
            node_marker.color.b = 0.0
            node_marker.color.a = 1.0
            marker_array.markers.append(node_marker)
            marker_id += 1

            # 2. Node label (white text)
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = stamp
            text_marker.ns = "labels"
            text_marker.id = marker_id
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = data["x"]
            text_marker.pose.position.y = data["y"]
            text_marker.pose.position.z = 0.3
            text_marker.scale.z = 0.18
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = f"{node_id}"
            marker_array.markers.append(text_marker)
            marker_id += 1

            # 3. Edge lines (cyan)
            for edge in data["edges"]:
                target_id = edge["to_node"]
                if target_id in self.map_data:
                    line_marker = Marker()
                    line_marker.header.frame_id = "map"
                    line_marker.header.stamp = stamp
                    line_marker.ns = "edges"
                    line_marker.id = marker_id
                    line_marker.type = Marker.LINE_STRIP
                    line_marker.action = Marker.ADD
                    line_marker.scale.x = 0.02
                    line_marker.color.r = 0.0
                    line_marker.color.g = 0.8
                    line_marker.color.b = 0.8
                    line_marker.color.a = 0.5

                    p_start = Point()
                    p_start.x = data["x"]
                    p_start.y = data["y"]
                    p_start.z = 0.02

                    p_end = Point()
                    p_end.x = self.map_data[target_id]["x"]
                    p_end.y = self.map_data[target_id]["y"]
                    p_end.z = 0.02

                    line_marker.points = [p_start, p_end]
                    marker_array.markers.append(line_marker)
                    marker_id += 1

        # 4. Dense active path (bright magenta line + dots)
        if len(self.active_path_points) >= 2:
            path_line = Marker()
            path_line.header.frame_id = "map"
            path_line.header.stamp = stamp
            path_line.ns = "active_path_line"
            path_line.id = marker_id
            path_line.type = Marker.LINE_STRIP
            path_line.action = Marker.ADD
            path_line.scale.x = 0.05
            path_line.color.r = 1.0
            path_line.color.g = 0.0
            path_line.color.b = 1.0  # Magenta
            path_line.color.a = 0.9
            for (px, py) in self.active_path_points:
                p = Point()
                p.x = px
                p.y = py
                p.z = 0.08
                path_line.points.append(p)
            marker_array.markers.append(path_line)
            marker_id += 1

        # 5. Waypoint dots along the dense path (small orange spheres)
        for i, (px, py) in enumerate(self.active_path_points[::3]):  # every 3rd for clarity
            wp_marker = Marker()
            wp_marker.header.frame_id = "map"
            wp_marker.header.stamp = stamp
            wp_marker.ns = "active_path_dots"
            wp_marker.id = marker_id
            wp_marker.type = Marker.SPHERE
            wp_marker.action = Marker.ADD
            wp_marker.pose.position.x = px
            wp_marker.pose.position.y = py
            wp_marker.pose.position.z = 0.1
            wp_marker.scale.x = 0.08
            wp_marker.scale.y = 0.08
            wp_marker.scale.z = 0.08
            wp_marker.color.r = 1.0
            wp_marker.color.g = 0.5
            wp_marker.color.b = 0.0  # Orange
            wp_marker.color.a = 0.9
            marker_array.markers.append(wp_marker)
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