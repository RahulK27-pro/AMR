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

        # Load the graph — check active map, warehouse_graph, or map candidates
        self.declare_parameter('map_name', 'warehouse_01')
        map_name_param = self.get_parameter('map_name').get_parameter_value().string_value or 'warehouse_01'

        candidates = []
        try:
            pkg_share = get_package_share_directory('agv_description')
            candidates.extend([
                os.path.join(pkg_share, 'maps', f'{map_name_param}_graph.json'),
                os.path.join(pkg_share, 'maps', 'warehouse_graph.json'),
                os.path.join(pkg_share, 'maps', 'warehouse_01_graph.json'),
                os.path.join(pkg_share, 'maps', 'warehouse_map_graph.json'),
            ])
        except Exception:
            pass

        ws_maps = os.path.join(os.environ.get('HOME', ''), 'AMR', 'AMR-main', 'src', 'agv_description', 'maps')
        candidates.extend([
            os.path.join(ws_maps, f'{map_name_param}_graph.json'),
            os.path.join(ws_maps, 'warehouse_graph.json'),
            os.path.join(ws_maps, 'warehouse_01_graph.json'),
            os.path.join(ws_maps, 'warehouse_map_graph.json'),
        ])

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

        # Deduplicate undirected edges to halve line vertex count
        seen_edges = set()
        self.unique_edge_pairs = []

        for edge in raw_data.get("edges", []):
            from_node = edge["from"]
            to_node = edge["to"]
            if from_node in self.map_data and to_node in self.map_data:
                pair = tuple(sorted([from_node, to_node]))
                if pair not in seen_edges:
                    seen_edges.add(pair)
                    self.unique_edge_pairs.append((from_node, to_node))

        # Pre-build static marker messages (high-efficiency SPHERE_LIST and LINE_LIST)
        self._build_static_graph_markers()

        # Publish static graph markers every 2.0 seconds
        self.timer = self.create_timer(2.0, self.publish_static_graph)
        self.get_logger().info(
            f"Graph Visualizer Active ({len(self.map_data)} nodes, {len(self.unique_edge_pairs)} edges). "
            f"Publishing batch SPHERE_LIST and LINE_LIST."
        )

    def _build_static_graph_markers(self):
        """Pre-generate efficient batched SPHERE_LIST and LINE_LIST markers."""
        # 1. Node spheres — single SPHERE_LIST marker for all nodes
        self.nodes_marker = Marker()
        self.nodes_marker.header.frame_id = "map"
        self.nodes_marker.ns = "graph_nodes"
        self.nodes_marker.id = 0
        self.nodes_marker.type = Marker.SPHERE_LIST
        self.nodes_marker.action = Marker.ADD
        self.nodes_marker.scale.x = 0.12
        self.nodes_marker.scale.y = 0.12
        self.nodes_marker.scale.z = 0.12
        self.nodes_marker.color.r = 1.0
        self.nodes_marker.color.g = 0.85
        self.nodes_marker.color.b = 0.1
        self.nodes_marker.color.a = 0.85

        for data in self.map_data.values():
            p = Point()
            p.x = float(data["x"])
            p.y = float(data["y"])
            p.z = 0.05
            self.nodes_marker.points.append(p)

        # 2. Edge lines — single LINE_LIST marker for all unique edges
        self.edges_marker = Marker()
        self.edges_marker.header.frame_id = "map"
        self.edges_marker.ns = "graph_edges"
        self.edges_marker.id = 1
        self.edges_marker.type = Marker.LINE_LIST
        self.edges_marker.action = Marker.ADD
        self.edges_marker.scale.x = 0.02
        self.edges_marker.color.r = 0.0
        self.edges_marker.color.g = 0.75
        self.edges_marker.color.b = 0.95
        self.edges_marker.color.a = 0.45

        for u, v in self.unique_edge_pairs:
            p1 = Point()
            p1.x = float(self.map_data[u]["x"])
            p1.y = float(self.map_data[u]["y"])
            p1.z = 0.02

            p2 = Point()
            p2.x = float(self.map_data[v]["x"])
            p2.y = float(self.map_data[v]["y"])
            p2.z = 0.02

            self.edges_marker.points.append(p1)
            self.edges_marker.points.append(p2)

    def path_callback(self, msg):
        """Update and publish the active dense path to display in RViz."""
        self.active_path_points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.publish_active_path()

    def publish_static_graph(self):
        """Publish the static graph nodes and edges."""
        now = self.get_clock().now().to_msg()
        self.nodes_marker.header.stamp = now
        self.edges_marker.header.stamp = now

        msg = MarkerArray()
        msg.markers.append(self.nodes_marker)
        msg.markers.append(self.edges_marker)
        self.marker_pub.publish(msg)

    def publish_active_path(self):
        """Publish active planned path and waypoints efficiently."""
        if not self.active_path_points:
            # If path cleared, delete active path markers
            msg = MarkerArray()
            del_line = Marker()
            del_line.header.frame_id = "map"
            del_line.ns = "active_path_line"
            del_line.id = 10
            del_line.action = Marker.DELETE
            msg.markers.append(del_line)

            del_dots = Marker()
            del_dots.header.frame_id = "map"
            del_dots.ns = "active_path_dots"
            del_dots.id = 11
            del_dots.action = Marker.DELETE
            msg.markers.append(del_dots)

            self.marker_pub.publish(msg)
            return

        now = self.get_clock().now().to_msg()
        msg = MarkerArray()

        # 1. Magenta path ribbon (LINE_STRIP)
        path_line = Marker()
        path_line.header.frame_id = "map"
        path_line.header.stamp = now
        path_line.ns = "active_path_line"
        path_line.id = 10
        path_line.type = Marker.LINE_STRIP
        path_line.action = Marker.ADD
        path_line.scale.x = 0.05
        path_line.color.r = 1.0
        path_line.color.g = 0.0
        path_line.color.b = 1.0  # Vibrant magenta
        path_line.color.a = 0.95

        for px, py in self.active_path_points:
            p = Point()
            p.x = px
            p.y = py
            p.z = 0.08
            path_line.points.append(p)
        msg.markers.append(path_line)

        # 2. Orange waypoint dots (SPHERE_LIST)
        dots_marker = Marker()
        dots_marker.header.frame_id = "map"
        dots_marker.header.stamp = now
        dots_marker.ns = "active_path_dots"
        dots_marker.id = 11
        dots_marker.type = Marker.SPHERE_LIST
        dots_marker.action = Marker.ADD
        dots_marker.scale.x = 0.08
        dots_marker.scale.y = 0.08
        dots_marker.scale.z = 0.08
        dots_marker.color.r = 1.0
        dots_marker.color.g = 0.5
        dots_marker.color.b = 0.0  # Orange
        dots_marker.color.a = 0.9

        for px, py in self.active_path_points[::3]:
            p = Point()
            p.x = px
            p.y = py
            p.z = 0.10
            dots_marker.points.append(p)
        msg.markers.append(dots_marker)

        self.marker_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GraphVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()