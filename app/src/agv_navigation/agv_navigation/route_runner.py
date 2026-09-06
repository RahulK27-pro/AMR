import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String, Bool
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

        # ============================================================
        # ROS PUBLISHERS
        # ============================================================

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.path_pub = self.create_publisher(
            Path,
            '/agv_dense_path',
            10
        )

        # App bridge: publish current AGV state
        self.state_pub = self.create_publisher(
            String,
            '/agv_state',
            10
        )

        # ============================================================
        # ROS SUBSCRIBERS
        # ============================================================

        # EKF-fused odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10
        )

        # RViz goal topics
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        self.goal_sub_alt = self.create_subscription(
            PoseStamped,
            '/goal',
            self.goal_callback,
            10
        )

        self.goal_sub_mb = self.create_subscription(
            PoseStamped,
            '/move_base_simple/goal',
            self.goal_callback,
            10
        )

        # Named goal from app / other ROS node
        self.named_goal_sub = self.create_subscription(
            String,
            '/named_goal',
            self.named_goal_callback,
            10
        )

        # LiDAR
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # App emergency stop
        self.estop_sub = self.create_subscription(
            Bool,
            '/agv_estop',
            self.estop_callback,
            10
        )

        # Publish AGV state at 5 Hz
        self.state_timer = self.create_timer(
            0.2,
            self.publish_state
        )

        # ============================================================
        # TF
        # ============================================================

        # TF Buffer and Listener for map -> base_link
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        self.using_map_tf = False

        # ============================================================
        # LOAD MAP
        # ============================================================

        self.map_path = os.path.join(
            os.environ['HOME'],
            'agv',
            'src',
            'AMR',
            'src',
            'agv_description',
            'maps',
            'warehouse_graph.json'
        )

        # Fallback path
        if not os.path.exists(self.map_path):
            self.map_path = os.path.join(
                os.environ['HOME'],
                'agv_ws',
                'src',
                'agv_description',
                'maps',
                'warehouse_graph.json'
            )

        if not os.path.exists(self.map_path):
            self.get_logger().error(
                f"Map file not found: {self.map_path}"
            )
            raise FileNotFoundError(self.map_path)

        with open(self.map_path, 'r') as f:
            raw_data = json.load(f)

        # ============================================================
        # LOAD NAMED LOCATIONS
        # ============================================================

        self.locations_path = os.path.join(
            os.environ['HOME'],
            'agv',
            'src',
            'AMR',
            'src',
            'agv_navigation',
            'config',
            'locations.json'
        )

        if not os.path.exists(self.locations_path):
            self.get_logger().warn(
                f"Named locations file not found: "
                f"{self.locations_path}"
            )
            self.locations = {}
        else:
            with open(self.locations_path, 'r') as f:
                self.locations = json.load(f)

        self.get_logger().info(
            f"Loaded {len(self.locations)} named locations: "
            f"{list(self.locations.keys())}"
        )

        # ============================================================
        # BUILD GRAPH
        # ============================================================

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

        # ============================================================
        # MAKE GRAPH BIDIRECTIONAL
        # ============================================================

        for node_id, data in list(self.map_data.items()):

            for edge in list(data["edges"]):

                target_id = edge["to_node"]
                weight = edge["distance_m"]

                if target_id in self.map_data:

                    target_edges = self.map_data[target_id]["edges"]

                    if not any(
                        e["to_node"] == node_id
                        for e in target_edges
                    ):
                        target_edges.append({
                            "to_node": node_id,
                            "distance_m": weight
                        })

        # ============================================================
        # NAVIGATION STATE
        # ============================================================

        # IDLE, PLANNING, NAVIGATING
        self.state = "IDLE"

        # Dense path:
        # [(x, y, theta), ...]
        self.path_plan = []

        # Current waypoint index
        self.current_target_index = 0

        # Raw Dijkstra path
        self.node_path = []

        # ============================================================
        # ROBOT POSE
        # ============================================================

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.odom_ready = False

        # ============================================================
        # OBSTACLES
        # ============================================================

        self.obstacles = np.array([])

        # ============================================================
        # MPPI PARAMETERS
        # ============================================================

        self.v_max = 0.8

        # Forward-only
        self.v_min = 0.0

        # Maximum angular velocity
        self.w_max = 1.8

        # Control timestep
        self.dt = 0.1

        # Prediction horizon
        self.horizon = 15

        # Number of sampled trajectories
        self.num_samples = 80

        # ============================================================
        # MPPI NOISE
        # ============================================================

        self.noise_v = 0.3
        self.noise_w = 0.5

        # Lower = sharper trajectory selection
        self.lambda_weight = 0.5

        # ============================================================
        # MPPI COST WEIGHTS
        # ============================================================

        self.w_dist = 4.0
        self.w_heading = 3.0

        # Collision should dominate
        self.w_collision = 5000.0

        # Robot clearance
        self.collision_radius = 0.30

        # ============================================================
        # CONTROL TIMER
        # ============================================================

        self.control_timer = self.create_timer(
            self.dt,
            self.control_loop
        )

        self.get_logger().info(
            "Route Runner Active. "
            "Waiting for Goal in RViz (2D Goal Pose)..."
        )

    # ================================================================
    # QUATERNION -> YAW
    # ================================================================

    def get_yaw_from_quaternion(self, q):

        siny_cosp = 2 * (
            q.w * q.z +
            q.x * q.y
        )

        cosy_cosp = 1 - 2 * (
            q.y * q.y +
            q.z * q.z
        )

        return math.atan2(
            siny_cosp,
            cosy_cosp
        )

    # ================================================================
    # FIND CLOSEST GRAPH NODE
    # ================================================================

    def find_closest_node(self, x, y):

        closest_node = None
        min_dist = float('inf')

        for node_id, data in self.map_data.items():

            dist = math.sqrt(
                (x - data["x"]) ** 2 +
                (y - data["y"]) ** 2
            )

            if dist < min_dist:
                min_dist = dist
                closest_node = node_id

        return closest_node

    # ================================================================
    # DIJKSTRA
    # ================================================================

    def calculate_dijkstra(
        self,
        start_node,
        target_node
    ):

        distances = {
            node: float('infinity')
            for node in self.map_data
        }

        distances[start_node] = 0

        previous_nodes = {
            node: None
            for node in self.map_data
        }

        priority_queue = [
            (0, start_node)
        ]

        while priority_queue:

            current_distance, current_node = heapq.heappop(
                priority_queue
            )

            if current_node == target_node:
                break

            if current_distance > distances[current_node]:
                continue

            for edge in self.map_data[current_node]["edges"]:

                neighbor = edge["to_node"]
                weight = edge["distance_m"]

                distance = (
                    current_distance +
                    weight
                )

                if distance < distances[neighbor]:

                    distances[neighbor] = distance

                    previous_nodes[neighbor] = current_node

                    heapq.heappush(
                        priority_queue,
                        (distance, neighbor)
                    )

        # Reconstruct path
        path = []

        current = target_node

        while current is not None:

            path.append(current)

            current = previous_nodes[current]

        path.reverse()

        if (
            not path or
            path[0] != start_node
        ):

            self.get_logger().error(
                f"NO VALID PATH FOUND FROM "
                f"{start_node} TO {target_node}!"
            )

            return []

        return path

    # ================================================================
    # UPDATE ROBOT POSE
    # ================================================================

    def update_robot_pose(self):

        try:

            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )

            self.current_x = (
                transform.transform.translation.x
            )

            self.current_y = (
                transform.transform.translation.y
            )

            self.current_yaw = (
                self.get_yaw_from_quaternion(
                    transform.transform.rotation
                )
            )

            if not self.using_map_tf:

                self.get_logger().info(
                    "Localization Active! "
                    "Received map -> base_link TF transform."
                )

                self.using_map_tf = True

            self.odom_ready = True

            return True

        except TransformException:

            if self.using_map_tf:

                self.get_logger().warn(
                    "AMCL TF lost! "
                    "Falling back to /odometry/filtered."
                )

                self.using_map_tf = False

            return self.odom_ready

    # ================================================================
    # ODOM CALLBACK
    # ================================================================

    def odom_callback(self, msg):

        # Use EKF odometry when map TF isn't available
        if not self.using_map_tf:

            self.current_x = (
                msg.pose.pose.position.x
            )

            self.current_y = (
                msg.pose.pose.position.y
            )

            self.current_yaw = (
                self.get_yaw_from_quaternion(
                    msg.pose.pose.orientation
                )
            )

            self.odom_ready = True

    # ================================================================
    # LIDAR CALLBACK
    # ================================================================

    def scan_callback(self, msg):

        self.update_robot_pose()

        ranges = np.array(
            msg.ranges
        )

        angles = (
            msg.angle_min +
            np.arange(len(ranges)) *
            msg.angle_increment
        )

        # Filter invalid ranges
        valid = (
            (ranges > msg.range_min) &
            (ranges < msg.range_max)
        )

        ranges = ranges[valid]
        angles = angles[valid]

        if len(ranges) == 0:

            self.obstacles = np.array([])

            return

        # Downsample LiDAR
        ranges = ranges[::3]
        angles = angles[::3]

        # ------------------------------------------------------------
        # Convert LiDAR points from robot frame to map frame
        # ------------------------------------------------------------

        ox_local = (
            ranges *
            np.cos(angles)
        )

        oy_local = (
            ranges *
            np.sin(angles)
        )

        cos_yaw = np.cos(
            self.current_yaw
        )

        sin_yaw = np.sin(
            self.current_yaw
        )

        ox_global = (
            self.current_x +
            ox_local * cos_yaw -
            oy_local * sin_yaw
        )

        oy_global = (
            self.current_y +
            ox_local * sin_yaw +
            oy_local * cos_yaw
        )

        self.obstacles = np.column_stack(
            (
                ox_global,
                oy_global
            )
        )

    # ================================================================
    # DENSIFY PATH
    # ================================================================

    def densify_path(
        self,
        node_path,
        step_m=0.30
    ):

        """
        Interpolate dense (x, y, theta)
        waypoints every step_m meters.
        """

        dense = []

        for i in range(
            len(node_path)
        ):

            nx = self.map_data[
                node_path[i]
            ]["x"]

            ny = self.map_data[
                node_path[i]
            ]["y"]

            # --------------------------------------------------------
            # First node
            # --------------------------------------------------------

            if i == 0:

                if len(node_path) > 1:

                    nx2 = self.map_data[
                        node_path[1]
                    ]["x"]

                    ny2 = self.map_data[
                        node_path[1]
                    ]["y"]

                    theta0 = math.atan2(
                        ny2 - ny,
                        nx2 - nx
                    )

                else:

                    theta0 = self.current_yaw

                dense.append(
                    (
                        nx,
                        ny,
                        theta0
                    )
                )

                continue

            # --------------------------------------------------------
            # Segment
            # --------------------------------------------------------

            px = self.map_data[
                node_path[i - 1]
            ]["x"]

            py = self.map_data[
                node_path[i - 1]
            ]["y"]

            seg_theta = math.atan2(
                ny - py,
                nx - px
            )

            seg_len = math.sqrt(
                (nx - px) ** 2 +
                (ny - py) ** 2
            )

            num_steps = max(
                1,
                int(seg_len / step_m)
            )

            for k in range(
                1,
                num_steps + 1
            ):

                t = k / num_steps

                ix = (
                    px +
                    t * (nx - px)
                )

                iy = (
                    py +
                    t * (ny - py)
                )

                dense.append(
                    (
                        ix,
                        iy,
                        seg_theta
                    )
                )

        # ------------------------------------------------------------
        # Smooth waypoint heading
        # ------------------------------------------------------------

        for i in range(
            len(dense) - 1
        ):

            x1, y1, _ = dense[i]
            x2, y2, _ = dense[i + 1]

            dense[i] = (
                x1,
                y1,
                math.atan2(
                    y2 - y1,
                    x2 - x1
                )
            )

        return dense

    # ================================================================
    # NAMED GOAL CALLBACK
    # ================================================================

    def named_goal_callback(self, msg):

        name = msg.data.strip().upper()

        if name not in self.locations:

            self.get_logger().error(
                f"Unknown named goal '{name}'. "
                f"Available: "
                f"{list(self.locations.keys())}"
            )

            return

        location = self.locations[name]

        goal_msg = PoseStamped()

        goal_msg.header.frame_id = "map"

        goal_msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        goal_msg.pose.position.x = (
            location["x"]
        )

        goal_msg.pose.position.y = (
            location["y"]
        )

        goal_msg.pose.orientation.w = 1.0

        self.get_logger().info(
            f"Named goal received: {name} -> "
            f"({location['x']:.3f}, "
            f"{location['y']:.3f})"
        )

        self.goal_callback(
            goal_msg
        )

    # ================================================================
    # GOAL CALLBACK
    # ================================================================

    def goal_callback(self, msg):

        self.update_robot_pose()

        if not self.odom_ready:

            self.get_logger().warn(
                "Waiting for localization/odometry "
                "before accepting goals."
            )

            return

        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y

        self.state = "PLANNING"

        self.get_logger().info(
            f"Received RViz Goal: "
            f"({goal_x:.2f}, {goal_y:.2f})"
        )

        start_node = self.find_closest_node(
            self.current_x,
            self.current_y
        )

        target_node = self.find_closest_node(
            goal_x,
            goal_y
        )

        self.get_logger().info(
            f"Snapping to nodes: "
            f"Start={start_node}, "
            f"Target={target_node}"
        )

        self.node_path = (
            self.calculate_dijkstra(
                start_node,
                target_node
            )
        )

        if not self.node_path:

            self.state = "IDLE"

            return

        # Densify path
        self.path_plan = (
            self.densify_path(
                self.node_path,
                step_m=0.30
            )
        )

        self.get_logger().info(
            f"DIJKSTRA PATH: "
            f"{self.node_path} "
            f"→ Densified to "
            f"{len(self.path_plan)} waypoints"
        )

        self.current_target_index = (
            1
            if len(self.path_plan) > 1
            else 0
        )

        self.state = "NAVIGATING"

        # ------------------------------------------------------------
        # Publish path to RViz
        # ------------------------------------------------------------

        ros_path = Path()

        ros_path.header.frame_id = "map"

        ros_path.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        for (
            px,
            py,
            ptheta
        ) in self.path_plan:

            pose = PoseStamped()

            pose.header.frame_id = "map"

            pose.pose.position.x = px
            pose.pose.position.y = py

            # theta -> quaternion
            pose.pose.orientation.z = (
                math.sin(ptheta / 2.0)
            )

            pose.pose.orientation.w = (
                math.cos(ptheta / 2.0)
            )

            ros_path.poses.append(
                pose
            )

        self.path_pub.publish(
            ros_path
        )

    # ================================================================
    # LOOKAHEAD TARGET
    # ================================================================

    def get_lookahead_target(
        self,
        lookahead_dist=1.0
    ):

        """
        Find a target point along the path
        using arc-length rather than Euclidean
        distance.
        """

        min_dist = float('inf')

        closest_idx = (
            self.current_target_index
        )

        search_window = min(
            len(self.path_plan),
            self.current_target_index + 20
        )

        # ------------------------------------------------------------
        # Find closest path point
        # ------------------------------------------------------------

        for i in range(
            self.current_target_index,
            search_window
        ):

            px, py, _ = (
                self.path_plan[i]
            )

            dist = math.sqrt(
                (px - self.current_x) ** 2 +
                (py - self.current_y) ** 2
            )

            if dist < min_dist:

                min_dist = dist
                closest_idx = i

        self.current_target_index = max(
            self.current_target_index,
            closest_idx
        )

        # ------------------------------------------------------------
        # Walk forward along path by arc length
        # ------------------------------------------------------------

        arc_length = 0.0

        target_idx = closest_idx

        for i in range(
            closest_idx,
            len(self.path_plan) - 1
        ):

            x1, y1, _ = (
                self.path_plan[i]
            )

            x2, y2, _ = (
                self.path_plan[i + 1]
            )

            arc_length += math.sqrt(
                (x2 - x1) ** 2 +
                (y2 - y1) ** 2
            )

            if arc_length >= lookahead_dist:

                target_idx = i + 1

                break

        else:

            target_idx = (
                len(self.path_plan) - 1
            )

        return (
            self.path_plan[target_idx],
            target_idx
        )

    # ================================================================
    # APP STATE PUBLISHER
    # ================================================================

    def publish_state(self):

        msg = String()

        msg.data = self.state

        self.state_pub.publish(
            msg
        )

    # ================================================================
    # EMERGENCY STOP
    # ================================================================

    def estop_callback(self, msg):

        if msg.data:

            self.get_logger().warn(
                "E-STOP received from app "
                "— halting navigation."
            )

            self.state = "IDLE"

            self.path_plan = []

            self.node_path = []

            self.current_target_index = 0

            # Immediately stop robot
            self.cmd_pub.publish(
                Twist()
            )

    # ================================================================
    # MPPI CONTROL LOOP
    # ================================================================

    def control_loop(self):

        self.update_robot_pose()

        if self.state != "NAVIGATING":

            return

        if not self.path_plan:

            self.state = "IDLE"

            self.cmd_pub.publish(
                Twist()
            )

            return

        # ------------------------------------------------------------
        # Check final destination
        # ------------------------------------------------------------

        final_x, final_y, _ = (
            self.path_plan[-1]
        )

        dist_to_final = math.sqrt(
            (final_x - self.current_x) ** 2 +
            (final_y - self.current_y) ** 2
        )

        if dist_to_final < 0.30:

            self.get_logger().info(
                "FINAL DESTINATION REACHED! "
                "Mission Complete."
            )

            self.cmd_pub.publish(
                Twist()
            )

            self.state = "IDLE"

            return

        # ------------------------------------------------------------
        # Lookahead target
        # ------------------------------------------------------------

        (
            target_x,
            target_y,
            target_theta
        ), target_idx = (
            self.get_lookahead_target(
                lookahead_dist=1.2
            )
        )

        # ============================================================
        # MPPI
        # ============================================================

        # ------------------------------------------------------------
        # 1. Sample control sequences
        # ------------------------------------------------------------

        v_seq = np.random.normal(
            0.2,
            self.noise_v,
            (
                self.num_samples,
                self.horizon
            )
        )

        w_seq = np.random.normal(
            0.0,
            self.noise_w,
            (
                self.num_samples,
                self.horizon
            )
        )

        # Clip velocity
        v_seq = np.clip(
            v_seq,
            self.v_min,
            self.v_max
        )

        # Clip angular velocity
        w_seq = np.clip(
            w_seq,
            -self.w_max,
            self.w_max
        )

        # ------------------------------------------------------------
        # 2. Rollout
        # ------------------------------------------------------------

        x_rollout = np.full(
            (
                self.num_samples,
                self.horizon
            ),
            self.current_x
        )

        y_rollout = np.full(
            (
                self.num_samples,
                self.horizon
            ),
            self.current_y
        )

        yaw_rollout = np.full(
            (
                self.num_samples,
                self.horizon
            ),
            self.current_yaw
        )

        for t in range(
            1,
            self.horizon
        ):

            yaw_rollout[:, t] = (
                yaw_rollout[:, t - 1] +
                w_seq[:, t - 1] *
                self.dt
            )

            x_rollout[:, t] = (
                x_rollout[:, t - 1] +
                v_seq[:, t - 1] *
                np.cos(
                    yaw_rollout[:, t - 1]
                ) *
                self.dt
            )

            y_rollout[:, t] = (
                y_rollout[:, t - 1] +
                v_seq[:, t - 1] *
                np.sin(
                    yaw_rollout[:, t - 1]
                ) *
                self.dt
            )

        # ------------------------------------------------------------
        # 3. Cost evaluation
        # ------------------------------------------------------------

        costs = np.zeros(
            self.num_samples
        )

        # ------------------------------------------------------------
        # Terminal distance
        # ------------------------------------------------------------

        terminal_dists = np.sqrt(
            (
                x_rollout[:, -1] -
                target_x
            ) ** 2 +
            (
                y_rollout[:, -1] -
                target_y
            ) ** 2
        )

        costs += (
            self.w_dist *
            terminal_dists
        )

        # ------------------------------------------------------------
        # Heading cost
        # ------------------------------------------------------------

        for t in range(
            self.horizon
        ):

            heading_error = np.arctan2(
                np.sin(
                    target_theta -
                    yaw_rollout[:, t]
                ),
                np.cos(
                    target_theta -
                    yaw_rollout[:, t]
                )
            )

            costs += (
                self.w_heading /
                self.horizon
            ) * np.abs(
                heading_error
            )

        # ------------------------------------------------------------
        # Cross-track error
        # ------------------------------------------------------------

        local_path_full = (
            self.path_plan[
                self.current_target_index:
                target_idx + 2
            ]
        )

        local_path = np.array(
            [
                (p[0], p[1])
                for p in local_path_full
            ]
        )

        if len(local_path) > 0:

            rollout_pts = np.stack(
                (
                    x_rollout,
                    y_rollout
                ),
                axis=-1
            )[:, :, np.newaxis, :]

            path_pts = (
                local_path[
                    np.newaxis,
                    np.newaxis,
                    :,
                    :
                ]
            )

            dists = np.linalg.norm(
                rollout_pts -
                path_pts,
                axis=-1
            )

            min_dists = np.min(
                dists,
                axis=-1
            )

            w_cross_track = 6.0

            costs += (
                w_cross_track *
                np.sum(
                    min_dists,
                    axis=-1
                )
            )

        # ------------------------------------------------------------
        # Collision cost
        # ------------------------------------------------------------

        if len(self.obstacles) > 0:

            for t in range(
                self.horizon
            ):

                pts = np.stack(
                    [
                        x_rollout[:, t],
                        y_rollout[:, t]
                    ],
                    axis=-1
                )[:, np.newaxis, :]

                obs = (
                    self.obstacles[
                        np.newaxis,
                        :,
                        :
                    ]
                )

                obstacle_distances = np.min(
                    np.linalg.norm(
                        pts - obs,
                        axis=-1
                    ),
                    axis=1
                )

                collision_mask = (
                    obstacle_distances <
                    self.collision_radius
                )

                costs[
                    collision_mask
                ] += self.w_collision

        # ------------------------------------------------------------
        # 4. MPPI weighted control
        # ------------------------------------------------------------

        beta = np.min(
            costs
        )

        weights = np.exp(
            -1.0 /
            self.lambda_weight *
            (costs - beta)
        )

        weight_sum = np.sum(
            weights
        )

        if weight_sum <= 1e-12:

            self.get_logger().warn(
                "MPPI weight sum too small. "
                "Stopping robot."
            )

            self.cmd_pub.publish(
                Twist()
            )

            return

        weights = (
            weights /
            weight_sum
        )

        optimal_v = np.sum(
            weights *
            v_seq[:, 0]
        )

        optimal_w = np.sum(
            weights *
            w_seq[:, 0]
        )

        # ------------------------------------------------------------
        # Execute command
        # ------------------------------------------------------------

        twist = Twist()

        twist.linear.x = float(
            optimal_v
        )

        twist.angular.z = float(
            optimal_w
        )

        self.cmd_pub.publish(
            twist
        )


# ====================================================================
# MAIN
# ====================================================================

def main(args=None):

    rclpy.init(
        args=args
    )

    node = RouteRunner()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()
