import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from nav_msgs.msg import Path
import tf2_ros
from tf2_ros import Buffer, TransformListener, TransformException
from ament_index_python.packages import get_package_share_directory

import py_trees
import json
import os


class CheckLocalization(py_trees.behaviour.Behaviour):
    """Condition behavior: verifies AMCL / TF localization is active."""
    def __init__(self, name, node, tf_buffer):
        super().__init__(name=name)
        self.node = node
        self.tf_buffer = tf_buffer

    def update(self):
        try:
            _ = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            return py_trees.common.Status.SUCCESS
        except TransformException:
            return py_trees.common.Status.FAILURE


class CheckGoalQueue(py_trees.behaviour.Behaviour):
    """Condition behavior: checks if an active goal or queued mission exists."""
    def __init__(self, name, node, blackboard):
        super().__init__(name=name)
        self.node = node
        self.blackboard = blackboard

    def update(self):
        if self.blackboard.get("current_goal") is not None or len(self.blackboard.get("goal_queue", [])) > 0:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class PlanTopologicalPath(py_trees.behaviour.Behaviour):
    """Action behavior: dispatches goal to the route planner and verifies path generation."""
    def __init__(self, name, node, blackboard):
        super().__init__(name=name)
        self.node = node
        self.blackboard = blackboard

    def update(self):
        current_goal = self.blackboard.get("current_goal")
        goal_queue = self.blackboard.get("goal_queue", [])

        if current_goal is None and len(goal_queue) > 0:
            next_goal = goal_queue.pop(0)
            self.blackboard.set("current_goal", next_goal)
            self.blackboard.set("goal_queue", goal_queue)
            self.node.get_logger().info(f"BT Orchestrator: Dispatched goal {next_goal} to planner.")

            msg = PoseStamped()
            msg.header.frame_id = "map"
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.pose.position.x = next_goal[0]
            msg.pose.position.y = next_goal[1]
            self.blackboard.get("goal_pub").publish(msg)

        if self.blackboard.get("current_goal") is not None:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class ExecuteMPPINavigation(py_trees.behaviour.Behaviour):
    """Action behavior: monitors MPPI navigation status from route_runner."""
    def __init__(self, name, node, blackboard):
        super().__init__(name=name)
        self.node = node
        self.blackboard = blackboard

    def update(self):
        status = self.blackboard.get("mission_status", "IDLE")

        if status in ["NAVIGATING", "YIELDING", "PLANNING"]:
            return py_trees.common.Status.RUNNING
        elif status in ["MISSION_COMPLETE", "IDLE"]:
            if self.blackboard.get("current_goal") is not None:
                self.node.get_logger().info("BT Orchestrator: Waypoint navigation complete.")
                self.blackboard.set("current_goal", None)
            return py_trees.common.Status.SUCCESS
        else:
            self.node.get_logger().warn(f"BT Orchestrator: Unexpected navigation status '{status}'.")
            return py_trees.common.Status.FAILURE


class BackUpRecovery(py_trees.behaviour.Behaviour):
    """Recovery behavior: reverses 0.3m if trapped in tight dynamic blockage."""
    def __init__(self, name, node, blackboard):
        super().__init__(name=name)
        self.node = node
        self.blackboard = blackboard
        self.start_time = None

    def _now_sec(self):
        return self.node.get_clock().now().nanoseconds / 1e9

    def initialise(self):
        self.start_time = self._now_sec()
        self.node.get_logger().warn("BT Recovery: Executing BackUp behavior (0.3m reverse)...")

    def update(self):
        elapsed = self._now_sec() - self.start_time
        if elapsed < 1.5:
            cmd = Twist()
            cmd.linear.x = -0.15
            self.blackboard.get("cmd_pub").publish(cmd)
            return py_trees.common.Status.RUNNING
        else:
            self.blackboard.get("cmd_pub").publish(Twist())
            self.node.get_logger().info("BT Recovery: BackUp complete.")
            return py_trees.common.Status.SUCCESS


class SpinRecovery(py_trees.behaviour.Behaviour):
    """Recovery behavior: rotates 60 degrees to clear local sensor view."""
    def __init__(self, name, node, blackboard):
        super().__init__(name=name)
        self.node = node
        self.blackboard = blackboard
        self.start_time = None

    def _now_sec(self):
        return self.node.get_clock().now().nanoseconds / 1e9

    def initialise(self):
        self.start_time = self._now_sec()
        self.node.get_logger().warn("BT Recovery: Executing Spin behavior (clear sensor view)...")

    def update(self):
        elapsed = self._now_sec() - self.start_time
        if elapsed < 2.0:
            cmd = Twist()
            cmd.angular.z = 0.5
            self.blackboard.get("cmd_pub").publish(cmd)
            return py_trees.common.Status.RUNNING
        else:
            self.blackboard.get("cmd_pub").publish(Twist())
            self.node.get_logger().info("BT Recovery: Spin complete.")
            return py_trees.common.Status.SUCCESS


class SimpleBlackboard:
    """Lightweight in-memory blackboard container for BT data sharing."""
    def __init__(self):
        self.data = {}

    def set(self, key, value):
        self.data[key] = value

    def get(self, key, default=None):
        return self.data.get(key, default)


class BTManagerNode(Node):
    def __init__(self):
        super().__init__('bt_manager')

        self.blackboard = SimpleBlackboard()

        # Load topological graph for node-ID → (x, y) lookup in sequence_cb
        try:
            pkg_share = get_package_share_directory('agv_description')
            graph_path = os.path.join(pkg_share, 'maps', 'warehouse_graph.json')
        except Exception:
            graph_path = os.path.join(
                os.environ['HOME'], 'AMR', 'AMR-main',
                'src', 'agv_description', 'maps', 'warehouse_graph.json'
            )
        with open(graph_path, 'r') as f:
            raw = json.load(f)
        self.graph_coords = {n['id']: (n['x'], n['y']) for n in raw.get('nodes', [])}
        self.get_logger().info(f"BT Manager: Loaded {len(self.graph_coords)}-node graph for sequence lookup.")

        # Publishers & Subscribers
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.blackboard.set("goal_pub", self.goal_pub)
        self.blackboard.set("cmd_pub", self.cmd_pub)

        self.seq_sub = self.create_subscription(String, '/goal_sequence', self.sequence_cb, 10)
        self.progress_sub = self.create_subscription(String, '/mission_progress', self.progress_cb, 10)

        # TF Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Build Behavior Tree
        self.root = self.create_tree()

        # Timer for ticking behavior tree at 10 Hz
        self.timer = self.create_timer(0.1, self.tick_tree)
        self.get_logger().info("BT Manager Node Active. Behavior Tree Orchestration running at 10 Hz.")

    def sequence_cb(self, msg):
        """Receives JSON node-ID list, resolves (x, y) coords and populates blackboard goal_queue."""
        try:
            node_ids = json.loads(msg.data)
            if not isinstance(node_ids, list):
                self.get_logger().error("BT Manager: /goal_sequence payload must be a JSON array of node IDs.")
                return

            coords = []
            unknown = []
            for nid in node_ids:
                if nid in self.graph_coords:
                    coords.append(self.graph_coords[nid])
                else:
                    unknown.append(nid)

            if unknown:
                self.get_logger().error(
                    f"BT Manager: Unknown node IDs in sequence (skipped): {unknown}"
                )
            if not coords:
                self.get_logger().warn("BT Manager: No valid goals in sequence — mission not started.")
                return

            self.blackboard.set("goal_queue", coords)
            self.blackboard.set("current_goal", None)
            self.blackboard.set("mission_status", "IDLE")
            self.get_logger().info(
                f"BT Manager: Mission loaded — {len(coords)} goal(s): {node_ids}"
            )
        except Exception as e:
            self.get_logger().error(f"BT Manager: Failed to parse /goal_sequence: {e}")


    def progress_cb(self, msg):
        try:
            data = json.loads(msg.data)
            self.blackboard.set("mission_status", data.get("state", "IDLE"))
        except Exception:
            pass

    def create_tree(self):
        root = py_trees.composites.Sequence(name="AGV_BT_Root", memory=True)

        check_loc = CheckLocalization("Check_Localization", self, self.tf_buffer)
        check_goal = CheckGoalQueue("Check_Goal_Queue", self, self.blackboard)
        plan_path = PlanTopologicalPath("Plan_Topological_Path", self, self.blackboard)

        nav_selector = py_trees.composites.Selector(name="Nav_Or_Recovery", memory=False)
        exec_nav = ExecuteMPPINavigation("Execute_MPPI_Nav", self, self.blackboard)

        recovery_seq = py_trees.composites.Sequence(name="Recovery_Sequence", memory=True)
        backup_act = BackUpRecovery("Backup_Action", self, self.blackboard)
        spin_act = SpinRecovery("Spin_Action", self, self.blackboard)

        recovery_seq.add_children([backup_act, spin_act])
        nav_selector.add_children([exec_nav, recovery_seq])

        root.add_children([check_loc, check_goal, plan_path, nav_selector])
        return root

    def tick_tree(self):
        self.root.tick_once()


def main(args=None):
    rclpy.init(args=args)
    node = BTManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
