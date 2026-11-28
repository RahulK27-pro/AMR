#!/usr/bin/env python3
"""Agent 1: Simulation & Autonomous Navigation Patrol Agent (sim_mission_agent).

Autonomously dispatches corridor patrol goals across the Gazebo map while monitoring
robot progress, obstacle clearance, and zero-stuck recovery for closed-loop dataset capture.
"""

import json
import math
import time
from typing import List, Tuple

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


HOSPITAL_PATROL_GOALS: List[Tuple[float, float]] = [
    (3.8, 0.0),
    (-3.6, 0.0),
    (3.8, 0.22),
    (-3.4, -0.18),
    (3.8, -0.15),
    (-3.6, 0.20),
]

WAREHOUSE_PATROL_GOALS: List[Tuple[float, float]] = [
    (2.5, 0.0),
    (-2.2, 0.0),
    (2.4, 1.2),
    (-2.0, -1.0),
]


class SimMissionAgent(Node):
    """Autonomous goal dispatcher and closed-loop evasion performance monitor."""

    def __init__(self):
        super().__init__("sim_mission_agent")

        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        dyn_desc = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter("map_name", "hospital_map")
        self.declare_parameter("target_laps", 6)
        self.declare_parameter("max_duration_sec", 150.0, dyn_desc)
        self.declare_parameter("goal_timeout_sec", 28.0, dyn_desc)

        self.map_name = str(self.get_parameter("map_name").value)
        self.target_laps = int(self.get_parameter("target_laps").value)
        self.max_duration_sec = float(self.get_parameter("max_duration_sec").value)
        self.goal_timeout_sec = float(self.get_parameter("goal_timeout_sec").value)

        self.goals = WAREHOUSE_PATROL_GOALS if "warehouse" in self.map_name else HOSPITAL_PATROL_GOALS
        self.goal_idx = 0
        self.completed_laps = 0
        self.stuck_events = 0
        self.yield_ticks = 0
        self.min_clearance_seen = 99.0

        self.agv_state = "IDLE"
        self.last_state_rx_wall = 0.0
        self.goal_sent_time = None
        self.goal_active = False
        self.last_progress_pos = None
        self.last_progress_wall = time.monotonic()
        self.start_wall = time.monotonic()
        self.done = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.status_pub = self.create_publisher(String, "/ml_pipeline/mission_status", 10)

        self.create_subscription(String, "/agv_state", self._state_cb, 10)
        self.create_subscription(LaserScan, "/scan", self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(Twist, "/cmd_vel", self._cmd_cb, 10)

        self.timer = self.create_timer(0.25, self._tick)
        self.get_logger().info(
            f"[AGENT_1: SIM_MISSION] Ready (map={self.map_name}, target_laps={self.target_laps}). "
            f"Waiting for localization TF & route_runner..."
        )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _state_cb(self, msg: String):
        self.agv_state = msg.data.strip()
        self.last_state_rx_wall = time.monotonic()
        if self.agv_state == "YIELDING":
            self.yield_ticks += 1

    def _scan_cb(self, msg: LaserScan):
        valid = [r for r in msg.ranges if 0.16 < r < msg.range_max and math.isfinite(r)]
        if valid:
            d = min(valid)
            if d < self.min_clearance_seen:
                self.min_clearance_seen = d

    def _cmd_cb(self, msg: Twist):
        if abs(msg.linear.x) > 0.03 or abs(msg.angular.z) > 0.08:
            self.last_progress_wall = time.monotonic()

    def _get_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            return float(t.transform.translation.x), float(t.transform.translation.y)
        except TransformException:
            return None

    def _send_next_goal(self):
        gx, gy = self.goals[self.goal_idx % len(self.goals)]
        self.goal_idx += 1

        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(gx)
        msg.pose.position.y = float(gy)
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)

        self.goal_active = True
        self.goal_sent_time = self._now_sec()
        self.last_progress_wall = time.monotonic()
        self.get_logger().info(
            f"[AGENT_1: SIM_MISSION] Dispatched autonomous patrol goal #{self.goal_idx}: ({gx:.2f}, {gy:.2f})"
        )

    def _publish_status(self):
        payload = {
            "completed_laps": self.completed_laps,
            "target_laps": self.target_laps,
            "agv_state": self.agv_state,
            "min_clearance_m": round(self.min_clearance_seen, 3),
            "stuck_events": self.stuck_events,
            "yield_ticks": self.yield_ticks,
            "elapsed_sec": round(time.monotonic() - self.start_wall, 1),
            "done": self.done,
        }
        m = String()
        m.data = json.dumps(payload)
        self.status_pub.publish(m)

    def _tick(self):
        if self.done:
            self._publish_status()
            return

        elapsed_wall = time.monotonic() - self.start_wall
        if self.completed_laps >= self.target_laps or elapsed_wall >= self.max_duration_sec:
            self.done = True
            self._publish_status()
            self.get_logger().info(
                f"[AGENT_1: SIM_MISSION] Mission complete: {self.completed_laps} laps in {elapsed_wall:.1f}s | "
                f"MinClearance={self.min_clearance_seen:.2f}m | StuckEvents={self.stuck_events}"
            )
            return

        pose = self._get_robot_pose()
        if pose is None:
            return

        # Wait until route_runner has published/is active (or after 3s TF warmup)
        if self.last_state_rx_wall == 0.0 and elapsed_wall < 3.0:
            return

        gx, gy = self.goals[(self.goal_idx - 1) % len(self.goals)] if self.goal_idx > 0 else (0.0, 0.0)
        dist_goal = math.hypot(gx - pose[0], gy - pose[1]) if self.goal_idx > 0 else 99.0

        if not self.goal_active:
            if self.agv_state in ("IDLE", "ARRIVED", ""):
                self._send_next_goal()
        else:
            sim_elapsed = (self._now_sec() - self.goal_sent_time) if self.goal_sent_time else 0.0
            if (self.agv_state == "ARRIVED" and dist_goal < 0.45) or (self.agv_state == "IDLE" and dist_goal < 0.25):
                self.completed_laps += 1
                self.goal_active = False
                self.get_logger().info(
                    f"[AGENT_1: SIM_MISSION] Lap {self.completed_laps}/{self.target_laps} completed at "
                    f"({pose[0]:.2f}, {pose[1]:.2f}) in {sim_elapsed:.1f}s!"
                )
            elif self.agv_state == "IDLE" and sim_elapsed > 3.0 and dist_goal >= 0.45:
                # route_runner was not ready when goal arrived; re-dispatch same goal without counting lap
                self.goal_idx = max(0, self.goal_idx - 1)
                self.goal_active = False
            elif sim_elapsed > self.goal_timeout_sec:
                self.stuck_events += 1
                self.goal_active = False
                self.get_logger().warn(
                    f"[AGENT_1: SIM_MISSION] Goal timeout ({sim_elapsed:.1f}s) — switching to next patrol waypoint."
                )

        self._publish_status()


def main(args=None):
    rclpy.init(args=args)
    node = SimMissionAgent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
