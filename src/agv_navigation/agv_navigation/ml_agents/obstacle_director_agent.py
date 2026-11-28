#!/usr/bin/env python3
"""Agent 2: Obstacle Spawner & Behavior Controller Agent (obstacle_director_agent).

Autonomously spawns, cycles, and drives diverse dynamic (2-leg pedestrian, moving cart)
and static temporary clutter obstacles inside the active Gazebo world, publishing exact
ground-truth scenario poses on /ml_pipeline/active_scenario for live dataset labeling.
"""

import json
import math
import os
import subprocess
import tempfile
import time
from typing import Dict, List, Optional

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from ament_index_python.packages import get_package_share_directory


SCENARIOS = [
    {
        "name": "pedestrian_aisle_crossing",
        "dyn_type": "pedestrian",
        "pattern": "aisle_crossing",
        "spawn": (1.6, -0.58, 1.57),
        "speed": 0.38,
        "travel_time": 2.8,
        "pause_time": 0.8,
        "clutter": [],
    },
    {
        "name": "pedestrian_corridor_walker",
        "dyn_type": "pedestrian",
        "pattern": "corridor_walker",
        "spawn": (2.6, 0.25, 3.1416),
        "speed": 0.36,
        "travel_time": 4.2,
        "pause_time": 0.7,
        "clutter": [],
    },
    {
        "name": "cart_crossing_and_clutter",
        "dyn_type": "cart",
        "pattern": "aisle_crossing",
        "spawn": (-1.3, -0.56, 1.57),
        "speed": 0.42,
        "travel_time": 2.6,
        "pause_time": 0.7,
        "clutter": [(1.9, 0.28, 0.24)],
    },
    {
        "name": "clutter_corridor_slalom",
        "dyn_type": None,
        "pattern": None,
        "spawn": None,
        "speed": 0.0,
        "travel_time": 0.0,
        "pause_time": 0.0,
        "clutter": [(1.4, -0.18, 0.24), (-1.4, 0.22, 0.24)],
    },
    {
        "name": "multi_pedestrian_and_clutter",
        "dyn_type": "pedestrian",
        "pattern": "stop_go",
        "spawn": (0.4, -0.56, 1.57),
        "speed": 0.38,
        "travel_time": 2.7,
        "pause_time": 1.0,
        "clutter": [(2.2, 0.28, 0.24)],
    },
    {
        "name": "cart_circular_corridor",
        "dyn_type": "cart",
        "pattern": "circular_patrol",
        "spawn": (0.2, -0.35, 0.0),
        "speed": 0.36,
        "travel_time": 3.0,
        "pause_time": 0.5,
        "clutter": [],
    },
]


CLUTTER_BOX_SDF = """<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="{name}">
    <static>true</static>
    <pose>0 0 0.35 0 0 0</pose>
    <link name="link">
      <collision name="col">
        <geometry><box><size>0.42 0.38 0.70</size></box></geometry>
      </collision>
      <visual name="vis">
        <geometry><box><size>0.42 0.38 0.70</size></box></geometry>
        <material>
          <ambient>0.92 0.72 0.10 1</ambient>
          <diffuse>0.95 0.76 0.14 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


class ObstacleDirectorAgent(Node):
    """Spawns, animates, and publishes ground truth for multi-class Gazebo obstacles."""

    def __init__(self):
        super().__init__("obstacle_director_agent")
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        self.declare_parameter("scenario_duration_sec", 18.0, ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter("world", "auto")

        self.scenario_duration_sec = float(self.get_parameter("scenario_duration_sec").value)
        world_param = str(self.get_parameter("world").value).strip()
        self.world = self._detect_gz_world() if (not world_param or world_param == "auto") else world_param

        pkg_agv = get_package_share_directory("agv_description")
        self.pedestrian_sdf = os.path.join(pkg_agv, "models", "human_pedestrian", "model.sdf")
        self.cart_sdf = os.path.join(pkg_agv, "models", "dynamic_obstacle", "model.sdf")
        self._tmp_dir = tempfile.mkdtemp(prefix="amr_clutter_")

        # Bridge Gazebo /world/<w>/dynamic_pose/info -> ROS TFMessage for exact GT world poses
        self.pose_topic = f"/world/{self.world}/dynamic_pose/info"
        self.pose_bridge_proc = subprocess.Popen(
            [
                "ros2", "run", "ros_gz_bridge", "parameter_bridge",
                f"{self.pose_topic}@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

        self.cmd_pub = self.create_publisher(Twist, "/dynamic_obstacle/cmd_vel", 10)
        self.scenario_pub = self.create_publisher(String, "/ml_pipeline/active_scenario", 10)

        self.create_subscription(TFMessage, self.pose_topic, self._gz_pose_cb, 10)
        self.create_subscription(Odometry, "/dynamic_obstacle/odom", self._dyn_odom_cb, 10)
        self.create_subscription(String, "/ml_pipeline/mission_status", self._mission_cb, 10)

        self.gz_poses: Dict[str, tuple] = {}
        self.last_odom_pose = None
        self.last_odom_vel = (0.0, 0.0)
        self.active_dyn_name = "dynamic_obstacle_1"
        self.active_dyn_spawn = (0.0, 0.0, 0.0)
        self.active_clutter: List[Dict] = []

        self.scenario_idx = -1
        self.scenario_start_wall = 0.0
        self.motion_state = "FORWARD"
        self.motion_state_start = 0.0
        self.mission_done = False

        # Spawn first scenario shortly after startup
        self.control_timer = self.create_timer(0.10, self._tick)
        self.get_logger().info(
            f"[AGENT_2: OBSTACLE_DIRECTOR] Active on Gazebo world '{self.world}' "
            f"(scenario_duration={self.scenario_duration_sec:.1f}s)."
        )

    @staticmethod
    def _detect_gz_world(default_world: str = "hospital") -> str:
        try:
            res = subprocess.run(
                [
                    "gz", "service", "-s", "/gazebo/worlds",
                    "--reqtype", "gz.msgs.Empty",
                    "--reptype", "gz.msgs.StringMsg_V",
                    "--timeout", "1500", "--req", "",
                ],
                capture_output=True, text=True, timeout=2.5,
            )
            if res.returncode == 0 and "data:" in res.stdout:
                for line in res.stdout.splitlines():
                    if "data:" in line:
                        w = line.split("data:")[-1].strip().strip('"').strip("'")
                        if w:
                            return w
        except Exception:
            pass
        return default_world

    def _gz_pose_cb(self, msg: TFMessage):
        for tr in msg.transforms:
            name = tr.child_frame_id
            if name:
                base_name = name.split("::")[0].split("/")[0]
                short_name = name.split("::")[0].split("/")[-1]
                pt = (float(tr.transform.translation.x), float(tr.transform.translation.y))
                self.gz_poses[name] = pt
                self.gz_poses[base_name] = pt
                self.gz_poses[short_name] = pt

    def _dyn_odom_cb(self, msg: Odometry):
        self.last_odom_pose = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))
        self.last_odom_vel = (float(msg.twist.twist.linear.x), float(msg.twist.twist.linear.y))

    def _mission_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            if data.get("done"):
                self.mission_done = True
        except Exception:
            pass

    def _gz_remove(self, entity_name: str):
        subprocess.run(
            [
                "gz", "service", "-s", f"/world/{self.world}/remove",
                "--reqtype", "gz.msgs.Entity",
                "--reptype", "gz.msgs.Boolean",
                "--timeout", "1200",
                "--req", f'name: "{entity_name}", type: MODEL',
            ],
            capture_output=True, text=True, timeout=2.5,
        )

    def _gz_create(self, sdf_file: str, name: str, x: float, y: float, z: float, yaw: float):
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        req = (
            f'sdf_filename: "{sdf_file}", name: "{name}", '
            f'pose: {{position: {{x: {x}, y: {y}, z: {z}}}, orientation: {{z: {qz:.5f}, w: {qw:.5f}}}}}'
        )
        res = subprocess.run(
            [
                "gz", "service", "-s", f"/world/{self.world}/create",
                "--reqtype", "gz.msgs.EntityFactory",
                "--reptype", "gz.msgs.Boolean",
                "--timeout", "2500",
                "--req", req,
            ],
            capture_output=True, text=True, timeout=3.5,
        )
        return res.returncode == 0 and "data: true" in res.stdout

    def _cleanup_entities(self):
        self.cmd_pub.publish(Twist())
        self._gz_remove(self.active_dyn_name)
        for c in self.active_clutter:
            self._gz_remove(c["name"])
        self.active_clutter = []

    def _activate_scenario(self, idx: int):
        sc = SCENARIOS[idx % len(SCENARIOS)]
        self._cleanup_entities()
        time.sleep(0.15)
        self.gz_poses.pop(self.active_dyn_name, None)
        self.last_odom_pose = None
        self.active_dyn_spawn = (0.0, 0.0, 0.0)

        if sc["dyn_type"] is not None:
            sx, sy, syaw = sc["spawn"]
            self.active_dyn_spawn = (float(sx), float(sy), float(syaw))
            if sc["dyn_type"] == "pedestrian":
                sdf_path, z_val = self.pedestrian_sdf, 0.01
            else:
                sdf_path, z_val = self.cart_sdf, 0.50
            ok = self._gz_create(sdf_path, self.active_dyn_name, sx, sy, z_val, syaw)
            self.gz_poses[self.active_dyn_name] = (sx, sy)
            self.get_logger().info(
                f"[AGENT_2: OBSTACLE_DIRECTOR] Scenario '{sc['name']}': spawned {sc['dyn_type']} "
                f"at ({sx:.2f}, {sy:.2f}, yaw={syaw:.2f}) ok={ok}"
            )

        for ci, (cx, cy, cr) in enumerate(sc["clutter"]):
            cname = f"clutter_crate_{ci + 1}"
            sdf_file = os.path.join(self._tmp_dir, f"{cname}.sdf")
            with open(sdf_file, "w") as f:
                f.write(CLUTTER_BOX_SDF.format(name=cname))
            self._gz_create(sdf_file, cname, cx, cy, 0.35, 0.3 * ci)
            self.active_clutter.append({"name": cname, "class_name": "CLUTTER", "x": cx, "y": cy, "radius": cr})

        self.scenario_idx = idx
        self.scenario_start_wall = time.monotonic()
        self.motion_state = "FORWARD"
        self.motion_state_start = self._now_sec()

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _drive_dynamic_obstacle(self, sc: Dict):
        twist = Twist()
        if sc["dyn_type"] is None:
            self.cmd_pub.publish(twist)
            return

        now = self._now_sec()
        if self.motion_state_start == 0.0:
            self.motion_state_start = now
        elapsed = now - self.motion_state_start

        pat = sc["pattern"]
        spd = float(sc["speed"])
        turn_spd = 1.05
        travel_t = float(sc["travel_time"])
        pause_t = float(sc["pause_time"])

        if pat in ("aisle_crossing", "corridor_walker", "stop_go"):
            if self.motion_state == "FORWARD":
                twist.linear.x = spd
                if pat == "stop_go" and 1.0 <= elapsed <= 2.1:
                    twist.linear.x = 0.0
                if elapsed >= travel_t:
                    self.motion_state = "PAUSE"
                    self.motion_state_start = now
            elif self.motion_state == "PAUSE":
                if elapsed >= pause_t:
                    self.motion_state = "ROTATING"
                    self.motion_state_start = now
            elif self.motion_state == "ROTATING":
                twist.angular.z = turn_spd
                if elapsed >= (math.pi / turn_spd):
                    self.motion_state = "FORWARD"
                    self.motion_state_start = now
        elif pat == "circular_patrol":
            twist.linear.x = spd
            twist.angular.z = 0.55

        self.cmd_pub.publish(twist)

    def _publish_scenario_gt(self, sc: Dict):
        dyn_list = []
        if sc["dyn_type"] is not None:
            sx, sy, syaw = self.active_dyn_spawn
            pos = self.gz_poses.get(self.active_dyn_name)
            moved_gz = pos is not None and math.hypot(pos[0] - sx, pos[1] - sy) > 0.02
            if not moved_gz and self.last_odom_pose is not None:
                ox, oy = self.last_odom_pose
                pos = (
                    sx + ox * math.cos(syaw) - oy * math.sin(syaw),
                    sy + ox * math.sin(syaw) + oy * math.cos(syaw),
                )
            elif pos is None and sc["spawn"] is not None:
                pos = (sx, sy)
            if pos is not None:
                dyn_list.append(
                    {
                        "name": self.active_dyn_name,
                        "class_name": "PERSON",
                        "dyn_type": sc["dyn_type"],
                        "x": round(float(pos[0]), 4),
                        "y": round(float(pos[1]), 4),
                        "radius": 0.28 if sc["dyn_type"] == "pedestrian" else 0.25,
                    }
                )

        payload = {
            "scenario_idx": self.scenario_idx,
            "scenario_name": sc["name"],
            "dynamic_obstacles": dyn_list,
            "clutter_obstacles": self.active_clutter,
        }
        m = String()
        m.data = json.dumps(payload)
        self.scenario_pub.publish(m)

    def _tick(self):
        if self.mission_done:
            self.cmd_pub.publish(Twist())
            return

        now_wall = time.monotonic()
        if self.scenario_idx < 0 or (now_wall - self.scenario_start_wall) >= self.scenario_duration_sec:
            self._activate_scenario(self.scenario_idx + 1)

        sc = SCENARIOS[self.scenario_idx % len(SCENARIOS)]
        self._drive_dynamic_obstacle(sc)
        self._publish_scenario_gt(sc)

    def destroy_node(self):
        try:
            self._cleanup_entities()
        except Exception:
            pass
        try:
            if self.pose_bridge_proc and self.pose_bridge_proc.poll() is None:
                self.pose_bridge_proc.terminate()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDirectorAgent()
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
