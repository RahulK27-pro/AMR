#!/usr/bin/env python3
"""
dynamic_obstacle_manager.py
===========================
Controls and animates dynamic obstacles (workers, moving carts) in Gazebo Harmonic
to test the AMR's dynamic avoidance, yielding, and re-routing capabilities.

Features:
- Autonomous Patrol Patterns:
    1. aisle_crossing   : Walks back and forth across warehouse aisles (cross traffic).
    2. corridor_walker  : Paces back and forth along a narrow corridor (head-on traffic).
    3. doorway_blocker  : Parks or paces inside the main doorway (x=3.0, y=2.8) to trigger re-routing.
    4. circular_patrol  : Moves in a continuous loop around warehouse pillars.
- Interactive Mode:
    - Easily driven with teleop: ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/dynamic_obstacle/cmd_vel

Usage:
    ros2 run agv_navigation dynamic_obstacle_manager --ros-args -p pattern:=aisle_crossing -p speed:=0.35
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time


class DynamicObstacleManager(Node):
    def __init__(self):
        super().__init__('dynamic_obstacle_manager')

        # Parameters
        self.declare_parameter('pattern', 'aisle_crossing')  # aisle_crossing, corridor_walker, doorway_blocker, circular_patrol
        self.declare_parameter('speed', 0.35)               # linear speed in m/s
        self.declare_parameter('turn_speed', 0.8)          # angular turn speed in rad/s
        self.declare_parameter('travel_time', 4.0)          # duration before reversing (seconds)
        self.declare_parameter('pause_time', 1.0)           # pause at turn-around (seconds)

        self.pattern = self.get_parameter('pattern').get_parameter_value().string_value
        self.speed = self.get_parameter('speed').get_parameter_value().double_value
        self.turn_speed = self.get_parameter('turn_speed').get_parameter_value().double_value
        self.travel_time = self.get_parameter('travel_time').get_parameter_value().double_value
        self.pause_time = self.get_parameter('pause_time').get_parameter_value().double_value

        # Publisher for obstacle velocity
        self.cmd_pub = self.create_publisher(Twist, '/dynamic_obstacle/cmd_vel', 10)

        # Optional odometry subscriber from obstacle
        self.odom_sub = self.create_subscription(Odometry, '/dynamic_obstacle/odom', self.odom_callback, 10)
        self.current_x = 0.0
        self.current_y = 0.0
        self.has_odom = False

        # State machine for pattern execution
        self.direction = 1  # 1 = forward, -1 = reverse
        self.state = 'FORWARD'  # FORWARD, PAUSE, ROTATING
        self.state_start_time = 0.0

        # Control loop at 10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info(f"Dynamic Obstacle Manager Active. Pattern: '{self.pattern}' at {self.speed} m/s")

    def _now_sec(self):
        """Return current ROS time as float seconds (sim-time aware)."""
        return self.get_clock().now().nanoseconds / 1e9

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.has_odom = True

    def control_loop(self):
        twist = Twist()
        now = self._now_sec()
        if self.state_start_time == 0.0:
            self.state_start_time = now
        elapsed = now - self.state_start_time

        if self.pattern == 'aisle_crossing':
            # Oscillates back and forth across an aisle
            if self.state == 'FORWARD':
                twist.linear.x = self.speed
                twist.angular.z = 0.0
                if elapsed >= self.travel_time:
                    self.state = 'PAUSE'
                    self.state_start_time = now
            elif self.state == 'PAUSE':
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                if elapsed >= self.pause_time:
                    self.state = 'ROTATING'
                    self.state_start_time = now
            elif self.state == 'ROTATING':
                twist.linear.x = 0.0
                twist.angular.z = self.turn_speed
                # 180 degree rotation: time = pi / turn_speed
                if elapsed >= (math.pi / self.turn_speed):
                    self.state = 'FORWARD'
                    self.state_start_time = now

        elif self.pattern == 'corridor_walker':
            # Back and forth along a corridor
            if self.state == 'FORWARD':
                twist.linear.x = self.speed * 0.8
                twist.angular.z = 0.0
                if elapsed >= self.travel_time * 1.5:
                    self.state = 'PAUSE'
                    self.state_start_time = now
            elif self.state == 'PAUSE':
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                if elapsed >= self.pause_time:
                    self.state = 'ROTATING'
                    self.state_start_time = now
            elif self.state == 'ROTATING':
                twist.linear.x = 0.0
                twist.angular.z = self.turn_speed
                if elapsed >= (math.pi / self.turn_speed):
                    self.state = 'FORWARD'
                    self.state_start_time = now

        elif self.pattern == 'doorway_blocker':
            # Moves slowly back and forth inside the doorway gap
            short_travel = 2.0
            if self.state == 'FORWARD':
                twist.linear.x = self.speed * 0.5
                if elapsed >= short_travel:
                    self.state = 'PAUSE'
                    self.state_start_time = now
            elif self.state == 'PAUSE':
                twist.linear.x = 0.0
                # Stays stationary for a long time to challenge robot re-routing
                if elapsed >= 6.0:
                    self.state = 'ROTATING'
                    self.state_start_time = now
            elif self.state == 'ROTATING':
                twist.angular.z = self.turn_speed
                if elapsed >= (math.pi / self.turn_speed):
                    self.state = 'FORWARD'
                    self.state_start_time = now

        elif self.pattern == 'circular_patrol':
            # Drives in a smooth continuous circle
            twist.linear.x = self.speed
            twist.angular.z = 0.4

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacleManager()
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
