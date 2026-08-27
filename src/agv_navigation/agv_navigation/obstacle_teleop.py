#!/usr/bin/env python3
"""
obstacle_teleop.py
==================
WASD Keyboard teleoperation for the dynamic obstacle in Gazebo Harmonic.

Controls:
  [W] : Move Forward
  [S] : Move Backward (Reverse)
  [A] : Turn Left
  [D] : Turn Right

Diagonal Movement:
  [Q] : Forward-Left
  [E] : Forward-Right
  [Z] : Reverse-Left
  [C] : Reverse-Right

Speed & Stopping:
  [Space] or [X] : Immediate Full Stop
  [+] or [=]     : Increase Speed (+10%)
  [-] or [_]     : Decrease Speed (-10%)

Quit:
  Ctrl-C or [Esc]
"""

import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

BANNER = """
=====================================================
  DYNAMIC OBSTACLE TELEOPERATION — WASD CONTROLLER   
=====================================================
  Target Topic: /dynamic_obstacle/cmd_vel

  Controls:
          [W] Forward
   [A] Left         [D] Right
          [S] Reverse

  Diagonals: [Q] Fwd-Left  [E] Fwd-Right
             [Z] Rev-Left  [C] Rev-Right

  Stop:      [Space] or [X]
  Speed:     [+] Increase  [-] Decrease
  Quit:      [Ctrl-C]
=====================================================
"""


class ObstacleTeleop(Node):
    def __init__(self):
        super().__init__('obstacle_teleop')

        self.declare_parameter('cmd_topic', '/dynamic_obstacle/cmd_vel')
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('angular_speed', 1.2)

        topic = self.get_parameter('cmd_topic').get_parameter_value().string_value
        self.linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value
        self.angular_speed = self.get_parameter('angular_speed').get_parameter_value().double_value

        self.pub = self.create_publisher(Twist, topic, 10)
        self.get_logger().info(f"WASD Teleop active on topic: {topic}")


def get_key(settings, timeout=0.1):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
        # Handle arrow escape sequences if user uses arrow keys
        if key == '\x1b':
            extra = sys.stdin.read(2)
            if extra == '[A':
                key = 'w'
            elif extra == '[B':
                key = 's'
            elif extra == '[C':
                key = 'd'
            elif extra == '[D':
                key = 'a'
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = ObstacleTeleop()

    print(BANNER)
    print(f"Current Settings: Speed = {node.linear_speed:.2f} m/s | Turn = {node.angular_speed:.2f} rad/s\n")

    target_linear = 0.0
    target_angular = 0.0
    last_key_time = 0.0
    key_timeout = 0.5  # Auto-stop after 0.5s of no keypress (deadman switch)

    try:
        while rclpy.ok():
            key = get_key(settings, timeout=0.08)
            now = node.get_clock().now().nanoseconds / 1e9

            if key:
                last_key_time = now

                # Directional Controls
                if key in ['w', 'W']:
                    target_linear = node.linear_speed
                    target_angular = 0.0
                    status = "FORWARD"
                elif key in ['s', 'S']:
                    target_linear = -node.linear_speed
                    target_angular = 0.0
                    status = "REVERSE"
                elif key in ['a', 'A']:
                    target_linear = 0.0
                    target_angular = node.angular_speed
                    status = "TURN LEFT"
                elif key in ['d', 'D']:
                    target_linear = 0.0
                    target_angular = -node.angular_speed
                    status = "TURN RIGHT"
                elif key in ['q', 'Q']:
                    target_linear = node.linear_speed
                    target_angular = node.angular_speed
                    status = "FWD-LEFT"
                elif key in ['e', 'E']:
                    target_linear = node.linear_speed
                    target_angular = -node.angular_speed
                    status = "FWD-RIGHT"
                elif key in ['z', 'Z']:
                    target_linear = -node.linear_speed
                    target_angular = -node.angular_speed
                    status = "REV-LEFT"
                elif key in ['c', 'C']:
                    target_linear = -node.linear_speed
                    target_angular = node.angular_speed
                    status = "REV-RIGHT"
                elif key in [' ', 'x', 'X']:
                    target_linear = 0.0
                    target_angular = 0.0
                    status = "STOPPED"
                elif key in ['+', '=']:
                    node.linear_speed = min(2.0, node.linear_speed * 1.1)
                    node.angular_speed = min(3.0, node.angular_speed * 1.1)
                    status = f"SPEED UP -> {node.linear_speed:.2f} m/s"
                elif key in ['-', '_']:
                    node.linear_speed = max(0.1, node.linear_speed * 0.9)
                    node.angular_speed = max(0.2, node.angular_speed * 0.9)
                    status = f"SPEED DOWN -> {node.linear_speed:.2f} m/s"
                elif key == '\x03':  # Ctrl-C
                    break
                else:
                    status = ""

                if status:
                    sys.stdout.write(f"\r[CMD] Action: {status:<18} | Lin: {target_linear:+.2f} m/s | Ang: {target_angular:+.2f} rad/s   ")
                    sys.stdout.flush()

            else:
                # Deadman switch: if no key pressed within timeout, decelerate to stop
                if (now - last_key_time) > key_timeout and (target_linear != 0.0 or target_angular != 0.0):
                    target_linear = 0.0
                    target_angular = 0.0
                    sys.stdout.write(f"\r[CMD] Action: IDLE (Released)     | Lin:  0.00 m/s | Ang:  0.00 rad/s   ")
                    sys.stdout.flush()

            # Publish velocity command
            twist = Twist()
            twist.linear.x = float(target_linear)
            twist.angular.z = float(target_angular)
            node.pub.publish(twist)

            rclpy.spin_once(node, timeout_sec=0.01)

    except Exception as e:
        print(f"\nTeleop error: {e}")
    finally:
        # Send a final stop command
        twist = Twist()
        node.pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print("\nRestored terminal settings. Exiting WASD teleop.")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
