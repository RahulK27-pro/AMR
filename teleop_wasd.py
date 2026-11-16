#!/usr/bin/env python3
"""
AMR Physical Hardware WASD Teleoperation Controller
===================================================
A clean, non-latching keyboard teleop controller for ROS 2 Jazzy.

Features:
  - HOLD-TO-MOVE: Robot moves ONLY while WASD / Arrow keys are pressed.
  - SPEED CONTROL BEFOREHAND: Use '+' and '-' to adjust linear/angular speeds.
  - IN-PLACE HUD: Crisp single-screen dashboard without scrolling clutter.
  - SAFETY WATCHDOG: Releasing a key immediately commands zero velocity.
  - ROS 2 PUBLISHER: Publishes standard geometry_msgs/msg/Twist on /cmd_vel.
"""

import os
import sys
import time
import signal
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist

# Speed Limits (calibrated for 70mm wheel chassis @ 122 RPM)
MIN_LINEAR_SPEED  = 0.04   # m/s (~10% duty)
MAX_LINEAR_SPEED  = 0.45   # m/s (100% max speed on 70mm wheels)
DEFAULT_LINEAR    = 0.18   # m/s (safe, responsive indoor cruising speed)
LINEAR_STEP       = 0.02   # m/s per '+' or '-' press

MIN_ANGULAR_SPEED = 0.20   # rad/s
MAX_ANGULAR_SPEED = 2.00   # rad/s
DEFAULT_ANGULAR   = 0.80   # rad/s
ANGULAR_STEP      = 0.10   # rad/s per '+' or '-' press

# Hold-to-move key release timeout
# Standard OS key-repeat delay is ~250ms with 20-30Hz repeat.
# 0.22s allows smooth continuous motion when held, and stops within 220ms when released.
KEY_TIMEOUT_SEC   = 0.22


class KeyReader:
    """Reads non-blocking single characters from standard input in raw mode."""
    def __init__(self):
        self.is_tty = sys.stdin.isatty()
        if self.is_tty:
            self.old_settings = termios.tcgetattr(sys.stdin)
        else:
            self.old_settings = None

    def read_key(self, timeout=0.08):
        if not self.is_tty:
            ch = sys.stdin.read(1)
            return ch if ch else 'q'

        try:
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], timeout)
            key = None
            if rlist:
                key = sys.stdin.read(1)
                # Handle multi-byte escape sequences (arrow keys, function keys)
                if key == '\x1b':
                    rlist2, _, _ = select.select([sys.stdin], [], [], 0.02)
                    if rlist2:
                        key += sys.stdin.read(2)
            return key
        finally:
            self.restore()

    def restore(self):
        if self.is_tty and self.old_settings:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            except Exception:
                pass


class AMRWasdTeleopNode(Node):
    def __init__(self):
        super().__init__('amr_wasd_teleop')

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', qos)

        self.linear_speed = DEFAULT_LINEAR
        self.angular_speed = DEFAULT_ANGULAR

        self.cmd_linear = 0.0
        self.cmd_angular = 0.0

        self.state_desc = "STOPPED"
        self.last_key_time = 0.0
        self.active_key = None

    def publish_twist(self, linear: float, angular: float, state_desc: str):
        self.cmd_linear = linear
        self.cmd_angular = angular
        self.state_desc = state_desc

        msg = Twist()
        msg.linear.x = float(linear)
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(angular)
        self.pub_cmd_vel.publish(msg)

    def stop(self, reason="STOPPED"):
        self.publish_twist(0.0, 0.0, reason)
        self.active_key = None


def render_hud(node: AMRWasdTeleopNode, domain_id: str):
    # ANSI clear and return home for flicker-free HUD
    sys.stdout.write("\033[H\033[J")

    # State badge color
    if "FORWARD" in node.state_desc:
        state_color = "\033[1;32m"  # Green
    elif "REVERSE" in node.state_desc:
        state_color = "\033[1;33m"  # Yellow
    elif "PIVOT" in node.state_desc or "CURVE" in node.state_desc:
        state_color = "\033[1;36m"  # Cyan
    else:
        state_color = "\033[1;31m"  # Red

    print("========================================================================")
    print("      AMR PHYSICAL ROBOT TELEOPERATION (WASD HOLD-TO-MOVE)              ")
    print("========================================================================")
    print(f" Target Topic   : /cmd_vel (geometry_msgs/Twist) | ROS Domain ID: {domain_id}")
    print(f" RMW Middleware : CycloneDDS (rmw_cyclonedds_cpp)")
    print("------------------------------------------------------------------------")
    print(f" Current State  : {state_color}{node.state_desc}\033[0m")
    print(f" Target Speeds  : Linear = \033[1;37m{node.linear_speed:.2f} m/s\033[0m  |  Turn = \033[1;37m{node.angular_speed:.2f} rad/s\033[0m")
    print(f" Commanded Vel  : Linear = {node.cmd_linear:+.2f} m/s  |  Turn = {node.cmd_angular:+.2f} rad/s")
    print("------------------------------------------------------------------------")
    print(" Driving Controls (HOLD TO MOVE — robot stops immediately on release):")
    print("        [W] / [↑] : Drive Forward     |   [+] or [=] : Speed Up   (+0.02 m/s)")
    print("        [S] / [↓] : Drive Reverse     |   [-] or [_] : Speed Down (-0.02 m/s)")
    print("        [A] / [←] : Pivot Turn Left   |   [1] : Preset Slow   (0.10 m/s)")
    print("        [D] / [→] : Pivot Turn Right  |   [2] : Preset Normal (0.20 m/s)")
    print("        [Q] / [E] : Curve Left/Right  |   [3] : Preset Fast   (0.35 m/s)")
    print("      [SPACE] / [X] : Emergency STOP  |   [Ctrl+C] or [Esc] : Exit")
    print("========================================================================")
    print(" Ready. Press and hold WASD to drive...", end='', flush=True)


def main():
    rclpy.init()
    node = AMRWasdTeleopNode()
    key_reader = KeyReader()

    domain_id = os.environ.get('ROS_DOMAIN_ID', '7')

    def cleanup_and_exit(signum=None, frame=None):
        node.stop("SHUTDOWN - EXITING")
        key_reader.restore()
        try:
            # Send 3 zero messages to ensure network delivery
            for _ in range(3):
                node.publish_twist(0.0, 0.0, "EXIT")
                rclpy.spin_once(node, timeout_sec=0.01)
                time.sleep(0.02)
        except Exception:
            pass
        print("\n[OK] Motors stopped. Teleoperation terminated safely.")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, cleanup_and_exit)
    signal.signal(signal.SIGTERM, cleanup_and_exit)

    need_refresh = True

    try:
        while rclpy.ok():
            if need_refresh:
                render_hud(node, domain_id)
                need_refresh = False

            key = key_reader.read_key(timeout=0.06)
            rclpy.spin_once(node, timeout_sec=0.001)

            now = time.time()

            # ---- HOLD-TO-MOVE TIMEOUT CHECK ----
            # If no key arrived for KEY_TIMEOUT_SEC and robot is not stopped, stop it.
            if node.active_key is not None:
                if (now - node.last_key_time) > KEY_TIMEOUT_SEC:
                    node.stop("STOPPED (KEY RELEASED)")
                    need_refresh = True
                    continue

            if key is None:
                continue

            need_refresh = True
            key_lower = key.lower()

            # Normalize arrow keys to wasd
            if key == '\x1b[A':
                key_lower = 'w'
            elif key == '\x1b[B':
                key_lower = 's'
            elif key == '\x1b[D':
                key_lower = 'a'
            elif key == '\x1b[C':
                key_lower = 'd'

            # ── WASD Navigation Controls (Hold to move) ────────────────────
            if key_lower == 'w':
                node.active_key = 'w'
                node.last_key_time = now
                node.publish_twist(+node.linear_speed, 0.0, "DRIVING FORWARD")

            elif key_lower == 's':
                node.active_key = 's'
                node.last_key_time = now
                node.publish_twist(-node.linear_speed, 0.0, "DRIVING REVERSE")

            elif key_lower == 'a':
                node.active_key = 'a'
                node.last_key_time = now
                node.publish_twist(0.0, +node.angular_speed, "PIVOT TURNING LEFT")

            elif key_lower == 'd':
                node.active_key = 'd'
                node.last_key_time = now
                node.publish_twist(0.0, -node.angular_speed, "PIVOT TURNING RIGHT")

            elif key_lower == 'q':
                node.active_key = 'q'
                node.last_key_time = now
                node.publish_twist(+node.linear_speed, +node.angular_speed * 0.7, "CURVE TURNING LEFT")

            elif key_lower == 'e':
                node.active_key = 'e'
                node.last_key_time = now
                node.publish_twist(+node.linear_speed, -node.angular_speed * 0.7, "CURVE TURNING RIGHT")

            # ── Emergency Stop ─────────────────────────────────────────────
            elif key_lower in ['x', ' ']:
                node.stop("EMERGENCY STOP [SPACE/X]")

            # ── Speed Controls Beforehand (+ / -) ──────────────────────────
            elif key in ['+', '=']:
                node.linear_speed = min(MAX_LINEAR_SPEED, round(node.linear_speed + LINEAR_STEP, 2))
                node.angular_speed = min(MAX_ANGULAR_SPEED, round(node.angular_speed + ANGULAR_STEP, 2))
                # If currently moving, update the active command immediately
                if node.active_key == 'w':
                    node.publish_twist(+node.linear_speed, 0.0, "DRIVING FORWARD")
                elif node.active_key == 's':
                    node.publish_twist(-node.linear_speed, 0.0, "DRIVING REVERSE")
                elif node.active_key == 'a':
                    node.publish_twist(0.0, +node.angular_speed, "PIVOT TURNING LEFT")
                elif node.active_key == 'd':
                    node.publish_twist(0.0, -node.angular_speed, "PIVOT TURNING RIGHT")

            elif key in ['-', '_']:
                node.linear_speed = max(MIN_LINEAR_SPEED, round(node.linear_speed - LINEAR_STEP, 2))
                node.angular_speed = max(MIN_ANGULAR_SPEED, round(node.angular_speed - ANGULAR_STEP, 2))
                if node.active_key == 'w':
                    node.publish_twist(+node.linear_speed, 0.0, "DRIVING FORWARD")
                elif node.active_key == 's':
                    node.publish_twist(-node.linear_speed, 0.0, "DRIVING REVERSE")
                elif node.active_key == 'a':
                    node.publish_twist(0.0, +node.angular_speed, "PIVOT TURNING LEFT")
                elif node.active_key == 'd':
                    node.publish_twist(0.0, -node.angular_speed, "PIVOT TURNING RIGHT")

            # ── Speed Presets ──────────────────────────────────────────────
            elif key == '1':
                node.linear_speed = 0.10
                node.angular_speed = 0.50

            elif key == '2':
                node.linear_speed = 0.20
                node.angular_speed = 0.80

            elif key == '3':
                node.linear_speed = 0.35
                node.angular_speed = 1.40

            # ── Quit ───────────────────────────────────────────────────────
            elif key_lower == 'exit' or key == '\x1b':
                cleanup_and_exit()

    finally:
        cleanup_and_exit()


if __name__ == '__main__':
    main()
