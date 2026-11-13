#!/usr/bin/env python3
"""
AMR Differential Drive Hardware & Odometry Node for Raspberry Pi 5
===================================================================
Hardware Interfaces:
  - SBC: Raspberry Pi 5 (RP1 Southbridge via lgpio)
  - Driver: RMCS-2305 Dual DC Motor Driver
  - Encoders: 6-Wire Quadrature Hall-Effect Encoders (3.3V Logic)

Calibrated Constants:
  - Wheel Radius (r)       : 0.070 m (70 mm)
  - Wheel Separation (L)   : 0.240 m (240 mm)
  - Ticks Per Revolution   : 4713 ticks (4x quadrature decoding)

Pinout (BCM Numbering):
  - Left Motor : PWM = GPIO 12 (Pin 32), DIR = GPIO 16 (Pin 36) [DIR: 0=Fwd, 1=Rev]
  - Right Motor: PWM = GPIO 13 (Pin 33), DIR = GPIO 26 (Pin 37) [DIR: 1=Fwd, 0=Rev]
  - Left Enc   : Ch A = GPIO 17 (Pin 11), Ch B = GPIO 27 (Pin 13) [Polarity: Inverted]
  - Right Enc  : Ch A = GPIO 22 (Pin 15), Ch B = GPIO 23 (Pin 16) [Polarity: Normal]
  - SLP1 & SLP2: Left unconnected / open (Hardware default awake)
  - GND        : Pin 39

Topics:
  - Subscribes: /cmd_vel (geometry_msgs/Twist)
  - Publishes : /odom (nav_msgs/Odometry)
  - Broadcasts: odom -> base_link TF (optional, default false when EKF is active)
"""

import math
import sys
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

# GPIO Pinout
LEFT_PWM_PIN  = 12
LEFT_DIR_PIN  = 16
RIGHT_PWM_PIN = 13
RIGHT_DIR_PIN = 26

LEFT_ENC_A    = 17
LEFT_ENC_B    = 27
RIGHT_ENC_A   = 22
RIGHT_ENC_B   = 23

PWM_FREQ      = 1000

# Quadrature lookup table
QUAD_TABLE = [
     0,  1, -1,  0,
    -1,  0,  0,  1,
     1,  0,  0, -1,
     0, -1,  1,  0
]


def euler_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class AMRHardwareDriveNode(Node):
    def __init__(self):
        super().__init__('amr_hardware_drive_node')

        # Declare ROS 2 Parameters
        self.declare_parameter('wheel_radius', 0.070)
        self.declare_parameter('wheel_separation', 0.240)
        self.declare_parameter('ticks_per_rev', 4713)
        self.declare_parameter('max_wheel_speed', 0.90)       # m/s (~122 RPM on 70mm wheel)
        self.declare_parameter('min_pwm', 15.0)               # % duty minimum deadband
        self.declare_parameter('max_pwm', 100.0)              # % duty maximum
        self.declare_parameter('odom_rate', 50.0)             # Hz
        self.declare_parameter('cmd_vel_timeout', 0.5)        # seconds safety watchdog
        self.declare_parameter('publish_tf', False)           # Set True only if EKF is disabled
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        # Read Parameters
        self.r = self.get_parameter('wheel_radius').value
        self.L = self.get_parameter('wheel_separation').value
        self.ticks_per_rev = float(self.get_parameter('ticks_per_rev').value)
        self.max_speed = self.get_parameter('max_wheel_speed').value
        self.min_pwm = self.get_parameter('min_pwm').value
        self.max_pwm = self.get_parameter('max_pwm').value
        self.cmd_timeout = self.get_parameter('cmd_vel_timeout').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.meters_per_tick = (2.0 * math.pi * self.r) / self.ticks_per_rev

        self.get_logger().info(
            f"Initializing AMR Hardware Drive Node: r={self.r:.3f}m, L={self.L:.3f}m, "
            f"ticks_per_rev={self.ticks_per_rev:.0f} ({self.meters_per_tick*1000:.4f} mm/tick)"
        )

        # Initialize all state variables BEFORE claiming hardware interrupts
        self.left_ticks = 0
        self.right_ticks = 0
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0

        self.left_state = 0
        self.right_state = 0

        # Odometry Pose State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_odom_time = self.get_clock().now()

        # Watchdog
        self.last_cmd_time = self.get_clock().now()

        # Hardware Initialization (lgpio)
        self.chip = None
        self.mock_mode = False
        self.callbacks = []

        self._init_hardware()

        # ROS 2 Subscriptions & Publishers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        # Timer for Odometry & Watchdog (50 Hz)
        odom_period = 1.0 / self.get_parameter('odom_rate').value
        self.odom_timer = self.create_timer(odom_period, self.update_odometry)

        self.get_logger().info("AMR Hardware Drive Node online & listening on /cmd_vel")

    def _init_hardware(self):
        try:
            import lgpio
            self.lgpio = lgpio

            for chip_num in [4, 0, 1]:
                try:
                    self.chip = lgpio.gpiochip_open(chip_num)
                    self.get_logger().info(f"Opened /dev/gpiochip{chip_num} via lgpio")
                    break
                except Exception:
                    self.chip = None

            if self.chip is None:
                raise RuntimeError("Failed to open gpiochip (4, 0, 1)")

            # Setup Motor Outputs
            self.lgpio.gpio_claim_output(self.chip, LEFT_DIR_PIN, 0)
            self.lgpio.gpio_claim_output(self.chip, RIGHT_DIR_PIN, 0)
            self.lgpio.gpio_claim_output(self.chip, LEFT_PWM_PIN, 0)
            self.lgpio.gpio_claim_output(self.chip, RIGHT_PWM_PIN, 0)

            # Setup Encoder Inputs & Alerts
            for pin in [LEFT_ENC_A, LEFT_ENC_B, RIGHT_ENC_A, RIGHT_ENC_B]:
                self.lgpio.gpio_claim_input(self.chip, pin)
                self.lgpio.gpio_claim_alert(self.chip, pin, self.lgpio.BOTH_EDGES)

            la = self.lgpio.gpio_read(self.chip, LEFT_ENC_A)
            lb = self.lgpio.gpio_read(self.chip, LEFT_ENC_B)
            self.left_state = (la << 1) | lb

            ra = self.lgpio.gpio_read(self.chip, RIGHT_ENC_A)
            rb = self.lgpio.gpio_read(self.chip, RIGHT_ENC_B)
            self.right_state = (ra << 1) | rb

            cb1 = self.lgpio.callback(self.chip, LEFT_ENC_A, self.lgpio.BOTH_EDGES, self._on_left_a)
            cb2 = self.lgpio.callback(self.chip, LEFT_ENC_B, self.lgpio.BOTH_EDGES, self._on_left_b)
            cb3 = self.lgpio.callback(self.chip, RIGHT_ENC_A, self.lgpio.BOTH_EDGES, self._on_right_a)
            cb4 = self.lgpio.callback(self.chip, RIGHT_ENC_B, self.lgpio.BOTH_EDGES, self._on_right_b)
            self.callbacks = [cb1, cb2, cb3, cb4]

            self.get_logger().info("Hardware GPIO & Encoder interrupts claimed successfully.")

        except Exception as e:
            self.get_logger().warn(f"Hardware initialization failed ({e}). Running in SIMULATION MOCK mode.")
            self.mock_mode = True

    # ── Encoder ISR Callbacks ──────────────────────────────────────────────
    def _on_left_a(self, chip, gpio, level, tick):
        lb = self.lgpio.gpio_read(self.chip, LEFT_ENC_B)
        new_state = (level << 1) | lb
        transition = (self.left_state << 2) | new_state
        delta = QUAD_TABLE[transition & 0x0F]
        # Left encoder polarity is inverted to match forward rotation
        self.left_ticks -= delta
        self.left_state = new_state

    def _on_left_b(self, chip, gpio, level, tick):
        la = self.lgpio.gpio_read(self.chip, LEFT_ENC_A)
        new_state = (la << 1) | level
        transition = (self.left_state << 2) | new_state
        delta = QUAD_TABLE[transition & 0x0F]
        self.left_ticks -= delta
        self.left_state = new_state

    def _on_right_a(self, chip, gpio, level, tick):
        rb = self.lgpio.gpio_read(self.chip, RIGHT_ENC_B)
        new_state = (level << 1) | rb
        transition = (self.right_state << 2) | new_state
        delta = QUAD_TABLE[transition & 0x0F]
        # Right encoder polarity is normal
        self.right_ticks += delta
        self.right_state = new_state

    def _on_right_b(self, chip, gpio, level, tick):
        ra = self.lgpio.gpio_read(self.chip, RIGHT_ENC_A)
        new_state = (ra << 1) | level
        transition = (self.right_state << 2) | new_state
        delta = QUAD_TABLE[transition & 0x0F]
        self.right_ticks += delta
        self.right_state = new_state

    # ── Motor Driver Output ────────────────────────────────────────────────
    def set_motors(self, v_left: float, v_right: float):
        if self.mock_mode or self.chip is None:
            return

        # Left Motor: 0 = Forward, 1 = Reverse
        fwd_left = (v_left >= 0.0)
        dir_left = 0 if fwd_left else 1
        duty_left = self._speed_to_duty(abs(v_left))

        # Right Motor: 1 = Forward, 0 = Reverse
        fwd_right = (v_right >= 0.0)
        dir_right = 1 if fwd_right else 0
        duty_right = self._speed_to_duty(abs(v_right))

        self.lgpio.gpio_write(self.chip, LEFT_DIR_PIN, dir_left)
        self.lgpio.tx_pwm(self.chip, LEFT_PWM_PIN, PWM_FREQ, duty_left)

        self.lgpio.gpio_write(self.chip, RIGHT_DIR_PIN, dir_right)
        self.lgpio.tx_pwm(self.chip, RIGHT_PWM_PIN, PWM_FREQ, duty_right)

        if duty_left > 0.0 or duty_right > 0.0:
            self.get_logger().info(
                f"[MOTORS] L: DIR={dir_left} PWM={duty_left:.1f}% ({v_left:+.2f}m/s) | "
                f"R: DIR={dir_right} PWM={duty_right:.1f}% ({v_right:+.2f}m/s)"
            )

    def _speed_to_duty(self, speed: float) -> float:
        if speed < 0.01:
            return 0.0
        ratio = min(1.0, speed / self.max_speed)
        duty = self.min_pwm + (self.max_pwm - self.min_pwm) * ratio
        return max(0.0, min(100.0, duty))

    def stop_motors(self):
        if self.mock_mode or self.chip is None:
            return
        self.lgpio.tx_pwm(self.chip, LEFT_PWM_PIN, PWM_FREQ, 0)
        self.lgpio.tx_pwm(self.chip, RIGHT_PWM_PIN, PWM_FREQ, 0)

    # ── Command Velocity Callback ──────────────────────────────────────────
    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()

        vx = msg.linear.x
        wz = msg.angular.z

        # Differential drive inverse kinematics
        v_left = vx - (wz * self.L / 2.0)
        v_right = vx + (wz * self.L / 2.0)

        self.set_motors(v_left, v_right)

    # ── 50 Hz Odometry & Watchdog Loop ─────────────────────────────────────
    def update_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_odom_time).nanoseconds / 1e9

        if dt <= 0.0:
            return

        # Watchdog check: stop motors if /cmd_vel timed out
        time_since_cmd = (current_time - self.last_cmd_time).nanoseconds / 1e9
        if time_since_cmd > self.cmd_timeout:
            self.stop_motors()

        # Capture encoder deltas
        cur_l = self.left_ticks
        cur_r = self.right_ticks

        delta_l = cur_l - self.prev_left_ticks
        delta_r = cur_r - self.prev_right_ticks

        self.prev_left_ticks = cur_l
        self.prev_right_ticks = cur_r

        # Distance traveled by each wheel
        d_left = delta_l * self.meters_per_tick
        d_right = delta_r * self.meters_per_tick

        d_center = (d_right + d_left) / 2.0
        delta_theta = (d_right - d_left) / self.L

        # Runge-Kutta 2nd Order Odometry Integration
        self.x += d_center * math.cos(self.theta + delta_theta / 2.0)
        self.y += d_center * math.sin(self.theta + delta_theta / 2.0)
        self.theta = math.atan2(math.sin(self.theta + delta_theta), math.cos(self.theta + delta_theta))

        # Velocities
        vx = d_center / dt
        wz = delta_theta / dt

        # Orientation Quaternion
        q = euler_to_quaternion(self.theta)

        # Build Odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q

        # Covariance matrices (x, y, z, roll, pitch, yaw)
        odom.pose.covariance = [
            0.001, 0.0,   0.0, 0.0, 0.0, 0.0,
            0.0,   0.001, 0.0, 0.0, 0.0, 0.0,
            0.0,   0.0,   1e6, 0.0, 0.0, 0.0,
            0.0,   0.0,   0.0, 1e6, 0.0, 0.0,
            0.0,   0.0,   0.0, 0.0, 1e6, 0.0,
            0.0,   0.0,   0.0, 0.0, 0.0, 0.01
        ]

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = wz

        odom.twist.covariance = [
            0.001, 0.0,   0.0, 0.0, 0.0, 0.0,
            0.0,   0.001, 0.0, 0.0, 0.0, 0.0,
            0.0,   0.0,   1e6, 0.0, 0.0, 0.0,
            0.0,   0.0,   0.0, 1e6, 0.0, 0.0,
            0.0,   0.0,   0.0, 0.0, 1e6, 0.0,
            0.0,   0.0,   0.0, 0.0, 0.0, 0.01
        ]

        self.odom_pub.publish(odom)

        # Optional TF Broadcast (odom -> base_link)
        if self.publish_tf and self.tf_broadcaster:
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = q
            self.tf_broadcaster.sendTransform(t)

        self.last_odom_time = current_time

    def destroy_node(self):
        self.stop_motors()
        if not self.mock_mode and self.chip is not None:
            for cb in self.callbacks:
                try:
                    cb.cancel()
                except Exception:
                    pass
            try:
                self.lgpio.gpiochip_close(self.chip)
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AMRHardwareDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down AMR Hardware Drive Node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

