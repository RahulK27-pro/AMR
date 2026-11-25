#!/usr/bin/env python3
"""
AMR Differential Drive Hardware & Odometry Node for Raspberry Pi 5
===================================================================
Hardware Interfaces:
  - SBC: Raspberry Pi 5 (RP1 Southbridge via lgpio)
  - Driver: RMCS-2305 Dual DC Motor Driver
  - Encoders: 6-Wire Quadrature Hall-Effect Encoders (3.3V Logic)

Calibrated Constants:
  - Wheel Diameter (D)     : 0.070 m (70 mm)
  - Wheel Radius (r)       : 0.035 m (35 mm)
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
try:
    from rclpy.executors import ExternalShutdownException
except ImportError:
    class ExternalShutdownException(Exception):
        pass
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
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
        self.declare_parameter('wheel_radius', 0.035)
        self.declare_parameter('wheel_separation', 0.240)
        self.declare_parameter('ticks_per_rev', 4713)
        self.declare_parameter('max_wheel_speed', 0.65)       # m/s free speed calibration
        self.declare_parameter('min_pwm', 18.0)               # retained for launch compatibility
        self.declare_parameter('max_pwm', 100.0)              # % duty absolute max
        self.declare_parameter('max_pwm_cap', 35.0)           # % duty safety ceiling for closed-loop PI
        self.declare_parameter('min_pivot_pwm', 16.0)         # retained for launch compatibility
        self.declare_parameter('max_pivot_pwm', 22.0)         # retained for launch compatibility
        self.declare_parameter('ff_offset', 10.5)             # % duty kinetic friction offset
        self.declare_parameter('ff_slope', 55.0)              # % duty per (m/s) feedforward slope
        self.declare_parameter('breakaway_pwm', 17.5)         # % duty static stiction assist while stationary
        self.declare_parameter('pid_kp', 75.0)                # % duty per (m/s) error
        self.declare_parameter('pid_ki', 150.0)               # % duty per m integral error
        self.declare_parameter('odom_rate', 50.0)             # Hz
        self.declare_parameter('cmd_vel_timeout', 1.5)        # seconds safety watchdog
        self.declare_parameter('publish_tf', False)           # Set True only if EKF is disabled
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('swap_encoders', False)
        self.declare_parameter('invert_odom_yaw', False)
        self.declare_parameter('invert_odom_linear', True)

        # Read Parameters
        self.r = self.get_parameter('wheel_radius').value
        self.L = self.get_parameter('wheel_separation').value
        self.ticks_per_rev = float(self.get_parameter('ticks_per_rev').value)
        self.max_speed = self.get_parameter('max_wheel_speed').value
        self.min_pwm = self.get_parameter('min_pwm').value
        self.max_pwm = self.get_parameter('max_pwm').value
        self.max_pwm_cap = float(self.get_parameter('max_pwm_cap').value)
        self.min_pivot_pwm = float(self.get_parameter('min_pivot_pwm').value)
        self.max_pivot_pwm = float(self.get_parameter('max_pivot_pwm').value)
        self.ff_offset = float(self.get_parameter('ff_offset').value)
        self.ff_slope = float(self.get_parameter('ff_slope').value)
        self.breakaway_pwm = float(self.get_parameter('breakaway_pwm').value)
        self.pid_kp = float(self.get_parameter('pid_kp').value)
        self.pid_ki = float(self.get_parameter('pid_ki').value)
        self.cmd_timeout = self.get_parameter('cmd_vel_timeout').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.swap_encoders = self.get_parameter('swap_encoders').value
        self.invert_odom_yaw = self.get_parameter('invert_odom_yaw').value
        self.invert_odom_linear = self.get_parameter('invert_odom_linear').value

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
        self._watchdog_stopped = True

        # Closed-loop per-wheel PI controller state
        self.target_vx = 0.0
        self.target_wz = 0.0
        self.active_duty_l = 0.0
        self.active_duty_r = 0.0
        self._last_motor_log_sec = 0.0
        self._wheel_pi = {
            'left':  {'v_filt': 0.0, 'integ': 0.0, 'prev_sign': 0},
            'right': {'v_filt': 0.0, 'integ': 0.0, 'prev_sign': 0},
        }

        # Hardware Initialization (lgpio)
        self.chip = None
        self.mock_mode = False
        self.callbacks = []

        self._init_hardware()

        # ROS 2 Subscriptions & Publishers
        self.estop_active = False
        self.estop_sub = self.create_subscription(
            Bool,
            '/agv_estop',
            self.estop_callback,
            10
        )
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
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

    # ── Motor Driver Output (50 Hz Per-Wheel Closed-Loop PI) ───────────────
    def _compute_wheel_duty(self, side: str, target_v: float, dt: float) -> float:
        """Compute closed-loop PWM duty (%) for one wheel from filtered encoder velocity."""
        st = self._wheel_pi[side]
        if abs(target_v) < 0.005:
            st['integ'] = 0.0
            st['prev_sign'] = 0
            return 0.0

        sign = 1 if target_v >= 0.0 else -1
        if sign != st['prev_sign']:
            # Reset integral windup on direction reversal so stops/pivots stay crisp
            st['integ'] = 0.0
            st['prev_sign'] = sign

        speed_tgt = abs(target_v)
        speed_meas = st['v_filt'] if sign > 0 else -st['v_filt']
        err = speed_tgt - speed_meas

        ff = self.ff_offset + self.ff_slope * speed_tgt
        # Temporary static-friction boost only while rotor is nearly stalled (< 0.008 m/s)
        kick = max(0.0, min(5.5, self.breakaway_pwm - ff)) if abs(st['v_filt']) < 0.008 else 0.0

        u_unsat = ff + kick + self.pid_kp * err + self.pid_ki * st['integ']
        duty = max(0.0, min(self.max_pwm_cap, u_unsat))

        # Conditional anti-windup: freeze integrator if saturated in the error direction
        if not ((duty >= self.max_pwm_cap and err > 0.0) or (duty <= 0.0 and err < 0.0)):
            st['integ'] = max(-0.20, min(0.20, st['integ'] + err * dt))

        return duty

    def _apply_closed_loop_motors(self, dt: float, now_sec: float):
        """Run independent L/R wheel PI loops and write GPIO PWM outputs."""
        if abs(self.target_vx) < 0.005 and abs(self.target_wz) < 0.005:
            self.stop_motors()
            return

        v_left = self.target_vx - (self.target_wz * self.L / 2.0)
        v_right = self.target_vx + (self.target_wz * self.L / 2.0)

        # Left Motor: 0 = Forward, 1 = Reverse | Right Motor: 1 = Forward, 0 = Reverse
        dir_left = 0 if v_left >= 0.0 else 1
        dir_right = 1 if v_right >= 0.0 else 0

        dt_ctrl = max(0.005, min(0.10, dt))
        duty_left = self._compute_wheel_duty('left', v_left, dt_ctrl)
        duty_right = self._compute_wheel_duty('right', v_right, dt_ctrl)

        self.active_duty_l = duty_left
        self.active_duty_r = duty_right

        if self.mock_mode or self.chip is None:
            return

        self.lgpio.gpio_write(self.chip, LEFT_DIR_PIN, dir_left)
        self.lgpio.tx_pwm(self.chip, LEFT_PWM_PIN, PWM_FREQ, duty_left)

        self.lgpio.gpio_write(self.chip, RIGHT_DIR_PIN, dir_right)
        self.lgpio.tx_pwm(self.chip, RIGHT_PWM_PIN, PWM_FREQ, duty_right)

        if (duty_left > 0.0 or duty_right > 0.0) and (now_sec - self._last_motor_log_sec >= 0.5):
            self._last_motor_log_sec = now_sec
            self.get_logger().info(
                f"[MOTORS PI] L: {v_left:+.3f}m/s (meas {self._wheel_pi['left']['v_filt']:+.3f}) PWM={duty_left:.1f}% | "
                f"R: {v_right:+.3f}m/s (meas {self._wheel_pi['right']['v_filt']:+.3f}) PWM={duty_right:.1f}%"
            )

    def stop_motors(self):
        self.target_vx = 0.0
        self.target_wz = 0.0
        self.active_duty_l = 0.0
        self.active_duty_r = 0.0
        for st in self._wheel_pi.values():
            st['integ'] = 0.0
            st['prev_sign'] = 0
        if self.mock_mode or self.chip is None:
            return
        self.lgpio.tx_pwm(self.chip, LEFT_PWM_PIN, PWM_FREQ, 0)
        self.lgpio.tx_pwm(self.chip, RIGHT_PWM_PIN, PWM_FREQ, 0)

    # ── Emergency Stop & Command Velocity Callbacks ────────────────────────
    def estop_callback(self, msg: Bool):
        if msg.data:
            if not self.estop_active:
                self.get_logger().warn("[E-STOP] /agv_estop triggered — halting motors & locking /cmd_vel.")
            self.estop_active = True
            self.stop_motors()
        else:
            if self.estop_active:
                self.get_logger().info("[E-STOP] /agv_estop cleared — /cmd_vel unlocked.")
            self.estop_active = False

    def cmd_vel_callback(self, msg: Twist):
        if self.estop_active:
            self.stop_motors()
            return
        now = self.get_clock().now()
        self.last_cmd_time = now

        vx = msg.linear.x
        wz = msg.angular.z

        if abs(vx) < 0.005 and abs(wz) < 0.005:
            self._watchdog_stopped = True
            self.stop_motors()
            return

        self._watchdog_stopped = False
        was_idle = (self.active_duty_l == 0.0 and self.active_duty_r == 0.0)
        self.target_vx = vx
        self.target_wz = wz

        # Apply immediately when waking from idle; ongoing ticks are regulated at 50 Hz in update_odometry
        if was_idle:
            self._apply_closed_loop_motors(0.02, now.nanoseconds / 1e9)

    # ── 50 Hz Odometry & Watchdog Loop ─────────────────────────────────────
    def update_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_odom_time).nanoseconds / 1e9

        if dt <= 0.0:
            return
        dt_ctrl = min(dt, 0.25)

        # Watchdog check: halt active motion once when /cmd_vel stream drops
        time_since_cmd = (current_time - self.last_cmd_time).nanoseconds / 1e9
        timed_out = (time_since_cmd > self.cmd_timeout)
        if timed_out:
            was_moving = (
                self.active_duty_l > 0.0 or self.active_duty_r > 0.0 or
                abs(self.target_vx) >= 0.005 or abs(self.target_wz) >= 0.005
            )
            if was_moving or not self._watchdog_stopped:
                self.stop_motors()
                self._watchdog_stopped = True
                if was_moving:
                    self.get_logger().warn(
                        f"[WATCHDOG] /cmd_vel silent for {time_since_cmd:.2f}s (> {self.cmd_timeout:.2f}s); motors halted."
                    )

        # Capture encoder deltas (swap channels if physically inverted)
        if self.swap_encoders:
            cur_l = self.right_ticks
            cur_r = self.left_ticks
        else:
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
        if self.invert_odom_linear:
            d_center = -d_center

        delta_theta = (d_right - d_left) / self.L
        if self.invert_odom_yaw:
            delta_theta = -delta_theta

        # Runge-Kutta 2nd Order Odometry Integration
        self.x += d_center * math.cos(self.theta + delta_theta / 2.0)
        self.y += d_center * math.sin(self.theta + delta_theta / 2.0)
        self.theta = math.atan2(math.sin(self.theta + delta_theta), math.cos(self.theta + delta_theta))

        # Velocities in verified kinematic robot frame (invariant to encoder channel inversion/swap)
        vx = d_center / dt_ctrl
        wz = delta_theta / dt_ctrl
        v_l_meas = vx - (wz * self.L / 2.0)
        v_r_meas = vx + (wz * self.L / 2.0)
        self._wheel_pi['left']['v_filt'] = 0.45 * v_l_meas + 0.55 * self._wheel_pi['left']['v_filt']
        self._wheel_pi['right']['v_filt'] = 0.45 * v_r_meas + 0.55 * self._wheel_pi['right']['v_filt']

        # 50 Hz Closed-loop per-wheel PI velocity control
        if not timed_out and not self.estop_active:
            self._apply_closed_loop_motors(dt_ctrl, current_time.nanoseconds / 1e9)

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

        # Broadcast JointState so robot_state_publisher articulates wheel transforms in RViz
        js = JointState()
        js.header.stamp = current_time.to_msg()
        js.header.frame_id = self.base_frame
        js.name = ['left_wheel_joint', 'right_wheel_joint']
        js.position = [
            (cur_l / self.ticks_per_rev) * 2.0 * math.pi,
            (cur_r / self.ticks_per_rev) * 2.0 * math.pi
        ]
        js.velocity = [
            (d_left / dt_ctrl) / self.r,
            (d_right / dt_ctrl) / self.r
        ]
        self.joint_pub.publish(js)

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
    except (KeyboardInterrupt, ExternalShutdownException):
        try:
            node.get_logger().info("Shutting down AMR Hardware Drive Node...")
        except Exception:
            pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

