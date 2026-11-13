#!/usr/bin/env python3
"""
AMR 6-Axis IMU Node (GY-91 / MPU-6500) for Raspberry Pi 5
=========================================================
Target Sensor:
  - MPU-6500 / MPU-9250 6-Axis Accelerometer + Gyroscope
  - Bus: I2C Bus 1 (/dev/i2c-1)
  - Address: 0x68 (AD0 -> GND)
  - Strictly 6-DOF (No magnetometer polling or I2C bypass, optimal for indoor EKF)

Outputs:
  - Publishes: /imu/data (sensor_msgs/Imu) at 50 Hz
"""

import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion

# MPU-6500 Register Map
MPU_ADDR           = 0x68
REG_PWR_MGMT_1     = 0x6B
REG_ACCEL_CONFIG   = 0x1C
REG_GYRO_CONFIG    = 0x1B
REG_ACCEL_XOUT_H   = 0x3B
REG_GYRO_XOUT_H    = 0x43
REG_WHO_AM_I       = 0x75

GRAVITY_MSS        = 9.80665
ACCEL_SCALE_2G     = 16384.0   # LSB / g
GYRO_SCALE_250DPS  = 131.0     # LSB / (deg/s)
DEG_TO_RAD         = math.pi / 180.0


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class AMRIMUNode(Node):
    def __init__(self):
        super().__init__('amr_imu_node')

        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x68)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('rate', 50.0)
        self.declare_parameter('calibrate_gyro', True)

        self.bus_num = self.get_parameter('i2c_bus').value
        self.addr = self.get_parameter('i2c_address').value
        self.frame_id = self.get_parameter('frame_id').value
        self.rate = self.get_parameter('rate').value
        self.do_calib = self.get_parameter('calibrate_gyro').value

        self.bus = None
        self.mock_mode = False

        # Gyro zero-rate bias
        self.bias_gx = 0.0
        self.bias_gy = 0.0
        self.bias_gz = 0.0

        # Orientation state (yaw integration)
        self.yaw = 0.0
        self.last_time = self.get_clock().now()

        self._init_i2c()

        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        timer_period = 1.0 / self.rate
        self.timer = self.create_timer(timer_period, self.read_and_publish)

        self.get_logger().info(f"AMR IMU Node online (Publishing /imu/data @ {self.rate} Hz)")

    def _init_i2c(self):
        try:
            try:
                from smbus2 import SMBus
            except ImportError:
                from smbus import SMBus

            self.bus = SMBus(self.bus_num)

            # Wake up chip (clear SLEEP bit in PWR_MGMT_1)
            self.bus.write_byte_data(self.addr, REG_PWR_MGMT_1, 0x00)
            time.sleep(0.05)

            # Check WHO_AM_I (0x70 for MPU6500, 0x71 for MPU9250)
            wai = self.bus.read_byte_data(self.addr, REG_WHO_AM_I)
            self.get_logger().info(f"IMU detected at 0x{self.addr:02X} (WHO_AM_I = 0x{wai:02X})")

            # Configure ±2g and ±250°/s
            self.bus.write_byte_data(self.addr, REG_ACCEL_CONFIG, 0x00)
            self.bus.write_byte_data(self.addr, REG_GYRO_CONFIG, 0x00)
            time.sleep(0.02)

            if self.do_calib:
                self._calibrate_gyro()

        except Exception as e:
            self.get_logger().warn(f"Failed to open I2C bus {self.bus_num} ({e}). Running in MOCK mode.")
            self.mock_mode = True

    def _calibrate_gyro(self):
        self.get_logger().info("Calibrating gyroscope zero-rate bias (keep robot stationary)...")
        samples = 100
        sum_gx, sum_gy, sum_gz = 0.0, 0.0, 0.0

        for _ in range(samples):
            try:
                gx, gy, gz = self._read_raw_gyro()
                sum_gx += gx
                sum_gy += gy
                sum_gz += gz
                time.sleep(0.01)
            except Exception:
                pass

        self.bias_gx = sum_gx / samples
        self.bias_gy = sum_gy / samples
        self.bias_gz = sum_gz / samples

        self.get_logger().info(
            f"Gyroscope calibration complete: bias_z = {self.bias_gz:.4f} rad/s"
        )

    def _read_raw_16(self, reg, count=3):
        data = self.bus.read_i2c_block_data(self.addr, reg, count * 2)
        vals = []
        for i in range(count):
            h = data[i * 2]
            l = data[i * 2 + 1]
            val = (h << 8) | l
            if val > 32767:
                val -= 65536
            vals.append(val)
        return vals

    def _read_raw_gyro(self):
        raw = self._read_raw_16(REG_GYRO_XOUT_H, 3)
        # Convert to rad/s
        gx = (raw[0] / GYRO_SCALE_250DPS) * DEG_TO_RAD
        gy = (raw[1] / GYRO_SCALE_250DPS) * DEG_TO_RAD
        gz = (raw[2] / GYRO_SCALE_250DPS) * DEG_TO_RAD
        return gx, gy, gz

    def read_and_publish(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0.0:
            return

        if self.mock_mode:
            ax, ay, az = 0.0, 0.0, GRAVITY_MSS
            gx, gy, gz = 0.0, 0.0, 0.0
        else:
            try:
                # Read Accel (XYZ)
                raw_acc = self._read_raw_16(REG_ACCEL_XOUT_H, 3)
                ax = (raw_acc[0] / ACCEL_SCALE_2G) * GRAVITY_MSS
                ay = (raw_acc[1] / ACCEL_SCALE_2G) * GRAVITY_MSS
                az = (raw_acc[2] / ACCEL_SCALE_2G) * GRAVITY_MSS

                # Read Gyro (XYZ)
                gx_raw, gy_raw, gz_raw = self._read_raw_gyro()
                gx = gx_raw - self.bias_gx
                gy = gy_raw - self.bias_gy
                gz = gz_raw - self.bias_gz

            except OSError:
                # Occasional I2C bus noise/transient NACK — skip sample safely
                self.last_time = current_time
                return

        # Simple yaw integration for planar orientation estimate
        self.yaw += gz * dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        # Roll and Pitch from gravity vector
        roll = math.atan2(ay, az)
        pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))

        q = euler_to_quaternion(roll, pitch, self.yaw)

        # Build sensor_msgs/Imu
        msg = Imu()
        msg.header.stamp = current_time.to_msg()
        msg.header.frame_id = self.frame_id

        msg.orientation = q
        msg.orientation_covariance = [
            1e-4, 0.0,  0.0,
            0.0,  1e-4, 0.0,
            0.0,  0.0,  1e-2
        ]

        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz
        msg.angular_velocity_covariance = [
            1e-4, 0.0,  0.0,
            0.0,  1e-4, 0.0,
            0.0,  0.0,  1e-4
        ]

        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        msg.linear_acceleration_covariance = [
            1e-2, 0.0,  0.0,
            0.0,  1e-2, 0.0,
            0.0,  0.0,  1e-2
        ]

        self.imu_pub.publish(msg)
        self.last_time = current_time

    def destroy_node(self):
        if self.bus:
            try:
                self.bus.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AMRIMUNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down AMR IMU Node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

