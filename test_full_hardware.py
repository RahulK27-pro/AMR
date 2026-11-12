#!/usr/bin/env python3
"""
AMR Full Hardware Validation & Data Acquisition Test
=====================================================
Tests EVERY component on the Raspberry Pi 5 AMR in a single run:

  1. GPIO Access          — Can we claim RP1 southbridge GPIO lines?
  2. I2C Bus              — Is the GY-91 (MPU9250) IMU detected at 0x68?
  3. IMU Data             — Accelerometer + Gyroscope + Magnetometer readings
  4. LiDAR Serial         — Is YDLIDAR X2 accessible on /dev/ttyUSB0?
  5. LiDAR Scan           — Are we receiving 360° point-cloud frames?
  6. Motor PWM            — Brief controlled spin of each motor
  7. Encoder Counting     — Quadrature pulse counting during motor spin
  8. Synchronized Test    — All sensors sampled together while motors spin

Hardware Pinout (from Master Wiring Schematic):
  Motors (RMCS-2305 Driver):
    Left:   PWM = GPIO 12 (Pin 32), DIR = GPIO 16 (Pin 36)
    Right:  PWM = GPIO 13 (Pin 33), DIR = GPIO 26 (Pin 37)
    SLP1/2 tied to 3.3V (always enabled)

  Encoders (6-wire quadrature, 3.3V logic):
    Left:   Ch A = GPIO 17 (Pin 11), Ch B = GPIO 27 (Pin 13)
    Right:  Ch A = GPIO 22 (Pin 15), Ch B = GPIO 23 (Pin 16)

  IMU (GY-91 / MPU9250):
    I2C Bus 1: SDA = GPIO 2 (Pin 3), SCL = GPIO 3 (Pin 5)
    I2C Address: 0x68 (AD0 = GND)

  LiDAR (YDLIDAR X2):
    USB 3.0 → /dev/ttyUSB0, 115200 baud, 360° 2D, ~10 Hz

  Power:
    12.8V 8Ah LiFePO4 → 20A Fuse → E-Stop → XL4015 Buck → 5.1V via GPIO Pins 2&4

Usage:
  python3 test_full_hardware.py              # Run all tests
  python3 test_full_hardware.py --skip-motors # Skip motor spin (bench-safe)
  python3 test_full_hardware.py --mock        # Simulate on PC without hardware
  python3 test_full_hardware.py --report      # Save JSON report to file

Requirements (install on Pi 5):
  sudo apt update && sudo apt install -y python3-lgpio python3-smbus2 i2c-tools
  pip3 install pyserial
"""

import sys
import os
import time
import json
import struct
import argparse
import threading
import traceback
from datetime import datetime
from collections import OrderedDict

# ══════════════════════════════════════════════════════════════════════════════
# Pin Configuration (BCM numbering from wiring schematic)
# ══════════════════════════════════════════════════════════════════════════════
LEFT_PWM_PIN   = 12   # Pin 32 — RMCS PWM1
LEFT_DIR_PIN   = 16   # Pin 36 — RMCS DIR1
RIGHT_PWM_PIN  = 13   # Pin 33 — RMCS PWM2
RIGHT_DIR_PIN  = 26   # Pin 37 — RMCS DIR2

LEFT_ENC_A     = 17   # Pin 11 — Left Encoder Channel A (Yellow)
LEFT_ENC_B     = 27   # Pin 13 — Left Encoder Channel B (White)
RIGHT_ENC_A    = 22   # Pin 15 — Right Encoder Channel A (Yellow)
RIGHT_ENC_B    = 23   # Pin 16 — Right Encoder Channel B (White)

IMU_I2C_BUS    = 1    # /dev/i2c-1
IMU_I2C_ADDR   = 0x68 # MPU9250 (AD0 → GND)
MAG_I2C_ADDR   = 0x0C # AK8963 magnetometer (internal to MPU9250)

LIDAR_PORT     = "/dev/ttyUSB0"
LIDAR_BAUD     = 115200

PWM_FREQ       = 1000  # Hz
TEST_MOTOR_DUTY = 30   # % duty for test spins (conservative)
TEST_MOTOR_SEC  = 2.0  # seconds per motor test

# MPU9250 Register Map (key registers)
MPU9250_WHO_AM_I   = 0x75
MPU9250_PWR_MGMT_1 = 0x6B
MPU9250_ACCEL_XOUT = 0x3B  # 6 bytes: AX_H, AX_L, AY_H, AY_L, AZ_H, AZ_L
MPU9250_GYRO_XOUT  = 0x43  # 6 bytes: GX_H, GX_L, GY_H, GY_L, GZ_H, GZ_L
MPU9250_INT_PIN_CFG = 0x37
AK8963_WHO_AM_I    = 0x00
AK8963_CNTL1       = 0x0A
AK8963_ST1         = 0x02
AK8963_HXL         = 0x03  # 6 bytes: HX_L, HX_H, HY_L, HY_H, HZ_L, HZ_H


# ══════════════════════════════════════════════════════════════════════════════
# Test Result Tracking
# ══════════════════════════════════════════════════════════════════════════════

class TestResults:
    """Collects pass/fail/skip results and data for each subsystem."""

    def __init__(self):
        self.tests = OrderedDict()
        self.start_time = datetime.now()

    def record(self, name, passed, message="", data=None):
        status = "PASS" if passed else "FAIL"
        self.tests[name] = {
            "status": status,
            "message": message,
            "data": data or {},
            "timestamp": datetime.now().isoformat(),
        }
        icon = "✅" if passed else "❌"
        print(f"  {icon}  {name}: {status} — {message}")

    def skip(self, name, reason=""):
        self.tests[name] = {
            "status": "SKIP",
            "message": reason,
            "data": {},
            "timestamp": datetime.now().isoformat(),
        }
        print(f"  ⏭️  {name}: SKIPPED — {reason}")

    def summary(self):
        total = len(self.tests)
        passed = sum(1 for t in self.tests.values() if t["status"] == "PASS")
        failed = sum(1 for t in self.tests.values() if t["status"] == "FAIL")
        skipped = sum(1 for t in self.tests.values() if t["status"] == "SKIP")
        return total, passed, failed, skipped

    def to_dict(self):
        total, passed, failed, skipped = self.summary()
        return {
            "run_date": self.start_time.isoformat(),
            "duration_sec": (datetime.now() - self.start_time).total_seconds(),
            "summary": {
                "total": total, "passed": passed,
                "failed": failed, "skipped": skipped,
            },
            "tests": dict(self.tests),
        }


# ══════════════════════════════════════════════════════════════════════════════
# GPIO / lgpio Hardware Layer
# ══════════════════════════════════════════════════════════════════════════════

class GPIOController:
    """Wraps lgpio for motor PWM + encoder pulse counting on Pi 5 RP1."""

    def __init__(self, mock=False):
        self.mock = mock
        self.chip = None
        self._encoder_counts = {
            "left_a": 0, "left_b": 0,
            "right_a": 0, "right_b": 0,
        }
        self._encoder_lock = threading.Lock()
        self._callbacks = []

        if mock:
            return

        import lgpio
        self.lgpio = lgpio

        # Pi 5 RP1 → typically gpiochip4 (kernel ≥ 6.6), fallback to 0
        for chip_num in [4, 0]:
            try:
                self.chip = lgpio.gpiochip_open(chip_num)
                print(f"  [GPIO] Opened /dev/gpiochip{chip_num}")
                break
            except Exception:
                self.chip = None

        if self.chip is None:
            raise RuntimeError("Cannot open any gpiochip. Are you on a Raspberry Pi 5?")

    # ── Motor Control ──────────────────────────────────────────────────────
    def setup_motors(self):
        if self.mock:
            return
        for pin in [LEFT_DIR_PIN, RIGHT_DIR_PIN]:
            self.lgpio.gpio_claim_output(self.chip, pin, 0)
        # PWM pins: start at 0% duty
        for pin in [LEFT_PWM_PIN, RIGHT_PWM_PIN]:
            self.lgpio.gpio_claim_output(self.chip, pin, 0)

    def set_motor(self, side, duty_pct, forward=True):
        if self.mock:
            # Simulate encoder pulses ~75 per second when motor is on
            if duty_pct > 0:
                with self._encoder_lock:
                    prefix = "left" if side == "left" else "right"
                    self._encoder_counts[f"{prefix}_a"] += 8
                    self._encoder_counts[f"{prefix}_b"] += 8
            return
        if side == "left":
            self.lgpio.gpio_write(self.chip, LEFT_DIR_PIN, 1 if forward else 0)
            self.lgpio.tx_pwm(self.chip, LEFT_PWM_PIN, PWM_FREQ, max(0, min(100, duty_pct)))
        else:
            self.lgpio.gpio_write(self.chip, RIGHT_DIR_PIN, 1 if forward else 0)
            self.lgpio.tx_pwm(self.chip, RIGHT_PWM_PIN, PWM_FREQ, max(0, min(100, duty_pct)))

    def stop_motors(self):
        if self.mock:
            return
        self.lgpio.tx_pwm(self.chip, LEFT_PWM_PIN, PWM_FREQ, 0)
        self.lgpio.tx_pwm(self.chip, RIGHT_PWM_PIN, PWM_FREQ, 0)

    # ── Encoder Reading ────────────────────────────────────────────────────
    def setup_encoders(self):
        if self.mock:
            return
        for pin in [LEFT_ENC_A, LEFT_ENC_B, RIGHT_ENC_A, RIGHT_ENC_B]:
            self.lgpio.gpio_claim_input(self.chip, pin)

        # Set up alert callbacks for rising edges on all encoder channels
        self.lgpio.gpio_claim_alert(self.chip, LEFT_ENC_A, self.lgpio.RISING_EDGE)
        self.lgpio.gpio_claim_alert(self.chip, LEFT_ENC_B, self.lgpio.RISING_EDGE)
        self.lgpio.gpio_claim_alert(self.chip, RIGHT_ENC_A, self.lgpio.RISING_EDGE)
        self.lgpio.gpio_claim_alert(self.chip, RIGHT_ENC_B, self.lgpio.RISING_EDGE)

        cb1 = self.lgpio.callback(self.chip, LEFT_ENC_A, self.lgpio.RISING_EDGE, self._enc_cb)
        cb2 = self.lgpio.callback(self.chip, LEFT_ENC_B, self.lgpio.RISING_EDGE, self._enc_cb)
        cb3 = self.lgpio.callback(self.chip, RIGHT_ENC_A, self.lgpio.RISING_EDGE, self._enc_cb)
        cb4 = self.lgpio.callback(self.chip, RIGHT_ENC_B, self.lgpio.RISING_EDGE, self._enc_cb)
        self._callbacks = [cb1, cb2, cb3, cb4]

    def _enc_cb(self, chip, gpio, level, tick):
        with self._encoder_lock:
            if gpio == LEFT_ENC_A:
                self._encoder_counts["left_a"] += 1
            elif gpio == LEFT_ENC_B:
                self._encoder_counts["left_b"] += 1
            elif gpio == RIGHT_ENC_A:
                self._encoder_counts["right_a"] += 1
            elif gpio == RIGHT_ENC_B:
                self._encoder_counts["right_b"] += 1

    def read_encoder_levels(self):
        """Read raw pin levels (for static connection check)."""
        if self.mock:
            return {"left_a": 0, "left_b": 0, "right_a": 0, "right_b": 0}
        return {
            "left_a": self.lgpio.gpio_read(self.chip, LEFT_ENC_A),
            "left_b": self.lgpio.gpio_read(self.chip, LEFT_ENC_B),
            "right_a": self.lgpio.gpio_read(self.chip, RIGHT_ENC_A),
            "right_b": self.lgpio.gpio_read(self.chip, RIGHT_ENC_B),
        }

    def get_encoder_counts(self):
        with self._encoder_lock:
            return dict(self._encoder_counts)

    def reset_encoder_counts(self):
        with self._encoder_lock:
            for k in self._encoder_counts:
                self._encoder_counts[k] = 0

    # ── Cleanup ────────────────────────────────────────────────────────────
    def cleanup(self):
        if self.mock:
            return
        try:
            self.stop_motors()
        except Exception:
            pass
        for cb in self._callbacks:
            try:
                cb.cancel()
            except Exception:
                pass
        try:
            self.lgpio.gpiochip_close(self.chip)
        except Exception:
            pass


# ══════════════════════════════════════════════════════════════════════════════
# I2C / IMU (MPU9250 + AK8963 Magnetometer)
# ══════════════════════════════════════════════════════════════════════════════

class IMUReader:
    """Reads accelerometer, gyroscope, and magnetometer from GY-91 (MPU9250)."""

    def __init__(self, bus=IMU_I2C_BUS, addr=IMU_I2C_ADDR, mock=False):
        self.mock = mock
        self.addr = addr
        self.bus_num = bus
        self.bus = None

        if mock:
            return

        try:
            from smbus2 import SMBus
            self.SMBus = SMBus
        except ImportError:
            # Fallback to smbus
            try:
                from smbus import SMBus
                self.SMBus = SMBus
            except ImportError:
                raise ImportError(
                    "Neither smbus2 nor smbus found. Install with:\n"
                    "  sudo apt install -y python3-smbus2\n"
                    "  OR: pip3 install smbus2"
                )

    def open(self):
        if self.mock:
            return
        self.bus = self.SMBus(self.bus_num)

    def close(self):
        if self.bus:
            try:
                self.bus.close()
            except Exception:
                pass

    def who_am_i(self):
        """Read WHO_AM_I register. Expected: 0x71 (MPU9250) or 0x73 (MPU9255)."""
        if self.mock:
            return 0x71
        return self.bus.read_byte_data(self.addr, MPU9250_WHO_AM_I)

    def wake_up(self):
        """Clear sleep bit in PWR_MGMT_1."""
        if self.mock:
            return
        self.bus.write_byte_data(self.addr, MPU9250_PWR_MGMT_1, 0x00)
        time.sleep(0.1)

    def enable_magnetometer_passthrough(self):
        """Enable I2C bypass so we can talk directly to AK8963 magnetometer."""
        if self.mock:
            return
        # Set BYPASS_EN bit in INT_PIN_CFG register
        self.bus.write_byte_data(self.addr, MPU9250_INT_PIN_CFG, 0x02)
        time.sleep(0.05)

    def mag_who_am_i(self):
        """Read AK8963 WHO_AM_I. Expected: 0x48."""
        if self.mock:
            return 0x48
        return self.bus.read_byte_data(MAG_I2C_ADDR, AK8963_WHO_AM_I)

    def mag_init(self):
        """Set AK8963 to continuous measurement mode 2 (100 Hz), 16-bit output."""
        if self.mock:
            return
        # Power down first
        self.bus.write_byte_data(MAG_I2C_ADDR, AK8963_CNTL1, 0x00)
        time.sleep(0.01)
        # 16-bit, continuous mode 2 (100 Hz)
        self.bus.write_byte_data(MAG_I2C_ADDR, AK8963_CNTL1, 0x16)
        time.sleep(0.01)

    def _read_raw_16(self, addr, reg, count=1):
        """Read count × 2 bytes as signed 16-bit big-endian values."""
        data = self.bus.read_i2c_block_data(addr, reg, count * 2)
        values = []
        for i in range(count):
            high = data[i * 2]
            low = data[i * 2 + 1]
            val = (high << 8) | low
            if val > 32767:
                val -= 65536
            values.append(val)
        return values

    def read_accel_raw(self):
        """Read raw accelerometer XYZ (default ±2g, 16384 LSB/g)."""
        if self.mock:
            return {"ax": 200, "ay": -150, "az": 16384}
        vals = self._read_raw_16(self.addr, MPU9250_ACCEL_XOUT, 3)
        return {"ax": vals[0], "ay": vals[1], "az": vals[2]}

    def read_gyro_raw(self):
        """Read raw gyroscope XYZ (default ±250°/s, 131 LSB/°/s)."""
        if self.mock:
            return {"gx": 5, "gy": -3, "gz": 1}
        vals = self._read_raw_16(self.addr, MPU9250_GYRO_XOUT, 3)
        return {"gx": vals[0], "gy": vals[1], "gz": vals[2]}

    def read_mag_raw(self):
        """Read raw magnetometer XYZ from AK8963 (little-endian!)."""
        if self.mock:
            return {"mx": 120, "my": -80, "mz": 200}
        # Check data ready
        st1 = self.bus.read_byte_data(MAG_I2C_ADDR, AK8963_ST1)
        if not (st1 & 0x01):
            time.sleep(0.01)
        # AK8963 data is little-endian (unlike MPU9250)
        data = self.bus.read_i2c_block_data(MAG_I2C_ADDR, AK8963_HXL, 7)  # 6 data + ST2
        mx = struct.unpack_from('<h', bytes(data[0:2]))[0]
        my = struct.unpack_from('<h', bytes(data[2:4]))[0]
        mz = struct.unpack_from('<h', bytes(data[4:6]))[0]
        return {"mx": mx, "my": my, "mz": mz}

    def read_accel_g(self):
        """Accelerometer in g units (±2g range)."""
        raw = self.read_accel_raw()
        scale = 16384.0
        return {k: v / scale for k, v in raw.items()}

    def read_gyro_dps(self):
        """Gyroscope in degrees per second (±250°/s range)."""
        raw = self.read_gyro_raw()
        scale = 131.0
        return {k: v / scale for k, v in raw.items()}


# ══════════════════════════════════════════════════════════════════════════════
# YDLIDAR X2 Serial Reader
# ══════════════════════════════════════════════════════════════════════════════

class LidarReader:
    """Reads raw scan frames from YDLIDAR X2 over serial."""

    HEADER = bytes([0xAA, 0x55])

    def __init__(self, port=LIDAR_PORT, baud=LIDAR_BAUD, mock=False):
        self.port = port
        self.baud = baud
        self.mock = mock
        self.ser = None

    def open(self):
        if self.mock:
            return
        import serial
        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baud,
            timeout=2.0,
            bytesize=8,
            parity='N',
            stopbits=1,
        )
        # Flush any stale data
        self.ser.reset_input_buffer()

    def close(self):
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass

    def read_one_frame(self, timeout=3.0):
        """
        Reads one scan packet from the YDLIDAR X2 protocol.
        Returns dict with header info and point count, or None on timeout.
        """
        if self.mock:
            time.sleep(0.1)  # Simulate ~10 Hz scan rate
            return {
                "type": 0x00,
                "sample_count": 40,
                "start_angle": 0.0,
                "end_angle": 45.0,
                "raw_bytes": 80,
            }

        start = time.time()
        while time.time() - start < timeout:
            # Sync to 0xAA 0x55 header
            b = self.ser.read(1)
            if not b:
                continue
            if b[0] != 0xAA:
                continue
            b2 = self.ser.read(1)
            if not b2 or b2[0] != 0x55:
                continue

            # Read packet header (8 bytes after sync)
            hdr = self.ser.read(8)
            if len(hdr) < 8:
                continue

            pkt_type = hdr[0]
            sample_count = hdr[1]
            start_angle = struct.unpack_from('<H', hdr, 2)[0] / 64.0
            end_angle = struct.unpack_from('<H', hdr, 4)[0] / 64.0

            # Read sample data (2 bytes per sample for X2)
            if sample_count > 0:
                data_bytes = self.ser.read(sample_count * 2)
            else:
                data_bytes = b''

            return {
                "type": pkt_type,
                "sample_count": sample_count,
                "start_angle": start_angle,
                "end_angle": end_angle,
                "raw_bytes": len(data_bytes),
            }

        return None

    def collect_frames(self, duration=2.0):
        """Collect scan frames for a given duration. Returns list of frame dicts."""
        frames = []
        start = time.time()
        while time.time() - start < duration:
            frame = self.read_one_frame(timeout=1.0)
            if frame:
                frames.append(frame)
        return frames


# ══════════════════════════════════════════════════════════════════════════════
# Individual Test Functions
# ══════════════════════════════════════════════════════════════════════════════

def test_gpio_access(results, mock=False):
    """Test 1: Can we open the RP1 GPIO chip?"""
    print("\n" + "━" * 70)
    print("  TEST 1: GPIO Access (RP1 Southbridge)")
    print("━" * 70)

    if mock:
        results.record("GPIO Access", True, "Mock mode — GPIO simulated", {"chip": "mock"})
        return True

    try:
        import lgpio
        for chip_num in [4, 0]:
            try:
                chip = lgpio.gpiochip_open(chip_num)
                info = f"/dev/gpiochip{chip_num} opened successfully"
                lgpio.gpiochip_close(chip)
                results.record("GPIO Access", True, info, {"chip": chip_num})
                return True
            except Exception:
                continue
        results.record("GPIO Access", False, "No gpiochip found (0 or 4)")
        return False
    except ImportError:
        results.record("GPIO Access", False,
                       "lgpio not installed. Run: sudo apt install -y python3-lgpio")
        return False
    except Exception as e:
        results.record("GPIO Access", False, str(e))
        return False


def test_i2c_bus(results, mock=False):
    """Test 2: I2C bus scan — is the IMU detected?"""
    print("\n" + "━" * 70)
    print("  TEST 2: I2C Bus Scan (GY-91 / MPU9250 IMU)")
    print("━" * 70)

    if mock:
        results.record("I2C Bus Scan", True, "Mock: IMU at 0x68", {"address": "0x68"})
        return True

    # First try i2cdetect command
    import subprocess
    try:
        out = subprocess.run(
            ["i2cdetect", "-y", str(IMU_I2C_BUS)],
            capture_output=True, text=True, timeout=5
        )
        print(f"  i2cdetect output:\n{out.stdout}")
        # Check if 0x68 appears
        if "68" in out.stdout:
            results.record("I2C Bus Scan", True,
                           f"MPU9250 detected at 0x68 on I2C bus {IMU_I2C_BUS}",
                           {"i2cdetect": out.stdout.strip()})
            return True
        else:
            results.record("I2C Bus Scan", False,
                           f"0x68 NOT found on I2C bus {IMU_I2C_BUS}. Check SDA/SCL wiring.",
                           {"i2cdetect": out.stdout.strip()})
            return False
    except FileNotFoundError:
        results.record("I2C Bus Scan", False,
                       "i2cdetect not found. Run: sudo apt install -y i2c-tools")
        return False
    except Exception as e:
        results.record("I2C Bus Scan", False, f"i2cdetect failed: {e}")
        return False


def test_imu_data(results, mock=False):
    """Test 3: Read IMU WHO_AM_I + accel/gyro/mag data."""
    print("\n" + "━" * 70)
    print("  TEST 3: IMU Data Acquisition (Accel + Gyro + Mag)")
    print("━" * 70)

    imu = IMUReader(mock=mock)
    try:
        imu.open()
        imu.wake_up()

        # WHO_AM_I check
        wai = imu.who_am_i()
        wai_ok = wai in (0x71, 0x73, 0x70)  # MPU9250=0x71, MPU9255=0x73
        print(f"  WHO_AM_I = 0x{wai:02X} ({'OK' if wai_ok else 'UNEXPECTED'})")

        if not wai_ok:
            results.record("IMU WHO_AM_I", False,
                           f"Got 0x{wai:02X}, expected 0x71 (MPU9250) or 0x73 (MPU9255)")
            return False

        results.record("IMU WHO_AM_I", True, f"0x{wai:02X}")

        # Read accelerometer (multiple samples)
        print("  Reading accelerometer (5 samples)...")
        accel_samples = []
        for _ in range(5):
            a = imu.read_accel_g()
            accel_samples.append(a)
            time.sleep(0.05)

        avg_ax = sum(s["ax"] for s in accel_samples) / len(accel_samples)
        avg_ay = sum(s["ay"] for s in accel_samples) / len(accel_samples)
        avg_az = sum(s["az"] for s in accel_samples) / len(accel_samples)
        print(f"    Avg: ax={avg_ax:+.3f}g  ay={avg_ay:+.3f}g  az={avg_az:+.3f}g")

        # At rest on flat surface, expect az ≈ ±1g
        az_ok = 0.5 < abs(avg_az) < 1.8
        results.record("IMU Accelerometer", az_ok,
                       f"ax={avg_ax:+.3f}g ay={avg_ay:+.3f}g az={avg_az:+.3f}g "
                       f"({'gravity detected' if az_ok else 'no gravity — check orientation or wiring'})",
                       {"avg_ax": avg_ax, "avg_ay": avg_ay, "avg_az": avg_az})

        # Read gyroscope
        print("  Reading gyroscope (5 samples)...")
        gyro_samples = []
        for _ in range(5):
            g = imu.read_gyro_dps()
            gyro_samples.append(g)
            time.sleep(0.05)

        avg_gx = sum(s["gx"] for s in gyro_samples) / len(gyro_samples)
        avg_gy = sum(s["gy"] for s in gyro_samples) / len(gyro_samples)
        avg_gz = sum(s["gz"] for s in gyro_samples) / len(gyro_samples)
        print(f"    Avg: gx={avg_gx:+.2f}°/s  gy={avg_gy:+.2f}°/s  gz={avg_gz:+.2f}°/s")

        # At rest, gyro should be near zero (within ±10°/s for uncalibrated)
        gyro_ok = all(abs(v) < 20 for v in [avg_gx, avg_gy, avg_gz])
        results.record("IMU Gyroscope", gyro_ok,
                       f"gx={avg_gx:+.2f} gy={avg_gy:+.2f} gz={avg_gz:+.2f} °/s "
                       f"({'stable at rest' if gyro_ok else 'excessive drift — check wiring'})",
                       {"avg_gx": avg_gx, "avg_gy": avg_gy, "avg_gz": avg_gz})

        # Read magnetometer (AK8963)
        print("  Enabling magnetometer passthrough...")
        try:
            imu.enable_magnetometer_passthrough()
            mag_wai = imu.mag_who_am_i()
            print(f"  AK8963 WHO_AM_I = 0x{mag_wai:02X} ({'OK' if mag_wai == 0x48 else 'UNEXPECTED'})")

            if mag_wai == 0x48:
                imu.mag_init()
                time.sleep(0.1)
                print("  Reading magnetometer (5 samples)...")
                mag_samples = []
                for _ in range(5):
                    m = imu.read_mag_raw()
                    mag_samples.append(m)
                    time.sleep(0.05)

                avg_mx = sum(s["mx"] for s in mag_samples) / len(mag_samples)
                avg_my = sum(s["my"] for s in mag_samples) / len(mag_samples)
                avg_mz = sum(s["mz"] for s in mag_samples) / len(mag_samples)
                print(f"    Avg: mx={avg_mx:+.0f}  my={avg_my:+.0f}  mz={avg_mz:+.0f}")

                # Basic sanity: at least one axis should be non-zero
                mag_ok = any(abs(v) > 5 for v in [avg_mx, avg_my, avg_mz])
                results.record("IMU Magnetometer", mag_ok,
                               f"mx={avg_mx:+.0f} my={avg_my:+.0f} mz={avg_mz:+.0f} "
                               f"({'field detected' if mag_ok else 'all near zero — check module'})",
                               {"avg_mx": avg_mx, "avg_my": avg_my, "avg_mz": avg_mz})
            else:
                results.record("IMU Magnetometer", False,
                               f"AK8963 WHO_AM_I = 0x{mag_wai:02X}, expected 0x48")
        except Exception as e:
            results.record("IMU Magnetometer", False, f"Mag passthrough failed: {e}")

        return True

    except Exception as e:
        results.record("IMU Data", False, f"Exception: {e}")
        traceback.print_exc()
        return False
    finally:
        imu.close()


def test_lidar_connection(results, mock=False):
    """Test 4: Check YDLIDAR X2 serial port exists and is accessible."""
    print("\n" + "━" * 70)
    print("  TEST 4: LiDAR Serial Connection (YDLIDAR X2)")
    print("━" * 70)

    if mock:
        results.record("LiDAR Serial Port", True, f"Mock: {LIDAR_PORT} simulated")
        return True

    # Check device node exists
    if not os.path.exists(LIDAR_PORT):
        # Try to find any ttyUSB device
        import glob
        usb_ports = glob.glob("/dev/ttyUSB*")
        if usb_ports:
            results.record("LiDAR Serial Port", False,
                           f"{LIDAR_PORT} not found, but found: {usb_ports}. "
                           f"Try: --lidar-port {usb_ports[0]}")
        else:
            results.record("LiDAR Serial Port", False,
                           f"{LIDAR_PORT} not found. No /dev/ttyUSB* devices. "
                           "Check USB cable and UART adapter board.")
        return False

    # Check read permission
    if not os.access(LIDAR_PORT, os.R_OK | os.W_OK):
        results.record("LiDAR Serial Port", False,
                       f"Permission denied on {LIDAR_PORT}. "
                       f"Run: sudo usermod -aG dialout $USER && reboot")
        return False

    results.record("LiDAR Serial Port", True,
                   f"{LIDAR_PORT} exists and accessible")
    return True


def test_lidar_data(results, mock=False):
    """Test 5: Read actual scan frames from YDLIDAR X2."""
    print("\n" + "━" * 70)
    print("  TEST 5: LiDAR Scan Data Acquisition")
    print("━" * 70)

    lidar = LidarReader(mock=mock)
    try:
        lidar.open()
        print(f"  Collecting scan frames for 3 seconds...")
        frames = lidar.collect_frames(duration=3.0)
        print(f"  Received {len(frames)} frames in 3 seconds")

        if len(frames) == 0:
            results.record("LiDAR Scan Data", False,
                           "No scan frames received in 3s. "
                           "Check: is the LiDAR motor spinning? Power OK?")
            return False

        total_points = sum(f["sample_count"] for f in frames)
        avg_samples = total_points / len(frames) if frames else 0
        frame_rate = len(frames) / 3.0

        # X2 should deliver roughly 7-12 Hz of full rotations
        # (multiple packets per rotation)
        data_ok = len(frames) >= 3  # At least 1 frame/sec

        results.record("LiDAR Scan Data", data_ok,
                       f"{len(frames)} frames, ~{frame_rate:.1f} frames/sec, "
                       f"avg {avg_samples:.0f} samples/frame, total {total_points} points",
                       {"frame_count": len(frames), "frame_rate_hz": frame_rate,
                        "total_points": total_points, "avg_samples_per_frame": avg_samples})
        return data_ok

    except ImportError:
        results.record("LiDAR Scan Data", False,
                       "pyserial not installed. Run: pip3 install pyserial")
        return False
    except Exception as e:
        results.record("LiDAR Scan Data", False, f"Exception: {e}")
        traceback.print_exc()
        return False
    finally:
        lidar.close()


def test_encoder_static(results, gpio, mock=False):
    """Test 6: Check encoder pin levels (static connectivity check)."""
    print("\n" + "━" * 70)
    print("  TEST 6: Encoder Pin Connectivity (Static Level Read)")
    print("━" * 70)

    if mock:
        results.record("Encoder Pins", True, "Mock: encoder pins simulated")
        return True

    try:
        levels = gpio.read_encoder_levels()
        print(f"  Left  Encoder:  Ch A (GPIO {LEFT_ENC_A}) = {levels['left_a']}, "
              f"Ch B (GPIO {LEFT_ENC_B}) = {levels['left_b']}")
        print(f"  Right Encoder:  Ch A (GPIO {RIGHT_ENC_A}) = {levels['right_a']}, "
              f"Ch B (GPIO {RIGHT_ENC_B}) = {levels['right_b']}")

        # We can't determine pass/fail from static levels alone (they depend
        # on wheel position), but we CAN check that pins are readable
        results.record("Encoder Pins", True,
                       f"Pins readable — L(A={levels['left_a']},B={levels['left_b']}) "
                       f"R(A={levels['right_a']},B={levels['right_b']})",
                       levels)
        return True
    except Exception as e:
        results.record("Encoder Pins", False, f"Failed to read encoder pins: {e}")
        return False


def test_motor_with_encoders(results, gpio, mock=False):
    """Test 7: Spin each motor briefly and count encoder pulses."""
    print("\n" + "━" * 70)
    print("  TEST 7: Motor Spin + Encoder Pulse Count")
    print("━" * 70)

    if mock:
        results.record("Left Motor + Encoder", True,
                       "Mock: simulated 150 pulses", {"pulses_a": 150, "pulses_b": 150})
        results.record("Right Motor + Encoder", True,
                       "Mock: simulated 150 pulses", {"pulses_a": 150, "pulses_b": 150})
        return True

    all_ok = True

    for side, enc_a_key, enc_b_key in [
        ("left", "left_a", "left_b"),
        ("right", "right_a", "right_b"),
    ]:
        label = side.upper()
        print(f"\n  ▶ Spinning {label} motor at {TEST_MOTOR_DUTY}% for {TEST_MOTOR_SEC}s...")

        gpio.reset_encoder_counts()
        time.sleep(0.1)  # Let encoder ISR settle

        gpio.set_motor(side, TEST_MOTOR_DUTY, forward=True)
        time.sleep(TEST_MOTOR_SEC)
        gpio.stop_motors()
        time.sleep(0.3)  # Let trailing pulses register

        counts = gpio.get_encoder_counts()
        pa = counts[enc_a_key]
        pb = counts[enc_b_key]
        print(f"    {label} Encoder: Ch A = {pa} pulses, Ch B = {pb} pulses")

        # Expect at least a few pulses if motor actually spun
        motor_ok = pa > 5 or pb > 5
        if not motor_ok:
            print(f"    ⚠️  Very few pulses — motor may not have spun, or encoder wiring issue.")

        results.record(f"{label} Motor + Encoder", motor_ok,
                       f"Ch A = {pa} pulses, Ch B = {pb} pulses in {TEST_MOTOR_SEC}s"
                       f" at {TEST_MOTOR_DUTY}% duty"
                       f" ({'OK' if motor_ok else 'low count — check motor/encoder wiring'})",
                       {"pulses_a": pa, "pulses_b": pb,
                        "duty_pct": TEST_MOTOR_DUTY, "duration_s": TEST_MOTOR_SEC})

        if not motor_ok:
            all_ok = False

    return all_ok


def test_synchronized(results, gpio, mock=False):
    """Test 8: All sensors sampled simultaneously while motors spin briefly."""
    print("\n" + "━" * 70)
    print("  TEST 8: Synchronized Full-System Data Acquisition")
    print("━" * 70)

    imu = IMUReader(mock=mock)
    lidar = LidarReader(mock=mock)
    sync_data = {
        "imu_samples": [],
        "lidar_frames": [],
        "encoder_start": {},
        "encoder_end": {},
        "duration_sec": 0,
    }

    try:
        imu.open()
        imu.wake_up()
        lidar.open()
        gpio.reset_encoder_counts()

        print(f"  Starting synchronized capture (motors at {TEST_MOTOR_DUTY}%, 3 seconds)...")
        print(f"  Sampling: IMU (accel+gyro) + LiDAR frames + Encoder counts")

        sync_data["encoder_start"] = gpio.get_encoder_counts()

        # Start motors (both forward for straight line)
        gpio.set_motor("left", TEST_MOTOR_DUTY, forward=True)
        gpio.set_motor("right", TEST_MOTOR_DUTY, forward=True)

        start = time.time()
        sample_interval = 0.05  # 20 Hz IMU sampling
        last_imu_sample = 0

        # Lidar reads in background thread
        lidar_frames = []
        lidar_stop = threading.Event()

        def lidar_worker():
            while not lidar_stop.is_set():
                frame = lidar.read_one_frame(timeout=0.5)
                if frame:
                    frame["capture_time"] = time.time() - start
                    lidar_frames.append(frame)

        lidar_thread = threading.Thread(target=lidar_worker, daemon=True)
        lidar_thread.start()

        # Main loop: sample IMU at 20 Hz for 3 seconds
        while time.time() - start < 3.0:
            now = time.time()
            if now - last_imu_sample >= sample_interval:
                accel = imu.read_accel_g()
                gyro = imu.read_gyro_dps()
                enc = gpio.get_encoder_counts()
                sync_data["imu_samples"].append({
                    "t": now - start,
                    "accel": accel,
                    "gyro": gyro,
                    "encoder": enc,
                })
                last_imu_sample = now
            time.sleep(0.005)

        # Stop everything
        gpio.stop_motors()
        lidar_stop.set()
        lidar_thread.join(timeout=2.0)

        sync_data["encoder_end"] = gpio.get_encoder_counts()
        sync_data["lidar_frames"] = lidar_frames
        sync_data["duration_sec"] = time.time() - start

        # Evaluate
        n_imu = len(sync_data["imu_samples"])
        n_lidar = len(lidar_frames)
        enc_delta = {
            k: sync_data["encoder_end"][k] - sync_data["encoder_start"].get(k, 0)
            for k in sync_data["encoder_end"]
        }

        print(f"\n  ── Synchronized Capture Results ──")
        print(f"  IMU samples collected : {n_imu} ({n_imu/3.0:.1f} Hz)")
        print(f"  LiDAR frames collected: {n_lidar} ({n_lidar/3.0:.1f} Hz)")
        print(f"  Encoder deltas        : {enc_delta}")

        imu_ok = n_imu >= 30  # expect ~60 at 20 Hz
        lidar_ok = n_lidar >= 3
        enc_ok = any(v > 5 for v in enc_delta.values())

        all_ok = imu_ok and lidar_ok and enc_ok
        status_parts = []
        if imu_ok:
            status_parts.append(f"IMU:{n_imu}samples✅")
        else:
            status_parts.append(f"IMU:{n_imu}samples❌")
        if lidar_ok:
            status_parts.append(f"LiDAR:{n_lidar}frames✅")
        else:
            status_parts.append(f"LiDAR:{n_lidar}frames❌")
        if enc_ok:
            status_parts.append(f"Enc:{enc_delta}✅")
        else:
            status_parts.append(f"Enc:{enc_delta}❌(no pulses)")

        results.record("Synchronized System Test", all_ok,
                       " | ".join(status_parts),
                       {"imu_samples": n_imu, "lidar_frames": n_lidar,
                        "encoder_delta": enc_delta, "duration": sync_data["duration_sec"]})

        return all_ok

    except Exception as e:
        gpio.stop_motors()
        results.record("Synchronized System Test", False, f"Exception: {e}")
        traceback.print_exc()
        return False
    finally:
        imu.close()
        lidar.close()


# ══════════════════════════════════════════════════════════════════════════════
# Main Entry Point
# ══════════════════════════════════════════════════════════════════════════════

def main():
    global TEST_MOTOR_DUTY, TEST_MOTOR_SEC, LIDAR_PORT

    parser = argparse.ArgumentParser(
        description="AMR Full Hardware Validation for Raspberry Pi 5",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 test_full_hardware.py               # Full test on Pi 5
  python3 test_full_hardware.py --skip-motors  # Skip motor tests (safe for bench)
  python3 test_full_hardware.py --mock         # Dry-run on laptop/PC
  python3 test_full_hardware.py --report       # Save JSON report
        """)
    parser.add_argument('--mock', action='store_true',
                        help='Run in simulation mode (no real hardware)')
    parser.add_argument('--skip-motors', action='store_true',
                        help='Skip motor spin tests (use for passive sensor checks)')
    parser.add_argument('--skip-lidar', action='store_true',
                        help='Skip LiDAR tests')
    parser.add_argument('--skip-imu', action='store_true',
                        help='Skip IMU tests')
    parser.add_argument('--report', action='store_true',
                        help='Save JSON test report to file')
    parser.add_argument('--report-file', type=str,
                        default='hardware_test_report.json',
                        help='Output JSON report path')
    parser.add_argument('--motor-duty', type=int, default=TEST_MOTOR_DUTY,
                        help=f'Motor test duty cycle %% (default: {TEST_MOTOR_DUTY})')
    parser.add_argument('--motor-time', type=float, default=TEST_MOTOR_SEC,
                        help=f'Motor test duration in seconds (default: {TEST_MOTOR_SEC})')
    parser.add_argument('--lidar-port', type=str, default=LIDAR_PORT,
                        help=f'LiDAR serial port (default: {LIDAR_PORT})')
    args = parser.parse_args()

    # Apply overrides
    TEST_MOTOR_DUTY = max(10, min(60, args.motor_duty))
    TEST_MOTOR_SEC = max(0.5, min(5.0, args.motor_time))
    LIDAR_PORT = args.lidar_port

    results = TestResults()
    gpio = None

    print("=" * 70)
    print("  AMR FULL HARDWARE VALIDATION — Raspberry Pi 5")
    print("  " + datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
    print("=" * 70)
    print(f"  Mode: {'MOCK / SIMULATION' if args.mock else 'LIVE HARDWARE'}")
    print(f"  Motor tests: {'SKIP' if args.skip_motors else f'{TEST_MOTOR_DUTY}% duty, {TEST_MOTOR_SEC}s'}")
    print(f"  LiDAR port: {LIDAR_PORT}")

    try:
        # ── Test 1: GPIO ──
        gpio_ok = test_gpio_access(results, mock=args.mock)

        # Initialize GPIO controller for later tests
        if gpio_ok or args.mock:
            gpio = GPIOController(mock=args.mock)
            try:
                gpio.setup_motors()
                gpio.setup_encoders()
            except Exception as e:
                print(f"  ⚠️  GPIO setup warning: {e}")
                if not args.mock:
                    traceback.print_exc()

        # ── Test 2: I2C Bus ──
        if not args.skip_imu:
            test_i2c_bus(results, mock=args.mock)
        else:
            results.skip("I2C Bus Scan", "Skipped by --skip-imu")

        # ── Test 3: IMU Data ──
        if not args.skip_imu:
            test_imu_data(results, mock=args.mock)
        else:
            results.skip("IMU Data", "Skipped by --skip-imu")

        # ── Test 4: LiDAR Connection ──
        if not args.skip_lidar:
            lidar_port_ok = test_lidar_connection(results, mock=args.mock)
        else:
            results.skip("LiDAR Serial Port", "Skipped by --skip-lidar")
            lidar_port_ok = False

        # ── Test 5: LiDAR Data ──
        if not args.skip_lidar and lidar_port_ok:
            test_lidar_data(results, mock=args.mock)
        elif args.skip_lidar:
            results.skip("LiDAR Scan Data", "Skipped by --skip-lidar")
        else:
            results.skip("LiDAR Scan Data", "Skipped — serial port not accessible")

        # ── Test 6: Encoder Static ──
        if gpio:
            test_encoder_static(results, gpio, mock=args.mock)
        else:
            results.skip("Encoder Pins", "Skipped — GPIO not available")

        # ── Test 7: Motor + Encoder ──
        if not args.skip_motors and gpio:
            print("\n  ⚠️  MOTORS WILL SPIN! Ensure robot is elevated on blocks.")
            if not args.mock:
                print("  Press ENTER to continue (or Ctrl+C to abort)...", end='', flush=True)
                try:
                    input()
                except KeyboardInterrupt:
                    results.skip("Motor + Encoder Tests", "Aborted by user")
                    raise
            test_motor_with_encoders(results, gpio, mock=args.mock)
        else:
            reason = "--skip-motors" if args.skip_motors else "GPIO not available"
            results.skip("Left Motor + Encoder", f"Skipped — {reason}")
            results.skip("Right Motor + Encoder", f"Skipped — {reason}")

        # ── Test 8: Synchronized Test ──
        if not args.skip_motors and gpio and not args.skip_imu and not args.skip_lidar:
            print("\n  ⚠️  SYNCHRONIZED TEST: Motors + IMU + LiDAR + Encoders together!")
            if not args.mock:
                print("  Press ENTER to continue (or Ctrl+C to abort)...", end='', flush=True)
                try:
                    input()
                except KeyboardInterrupt:
                    results.skip("Synchronized System Test", "Aborted by user")
                    raise
            test_synchronized(results, gpio, mock=args.mock)
        else:
            reasons = []
            if args.skip_motors:
                reasons.append("motors skipped")
            if args.skip_imu:
                reasons.append("IMU skipped")
            if args.skip_lidar:
                reasons.append("LiDAR skipped")
            if not gpio:
                reasons.append("GPIO unavailable")
            results.skip("Synchronized System Test",
                         f"Skipped — {', '.join(reasons) if reasons else 'prerequisites not met'}")

    except KeyboardInterrupt:
        print("\n\n  [!] Test run interrupted by user.")
    finally:
        if gpio:
            gpio.cleanup()

    # ── Final Report ──
    print("\n" + "═" * 70)
    print("  FINAL TEST REPORT")
    print("═" * 70)

    total, passed, failed, skipped = results.summary()
    for name, info in results.tests.items():
        icon = {"PASS": "✅", "FAIL": "❌", "SKIP": "⏭️"}[info["status"]]
        print(f"  {icon}  {info['status']:4s}  {name}: {info['message']}")

    print("─" * 70)
    color_pass = "\033[1;32m" if failed == 0 else "\033[1;33m"
    color_fail = "\033[1;31m" if failed > 0 else "\033[1;32m"
    color_reset = "\033[0m"

    print(f"  Total: {total}  |  "
          f"{color_pass}Passed: {passed}{color_reset}  |  "
          f"{color_fail}Failed: {failed}{color_reset}  |  "
          f"Skipped: {skipped}")

    if failed == 0 and skipped == 0:
        print(f"\n  {color_pass}🎉  ALL HARDWARE COMPONENTS VERIFIED — READY FOR AMR PROJECT!{color_reset}")
    elif failed == 0:
        print(f"\n  {color_pass}✅  All tested components passed. "
              f"({skipped} tests skipped){color_reset}")
    else:
        print(f"\n  {color_fail}⚠️  {failed} test(s) FAILED. "
              f"Review the issues above before proceeding.{color_reset}")

    print("═" * 70)

    # Save report
    if args.report:
        report_data = results.to_dict()
        report_data["hardware_config"] = {
            "left_motor": {"pwm": LEFT_PWM_PIN, "dir": LEFT_DIR_PIN},
            "right_motor": {"pwm": RIGHT_PWM_PIN, "dir": RIGHT_DIR_PIN},
            "left_encoder": {"ch_a": LEFT_ENC_A, "ch_b": LEFT_ENC_B},
            "right_encoder": {"ch_a": RIGHT_ENC_A, "ch_b": RIGHT_ENC_B},
            "imu": {"bus": IMU_I2C_BUS, "addr": f"0x{IMU_I2C_ADDR:02X}"},
            "lidar": {"port": LIDAR_PORT, "baud": LIDAR_BAUD},
        }

        with open(args.report_file, 'w') as f:
            json.dump(report_data, f, indent=2, default=str)
        print(f"\n  📄 Report saved to: {os.path.abspath(args.report_file)}")


if __name__ == '__main__':
    main()
