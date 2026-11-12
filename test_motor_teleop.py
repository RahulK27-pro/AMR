#!/usr/bin/env python3
"""
AMR Motor Hardware Testing & WASD Teleoperation Script
======================================================
Target Hardware:
  - SBC: Raspberry Pi 5 (Ubuntu 24.04 / ROS 2 Jazzy)
  - Driver: RMCS-2305 Dual DC Motor Driver (PWM Mode)
  - Motors: 12V DC Geared Motors (Differential Drive)

Pinout (RP1 40-Pin GPIO Header):
  - Left Motor:
      * PWM1: GPIO 12 (Physical Pin 32)
      * DIR1: GPIO 16 (Physical Pin 36)
  - Right Motor:
      * PWM2: GPIO 13 (Physical Pin 33)
      * DIR2: GPIO 26 (Physical Pin 37)
  - Sleep / Enable:
      * SLP1 & SLP2 tied to 3.3V Power (Pin 17) -> Always Enabled
  - Logic Ground:
      * RMCS GND tied to Pi GND (Pin 39)

Controls:
  [W] : Forward
  [S] : Reverse
  [A] : Pivot Turn Left
  [D] : Pivot Turn Right
  [X] or [SPACE] : Emergency STOP
  [+] or [=]     : Increase Base Speed (+5%)
  [-] or [_]     : Decrease Base Speed (-5%)
  [1] : Solo Test Left Motor Forward
  [2] : Solo Test Right Motor Forward
  [I] : Toggle Left Motor Direction Invert
  [O] : Toggle Right Motor Direction Invert
  [Q] or [Ctrl+C]: Safe Exit (Stop motors and release GPIO)
"""

import sys
import os
import time
import signal
import argparse
import select
import termios
import tty

# Default Pin Configurations (BCM numbering)
LEFT_PWM_PIN = 12   # Pin 32
LEFT_DIR_PIN = 16   # Pin 36
RIGHT_PWM_PIN = 13  # Pin 33
RIGHT_DIR_PIN = 26  # Pin 37

DEFAULT_PWM_FREQ = 1000  # 1 kHz
DEFAULT_SPEED = 40       # Start at 40% for safety on bench


# ==============================================================================
# Hardware Abstraction Layer (HAL)
# Supports:
#   1. gpiozero (with lgpio backend on Raspberry Pi 5)
#   2. direct lgpio (RP1 Southbridge controller)
#   3. Simulation / Mock fallback (for testing on PC / WSL2 without hardware)
# ==============================================================================

class HardwareDriver:
    """Base class for motor hardware control."""
    def set_left(self, duty_percent: float, forward: bool):
        raise NotImplementedError

    def set_right(self, duty_percent: float, forward: bool):
        raise NotImplementedError

    def stop_all(self):
        raise NotImplementedError

    def cleanup(self):
        raise NotImplementedError


class GpiozeroDriver(HardwareDriver):
    """Driver using gpiozero library (recommended on Ubuntu 24.04)."""
    def __init__(self, left_pwm_pin, left_dir_pin, right_pwm_pin, right_dir_pin, freq=DEFAULT_PWM_FREQ):
        from gpiozero import PWMOutputDevice, DigitalOutputDevice
        print(f"[HW] Initializing gpiozero backend (PWM Freq: {freq} Hz)...")
        self.left_pwm = PWMOutputDevice(left_pwm_pin, frequency=freq)
        self.left_dir = DigitalOutputDevice(left_dir_pin)
        self.right_pwm = PWMOutputDevice(right_pwm_pin, frequency=freq)
        self.right_dir = DigitalOutputDevice(right_dir_pin)
        self.stop_all()

    def set_left(self, duty_percent: float, forward: bool):
        self.left_dir.value = 1 if forward else 0
        self.left_pwm.value = max(0.0, min(1.0, duty_percent / 100.0))

    def set_right(self, duty_percent: float, forward: bool):
        self.right_dir.value = 1 if forward else 0
        self.right_pwm.value = max(0.0, min(1.0, duty_percent / 100.0))

    def stop_all(self):
        self.left_pwm.value = 0.0
        self.right_pwm.value = 0.0

    def cleanup(self):
        self.stop_all()
        self.left_pwm.close()
        self.left_dir.close()
        self.right_pwm.close()
        self.right_dir.close()


class LgpioDriver(HardwareDriver):
    """Driver using direct lgpio library (RP1 southbridge on Pi 5)."""
    def __init__(self, left_pwm_pin, left_dir_pin, right_pwm_pin, right_dir_pin, freq=DEFAULT_PWM_FREQ):
        import lgpio
        self.lgpio = lgpio
        self.freq = freq
        self.lpwm = left_pwm_pin
        self.ldir = left_dir_pin
        self.rpwm = right_pwm_pin
        self.rdir = right_dir_pin

        # Pi 5 RP1 header is typically gpiochip4 or gpiochip0
        self.chip = None
        for chip_num in [4, 0, 1, 2, 3]:
            try:
                self.chip = lgpio.gpiochip_open(chip_num)
                # Verify claiming pins
                lgpio.gpio_claim_output(self.chip, self.ldir)
                lgpio.gpio_claim_output(self.chip, self.rdir)
                lgpio.gpio_claim_output(self.chip, self.lpwm)
                lgpio.gpio_claim_output(self.chip, self.rpwm)
                print(f"[HW] Successfully opened gpiochip{chip_num} via lgpio.")
                break
            except Exception:
                if self.chip is not None:
                    try:
                        lgpio.gpiochip_close(self.chip)
                    except Exception:
                        pass
                    self.chip = None

        if self.chip is None:
            raise RuntimeError("Failed to open any compatible gpiochip via lgpio on Raspberry Pi 5.")

        self.stop_all()

    def set_left(self, duty_percent: float, forward: bool):
        val = 1 if forward else 0
        duty = max(0.0, min(100.0, float(duty_percent)))
        self.lgpio.gpio_write(self.chip, self.ldir, val)
        self.lgpio.tx_pwm(self.chip, self.lpwm, self.freq, duty)

    def set_right(self, duty_percent: float, forward: bool):
        val = 1 if forward else 0
        duty = max(0.0, min(100.0, float(duty_percent)))
        self.lgpio.gpio_write(self.chip, self.rdir, val)
        self.lgpio.tx_pwm(self.chip, self.rpwm, self.freq, duty)

    def stop_all(self):
        self.lgpio.tx_pwm(self.chip, self.lpwm, self.freq, 0)
        self.lgpio.tx_pwm(self.chip, self.rpwm, self.freq, 0)

    def cleanup(self):
        self.stop_all()
        try:
            self.lgpio.gpiochip_close(self.chip)
        except Exception:
            pass


class MockDriver(HardwareDriver):
    """Simulation driver for testing on PC / WSL2 without GPIO hardware."""
    def __init__(self):
        print("\n" + "=" * 60)
        print(" [!] NOTICE: RUNNING IN SIMULATION / MOCK MODE")
        print(" No Raspberry Pi 5 GPIO detected on this machine.")
        print(" Motor commands will be displayed virtually.")
        print(" To run on Raspberry Pi 5 hardware, install:")
        print("   sudo apt update && sudo apt install -y python3-gpiozero python3-lgpio")
        print("=" * 60 + "\n")
        self.left_duty = 0.0
        self.left_fwd = True
        self.right_duty = 0.0
        self.right_fwd = True

    def set_left(self, duty_percent: float, forward: bool):
        self.left_duty = duty_percent
        self.left_fwd = forward

    def set_right(self, duty_percent: float, forward: bool):
        self.right_duty = duty_percent
        self.right_fwd = forward

    def stop_all(self):
        self.left_duty = 0.0
        self.right_duty = 0.0

    def cleanup(self):
        self.stop_all()


def init_driver(left_pwm, left_dir, right_pwm, right_dir, freq, force_mock=False):
    """Factory to instantiate the appropriate driver."""
    if force_mock:
        return MockDriver()

    # Try gpiozero first
    try:
        return GpiozeroDriver(left_pwm, left_dir, right_pwm, right_dir, freq)
    except Exception as e_gpiozero:
        # Try direct lgpio next
        try:
            return LgpioDriver(left_pwm, left_dir, right_pwm, right_dir, freq)
        except Exception as e_lgpio:
            print(f"[WARN] Hardware drivers unavailable (gpiozero: {e_gpiozero}, lgpio: {e_lgpio}).")
            return MockDriver()


# ==============================================================================
# Non-blocking Terminal Key Reader
# ==============================================================================

class KeyReader:
    def __init__(self):
        self.is_tty = sys.stdin.isatty()
        if self.is_tty:
            self.old_settings = termios.tcgetattr(sys.stdin)
        else:
            self.old_settings = None

    def read_key(self, timeout=0.1):
        if not self.is_tty:
            ch = sys.stdin.read(1)
            if not ch:  # EOF
                time.sleep(timeout)
                return 'q'
            return ch

        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        key = None
        if rlist:
            key = sys.stdin.read(1)
            # Handle escape sequences (e.g. arrow keys)
            if key == '\x1b':
                rlist2, _, _ = select.select([sys.stdin], [], [], 0.02)
                if rlist2:
                    key += sys.stdin.read(2)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        return key

    def restore(self):
        if self.is_tty and self.old_settings:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            except Exception:
                pass


# ==============================================================================
# Interactive Motor Controller Application
# ==============================================================================

class MotorTestApp:
    def __init__(self, driver, base_speed=DEFAULT_SPEED, invert_left=False, invert_right=True, watchdog_sec=0.0):
        self.driver = driver
        self.speed = base_speed
        self.invert_left = invert_left
        self.invert_right = invert_right
        self.watchdog_sec = watchdog_sec

        self.last_cmd_time = time.time()
        self.state_name = "STOPPED"

        self.cur_left_pwm = 0.0
        self.cur_left_dir_hw = False
        self.cur_right_pwm = 0.0
        self.cur_right_dir_hw = False

    def apply_motion(self, left_target: float, right_target: float, state_desc: str):
        """
        Applies motion to left and right motors.
        target > 0 : Logical Forward
        target < 0 : Logical Backward
        target == 0: Stop
        """
        self.state_name = state_desc
        self.last_cmd_time = time.time()

        # Left Motor
        left_speed = abs(left_target)
        left_logical_fwd = (left_target >= 0)
        # Apply hardware polarity invert
        self.cur_left_dir_hw = (not left_logical_fwd) if self.invert_left else left_logical_fwd
        self.cur_left_pwm = left_speed
        self.driver.set_left(left_speed, self.cur_left_dir_hw)

        # Right Motor
        right_speed = abs(right_target)
        right_logical_fwd = (right_target >= 0)
        # Apply hardware polarity invert
        self.cur_right_dir_hw = (not right_logical_fwd) if self.invert_right else right_logical_fwd
        self.cur_right_pwm = right_speed
        self.driver.set_right(right_speed, self.cur_right_dir_hw)

    def stop(self):
        self.state_name = "STOPPED"
        self.cur_left_pwm = 0.0
        self.cur_right_pwm = 0.0
        self.driver.stop_all()

    def check_watchdog(self):
        if self.watchdog_sec > 0.0 and self.state_name != "STOPPED":
            if (time.time() - self.last_cmd_time) > self.watchdog_sec:
                self.stop()
                self.state_name = "WATCHDOG AUTO-STOP"

    def print_hud(self):
        # ANSI Escape Codes for crisp HUD without scroll flicker
        os.system('clear' if os.name == 'posix' else 'cls')
        print("=" * 68)
        print("   AMR HARDWARE MOTOR CONTROLLER & WASD TELEOPERATION (Pi 5)")
        print("=" * 68)
        print(f" Wiring Pinout: Left Motor (PWM=GPIO {LEFT_PWM_PIN}, DIR=GPIO {LEFT_DIR_PIN})")
        print(f"                Right Motor (PWM=GPIO {RIGHT_PWM_PIN}, DIR=GPIO {RIGHT_DIR_PIN})")
        print(f" Polarity Invert: Left = {'[INVERTED]' if self.invert_left else '[NORMAL]'}, "
              f"Right = {'[INVERTED]' if self.invert_right else '[NORMAL]'}")
        print("-" * 68)
        print(f" Current State : \033[1;32m{self.state_name}\033[0m")
        print(f" Target Speed  : \033[1;33m{self.speed}%\033[0m  (Adjust with '+' / '-')")
        if self.watchdog_sec > 0:
            print(f" Watchdog Timer: {self.watchdog_sec:.1f}s active")
        else:
            print(" Watchdog Timer: Disabled (Press [SPACE] or [X] to stop)")
        print("-" * 68)
        print(" Telemetry:")
        print(f"   LEFT WHEEL  -> PWM: {self.cur_left_pwm:5.1f}% | DIR Pin Level: {'HIGH (1)' if self.cur_left_dir_hw else 'LOW  (0)'}")
        print(f"   RIGHT WHEEL -> PWM: {self.cur_right_pwm:5.1f}% | DIR Pin Level: {'HIGH (1)' if self.cur_right_dir_hw else 'LOW  (0)'}")
        print("-" * 68)
        print(" Key Commands:")
        print("      [W] : Forward             |  [+] : Speed +5%      |  [1] : Solo Left")
        print("  [A] [S] [D] : Left/Back/Right |  [-] : Speed -5%      |  [2] : Solo Right")
        print("  [X] / [SPACE] : STOP NOW      |  [I] : Invert Left DIR|  [O] : Invert Right DIR")
        print("  [Q] or [Ctrl+C] : Quit safely")
        print("=" * 68)
        print(" Press a key to send command... ", end='', flush=True)


def main():
    parser = argparse.ArgumentParser(description="AMR Motor Hardware Testing & WASD Teleop for Raspberry Pi 5")
    parser.add_argument('--speed', type=int, default=DEFAULT_SPEED, help=f"Initial speed percentage (10-100, default {DEFAULT_SPEED})")
    parser.add_argument('--freq', type=int, default=DEFAULT_PWM_FREQ, help=f"PWM frequency in Hz (default {DEFAULT_PWM_FREQ})")
    parser.add_argument('--invert-left', action='store_true', help="Invert left motor direction polarity")
    parser.add_argument('--invert-right', action='store_true', default=True, help="Invert right motor direction polarity (default True for differential drive)")
    parser.add_argument('--watchdog', type=float, default=0.0, help="Safety auto-stop timeout in seconds (0 = disabled, e.g. 1.5)")
    parser.add_argument('--mock', action='store_true', help="Force mock/simulation mode without accessing GPIOs")
    args = parser.parse_args()

    # Safety clamp initial speed
    initial_speed = max(10, min(100, args.speed))

    driver = init_driver(
        LEFT_PWM_PIN, LEFT_DIR_PIN,
        RIGHT_PWM_PIN, RIGHT_DIR_PIN,
        args.freq,
        force_mock=args.mock
    )

    app = MotorTestApp(
        driver=driver,
        base_speed=initial_speed,
        invert_left=args.invert_left,
        invert_right=args.invert_right,
        watchdog_sec=args.watchdog
    )

    key_reader = KeyReader()

    def signal_handler(sig, frame):
        print("\n[INFO] Interrupt signal received. Stopping motors...")
        app.stop()
        driver.cleanup()
        key_reader.restore()
        print("[INFO] GPIOs cleared. Goodbye!")
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        need_refresh = True
        while True:
            if need_refresh:
                app.print_hud()
                need_refresh = False

            key = key_reader.read_key(timeout=0.1)
            app.check_watchdog()

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

            # Navigation Controls (WASD & Arrow Keys)
            if key_lower == 'w':
                # Both wheels drive forward
                app.apply_motion(+app.speed, +app.speed, "DRIVING FORWARD")

            elif key_lower == 's':
                # Both wheels drive reverse
                app.apply_motion(-app.speed, -app.speed, "DRIVING REVERSE")

            elif key_lower == 'a':
                # Pivot Turn Left: Left wheel backward, Right wheel forward
                app.apply_motion(-app.speed, +app.speed, "PIVOT TURNING LEFT")

            elif key_lower == 'd':
                # Pivot Turn Right: Left wheel forward, Right wheel backward
                app.apply_motion(+app.speed, -app.speed, "PIVOT TURNING RIGHT")

            elif key_lower in ['x', ' ']:
                # Emergency Stop
                app.stop()

            # Speed Adjustments
            elif key in ['+', '=']:
                app.speed = min(100, app.speed + 5)
                # If currently moving, update speed immediately
                if app.cur_left_pwm > 0 or app.cur_right_pwm > 0:
                    sign_l = -1 if "TURNING LEFT" in app.state_name or "REVERSE" in app.state_name else 1
                    sign_r = -1 if "TURNING RIGHT" in app.state_name or "REVERSE" in app.state_name else 1
                    app.apply_motion(sign_l * app.speed, sign_r * app.speed, app.state_name)

            elif key in ['-', '_']:
                app.speed = max(10, app.speed - 5)
                if app.cur_left_pwm > 0 or app.cur_right_pwm > 0:
                    sign_l = -1 if "TURNING LEFT" in app.state_name or "REVERSE" in app.state_name else 1
                    sign_r = -1 if "TURNING RIGHT" in app.state_name or "REVERSE" in app.state_name else 1
                    app.apply_motion(sign_l * app.speed, sign_r * app.speed, app.state_name)

            # Polarity Inversion Toggles (Live correction)
            elif key_lower == 'i':
                app.invert_left = not app.invert_left
                # Re-apply current motion with new inversion
                if app.cur_left_pwm > 0 or app.cur_right_pwm > 0:
                    sign_l = -1 if "TURNING LEFT" in app.state_name or "REVERSE" in app.state_name else 1
                    sign_r = -1 if "TURNING RIGHT" in app.state_name or "REVERSE" in app.state_name else 1
                    app.apply_motion(sign_l * app.speed, sign_r * app.speed, app.state_name)

            elif key_lower == 'o':
                app.invert_right = not app.invert_right
                if app.cur_left_pwm > 0 or app.cur_right_pwm > 0:
                    sign_l = -1 if "TURNING LEFT" in app.state_name or "REVERSE" in app.state_name else 1
                    sign_r = -1 if "TURNING RIGHT" in app.state_name or "REVERSE" in app.state_name else 1
                    app.apply_motion(sign_l * app.speed, sign_r * app.speed, app.state_name)

            # Bench Solo Motor Tests
            elif key == '1':
                # Test Left Motor only
                app.apply_motion(+app.speed, 0, "SOLO TEST: LEFT MOTOR ONLY")

            elif key == '2':
                # Test Right Motor only
                app.apply_motion(0, +app.speed, "SOLO TEST: RIGHT MOTOR ONLY")

            # Exit
            elif key_lower == 'q' or key == '\x1b':
                print("\n[INFO] Exiting...")
                break

    finally:
        app.stop()
        driver.cleanup()
        key_reader.restore()
        print("\n[INFO] Safe shutdown complete. Motors stopped.")


if __name__ == '__main__':
    main()
