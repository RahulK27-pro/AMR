#!/usr/bin/env python3
"""
AMR Wheel Encoder Calibration & Ticks-Per-Revolution Meter
==========================================================
Target SBC: Raspberry Pi 5 (RP1 Southbridge via lgpio)
Encoders: 6-Wire Hall-Effect Quadrature Encoders (3.3V Logic)

Pinout (RP1 40-Pin Header):
  - Left Wheel Encoder:
      * Channel A (Yellow): GPIO 17 (Pin 11)
      * Channel B (White) : GPIO 27 (Pin 13)
  - Right Wheel Encoder:
      * Channel A (Yellow): GPIO 22 (Pin 15)
      * Channel B (White) : GPIO 23 (Pin 16)

Purpose:
  Measure exact ticks per 360° wheel revolution before ROS 2 integration.
  Displays both:
    - 4x Full Quadrature Ticks (Bidirectional, highest resolution)
    - 1x Channel A Rising-Edge Pulses (Single-channel count)

Procedure:
  1. Keep robot elevated on blocks.
  2. Put a piece of tape at 12 o'clock on the wheel rim.
  3. Press [R] to zero all counters.
  4. Slowly rotate the wheel by hand EXACTLY 1 full turn (360°) back to 12 o'clock.
  5. Note down the recorded ticks!

Controls:
  [R]     : Reset / Zero counters
  [SPACE] : Snapshot current reading (saves to history)
  [Q]     : Quit
"""

import sys
import os
import time
import signal
import select
import termios
import tty
import argparse

# Default GPIO Pinout (BCM numbering from schematic)
LEFT_ENC_A  = 17   # Pin 11
LEFT_ENC_B  = 27   # Pin 13
RIGHT_ENC_A = 22   # Pin 15
RIGHT_ENC_B = 23   # Pin 16

# Quadrature transition lookup table (index: [prev_state << 2 | curr_state])
# States: 00(0), 01(1), 10(2), 11(3)
# Valid forward:  0->1 (+1), 1->3 (+1), 3->2 (+1), 2->0 (+1)
# Valid reverse:  0->2 (-1), 2->3 (-1), 3->1 (-1), 1->0 (-1)
QUAD_TABLE = [
     0,  1, -1,  0,  # 00 -> 00, 01, 10, 11
    -1,  0,  0,  1,  # 01 -> 00, 01, 10, 11
     1,  0,  0, -1,  # 10 -> 00, 01, 10, 11
     0, -1,  1,  0   # 11 -> 00, 01, 10, 11
]


class MockHardware:
    """Simulated hardware for PC testing."""
    def __init__(self):
        print("\n" + "=" * 62)
        print(" [!] RUNNING IN SIMULATION / MOCK MODE")
        print(" (Use on Raspberry Pi 5 to read real encoder pulses)")
        print("=" * 62 + "\n")


class EncoderReader:
    def __init__(self, mock=False):
        self.mock = mock
        self.chip = None
        self.callbacks = []

        # Counts
        self.left_4x = 0
        self.left_1x_a = 0
        self.left_1x_b = 0
        self.left_state = 0

        self.right_4x = 0
        self.right_1x_a = 0
        self.right_1x_b = 0
        self.right_state = 0

        self.snapshots = []

        if not mock:
            self._init_gpio()

    def _init_gpio(self):
        import lgpio
        self.lgpio = lgpio

        # Open RP1 southbridge gpiochip (gpiochip4 on Pi 5 kernel >= 6.6, fallback 0)
        for chip_num in [4, 0, 1]:
            try:
                self.chip = lgpio.gpiochip_open(chip_num)
                print(f"[HW] Opened /dev/gpiochip{chip_num} for encoders.")
                break
            except Exception:
                self.chip = None

        if self.chip is None:
            raise RuntimeError("Cannot open any gpiochip. Are you running on Raspberry Pi 5?")

        # Claim input pins with pull-ups enabled if supported
        for pin in [LEFT_ENC_A, LEFT_ENC_B, RIGHT_ENC_A, RIGHT_ENC_B]:
            self.lgpio.gpio_claim_input(self.chip, pin)
            self.lgpio.gpio_claim_alert(self.chip, pin, self.lgpio.BOTH_EDGES)

        # Initial pin states
        la = self.lgpio.gpio_read(self.chip, LEFT_ENC_A)
        lb = self.lgpio.gpio_read(self.chip, LEFT_ENC_B)
        self.left_state = (la << 1) | lb

        ra = self.lgpio.gpio_read(self.chip, RIGHT_ENC_A)
        rb = self.lgpio.gpio_read(self.chip, RIGHT_ENC_B)
        self.right_state = (ra << 1) | rb

        # Register callbacks
        cb1 = self.lgpio.callback(self.chip, LEFT_ENC_A, self.lgpio.BOTH_EDGES, self._on_left_a)
        cb2 = self.lgpio.callback(self.chip, LEFT_ENC_B, self.lgpio.BOTH_EDGES, self._on_left_b)
        cb3 = self.lgpio.callback(self.chip, RIGHT_ENC_A, self.lgpio.BOTH_EDGES, self._on_right_a)
        cb4 = self.lgpio.callback(self.chip, RIGHT_ENC_B, self.lgpio.BOTH_EDGES, self._on_right_b)
        self.callbacks = [cb1, cb2, cb3, cb4]

    def _on_left_a(self, chip, gpio, level, tick):
        if level == 1:
            self.left_1x_a += 1
        lb = self.lgpio.gpio_read(self.chip, LEFT_ENC_B)
        new_state = (level << 1) | lb
        transition = (self.left_state << 2) | new_state
        delta = QUAD_TABLE[transition & 0x0F]
        self.left_4x += delta
        self.left_state = new_state

    def _on_left_b(self, chip, gpio, level, tick):
        if level == 1:
            self.left_1x_b += 1
        la = self.lgpio.gpio_read(self.chip, LEFT_ENC_A)
        new_state = (la << 1) | level
        transition = (self.left_state << 2) | new_state
        delta = QUAD_TABLE[transition & 0x0F]
        self.left_4x += delta
        self.left_state = new_state

    def _on_right_a(self, chip, gpio, level, tick):
        if level == 1:
            self.right_1x_a += 1
        rb = self.lgpio.gpio_read(self.chip, RIGHT_ENC_B)
        new_state = (level << 1) | rb
        transition = (self.right_state << 2) | new_state
        delta = QUAD_TABLE[transition & 0x0F]
        self.right_4x += delta
        self.right_state = new_state

    def _on_right_b(self, chip, gpio, level, tick):
        if level == 1:
            self.right_1x_b += 1
        ra = self.lgpio.gpio_read(self.chip, RIGHT_ENC_A)
        new_state = (ra << 1) | level
        transition = (self.right_state << 2) | new_state
        delta = QUAD_TABLE[transition & 0x0F]
        self.right_4x += delta
        self.right_state = new_state

    def reset(self):
        self.left_4x = 0
        self.left_1x_a = 0
        self.left_1x_b = 0
        self.right_4x = 0
        self.right_1x_a = 0
        self.right_1x_b = 0

    def snapshot(self):
        snap = {
            "time": time.strftime("%H:%M:%S"),
            "left_4x": self.left_4x,
            "left_1x": self.left_1x_a,
            "right_4x": self.right_4x,
            "right_1x": self.right_1x_a
        }
        self.snapshots.append(snap)
        return snap

    def cleanup(self):
        if self.mock or self.chip is None:
            return
        for cb in self.callbacks:
            try:
                cb.cancel()
            except Exception:
                pass
        try:
            self.lgpio.gpiochip_close(self.chip)
        except Exception:
            pass


class KeyReader:
    def __init__(self):
        self.is_tty = sys.stdin.isatty()
        self.old_settings = termios.tcgetattr(sys.stdin) if self.is_tty else None

    def read_key(self, timeout=0.1):
        if not self.is_tty:
            ch = sys.stdin.read(1)
            if not ch:
                time.sleep(timeout)
                return 'q'
            return ch
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        key = None
        if rlist:
            key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        return key

    def restore(self):
        if self.is_tty and self.old_settings:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            except Exception:
                pass


def print_hud(reader, simulated_wheel_r=0.025):
    os.system('clear' if os.name == 'posix' else 'cls')
    print("=" * 72)
    print("   AMR WHEEL ENCODER CALIBRATION & TICKS-PER-REV MEASUREMENT TOOL")
    print("=" * 72)
    print(" Procedure:")
    print("   1. Keep robot elevated on blocks (wheels spin freely).")
    print("   2. Place a small tape marker at 12 o'clock on the wheel rim.")
    print("   3. Press [R] to ZERO the counters.")
    print("   4. Slowly rotate the wheel by hand EXACTLY 1 full turn (360°).")
    print("   5. Press [SPACE] to snapshot the reading.")
    print("-" * 72)
    print(f" LEFT WHEEL ENCODER  (Ch A: GPIO {LEFT_ENC_A}, Ch B: GPIO {LEFT_ENC_B}):")
    print(f"   ► 4x Quadrature Ticks : \033[1;32m{reader.left_4x:>7}\033[0m ticks (Bidirectional, 4 edges)")
    print(f"     1x Ch A Pulses      : \033[1;33m{reader.left_1x_a:>7}\033[0m pulses (Rising edge only)")
    print(f"     1x Ch B Pulses      : \033[1;33m{reader.left_1x_b:>7}\033[0m pulses (Rising edge only)")
    print("-" * 72)
    print(f" RIGHT WHEEL ENCODER (Ch A: GPIO {RIGHT_ENC_A}, Ch B: GPIO {RIGHT_ENC_B}):")
    print(f"   ► 4x Quadrature Ticks : \033[1;32m{reader.right_4x:>7}\033[0m ticks (Bidirectional, 4 edges)")
    print(f"     1x Ch A Pulses      : \033[1;33m{reader.right_1x_a:>7}\033[0m pulses (Rising edge only)")
    print(f"     1x Ch B Pulses      : \033[1;33m{reader.right_1x_b:>7}\033[0m pulses (Rising edge only)")
    print("-" * 72)

    if reader.snapshots:
        print(" Saved Snapshots for 1 Full Turn (360°):")
        for idx, snap in enumerate(reader.snapshots[-3:], 1):
            print(f"   #{idx} [{snap['time']}] -> Left 4x: {snap['left_4x']:>5} | Right 4x: {snap['right_4x']:>5} "
                  f"| Left 1x: {snap['left_1x']:>5} | Right 1x: {snap['right_1x']:>5}")
        print("-" * 72)

    print(" Controls:  [R] = Zero/Reset  |  [SPACE] = Save Snapshot  |  [Q] = Quit")
    print("=" * 72)
    print(" Rotate wheel by hand now... (Listening) ", end='', flush=True)


def main():
    parser = argparse.ArgumentParser(description="Measure wheel ticks per revolution on Raspberry Pi 5")
    parser.add_argument('--mock', action='store_true', help="Force mock/simulation mode")
    args = parser.parse_args()

    # Detect hardware or fallback
    reader = None
    try:
        if args.mock:
            reader = EncoderReader(mock=True)
        else:
            try:
                reader = EncoderReader(mock=False)
            except Exception as e:
                print(f"[WARN] Hardware error: {e}. Falling back to mock mode.")
                reader = EncoderReader(mock=True)
    except Exception as e:
        print(f"[ERROR] Failed to start: {e}")
        sys.exit(1)

    key_reader = KeyReader()

    def signal_handler(sig, frame):
        reader.cleanup()
        key_reader.restore()
        print("\n[INFO] Exited cleanly.")
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        last_display = 0
        while True:
            now = time.time()
            if now - last_display > 0.1:
                print_hud(reader)
                last_display = now

            key = key_reader.read_key(timeout=0.05)
            if key is not None:
                key_l = key.lower()
                if key_l == 'r':
                    reader.reset()
                elif key == ' ':
                    reader.snapshot()
                elif key_l == 'q':
                    break
                elif reader.mock:
                    # In mock mode, pressing 'l' or 'r' simulates ticks
                    if key_l == 'l':
                        reader.left_4x += 20
                        reader.left_1x_a += 5
                    elif key_l == 'k':
                        reader.right_4x += 20
                        reader.right_1x_a += 5

    finally:
        reader.cleanup()
        key_reader.restore()
        print("\n\n" + "=" * 62)
        print(" CALIBRATION SUMMARY:")
        if reader.snapshots:
            last = reader.snapshots[-1]
            print(f"  Last Recorded Full Turn (360°):")
            print(f"    Left Wheel  -> 4x Ticks: {last['left_4x']} | 1x Pulses: {last['left_1x']}")
            print(f"    Right Wheel -> 4x Ticks: {last['right_4x']} | 1x Pulses: {last['right_1x']}")
        else:
            print(f"  Current Counters:")
            print(f"    Left Wheel  -> 4x Ticks: {reader.left_4x} | 1x Pulses: {reader.left_1x_a}")
            print(f"    Right Wheel -> 4x Ticks: {reader.right_4x} | 1x Pulses: {reader.right_1x_a}")
        print("=" * 62)


if __name__ == '__main__':
    main()
