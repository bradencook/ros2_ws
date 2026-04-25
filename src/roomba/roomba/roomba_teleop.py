#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import sys
import termios
import tty
import select
import json
import time
import os

# Speed presets: (linear m/s, angular rad/s)
SPEED_PRESETS = [
    (0.05, 0.3),   # 1 = crawl
    (0.10, 0.5),   # 2 = slow
    (0.20, 1.0),   # 3 = normal
    (0.30, 1.5),   # 4 = fast
    (0.40, 2.0),   # 5 = full
]

# Pi battery (3S Li-ion via INA219)
PI_BATT_MIN_V = 9.0    # 3.0V/cell — empty
PI_BATT_MAX_V = 12.6   # 4.2V/cell — fully charged

# Roomba battery (NiMH pack, 12 cells)
ROOMBA_BATT_MIN_V = 12.0   # ~1.0V/cell — safe cutoff
ROOMBA_BATT_MAX_V = 16.8   # ~1.4V/cell — full charge


def voltage_to_percent(voltage, min_v, max_v):
    if max_v <= min_v:
        return 0.0
    pct = (voltage - min_v) / (max_v - min_v) * 100.0
    return max(0.0, min(100.0, pct))


class RoombaTeleop(Node):
    def __init__(self):
        super().__init__('roomba_teleop')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        self.subscription = self.create_subscription(
            String, 'roomba/sensors', self.sensor_callback, 10
        )
        self.latest_sensors = None

        # Speed control — start at preset 3 (normal)
        self.speed_index = 2

        # Pi battery via INA219
        self.pi_voltage = None
        try:
            from batteries.INA219 import INA219
            self._ina219 = INA219(addr=0x41)
            self.get_logger().info('INA219 Pi battery monitor connected')
        except Exception as e:
            self._ina219 = None
            self.get_logger().warn(f'INA219 not available: {e}')

    @property
    def speed(self):
        return SPEED_PRESETS[self.speed_index][0]

    @property
    def turn(self):
        return SPEED_PRESETS[self.speed_index][1]

    def sensor_callback(self, msg):
        try:
            self.latest_sensors = json.loads(msg.data)
        except Exception:
            pass

    def publish_twist(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.publisher_.publish(msg)

    def read_pi_battery(self):
        if self._ina219 is None:
            return
        try:
            self.pi_voltage = self._ina219.getBusVoltage_V()
        except Exception:
            pass


def get_key():
    dr, _, _ = select.select([sys.stdin], [], [], 0.05)
    if dr:
        return sys.stdin.read(1)
    return None


def setup_terminal():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    return old


def restore_terminal(old):
    termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old)


def bar(pct, width=10):
    filled = round(pct / 100.0 * width)
    return '\u2588' * filled + '\u2591' * (width - filled)


def main(args=None):
    rclpy.init(args=args)
    node = RoombaTeleop()

    old_settings = setup_terminal()

    print("\nRoomba Teleop")
    print("─" * 40)
    print("  WASD / Arrows  = drive")
    print("  1-5            = speed preset")
    print("  Space          = stop")
    print("  q              = quit")
    print("─" * 40)
    print()

    last_pi_read = 0.0

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0)

            # Read Pi battery every 2 seconds
            now = time.monotonic()
            if now - last_pi_read > 2.0:
                node.read_pi_battery()
                last_pi_read = now

            key = get_key()

            if key == '\x1b':
                key += sys.stdin.read(2)
                if key == '\x1b[A':    # up
                    node.publish_twist(node.speed, 0.0)
                elif key == '\x1b[B':  # down
                    node.publish_twist(-node.speed, 0.0)
                elif key == '\x1b[C':  # right
                    node.publish_twist(0.0, -node.turn)
                elif key == '\x1b[D':  # left
                    node.publish_twist(0.0, node.turn)

            elif key in ['w', 'W']:
                node.publish_twist(node.speed, 0.0)
            elif key in ['s', 'S']:
                node.publish_twist(-node.speed, 0.0)
            elif key in ['a', 'A']:
                node.publish_twist(0.0, node.turn)
            elif key in ['d', 'D']:
                node.publish_twist(0.0, -node.turn)

            elif key == ' ':
                node.publish_twist(0.0, 0.0)

            elif key in ['1', '2', '3', '4', '5']:
                node.speed_index = int(key) - 1

            elif key == 'q':
                break

            # --- Build status line ---
            parts = []

            # Speed
            level = node.speed_index + 1
            parts.append(f"Spd:{level} ({node.speed:.2f}m/s)")

            # Roomba battery
            if node.latest_sensors:
                charge = node.latest_sensors.get('25', {}).get('battery_charge_mah', 0)
                capacity = node.latest_sensors.get('26', {}).get('battery_capacity_mah', 0)
                voltage_mv = node.latest_sensors.get('22', {}).get('voltage_mv', 0)
                voltage_v = voltage_mv / 1000.0

                if capacity > 0:
                    roomba_pct = max(0.0, min(100.0, charge / capacity * 100.0))
                else:
                    roomba_pct = voltage_to_percent(voltage_v, ROOMBA_BATT_MIN_V, ROOMBA_BATT_MAX_V)

                parts.append(f"Roomba:{bar(roomba_pct,8)} {roomba_pct:.0f}% {voltage_v:.1f}V")

                # Bumps
                bumps = node.latest_sensors.get('7', {})
                bl = bumps.get('bump_left', False)
                br = bumps.get('bump_right', False)
                if bl or br:
                    bump_str = ""
                    if bl:
                        bump_str += "L"
                    if br:
                        bump_str += "R"
                    parts.append(f"BUMP:{bump_str}")

            # Pi battery
            if node.pi_voltage is not None:
                pi_pct = voltage_to_percent(node.pi_voltage, PI_BATT_MIN_V, PI_BATT_MAX_V)
                parts.append(f"Pi:{bar(pi_pct,8)} {pi_pct:.0f}% {node.pi_voltage:.1f}V")

            try:
                cols = os.get_terminal_size().columns
            except OSError:
                cols = 80
            line = ' | '.join(parts)[:cols - 1]
            sys.stdout.write(f"\r\033[K{line}")
            sys.stdout.flush()

    finally:
        node.publish_twist(0.0, 0.0)
        restore_terminal(old_settings)
        node.destroy_node()
        rclpy.shutdown()
        print("\nStopped")


if __name__ == '__main__':
    main()
