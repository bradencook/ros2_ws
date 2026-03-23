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

SPEED = 0.2  # m/s
TURN = 1.0   # rad/s

class RoombaTeleop(Node):
    def __init__(self):
        super().__init__('roomba_teleop')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscribe to sensors to continue displaying them
        self.subscription = self.create_subscription(
            String,
            'roomba/sensors',
            self.sensor_callback,
            10
        )
        self.latest_sensors = None

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

def main(args=None):
    rclpy.init(args=args)
    node = RoombaTeleop()

    old_settings = setup_terminal()

    print("\nRoomba ROS 2 Teleop\n")
    print("Arrow keys or WASD = drive")
    print("Space = stop")
    print("q = quit\n")

    try:
        while rclpy.ok():
            # Spin once to process callbacks (like the sensor subscriber)
            rclpy.spin_once(node, timeout_sec=0)

            key = get_key()

            if key == '\x1b':
                # Handle arrow keys
                key += sys.stdin.read(2)
                if key == '\x1b[A':  # up
                    node.publish_twist(SPEED, 0.0)
                elif key == '\x1b[B':  # down
                    node.publish_twist(-SPEED, 0.0)
                elif key == '\x1b[C':  # right
                    node.publish_twist(0.0, -TURN)
                elif key == '\x1b[D':  # left
                    node.publish_twist(0.0, TURN)

            # --- WASD controls ---
            elif key in ['w', 'W']:
                node.publish_twist(SPEED, 0.0)
            elif key in ['s', 'S']:
                node.publish_twist(-SPEED, 0.0)
            elif key in ['a', 'A']:
                node.publish_twist(0.0, TURN)
            elif key in ['d', 'D']:
                node.publish_twist(0.0, -TURN)
            
            # --- Stop ---
            elif key == ' ':
                node.publish_twist(0.0, 0.0)
            
            # --- Quit ---
            elif key == 'q':
                break

            # Print sensors if available
            if node.latest_sensors:
                # Need to map the JSON IDs to the sensor values as output in roomba_node.py
                # roomba_node sends string IDs like '22' for voltage.
                voltage_mv = node.latest_sensors.get('22', {}).get('voltage_mv', 0)
                voltage_v = voltage_mv / 1000.0

                bumps = node.latest_sensors.get('7', {})
                bump_left = bumps.get('bump_left', False)
                bump_right = bumps.get('bump_right', False)

                print(
                    f"\rVoltage: {voltage_v:.2f}V | "
                    f"BumpL: {int(bump_left)} | "
                    f"BumpR: {int(bump_right)}      ",
                    end=""
                )

    finally:
        # Stop the robot before exiting
        node.publish_twist(0.0, 0.0)
        restore_terminal(old_settings)
        node.destroy_node()
        rclpy.shutdown()
        print("\nStopped")

if __name__ == '__main__':
    main()
