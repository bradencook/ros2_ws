#!/usr/bin/env python3
"""BNO08x IMU driver for ROS 2.

Publishes sensor_msgs/Imu on /imu/data. Does NOT publish any TF — the EKF
(robot_localization) owns odom->base_link. Set frame_id via parameter.
"""
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

import board
import busio
from adafruit_bno08x import (
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_LINEAR_ACCELERATION,
)
from adafruit_bno08x.i2c import BNO08X_I2C


class BNO08xNode(Node):
    def __init__(self):
        super().__init__('bno08x_node')

        self.declare_parameter('i2c_address', 0x4B)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate_hz', 50.0)

        addr = self.get_parameter('i2c_address').value
        self.frame_id = self.get_parameter('frame_id').value
        rate = self.get_parameter('publish_rate_hz').value

        # Init I2C + sensor
        i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
        self.bno = BNO08X_I2C(i2c, address=addr)
        time.sleep(0.5)  # chip boot settle

        self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
        self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
        self.bno.enable_feature(BNO_REPORT_LINEAR_ACCELERATION)

        self.pub = self.create_publisher(Imu, '/imu/data', 10)
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f'BNO08x IMU online — addr=0x{addr:02x}, frame={self.frame_id}, rate={rate}Hz'
        )

    def _tick(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        try:
            qi, qj, qk, qw = self.bno.quaternion
            msg.orientation.x = float(qi)
            msg.orientation.y = float(qj)
            msg.orientation.z = float(qk)
            msg.orientation.w = float(qw)

            gx, gy, gz = self.bno.gyro
            msg.angular_velocity.x = float(gx)
            msg.angular_velocity.y = float(gy)
            msg.angular_velocity.z = float(gz)

            ax, ay, az = self.bno.linear_acceleration
            msg.linear_acceleration.x = float(ax)
            msg.linear_acceleration.y = float(ay)
            msg.linear_acceleration.z = float(az)

            # Covariances — BNO08x fused outputs are high quality.
            # Diagonal only; tune later if EKF weights look off.
            msg.orientation_covariance = [
                0.01, 0.0,  0.0,
                0.0,  0.01, 0.0,
                0.0,  0.0,  0.01,
            ]
            msg.angular_velocity_covariance = [
                0.001, 0.0,   0.0,
                0.0,   0.001, 0.0,
                0.0,   0.0,   0.001,
            ]
            msg.linear_acceleration_covariance = [
                0.1, 0.0, 0.0,
                0.0, 0.1, 0.0,
                0.0, 0.0, 0.1,
            ]

            self.pub.publish(msg)
        except Exception as e:
            self.get_logger().warning(f'BNO08x read error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = BNO08xNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
