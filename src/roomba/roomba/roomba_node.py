#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import threading
import json
import time

from roomba.driver import drive_direct, stream, pause_stream, startup, SENSOR_PACKETS, passive

class RoombaNode(Node):
    def __init__(self):
        super().__init__('roomba_node')

        # Lock for serial port access
        self.serial_lock = threading.Lock()

        # Flag to control background thread
        self.running = True

        # Subscribe to cmd_vel to drive the robot
        self.sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Publish sensor data as JSON
        self.sensor_pub = self.create_publisher(String, 'roomba/sensors', 10)

        # Wheel odometry — consumed by robot_localization EKF.
        # We do NOT publish odom->base_link TF anymore; EKF owns that.
        self.odom_pub = self.create_publisher(Odometry, 'wheel/odometry', 50)

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.prev_left_encoder = None
        self.prev_right_encoder = None
        self.prev_time = None

        # Roomba kinematics constants
        # wheel_diameter is the *effective rolling diameter*, not physical.
        # It absorbs tire compression + slip for the dominant surface.
        # Calibrated on plush carpet over two passes (0.0698 -> 0.0399 -> 0.0431).
        self.wheelbase = 0.2370       # meters (effective track width, calibrated)
        self.wheel_diameter = 0.0431  # meters (carpet-calibrated rolling diameter)
        self.ticks_per_rev = 508.8
        self.meters_per_tick = (math.pi * self.wheel_diameter) / self.ticks_per_rev

        # Wheelbase for cmd_vel conversion (mm)
        self.cmd_wheelbase_mm = 235.0

        # Minimum wheel speed (mm/s) that can actually overcome static friction on
        # plush carpet. Nav2 issues near-zero velocities during final goal orientation;
        # without this floor the motors stall and the robot gets stuck centimeters away.
        self.min_wheel_speed_mm = 30

        # Initialize Roomba
        with self.serial_lock:
            startup(full_mode=True)
            stream(SENSOR_PACKETS)

        # Start sensor streaming in a background thread
        threading.Thread(target=self.stream_sensors, daemon=True).start()

        self.get_logger().info(
            f'Roomba node started — encoder odom with '
            f'wheelbase={self.wheelbase}m, wheel_dia={self.wheel_diameter}m, '
            f'meters_per_tick={self.meters_per_tick:.6f}'
        )

    def cmd_vel_callback(self, msg: Twist):
        """Convert Twist to Roomba differential drive commands."""
        v_mm = msg.linear.x * 1000.0
        w = msg.angular.z

        right_vel = int(v_mm + (w * self.cmd_wheelbase_mm / 2.0))
        left_vel = int(v_mm - (w * self.cmd_wheelbase_mm / 2.0))

        # Clamp to Roomba OI limits
        right_vel = max(-500, min(500, right_vel))
        left_vel = max(-500, min(500, left_vel))

        # Apply per-wheel deadband so tiny Nav2 goal-approach commands still move.
        right_vel = self._apply_min_speed(right_vel)
        left_vel = self._apply_min_speed(left_vel)

        with self.serial_lock:
            drive_direct(left_vel, right_vel)

    def _apply_min_speed(self, v_mm: int) -> int:
        """Floor non-zero wheel speeds at the stall threshold, preserving sign."""
        if v_mm == 0:
            return 0
        if abs(v_mm) < self.min_wheel_speed_mm:
            return self.min_wheel_speed_mm if v_mm > 0 else -self.min_wheel_speed_mm
        return v_mm

    def _encoder_delta(self, current, previous):
        """Compute signed encoder delta handling unsigned 16-bit wrap-around."""
        delta = current - previous
        if delta > 32768:
            delta -= 65536
        elif delta < -32768:
            delta += 65536
        return delta

    def stream_sensors(self):
        """Continuously read Roomba sensor stream, compute odometry, and publish."""
        from roomba.driver import ser, PACKET_SIZES, decode_packet

        while self.running:
            try:
                with self.serial_lock:
                    header = ser.read(1)
                    if not header or header[0] != 19:
                        continue

                    length = ser.read(1)[0]
                    payload = ser.read(length)
                    checksum = ser.read(1)[0]

                # Verify checksum
                frame = bytes([19, length]) + payload + bytes([checksum])
                if (sum(frame) & 0xFF) != 0:
                    self.get_logger().warning("Sensor checksum error")
                    continue

                # Parse sensor data
                i = 0
                parsed = {}
                while i < len(payload):
                    packet_id = payload[i]
                    i += 1

                    size = PACKET_SIZES.get(packet_id)
                    if size is None:
                        self.get_logger().warning(f"Unknown sensor packet: {packet_id}")
                        break

                    data = payload[i:i+size]
                    i += size
                    parsed[str(packet_id)] = decode_packet(packet_id, data)

                # Publish raw sensor data
                parsed["timestamp"] = time.time()
                sensor_msg = String()
                sensor_msg.data = json.dumps(parsed)
                self.sensor_pub.publish(sensor_msg)

                # --- ENCODER-BASED ODOMETRY ---
                left_data = parsed.get("43")
                right_data = parsed.get("44")

                if left_data is None or right_data is None:
                    continue

                left_enc = left_data.get("encoder_left")
                right_enc = right_data.get("encoder_right")

                if left_enc is None or right_enc is None:
                    continue

                current_ros_time = self.get_clock().now()

                if self.prev_left_encoder is None:
                    # First reading — just store and publish a zero-velocity odom
                    self.prev_left_encoder = left_enc
                    self.prev_right_encoder = right_enc
                    self.prev_time = current_ros_time
                    self._publish_odom(current_ros_time, 0.0, 0.0)
                    continue

                # Compute encoder deltas with wrap-around handling
                d_left_ticks = self._encoder_delta(left_enc, self.prev_left_encoder)
                d_right_ticks = self._encoder_delta(right_enc, self.prev_right_encoder)

                self.prev_left_encoder = left_enc
                self.prev_right_encoder = right_enc

                # Convert to meters
                d_left = d_left_ticks * self.meters_per_tick
                d_right = d_right_ticks * self.meters_per_tick

                # Differential drive kinematics
                d_center = (d_left + d_right) / 2.0
                d_theta = (d_right - d_left) / self.wheelbase

                # Update pose (mid-point integration)
                self.x += d_center * math.cos(self.theta + d_theta / 2.0)
                self.y += d_center * math.sin(self.theta + d_theta / 2.0)
                self.theta += d_theta
                self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

                # Velocities computed from actual elapsed time (not hardcoded dt).
                # Protects against stream-rate jitter that otherwise biases vx.
                dt = (current_ros_time - self.prev_time).nanoseconds / 1e9
                self.prev_time = current_ros_time
                if dt > 0.0:
                    vx = d_center / dt
                    vth = d_theta / dt
                else:
                    vx = 0.0
                    vth = 0.0

                self._publish_odom(current_ros_time, vx, vth)

            except Exception as e:
                if self.running:
                    self.get_logger().error(f"Sensor read error: {e}")
                time.sleep(0.1)

    def _publish_odom(self, stamp, vx, vth):
        """Publish wheel odometry message. EKF handles TF and fusion."""
        time_msg = stamp.to_msg()

        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(self.theta / 2.0)
        q.w = math.cos(self.theta / 2.0)

        odom = Odometry()
        odom.header.stamp = time_msg
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vth

        # Covariance is row-major 6x6 for (x,y,z,roll,pitch,yaw).
        # Mark yaw pose + yaw-rate twist as untrusted so EKF uses IMU instead.
        odom.pose.covariance[0] = 0.01     # x
        odom.pose.covariance[7] = 0.01     # y
        odom.pose.covariance[35] = 1e6     # yaw — slips on carpet, don't trust
        odom.twist.covariance[0] = 0.01    # vx
        odom.twist.covariance[35] = 1e6    # vyaw — don't trust

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = RoombaNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        with node.serial_lock:
            pause_stream()
            passive()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
