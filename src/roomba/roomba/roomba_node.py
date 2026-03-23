#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from std_msgs.msg import String
import threading
import json
import time
import logging

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

        # Optional: publish sensor data
        self.sensor_pub = self.create_publisher(String, 'roomba/sensors', 10)
        
        # Odom & TF
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.prev_left_encoder = None
        self.prev_right_encoder = None
        self.prev_odom_time = None
        
        # Roomba kinematics constants
        self.wheelbase = 0.235  # meters
        self.wheel_diameter = 0.072  # meters
        self.ticks_per_rev = 508.8
        self.meters_per_tick = (math.pi * self.wheel_diameter) / self.ticks_per_rev
        
        # Initialize Roomba
        with self.serial_lock:
            startup(full_mode=True)  # Use full mode for complete control
            stream(SENSOR_PACKETS)  # Start streaming sensors

        # Start sensor streaming in a background thread
        threading.Thread(target=self.stream_sensors, daemon=True).start()

    def cmd_vel_callback(self, msg: Twist):
        """
        Converts Twist messages to Roomba drive commands.
        Using drive_direct for better differential control, including turn-in-place.
        """
        wheelbase_mm = 235.0
        
        v_mm = msg.linear.x * 1000.0  # convert m/s to mm/s
        w = msg.angular.z             # rad/s
        
        # Calculate left and right wheel velocities
        right_vel = int(v_mm + (w * wheelbase_mm / 2.0))
        left_vel = int(v_mm - (w * wheelbase_mm / 2.0))
        
        # Clamp velocities to -500 to 500 mm/s (as specified in the Open Interface)
        right_vel = max(-500, min(500, right_vel))
        left_vel = max(-500, min(500, left_vel))

        with self.serial_lock:
            drive_direct(left_vel, right_vel)

    def stream_sensors(self):
        """
        Continuously read Roomba sensor stream and publish as JSON.
        """
        from roomba.driver import ser, PACKET_SIZES, decode_packet
        import struct
        
        while self.running:
            try:
                with self.serial_lock:
                    header = ser.read(1)
                    if not header or header[0] != 19:  # stream header
                        continue
                    
                    length = ser.read(1)[0]
                    payload = ser.read(length)
                    checksum = ser.read(1)[0]
                
                # Verify checksum outside lock
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
                
                # Publish sensor data
                parsed["timestamp"] = time.time()
                msg = String()
                msg.data = json.dumps(parsed)
                self.sensor_pub.publish(msg)
                
                # --- ODOMETRY COMPUTATION ---
                left_enc_data = parsed.get("43")
                right_enc_data = parsed.get("44")
                
                left_enc = left_enc_data.get("encoder_left") if isinstance(left_enc_data, dict) else None
                right_enc = right_enc_data.get("encoder_right") if isinstance(right_enc_data, dict) else None
                
                if left_enc is not None and right_enc is not None:
                    current_ros_time = self.get_clock().now()
                    
                    if self.prev_left_encoder is None or self.prev_odom_time is None:
                        self.prev_left_encoder = left_enc
                        self.prev_right_encoder = right_enc
                        self.prev_odom_time = current_ros_time
                        continue
                        
                    dt = (current_ros_time - self.prev_odom_time).nanoseconds / 1e9
                    self.prev_odom_time = current_ros_time
                    
                    if dt > 0:
                        # Calculate delta ticks (handling 16-bit unsigned wraparound)
                        d_left = left_enc - self.prev_left_encoder
                        if d_left < -32768:
                            d_left += 65536
                        elif d_left > 32768:
                            d_left -= 65536
                            
                        d_right = right_enc - self.prev_right_encoder
                        if d_right < -32768:
                            d_right += 65536
                        elif d_right > 32768:
                            d_right -= 65536
                            
                        self.prev_left_encoder = left_enc
                        self.prev_right_encoder = right_enc
                        
                        # Convert to meters
                        dist_left = d_left * self.meters_per_tick
                        dist_right = d_right * self.meters_per_tick
                        
                        # Kinematics
                        d_center = (dist_right + dist_left) / 2.0
                        d_theta = (dist_right - dist_left) / self.wheelbase
                        
                        # Update state
                        self.x += d_center * math.cos(self.theta + (d_theta / 2.0))
                        self.y += d_center * math.sin(self.theta + (d_theta / 2.0))
                        self.theta += d_theta
                        
                        # Calculate speeds
                        vx = d_center / dt
                        vth = d_theta / dt
                        
                        # Publish Odom & TF
                        time_msg = current_ros_time.to_msg()
                        
                        q = Quaternion()
                        q.x = 0.0
                        q.y = 0.0
                        q.z = math.sin(self.theta / 2.0)
                        q.w = math.cos(self.theta / 2.0)
                        
                        # 1. Transform Over TF
                        t = TransformStamped()
                        t.header.stamp = time_msg
                        t.header.frame_id = 'odom'
                        t.child_frame_id = 'base_link'
                        t.transform.translation.x = self.x
                        t.transform.translation.y = self.y
                        t.transform.translation.z = 0.0
                        t.transform.rotation = q
                        
                        self.tf_broadcaster.sendTransform(t)
                        
                        # 2. Odometry Message
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
                        
                        self.odom_pub.publish(odom)
                
            except Exception as e:
                if self.running:  # Only log error if not shutting down
                    self.get_logger().error(f"Sensor read error: {e}")
                time.sleep(0.1)


def main(args=None):
    rclpy.init(args=args)
    node = RoombaNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False  # Signal thread to stop
        with node.serial_lock:
            pause_stream()
            passive()  # Return roomba to passive mode
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()