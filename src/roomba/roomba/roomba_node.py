#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import threading
import json
import time
import logging

from roomba.driver import drive, stream, pause_stream, startup, SENSOR_PACKETS, passive

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
        
        # Initialize Roomba
        with self.serial_lock:
            startup(full_mode=True)  # Use full mode for complete control
            stream(SENSOR_PACKETS)  # Start streaming sensors

        # Start sensor streaming in a background thread
        threading.Thread(target=self.stream_sensors, daemon=True).start()

    def cmd_vel_callback(self, msg: Twist):
        """
        Converts Twist messages to Roomba drive commands.
        This example uses simple differential drive mapping:
        - linear.x = forward/backward speed
        - angular.z = rotation
        """
        # Constants for converting Twist to Roomba drive
        # Roomba radius for turning (mm) approximation
        if msg.angular.z == 0:
            radius = 32768  # straight
        elif msg.linear.x == 0:
            radius = 1 if msg.angular.z > 0 else -1  # turn in place
        else:
            radius = int(msg.linear.x / msg.angular.z * 1000)  # crude approximation

        velocity = int(msg.linear.x * 1000)  # convert m/s to mm/s
        velocity = max(-500, min(500, velocity))  # clamp
        radius = max(-2000, min(2000, radius))    # clamp

        with self.serial_lock:
            drive(velocity, radius)

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