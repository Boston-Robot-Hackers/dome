#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import sys
import math
from datetime import datetime

class IMUCSVLogger(Node):
    def __init__(self):
        super().__init__('imu_csv_logger')
        
        # Counter for messages
        self.msg_count = 0
        self.max_messages = 10
        
        # Create a subscription to the IMU topic
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data_raw',
            self.imu_callback,
            10  # QoS profile depth
        )
        
        # Write CSV header
        print("timestamp, roll, pitch, yaw")
        
        self.get_logger().info('IMU CSV Logger started - listening on /imu/data_raw')
        self.get_logger().info(f'Will capture {self.max_messages} messages')

    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)
            
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Convert from radians to degrees
        roll = math.degrees(roll)
        pitch = math.degrees(pitch)
        yaw = math.degrees(yaw)
        
        return roll, pitch, yaw

    def imu_callback(self, msg):
        self.msg_count += 1
        
        # Format timestamp as HH:MM:SS
        timestamp = datetime.fromtimestamp(msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9).strftime('%H:%M:%S')
        
        # Convert quaternion to Euler angles
        roll, pitch, yaw = self.quaternion_to_euler(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
        
        # Print IMU data in CSV format with 2 decimal places and spaces after commas
        print(f"{timestamp}, {roll:.2f}, {pitch:.2f}, {yaw:.2f}")
        
        # Flush stdout to ensure data is written immediately 
        sys.stdout.flush()

                # Check if we've received enough messages
        if self.msg_count >= self.max_messages:
            self.get_logger().info('Received 10 messages. Shutting down...')
            # Signal the main loop to stop
            rclpy.shutdown()

        
def main(args=None):
    rclpy.init(args=args)
    
    node = IMUCSVLogger()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()