#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import sys
from datetime import datetime
from tf_transformations import euler_from_quaternion
import math

class IMUCSVLogger(Node):
    def __init__(self):
        super().__init__('imu_csv_logger')
        # Counter for messages
        self.msg_count = 0
        self.max_messages = 100
        # Create a subscription to the IMU topic
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data_raw',
            self.imu_callback,
            10  # QoS profile depth
        )
        # Write CSV header
        print("timestamp, roll, pitch, yaw, ang_vel_x, ang_vel_y, ang_vel_z, lin_acc_x, lin_acc_y, lin_acc_z")
        self.get_logger().info('IMU CSV Logger started - listening on /imu/data_raw')
        self.get_logger().info(f'Will capture {self.max_messages} messages')

    def imu_callback(self, msg):
        self.msg_count += 1
        # Format timestamp as HH:MM:SS
        timestamp = datetime.fromtimestamp(msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9).strftime('%H:%M:%S')
        
        # Convert quaternion to Euler angles using tf2
        quaternion = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]
        
        # Get Euler angles in radians (roll, pitch, yaw)
        (roll, pitch, yaw) = euler_from_quaternion(quaternion)
        
        # Convert from radians to degrees
        roll = math.degrees(roll)
        pitch = math.degrees(pitch)
        yaw = math.degrees(yaw)
        
        # Get angular velocity data
        ang_vel_x = msg.angular_velocity.x
        ang_vel_y = msg.angular_velocity.y
        ang_vel_z = msg.angular_velocity.z
        
        # Get linear acceleration data
        lin_acc_x = msg.linear_acceleration.x
        lin_acc_y = msg.linear_acceleration.y
        lin_acc_z = msg.linear_acceleration.z
        
        # Print IMU data in CSV format with 2 decimal places and spaces after commas
        print(f"{timestamp}, {roll:.2f}, {pitch:.2f}, {yaw:.2f}, " + 
              f"{ang_vel_x:.2f}, {ang_vel_y:.2f}, {ang_vel_z:.2f}, " + 
              f"{lin_acc_x:.2f}, {lin_acc_y:.2f}, {lin_acc_z:.2f}")
        
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