#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import sys
from datetime import datetime
from tf_transformations import euler_from_quaternion
import math

class MultiIMUCSVLogger(Node):
    def __init__(self):
        super().__init__('multi_imu_csv_logger')
        # Counter for messages from each topic
        self.msg_count = {
            'raw': 0,
            'data': 0,
            'corrected': 0
        }
        self.max_messages = 10
        
        # Create subscriptions to the three IMU topics
        self.raw_subscription = self.create_subscription(
            Imu,
            '/imu/data_raw',
            lambda msg: self.imu_callback(msg, 'raw'),
            10
        )
        
        self.data_subscription = self.create_subscription(
            Imu,
            '/imu/data',
            lambda msg: self.imu_callback(msg, 'data'),
            10
        )
        
        self.corrected_subscription = self.create_subscription(
            Imu,
            '/imu/corrected',
            lambda msg: self.imu_callback(msg, 'corrected'),
            10
        )
        
        # Track which topics have completed
        self.completed_topics = set()
        
        # Write CSV header
        print("topic, timestamp, roll, pitch, yaw, ang_vel_x, ang_vel_y, ang_vel_z, lin_acc_x, lin_acc_y, lin_acc_z")
        self.get_logger().info('Multi-IMU CSV Logger started - listening on three IMU topics')
        self.get_logger().info(f'Will capture {self.max_messages} messages from each topic')

    def imu_callback(self, msg, topic_type):
        # Skip if we've already received enough messages from this topic
        if self.msg_count[topic_type] >= self.max_messages:
            return
            
        self.msg_count[topic_type] += 1
        
        # Format timestamp as HH:MM:SS
        timestamp = datetime.fromtimestamp(msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9).strftime('%H:%M:%S')
        
        # Get the human-readable topic name
        topic_name = f"imu_{topic_type}"
        
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
        print(f"{topic_name}, {timestamp}, {roll:.2f}, {pitch:.2f}, {yaw:.2f}, " + 
              f"{ang_vel_x:.2f}, {ang_vel_y:.2f}, {ang_vel_z:.2f}, " + 
              f"{lin_acc_x:.2f}, {lin_acc_y:.2f}, {lin_acc_z:.2f}")
        
        # Flush stdout to ensure data is written immediately
        sys.stdout.flush()
        
        # Check if we've received enough messages from this topic
        if self.msg_count[topic_type] >= self.max_messages:
            self.get_logger().info(f'Received {self.max_messages} messages from {topic_name}')
            self.completed_topics.add(topic_type)
            
            # Check if all topics have completed
            if len(self.completed_topics) == 3:
                self.get_logger().info('Received required messages from all topics. Shutting down...')
                rclpy.shutdown()
                
    def print_status(self):
        """Print current message count status"""
        self.get_logger().info(f"Messages received - raw: {self.msg_count['raw']}, data: {self.msg_count['data']}, corrected: {self.msg_count['corrected']}")

def main(args=None):
    rclpy.init(args=args)
    node = MultiIMUCSVLogger()
    
    # Create a timer to periodically print status
    status_timer = node.create_timer(5.0, node.print_status)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        status_timer.cancel()
        node.destroy_node()
        print("\nLogging complete.")

if __name__ == '__main__':
    main()