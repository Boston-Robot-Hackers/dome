#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import sys
from datetime import datetime
from tf_transformations import euler_from_quaternion
import math

class MultiSensorCSVLogger(Node):
    def __init__(self):
        super().__init__('multi_sensor_csv_logger')
        # Counter for messages from each topic
        self.msg_count = {
            'raw': 0,
            'corrected': 0,
            'odom': 0
        }
        self.max_messages = 10
        
        # Create subscriptions to the IMU topics
        self.raw_subscription = self.create_subscription(
            Imu,
            '/imu/data_raw',
            lambda msg: self.imu_callback(msg, 'raw'),
            10
        )
        
        self.corrected_subscription = self.create_subscription(
            Imu,
            '/imu/corrected_data',
            lambda msg: self.imu_callback(msg, 'corrected'),
            10
        )
        
        # Create subscription to the odom topic
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Track which topics have completed
        self.completed_topics = set()
        
        # Write CSV header
        print("data")
        self.get_logger().info('Multi-Sensor CSV Logger started - listening on IMU and odometry topics')

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
        
        # Format all data into a single string
        data_str = f"{topic_name} {timestamp} r={roll:.2f} p={pitch:.2f} y={yaw:.2f} vx={ang_vel_x:.2f} vy={ang_vel_y:.2f} vz={ang_vel_z:.2f}"
        
        # Print as a single column
        print(data_str)
        
        # Flush stdout to ensure data is written immediately
        sys.stdout.flush()
        
        # Check if we've received enough messages from this topic
        if self.msg_count[topic_type] >= self.max_messages:
            self.completed_topics.add(topic_type)
            
            # Check if all topics have completed
            if len(self.completed_topics) == 3:
                rclpy.shutdown()

    def odom_callback(self, msg):
        # Skip if we've already received enough messages from this topic
        if self.msg_count['odom'] >= self.max_messages:
            return
            
        self.msg_count['odom'] += 1
        
        # Format timestamp as HH:MM:SS
        timestamp = datetime.fromtimestamp(msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9).strftime('%H:%M:%S')
        
        # Get the human-readable topic name
        topic_name = "odom"
        
        # Convert quaternion to Euler angles using tf2
        quaternion = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]
        
        # Get Euler angles in radians (roll, pitch, yaw)
        (roll, pitch, yaw) = euler_from_quaternion(quaternion)
        
        # Convert from radians to degrees
        roll = math.degrees(roll)
        pitch = math.degrees(pitch)
        yaw = math.degrees(yaw)
        
        # Get angular velocity data from odom
        ang_vel_x = msg.twist.twist.angular.x
        ang_vel_y = msg.twist.twist.angular.y
        ang_vel_z = msg.twist.twist.angular.z
        
        # Format all data into a single string
        data_str = f"{topic_name} {timestamp} r={roll:.2f} p={pitch:.2f} y={yaw:.2f} vx={ang_vel_x:.2f} vy={ang_vel_y:.2f} vz={ang_vel_z:.2f}"
        
        # Print as a single column
        print(data_str)
        
        # Flush stdout to ensure data is written immediately
        sys.stdout.flush()
        
        # Check if we've received enough messages from this topic
        if self.msg_count['odom'] >= self.max_messages:
            self.completed_topics.add('odom')
            
            # Check if all topics have completed
            if len(self.completed_topics) == 3:
                rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = MultiSensorCSVLogger()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()