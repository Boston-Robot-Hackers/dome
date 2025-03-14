#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import sys
from datetime import datetime
from tf_transformations import euler_from_quaternion
import math

class SensorTrace(Node):
    def __init__(self):
        super().__init__('sensor_trace')
        # Counter for messages from each topic
        self.msg_count = {
            'raw': 0,
            'corrected': 0,
            'odom': 0,
            'odom_unfiltered': 0
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
        
        # Create subscription to the odom topics
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            lambda msg: self.odom_callback(msg, 'odom'),
            10
        )
        
        self.odom_unfiltered_subscription = self.create_subscription(
            Odometry,
            '/odom/unfiltered',
            lambda msg: self.odom_callback(msg, 'odom_unfiltered'),
            10
        )
        
        # Track which topics have completed
        self.completed_topics = set()
        
        # Add a timer to check if we should exit
        self.exit_timer = self.create_timer(2.0, self.check_exit)
        
        # Write header with fixed-width columns
        header = f"{'topic':<15} {'timestamp':<12} {'roll':>8} {'pitch':>8} {'yaw':>8} {'ang_vel_x':>10} {'ang_vel_y':>10} {'ang_vel_z':>10}"
        print(header)
        print("-" * len(header))  # Print a separator line

    def check_exit(self):
        # Check if we've collected enough data from all topics
        all_complete = all(count >= self.max_messages for count in self.msg_count.values())
        
        if all_complete:
            self.get_logger().info("All topics have received enough messages. Exiting...")
            # Use sys.exit directly to force exit
            import sys
            sys.exit(0)

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
        
        # Format all data into fixed-width columns
        data_str = f"{topic_name:<15} {timestamp:<12} {roll:8.2f} {pitch:8.2f} {yaw:8.2f} {ang_vel_x:10.2f} {ang_vel_y:10.2f} {ang_vel_z:10.2f}"
        
        # Print the formatted string
        print(data_str)
        
        # Flush stdout to ensure data is written immediately
        sys.stdout.flush()
        
        # Check if we've received enough messages from this topic
        if self.msg_count[topic_type] >= self.max_messages:
            self.completed_topics.add(topic_type)

    def odom_callback(self, msg, topic_type):
        # Skip if we've already received enough messages from this topic
        if self.msg_count[topic_type] >= self.max_messages:
            return
            
        self.msg_count[topic_type] += 1
        
        # Format timestamp as HH:MM:SS
        timestamp = datetime.fromtimestamp(msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9).strftime('%H:%M:%S')
        
        # Get the human-readable topic name
        topic_name = topic_type
        
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
        
        # Format all data into fixed-width columns (matching the IMU format)
        data_str = f"{topic_name:<15} {timestamp:<12} {roll:8.2f} {pitch:8.2f} {yaw:8.2f} {ang_vel_x:10.2f} {ang_vel_y:10.2f} {ang_vel_z:10.2f}"
        
        # Print the formatted string
        print(data_str)
        
        # Flush stdout to ensure data is written immediately
        sys.stdout.flush()
        
        # Check if we've received enough messages from this topic
        if self.msg_count[topic_type] >= self.max_messages:
            self.completed_topics.add(topic_type)

def main(args=None):
    rclpy.init(args=args)
    node = SensorTrace()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()