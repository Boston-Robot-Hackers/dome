#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import csv
import sys
from datetime import datetime

class SensorCSV(Node):
    def __init__(self):
        super().__init__('sensor_csv')
        
        # Initialize data storage
        self.latest_filtered_yaw = None
        self.latest_unfiltered_yaw = None
        
        # Create subscriptions to the odom topics
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.filtered_odom_callback,
            10
        )
        
        self.odom_unfiltered_subscription = self.create_subscription(
            Odometry,
            '/odom/unfiltered',
            self.unfiltered_odom_callback,
            10
        )
        
        # Set up CSV writer with custom formatting (space after comma)
        self.csv_writer = csv.writer(sys.stdout, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        
        # Write CSV header (manually to ensure space after comma)
        print("Time, Filtered_Yaw, Unfiltered_Yaw")
        
        # Create timer for periodic CSV output (every 0.5 seconds)
        self.timer = self.create_timer(0.5, self.write_csv_row)
        
    def filtered_odom_callback(self, msg):
        # Extract yaw from filtered odometry quaternion
        quaternion = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]
        
        # Get Euler angles in radians (roll, pitch, yaw)
        _, _, yaw = euler_from_quaternion(quaternion)
        
        # Store the yaw value
        self.latest_filtered_yaw = yaw
        
    def unfiltered_odom_callback(self, msg):
        # Extract yaw from unfiltered odometry quaternion
        quaternion = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]
        
        # Get Euler angles in radians (roll, pitch, yaw)
        _, _, yaw = euler_from_quaternion(quaternion)
        
        # Store the yaw value
        self.latest_unfiltered_yaw = yaw
        
    def write_csv_row(self):
        # Get current time in HH:MM:SS format
        current_time = datetime.now().strftime('%H:%M:%S')
        
        # Check if we have both yaw values
        if self.latest_filtered_yaw is not None and self.latest_unfiltered_yaw is not None:
            # Convert radians to degrees and format to 2 decimal places
            filtered_yaw_deg = round(self.latest_filtered_yaw * 180.0 / 3.141592653589793, 2)
            unfiltered_yaw_deg = round(self.latest_unfiltered_yaw * 180.0 / 3.141592653589793, 2)
            
            # Write CSV row with space after each comma and 2 decimal places
            print(f"{current_time}, {filtered_yaw_deg:.2f}, {unfiltered_yaw_deg:.2f}")
            
            # Ensure output is flushed
            sys.stdout.flush()

def main(args=None):
    rclpy.init(args=args)
    node = SensorCSV()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()