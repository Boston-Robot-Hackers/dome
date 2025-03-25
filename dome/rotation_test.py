#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion
import sys
from datetime import datetime
import time

class RotationTest(Node):
    def __init__(self, rotation_speed=0.5, duration=20):
        super().__init__('rotation_test')
        
        self.rotation_speed = rotation_speed
        self.duration = duration
        self.start_time = None
        
        # Initialize data storage
        self.filtered_yaw = None
        self.unfiltered_yaw = None
        self.initial_filtered_yaw = None
        self.total_rotation = 0.0
        self.prev_filtered_yaw = None
        
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
        
        # Create publisher for cmd_vel
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Create timer for publishing and printing
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Log the test parameters
        self.get_logger().info(f"Starting rotation test with speed: {rotation_speed} rad/s until 360° rotation")
        
        # Print CSV header
        print("Time, Filtered_Yaw, Unfiltered_Yaw, Total_Rotation")
        
    def filtered_odom_callback(self, msg):
        quaternion = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]
        _, _, yaw = euler_from_quaternion(quaternion)
        
        # Store the first yaw value as initial
        if self.initial_filtered_yaw is None:
            self.initial_filtered_yaw = yaw
            self.prev_filtered_yaw = yaw
        
        # Calculate rotation since last measurement
        if self.prev_filtered_yaw is not None:
            # Handle the wrap-around from -pi to pi
            diff = yaw - self.prev_filtered_yaw
            if diff > 3.0:  # More than 3 radians jump, must have wrapped from +pi to -pi
                diff -= 2 * 3.1415926535
            elif diff < -3.0:  # More than 3 radians jump, must have wrapped from -pi to +pi
                diff += 2 * 3.1415926535
                
            self.total_rotation += diff
        
        self.filtered_yaw = yaw
        self.prev_filtered_yaw = yaw
        
    def unfiltered_odom_callback(self, msg):
        quaternion = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]
        _, _, yaw = euler_from_quaternion(quaternion)
        self.unfiltered_yaw = yaw
        
    def timer_callback(self):
        # Initialize start time on first callback
        if self.start_time is None:
            self.start_time = time.time()
            
        # Get elapsed time
        elapsed = time.time() - self.start_time
        
        # Check if we've completed 360 degrees (2π radians)
        if abs(self.total_rotation) >= 2 * 3.1415926535:
            self.get_logger().info(f"Rotation test completed: {abs(self.total_rotation * 180.0 / 3.1415926535):.2f}° rotated")
            self.publish_zero_velocity()
            sys.exit(0)
            
        # Publish rotation command
        self.publish_rotation()
        
        # Get current time in HH:MM:SS format
        current_time = datetime.now().strftime('%H:%M:%S')
        
        # Print yaw values if available
        if self.filtered_yaw is not None and self.unfiltered_yaw is not None:
            # Convert radians to degrees
            filtered_yaw_deg = round(self.filtered_yaw * 180.0 / 3.1415926535, 2)
            unfiltered_yaw_deg = round(self.unfiltered_yaw * 180.0 / 3.1415926535, 2)
            total_rotation_deg = round(self.total_rotation * 180.0 / 3.1415926535, 2)
            
            # Print CSV formatted output
            print(f"{current_time}, {filtered_yaw_deg:.2f}, {unfiltered_yaw_deg:.2f}, {total_rotation_deg:.2f}")
            
    def publish_rotation(self):
        msg = Twist()
        msg.angular.z = self.rotation_speed
        self.cmd_vel_publisher.publish(msg)
        
    def publish_zero_velocity(self):
        msg = Twist()
        msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(msg)
        # Publish multiple times to ensure the robot stops
        for _ in range(5):
            self.cmd_vel_publisher.publish(msg)
            time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    
    # Parse command line arguments for rotation speed and duration
    rotation_speed = 0.5
    duration = 20
    
    if len(sys.argv) >= 2:
        try:
            rotation_speed = float(sys.argv[1])
        except ValueError:
            print(f"# Invalid rotation speed: {sys.argv[1]}. Using default: 0.5 rad/s", file=sys.stderr)
    
    if len(sys.argv) >= 3:
        try:
            duration = int(sys.argv[2])
        except ValueError:
            print(f"# Invalid duration: {sys.argv[2]}. Using default: 20 seconds", file=sys.stderr)
    
    node = RotationTest(rotation_speed, duration)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Test interrupted by user")
        node.publish_zero_velocity()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()