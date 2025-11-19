#!/usr/bin/env python3

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math

class VelocityController(Node):
    def __init__(self):
        super().__init__('velocity_controller')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 1Hz control loop
        self.velocities = [0.4, 0.3, 0.2, 0.1]
        self.current_velocity_index = 0
        self.start_time = time.time()
        self.state = 'forward'  # States: forward, wait, backward
        self.completed = False
        self.rate = self.create_rate(0.5)  # 0.5 Hz = 2 second period

    def timer_callback(self):
        if self.completed:
            self.get_logger().info('Sequence completed. Shutting down...')
            self.destroy_timer(self.timer)
            rclpy.shutdown()
            return

        current_time = time.time()
        elapsed_time = current_time - self.start_time
        msg = Twist()

        if self.state == 'forward':
            self.get_logger().info('Forward...')
            if elapsed_time < 2.0:  # Forward for 2 seconds
                msg.linear.x = self.velocities[self.current_velocity_index]
                self.publisher.publish(msg)
            else:
                self.state = 'wait1'
                self.start_time = current_time
        elif self.state == 'wait1':
            if elapsed_time < 1.0:  # Wait for 1 second
                # Send zero velocity during wait
                self.publisher.publish(Twist())
            else:
                self.state = 'rotate'
                self.start_time = current_time
        elif self.state == 'rotate':
            self.get_logger().info('Rotate...')
            if elapsed_time < 2.0:
                msg.angular.z = math.pi/2# rotate at 18 degrees per second 
                self.publisher.publish(msg)
            else:
                self.state = 'wait2'
                self.start_time = time.time()
        elif self.state == 'wait2':
            if elapsed_time < 1.0:
                # Send zero velocity during wait
                self.publisher.publish(Twist())
            else:
                self.state = 'backward'
                self.start_time = current_time
        elif self.state == 'backward':
            self.get_logger().info('Backward...')
            if elapsed_time < 2.0:  # Backward for 2 seconds
                msg.linear.x = self.velocities[self.current_velocity_index]
                self.publisher.publish(msg)
            else:
                self.current_velocity_index += 1
                if self.current_velocity_index >= len(self.velocities):
                    self.completed = True
                else:
                    self.state = 'rotate2'
                    self.start_time = current_time
        elif self.state == 'rotate2':
            self.get_logger().info('rotate2...')
            if elapsed_time < 2.0:
                msg.angular.z = math.pi/2.0 # rotate at 18 degrees per second 
                self.publisher.publish(msg)
            else:
                self.state = 'wait3'
                self.start_time = time.time()
        elif self.state == 'wait3': 
            if elapsed_time < 1.0:
                # Send zero velocity during wait
                self.publisher.publish(Twist())
            else:
                self.state = 'forward'
                self.start_time = current_time
                self.get_logger().info('Moving to next velocity: {}'.format(self.velocities[self.current_velocity_index]))
def main():
    rclpy.init()
    controller = VelocityController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    
    # Ensure we stop the robot before shutting down
    stop_msg = Twist()
    controller.publisher.publish(stop_msg)
    
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()