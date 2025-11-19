#!/usr/bin/env python3

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math
from typing import Optional

class VelocityController(Node):
    def __init__(self):
        super().__init__('velocity_controller')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.velocities = [0.4, 0.3, 0.2, 0.1]
        self.current_velocity_index = 0
        self.start_time = time.time()
        self.state = 'forward'
        self.completed = False
        self.rate = self.create_rate(0.5)

    def create_twist_msg(self, linear_x: float = 0.0, angular_z: float = 0.0) -> Twist:
        """Create a Twist message with the given linear and angular velocities."""
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        return msg

    def handle_state_transition(self, current_state: str, duration: float) -> Optional[str]:
        """Handle state transition based on elapsed time."""
        elapsed_time = time.time() - self.start_time
        if elapsed_time >= duration:
            self.start_time = time.time()
            return self.get_next_state(current_state)
        return None

    def get_next_state(self, current_state: str) -> str:
        """Get the next state in the sequence."""
        state_sequence = {
            'forward': 'wait1',
            'wait1': 'rotate',
            'rotate': 'wait2',
            'wait2': 'backward',
            'backward': 'wait3',
            'wait3': 'rotate2',
            'rotate2': 'wait4',
            'wait4': 'forward'
        }
        return state_sequence.get(current_state, current_state)

    def handle_movement(self, state: str) -> Twist:
        """Generate appropriate Twist message based on current state."""
        current_velocity = self.velocities[self.current_velocity_index]
        
        movement_commands = {
            'forward': (current_velocity, 0.0),
            'backward': (current_velocity, 0.0),
            'rotate': (0.0, math.pi/2.0),
            'rotate2': (0.0, math.pi/2.0),
            'wait1': (0.0, 0.0),
            'wait2': (0.0, 0.0),
            'wait3': (0.0, 0.0),
            'wait4': (0.0, 0.0)
        }
        
        linear_x, angular_z = movement_commands.get(state, (0.0, 0.0))
        return self.create_twist_msg(linear_x, angular_z)

    def timer_callback(self):
        if self.completed:
            self.get_logger().info('Sequence completed. Shutting down...')
            self.destroy_timer(self.timer)
            rclpy.shutdown()
            return

        # Handle state transitions
        state_durations = {
            'forward': 2.0,
            'backward': 2.0,
            'rotate': 2.0,
            'rotate2': 2.0,
            'wait1': 1.0,
            'wait2': 1.0,
            'wait3': 1.0,
            'wait4': 1.0
        }

        if self.state in ['forward', 'backward', 'rotate', 'rotate2']:
            self.get_logger().info(f'{self.state}:{self.velocities[self.current_velocity_index]}...')

        next_state = self.handle_state_transition(self.state, state_durations[self.state])
        if next_state:
            if next_state == 'forward' and self.state == 'wait4':
                self.current_velocity_index += 1
                if self.current_velocity_index >= len(self.velocities):
                    self.completed = True
                    return
                self.get_logger().info(f'Moving to next velocity: {self.velocities[self.current_velocity_index]}')
            self.state = next_state

        # Generate and publish movement command
        msg = self.handle_movement(self.state)
        self.publisher.publish(msg)

def main():
    rclpy.init()
    controller = VelocityController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    
    # Ensure we stop the robot before shutting down
    controller.publisher.publish(Twist())
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()