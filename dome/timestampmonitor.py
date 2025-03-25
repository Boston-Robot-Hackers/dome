#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.time import Time
import numpy as np

class TimestampCorrectionNode(Node):
    def __init__(self):
        super().__init__('timestamp_correction_node')
        
        # Parameters (adjustable via ROS params)
        self.input_topic = self.declare_parameter('input_topic', '/scan').value
        self.output_topic = self.declare_parameter('output_topic', '/scan_corrected').value
        self.delay_compensation = self.declare_parameter('delay_compensation', 0.06).value  # 60ms default
        self.time_correction_mode = self.declare_parameter('time_correction_mode', 'current_time').value
        self.log_interval = self.declare_parameter('log_interval', 100).value
        
        # Statistics
        self.processed_msgs = 0
        self.sum_delay = 0.0
        
        # Setup QoS - match the input profile for best compatibility
        input_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create output QoS with more reliable settings if needed
        output_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Keep consistent with input
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscription and publisher
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.input_topic,
            self.scan_callback,
            input_qos
        )
        
        self.scan_pub = self.create_publisher(
            LaserScan,
            self.output_topic,
            output_qos
        )
        
        self.get_logger().info(
            f'Started timestamp correction: {self.input_topic} -> {self.output_topic}'
            f' with {self.delay_compensation}s delay compensation'
        )
    
    def scan_callback(self, msg):
        """Process scan messages and correct timestamps"""
        # Create a new message (copy of the input)
        corrected_msg = LaserScan()
        corrected_msg = msg
        
        # Store original timestamp for logging
        original_stamp = Time.from_msg(msg.header.stamp)
        original_time_sec = original_stamp.nanoseconds / 1e9
        
        # Apply the timestamp correction strategy based on mode
        if self.time_correction_mode == 'current_time':
            # Use current time minus compensation delay
            current_time = self.get_clock().now()
            corrected_stamp = current_time - rclpy.duration.Duration(seconds=self.delay_compensation)
            corrected_msg.header.stamp = corrected_stamp.to_msg()
        
        elif self.time_correction_mode == 'offset':
            # Subtract the compensation delay from the original timestamp
            corrected_stamp = original_stamp - rclpy.duration.Duration(seconds=self.delay_compensation)
            corrected_msg.header.stamp = corrected_stamp.to_msg()
        
        # Publish corrected message
        self.scan_pub.publish(corrected_msg)
        
        # Update statistics
        self.processed_msgs += 1
        
        # Log periodically
        if self.processed_msgs % self.log_interval == 0:
            corrected_stamp = Time.from_msg(corrected_msg.header.stamp)
            corrected_time_sec = corrected_stamp.nanoseconds / 1e9
            time_diff = original_time_sec - corrected_time_sec
            
            self.get_logger().info(
                f'Processed {self.processed_msgs} messages. '
                f'Last correction: {time_diff:.6f}s'
            )

def main(args=None):
    rclpy.init(args=args)
    node = TimestampCorrectionNode()
    
    try:
        rclpy.spin(node)
    except Ke