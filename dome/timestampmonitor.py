#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
import numpy as np
from threading import Lock
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import datetime

class TimestampMonitor(Node):
    def __init__(self):
        super().__init__('timestamp_monitor')
        
        # Parameters (make these configurable via ROS params)
        self.laser_topic = self.declare_parameter('laser_topic', '/scan').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.laser_frame = self.declare_parameter('laser_frame', 'laser').value
        self.history_length = self.declare_parameter('history_length', 100).value
        
        # TF Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscription to laser scan with best-effort reliability
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            depth=10
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.laser_topic,
            self.scan_callback,
            qos_profile)
        
        # Data storage
        self.lock = Lock()
        self.scan_timestamps = []
        self.tf_timestamps = []
        self.time_deltas = []
        self.scan_time_in_sec = []
        self.tf_time_in_sec = []
        
        # Calculate statistics
        self.max_delay = 0.0
        self.min_delay = 0.0
        self.avg_delay = 0.0
        self.dropped_msgs = 0
        self.early_timestamps = 0
        self.received_msgs = 0
        
        # Logging
        self.get_logger().info(f'Monitoring timestamps between {self.laser_frame} and {self.odom_frame}')
        
        # Create stats publisher timer
        self.create_timer(1.0, self.publish_stats)
        
        # Setup plotting if matplotlib is available
        self.setup_plot()
    
    def scan_callback(self, msg):
        """Process incoming laser scan messages and compare with TF timestamps."""
        self.received_msgs += 1
        
        # Convert ROS time to seconds for comparison
        scan_time = Time.from_msg(msg.header.stamp)
        scan_time_sec = scan_time.nanoseconds / 1e9
        
        # Try to get the corresponding transform
        try:
            # Look up the transform at the exact time of the laser scan
            trans = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.laser_frame,
                msg.header.stamp,
                timeout=Duration(seconds=0.1)
            )
            
            # Get the timestamp of the transform
            tf_time = Time.from_msg(trans.header.stamp)
            tf_time_sec = tf_time.nanoseconds / 1e9
            
            # Calculate time delta
            time_delta = scan_time_sec - tf_time_sec
            
            with self.lock:
                # Store data (keep only the last N samples)
                self.scan_timestamps.append(scan_time_sec)
                self.tf_timestamps.append(tf_time_sec)
                self.time_deltas.append(time_delta)
                self.scan_time_in_sec.append(datetime.datetime.fromtimestamp(scan_time_sec).strftime('%H:%M:%S.%f'))
                self.tf_time_in_sec.append(datetime.datetime.fromtimestamp(tf_time_sec).strftime('%H:%M:%S.%f'))
                
                if len(self.time_deltas) > self.history_length:
                    self.scan_timestamps.pop(0)
                    self.tf_timestamps.pop(0)
                    self.time_deltas.pop(0)
                    self.scan_time_in_sec.pop(0)
                    self.tf_time_in_sec.pop(0)
                
                # Update statistics
                if time_delta < 0:
                    self.early_timestamps += 1
                
                # Update extremes
                if len(self.time_deltas) == 1:
                    self.max_delay = time_delta
                    self.min_delay = time_delta
                else:
                    self.max_delay = max(self.max_delay, time_delta)
                    self.min_delay = min(self.min_delay, time_delta)
                
                # Update average
                self.avg_delay = sum(self.time_deltas) / len(self.time_deltas)
            
            # Log problematic deltas
            if abs(time_delta) > 0.1:  # 100ms threshold, adjust as needed
                self.get_logger().warning(
                    f'Large timestamp delta: {time_delta:.6f}s, scan: {self.scan_time_in_sec[-1]}, '
                    f'tf: {self.tf_time_in_sec[-1]}'
                )
        
        except Exception as e:
            self.dropped_msgs += 1
            self.get_logger().warning(f'Failed to get transform for laser scan: {e}')
    
    def publish_stats(self):
        """Publish statistics about timestamp differences."""
        with self.lock:
            if not self.time_deltas:
                return
            
            self.get_logger().info(
                f'\n--- Timestamp Statistics ---\n'
                f'Received msgs: {self.received_msgs}\n'
                f'Dropped msgs: {self.dropped_msgs} ({self.dropped_msgs/max(1, self.received_msgs)*100:.1f}%)\n'
                f'Scans with early timestamps: {self.early_timestamps}\n'
                f'Current delay: {self.time_deltas[-1]:.6f}s\n'
                f'Min delay: {self.min_delay:.6f}s\n'
                f'Max delay: {self.max_delay:.6f}s\n'
                f'Avg delay: {self.avg_delay:.6f}s\n'
                f'----------------------------'
            )
            
            # Update the plot if available
            if hasattr(self, 'line'):
                self.update_plot()
    
    def setup_plot(self):
        """Set up real-time plotting if matplotlib is available."""
        try:
            # Create the figure and axes
            self.fig, self.ax = plt.subplots(figsize=(10, 6))
            self.line, = self.ax.plot([], [], 'r-', label='Timestamp Delay')
            self.ax.axhline(y=0, color='k', linestyle='-', alpha=0.3)
            
            # Configure plot
            self.ax.set_xlabel('Sample')
            self.ax.set_ylabel('Delay (seconds)')
            self.ax.set_title('Laser Scan vs. TF Timestamp Delay')
            self.ax.grid(True)
            self.ax.legend()
            
            # Initialize plot with empty data
            self.update_plot()
            
            # Show the plot in non-blocking mode
            plt.ion()
            plt.show(block=False)
        
        except ImportError:
            self.get_logger().warning('Matplotlib not available. Plotting disabled.')
    
    def update_plot(self):
        """Update the real-time plot with latest data."""
        try:
            if hasattr(self, 'line') and self.time_deltas:
                # Update data
                self.line.set_data(range(len(self.time_deltas)), self.time_deltas)
                
                # Adjust y limits to fit the data with some padding
                if len(self.time_deltas) > 1:
                    y_min = min(self.time_deltas) - 0.01
                    y_max = max(self.time_deltas) + 0.01
                    self.ax.set_ylim(y_min, y_max)
                
                # Adjust x limits
                self.ax.set_xlim(0, max(len(self.time_deltas), 10))
                
                # Update the figure
                self.fig.canvas.draw_idle()
                self.fig.canvas.flush_events()
        except Exception as e:
            self.get_logger().error(f'Error updating plot: {e}')
    
    def save_report(self, filename='timestamp_report.txt'):
        """Save a detailed report of the timestamp analysis."""
        with open(filename, 'w') as f:
            f.write(f"Timestamp Analysis Report\n")
            f.write(f"========================\n\n")
            f.write(f"Generated: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"Laser topic: {self.laser_topic}\n")
            f.write(f"Frames: {self.laser_frame} -> {self.odom_frame}\n\n")
            
            f.write(f"Statistics:\n")
            f.write(f"  Received messages: {self.received_msgs}\n")
            f.write(f"  Dropped messages: {self.dropped_msgs} ({self.dropped_msgs/max(1, self.received_msgs)*100:.1f}%)\n")
            f.write(f"  Early timestamps: {self.early_timestamps}\n")
            f.write(f"  Min delay: {self.min_delay:.6f}s\n")
            f.write(f"  Max delay: {self.max_delay:.6f}s\n")
            f.write(f"  Avg delay: {self.avg_delay:.6f}s\n\n")
            
            f.write(f"Last {len(self.time_deltas)} Samples:\n")
            f.write(f"Sample,Scan Timestamp,TF Timestamp,Delta(s)\n")
            
            for i in range(len(self.time_deltas)):
                f.write(f"{i},{self.scan_time_in_sec[i]},{self.tf_time_in_sec[i]},{self.time_deltas[i]:.6f}\n")

def main(args=None):
    rclpy.init(args=args)
    node = TimestampMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Save a report before shutting down
        node.save_report()
        node.get_logger().info('Saved timestamp report to timestamp_report.txt')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()