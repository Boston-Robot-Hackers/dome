#!/usr/bin/env python3

import argparse
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import threading
import time
import sys
from collections import defaultdict, deque
from rosidl_runtime_py.utilities import get_message
import importlib

class TopicHzMonitor(Node):
    def __init__(self, topics, window_size=100):
        super().__init__('topic_hz_monitor')
        
        self.topics = topics
        self.window_size = window_size
        self.subscribers = {}
        self.timestamps = defaultdict(lambda: deque(maxlen=window_size))
        self.message_counts = defaultdict(int)
        self.topic_types = {}
        
        # Discover topic types and create subscribers
        self._setup_subscribers()
        
        # Start monitoring thread
        self.monitoring = True
        self.monitor_thread = threading.Thread(target=self._monitor_loop)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()
    
    def _setup_subscribers(self):
        """Discover topic types and create subscribers"""
        topic_names_and_types = self.get_topic_names_and_types()
        topic_dict = {name: types for name, types in topic_names_and_types}
        
        for topic in self.topics:
            if topic not in topic_dict:
                self.get_logger().warn(f"Topic '{topic}' not found. Available topics: {list(topic_dict.keys())}")
                continue
            
            # Get the first available type for the topic
            topic_type = topic_dict[topic][0]
            self.topic_types[topic] = topic_type
            
            try:
                # Import the message type
                msg_class = get_message(topic_type)
                
                # Create subscriber
                subscriber = self.create_subscription(
                    msg_class,
                    topic,
                    lambda msg, t=topic: self._message_callback(t),
                    qos_profile=qos_profile_sensor_data
                )
                
                self.subscribers[topic] = subscriber
                self.get_logger().info(f"Subscribed to {topic} ({topic_type})")
                
            except Exception as e:
                self.get_logger().error(f"Failed to subscribe to {topic}: {str(e)}")
    
    def _message_callback(self, topic):
        """Callback for receiving messages"""
        current_time = time.time()
        self.timestamps[topic].append(current_time)
        self.message_counts[topic] += 1
    
    def _calculate_hz(self, topic):
        """Calculate Hz for a specific topic"""
        if topic not in self.timestamps or len(self.timestamps[topic]) < 2:
            return 0.0
        
        timestamps = list(self.timestamps[topic])
        if len(timestamps) < 2:
            return 0.0
        
        # Calculate time span
        time_span = timestamps[-1] - timestamps[0]
        if time_span == 0:
            return 0.0
        
        # Calculate frequency
        num_messages = len(timestamps) - 1
        hz = num_messages / time_span
        
        return hz
    
    def _monitor_loop(self):
        """Main monitoring loop that prints statistics"""
        while self.monitoring and rclpy.ok():
            try:
                # Clear screen (optional - comment out if not desired)
                print("\033[2J\033[H", end="")
                
                print("=" * 80)
                print(f"ROS2 Topic Hz Monitor - {time.strftime('%H:%M:%S')}")
                print("=" * 80)
                print(f"{'Topic':<40} {'Type':<25} {'Hz':<10} {'Count':<10}")
                print("-" * 80)
                
                for topic in self.topics:
                    if topic in self.subscribers:
                        hz = self._calculate_hz(topic)
                        count = self.message_counts[topic]
                        topic_type = self.topic_types.get(topic, "Unknown")
                        
                        # Truncate long topic names and types for display
                        display_topic = topic[:39] if len(topic) > 39 else topic
                        display_type = topic_type.split('/')[-1][:24] if len(topic_type) > 24 else topic_type
                        
                        print(f"{display_topic:<40} {display_type:<25} {hz:>6.2f}{'':<4} {count:<10}")
                    else:
                        print(f"{topic:<40} {'Not found':<25} {'N/A':<10} {'N/A':<10}")
                
                print("=" * 80)
                print("Press Ctrl+C to exit")
                
                time.sleep(1.0)  # Update every second
                
            except KeyboardInterrupt:
                break
            except Exception as e:
                self.get_logger().error(f"Error in monitoring loop: {str(e)}")
    
    def stop_monitoring(self):
        """Stop the monitoring loop"""
        self.monitoring = False


def main():
    parser = argparse.ArgumentParser(description='Monitor ROS2 topic frequencies (Hz)')
    parser.add_argument('topics', nargs='+', help='List of topic names to monitor')
    parser.add_argument('--window-size', '-w', type=int, default=100,
                       help='Window size for Hz calculation (default: 100)')
    
    args = parser.parse_args()
    
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Create and run the monitor
        monitor = TopicHzMonitor(args.topics, args.window_size)
        
        print(f"Monitoring {len(args.topics)} topics...")
        print("Topics:", ", ".join(args.topics))
        print("Press Ctrl+C to exit\n")
        
        # Spin the node
        rclpy.spin(monitor)
        
    except KeyboardInterrupt:
        print("\nShutting down...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'monitor' in locals():
            monitor.stop_monitoring()
        rclpy.shutdown()


if __name__ == '__main__':
    main()