#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import time
from collections import defaultdict
from tf2_msgs.msg import TFMessage


class TFTreeNode(Node):
    def __init__(self):
        super().__init__('tf_tree_viewer')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.frames_seen = set()
        self.parent_child_relationships = {}  # child -> parent mapping
        
        # Subscribe to TF topics to collect frame names and relationships
        self.tf_subscription = self.create_subscription(
            TFMessage, '/tf', self.tf_callback, 10)
        self.tf_static_subscription = self.create_subscription(
            TFMessage, '/tf_static', self.tf_callback, 10)
        
    def tf_callback(self, msg):
        # Collect frame names and relationships from TF messages
        for transform in msg.transforms:
            parent_frame = transform.header.frame_id
            child_frame = transform.child_frame_id
            
            self.frames_seen.add(parent_frame)
            self.frames_seen.add(child_frame)
            self.parent_child_relationships[child_frame] = parent_frame
            
    def get_tf_tree(self):
        # Wait for TF data to accumulate
        self.get_logger().info("Waiting for TF data...")
        
        # Spin for longer to collect more frames
        start_time = time.time()
        while time.time() - start_time < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        
        frames = list(self.frames_seen)
        self.get_logger().info(f"Found frames: {frames}")
        self.get_logger().info(f"Parent-child relationships: {self.parent_child_relationships}")
        
        if not frames:
            return {}
        
        # Build tree structure using the relationships we collected
        tree = {}
        for frame in frames:
            tree[frame] = {'parent': None, 'children': []}
        
        # Set parent-child relationships from what we observed
        for child, parent in self.parent_child_relationships.items():
            if child in tree and parent in tree:
                tree[child]['parent'] = parent
                if child not in tree[parent]['children']:
                    tree[parent]['children'].append(child)
        
        # Set parent-child relationships from what we observed
        for child, parent in self.parent_child_relationships.items():
            if child in tree and parent in tree:
                tree[child]['parent'] = parent
                if child not in tree[parent]['children']:
                    tree[parent]['children'].append(child)
        
        self.get_logger().info(f"Final relationships: {self.parent_child_relationships}")
        
        return tree
    
    def get_frame_info(self, frame, parent=None):
        info = []
        
        if parent:
            try:
                # Get transform from parent to this frame
                transform = self.tf_buffer.lookup_transform(
                    parent, frame, rclpy.time.Time()
                )
                
                # Get translation distance
                t = transform.transform.translation
                dist = (t.x**2 + t.y**2 + t.z**2)**0.5
                info.append(f"dist: {dist:.3f}m")
                
                # Check if it's recent (< 1 second old)
                now = self.get_clock().now()
                stamp_time = rclpy.time.Time.from_msg(transform.header.stamp)
                age = (now - stamp_time).nanoseconds / 1e9
                if age > 1.0:
                    info.append(f"old: {age:.1f}s")
                
            except (LookupException, ConnectivityException, ExtrapolationException):
                info.append("no transform")
        
        return " | ".join(info) if info else ""


def print_tree(tree, tf_node, frame=None, indent=0, visited=None):
    if visited is None:
        visited = set()
    
    if frame is None:
        # Find root frames (no parent)
        roots = [f for f, data in tree.items() if data['parent'] is None]
        if not roots:
            print("No root frames found!")
            return
        
        print("TF Tree:")
        print("-" * 40)
        for root in sorted(roots):
            print_tree(tree, tf_node, root, 0, visited.copy())
        return
    
    if frame in visited:
        print("  " * indent + f"├─ {frame} [LOOP]")
        return
    
    visited.add(frame)
    
    # Get frame info
    parent = tree[frame]['parent']
    info = tf_node.get_frame_info(frame, parent)
    
    # Print current frame with vertical line for all frames
    prefix = "├─ "
    line = "  " * indent + f"{prefix}{frame}"
    if info:
        line += f" [{info}]"
    print(line)
    
    # Print children
    for child in sorted(tree[frame]['children']):
        print_tree(tree, tf_node, child, indent + 1, visited.copy())


def main():
    rclpy.init()
    
    try:
        node = TFTreeNode()
        print("Collecting TF data...")
        
        tree = node.get_tf_tree()
        
        if tree:
            print_tree(tree, node)
        else:
            print("No TF frames found!")
            print("Make sure TF publishers are running.")
    
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()