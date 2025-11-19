import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
import yaml
import sys
from datetime import datetime

class TransformLister(Node):
    def __init__(self):
        super().__init__('transform_lister')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Wait 5 seconds for transforms to populate, then print and exit
        self.create_timer(5.0, self.list_transforms_once)
        
    def list_transforms_once(self):
        self.list_all_transforms()
        self.destroy_node()
        sys.exit(0)
        
    def list_all_transforms(self):
        try:
            # Get all frame IDs as YAML
            all_frames_yaml = self.tf_buffer.all_frames_as_yaml()
            
            # Parse YAML to get frame names and relationships
            frames_dict = yaml.safe_load(all_frames_yaml)
            
            # Find root frames (frames that appear as parents but don't exist as actual frames)
            all_frame_names = set(frames_dict.keys())
            all_parents = set()
            
            for frame_info in frames_dict.values():
                parent = frame_info.get('parent')
                if parent and parent != '':
                    all_parents.add(parent)
            
            # Root frames are parents that don't exist as actual frames
            root_frame_names = all_parents - all_frame_names
            
            root_frames = []
            child_frames = []
            
            # Add root frames (these won't have frame_info, so create dummy entries)
            for root_name in sorted(root_frame_names):
                root_frames.append((root_name, {'parent': '', 'most_recent_transform': 0.0, 'oldest_transform': 0.0}))
            
            # All actual frames are child frames
            for frame_id, frame_info in sorted(frames_dict.items()):
                child_frames.append((frame_id, frame_info))
            
            # Sort both lists
            root_frames.sort(key=lambda x: x[0])
            child_frames.sort(key=lambda x: x[0])
            
            # Create formatted table output
            print("\n" + "="*85)
            print("Transform Frames (Root frames first)")
            print("="*85)
            print(f"{'Frame':<23} | {'Parent':<23} | {'Recent':<12} | {'Oldest'}")
            print("-"*85)
            
            def format_time(timestamp):
                if timestamp == 0.0:
                    return "never"
                else:
                    dt = datetime.fromtimestamp(timestamp)
                    return dt.strftime("%H:%M:%S.%f")[:-3]
            
            def format_frame_name(name):
                if len(name) > 20:
                    return f"...{name[-20:]}"
                else:
                    return name
            
            # Display root frames first
            for frame_id, frame_info in root_frames:
                formatted_frame = format_frame_name(frame_id)
                parent = frame_info.get('parent', 'ROOT')
                recent = format_time(frame_info.get('most_recent_transform', 0.0))
                oldest = format_time(frame_info.get('oldest_transform', 0.0))
                print(f"{formatted_frame:<23} | {parent:<23} | {recent:<12} | {oldest}")
            
            # Then display child frames
            for frame_id, frame_info in child_frames:
                formatted_frame = format_frame_name(frame_id)
                parent = frame_info.get('parent', 'None')
                recent = format_time(frame_info.get('most_recent_transform', 0.0))
                oldest = format_time(frame_info.get('oldest_transform', 0.0))
                print(f"{formatted_frame:<23} | {parent:<23} | {recent:<12} | {oldest}")
            
            print("="*85)
            
            # Also get just the frame names as a list
            frame_names = list(frames_dict.keys())
            return frame_names
            
        except Exception as e:
            self.get_logger().error(f"Error getting transforms: {e}")
            return []

def main():
    rclpy.init()
    node = TransformLister()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()