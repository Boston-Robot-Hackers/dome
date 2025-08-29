# Session 01 - TF Tree Viewer & Git Cleanup

## Start

**Read trace_tf.py and figure out how it works**

## About trace_tf.py

The `trace_tf.py` program is a ROS2 diagnostic tool that visualizes the Transform (TF) tree structure of a robot system. It shows how coordinate frames are connected in a hierarchical tree format.

**What it does:**
- Discovers all TF frames in the system
- Maps parent-child relationships between frames  
- Displays the tree structure with transform distances and timing information
- Shows warnings for old/stale transforms

**How it works:**
1. Subscribes to `/tf` and `/tf_static` topics to capture transform messages
2. Extracts frame names and parent-child relationships from transform headers
3. Builds a tree data structure showing the frame hierarchy
4. Uses the TF buffer to look up transform details (distance, age) between connected frames
5. Renders the tree with ASCII art showing the complete coordinate frame structure

This is useful for debugging robot localization, sensor mounting, and transform publication issues.

## Fixed TF Tree Visualization Script

Fixed `trace_tf.py` which had multiple issues:

1. **API Issues**: Replaced private TF2 methods (`_get_frame_list()`, `_get_parent()`) with proper public API
2. **Data Collection**: Added TF message subscribers to `/tf` and `/tf_static` topics to capture frame relationships
3. **Time Calculation**: Fixed TypeError in age calculation by properly converting timestamp types
4. **Tree Display**: Added vertical lines (`├─`) to all frames including root frames
5. **Relationship Logic**: Removed problematic orphan discovery that created circular dependencies

## Final Working Implementation

- Captures frame names and parent-child relationships directly from TF messages
- Only uses relationships observed in actual TF publications
- Properly identifies root frames (those not appearing as children)
- Displays complete TF tree with distances and timing info

## Git Configuration

- Created `.gitignore` to exclude log files (`*.log`, `log/`, `logs/`)
- Removed existing log files from git tracking using `git rm -r --cached log/`

## Result

Script now correctly displays the robot's complete TF tree structure showing all 8 frames with proper hierarchy.