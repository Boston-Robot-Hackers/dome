# OAK-D Lite vs. Orbbec Gemini 330
A concise comparison for SLAM, navigation, and AI perception.

## 1. Overview

| Feature | OAK-D Lite | Orbbec Gemini 330 |
|--------|------------|-------------------|
| Sensor type | Stereo depth + RGB | Structured-light / ToF + RGB |
| On-device AI | Yes (Myriad-X VPU) | No |
| Depth range | ~0.3–3 m | ~0.2–5 m |
| Depth stability | Texture-dependent | High (active depth) |
| Best use | AI + spatial AI | SLAM + navigation |

## 2. OAK-D Lite

### Pros
- Runs YOLO and other models on-device.
- Produces 2D and 3D detections without loading the Pi.
- Simple hardware (single USB-C).
- Flexible DepthAI pipelines.

### Cons
- Stereo depth is noisy and texture-dependent.
- Less stable for SLAM and mapping.
- Requires more tuning for consistent depth.

## 3. Orbbec Gemini 330

### Pros
- Active depth sensing with low noise.
- Excellent for SLAM, mapping, and navigation.
- Clean, consistent point clouds.
- Strong ROS2 driver with depth registration.

### Cons
- No on-device AI.
- Requires external compute for YOLO.
- No built-in spatial AI (3D fusion must be done manually).

## 4. ROS2 Support and Configuration

### OAK-D Lite
- Easy bring-up with `depthai_ros_driver`.
- Publishes RGB, depth, point cloud, and neural detections.
- Depth tuning is more environment-sensitive.

### Orbbec Gemini 330
- Clean ROS2 topics (RGB, depth, point cloud, camera info).
- Depth registration is a simple launch option.
- Works smoothly with SLAM Toolbox, RTAB-Map, and Nav2.

## 5. Recommended Use

- Choose **OAK-D Lite** for AI-centric robots needing on-device inference.
- Choose **Orbbec Gemini 330** for SLAM, navigation, and stable geometry.
- A hybrid setup (Gemini for geometry, OAK-D for AI) provides the strongest overall stack.

