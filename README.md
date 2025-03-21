# ROS2 Workspace for Object Detection and Sensor Fusion

This workspace contains several ROS2 packages for object detection, sensor fusion, and related functionalities. The packages are designed to work together to provide a complete solution for autonomous vehicle perception.

## Packages Overview

### 1. cone_detection
- Purpose: Detects cones from LiDAR point cloud data
- Features:
  - Point cloud processing using PCL
  - Cone detection and classification
  - Visualization of detected cones
  - Custom interface integration

### 2. custom_interface
- Purpose: Provides custom message and service interfaces
- Features:
  - Custom message definitions
  - Service interface definitions
  - Shared data structures for inter-package communication

### 3. hungarian_association
- Purpose: Implements Hungarian algorithm for data association
- Features:
  - YOLO-LiDAR fusion using Hungarian algorithm
  - Object tracking and association
  - Python-based implementation with numpy and scipy

### 4. yolo_ros
- Purpose: YOLO object detection integration for ROS2
- Features:
  - Multiple YOLO version support (v5-v12)
  - Real-time object detection
  - Debug and visualization nodes
  - 3D detection capabilities

### 5. cone_projection
- Purpose: Projects detected cones onto different coordinate frames
- Features:
  - Cone position projection
  - Visualization of projected cones
  - YAML configuration support
  - Sorted cone visualization

### 6. ros2_camera_lidar_fusion
- Purpose: Camera-LiDAR sensor fusion and calibration
- Features:
  - Intrinsic and extrinsic calibration
  - LiDAR point projection onto camera images
  - Data recording and synchronization
  - Point correspondence selection
  - Visualization tools

### 7. usb_cam
- Purpose: USB camera interface for ROS2
- Features:
  - USB camera driver
  - Image capture and publishing
  - Configuration support
  - Testing utilities

## Dependencies

- ROS2 Humble
- Python 3.10+
- PCL (Point Cloud Library)
- OpenCV
- NumPy
- SciPy
- YOLO


## Usage

Each package can be launched individually or as part of a larger system. For example:

```bash
# Launch cone detection
ros2 launch cone_detection cone_detection_launch.py

# Launch YOLO detection
ros2 run yolo_ros yolo_debug_node

# Launch camera-LiDAR fusion
ros2 run ros2_camera_lidar_fusion project_boxes_cones_points 

# Launch hungarian association
ros2 run hungarian_association hungarian_association_node
```

## License

This workspace contains packages with different licenses:
- yolo_ros: GPL-3
- ros2_camera_lidar_fusion: MIT
- Other packages: Apache-2.0

Please refer to individual package licenses for more details.
