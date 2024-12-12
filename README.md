# Object Position Detection on Map

## Overview
This Ros2 project aims to determine the position of an object on a map using LiDAR or camera images. It provides a way to track objects' positions visualize them as markers on rviz2.

## Features
- **Object Position Tracking**: Track the position of objects on a map.
- **Multiple Input Methods**: Support for LiDAR or camera.
- **Map Visualization**: Display object positions on a map on rviz2.

## Table of Contents
- [Installation](#installation)
- [Usage](#usage)

## Installation

### Prerequisites
- Python 3.8+ (or specify other required languages)
- Required libraries:
  - numpy>=1.23.0
  - opencv-python>=4.8.1.78
  - typing-extensions>=4.4.0
  - ultralytics==8.3.26
  - super-gradients==3.7.1
  - lap==0.4.0
  - setuptools==58.2.0
  - transformers==4.47.0
  - sphinx==8.0.0
  - sphinx-rtd-theme==3.0.0
  
### Steps
1. Clone the repository into your workspace/src/:
   ```bash
   git clone https://github.com/yourusername/object-position-detection.git

## Usage

### Steps
Follow these steps to set up and run the project:
1. **Build the package**:
   To build the package, navigate to your ROS 2 workspace and run:
   ```bash
   colcon build
   ```
2. **Source the workspace**:
   After building, source the workspace to set up the environment:
   ```bash
   source install/setup.bash
   ```
3. **Launch the robot initializer**:
   Start the system by launching the initializer:
   ```bash
   ros2 launch robot_initializer robot_initializer.launch.py
   ```
4. **Run the object processing node**:
    Process detected objects using the following command:
    ```bash
    ros2 run robot_initializer detected_object_processor
    ```

### Depth Detection Methods
There are three methods available to determine object depth:
1. **Using the entire bounding box**:
  This method determines the depth by evaluating the full range of the bounding box using LiDAR values.
2. **Using the center of the bounding box (default)**:
  This method determines depth by using the center point of the bounding box and its corresponding LiDAR value.
3. **Using the Depth-Anything-V2 model**:
  This method leverages the Depth-Anything-V2 model to predict depth.

To choose a different method, modify the detection_mode parameter in the detected_object_processor.py file.
Set the parameter to one of the following options:

&nbsp; &nbsp; \[0] - Using the entire bounding box \
&nbsp; &nbsp; \[1] (default) - Using the center of the bounding box \
&nbsp; &nbsp; \[2] - Using the Depth-Anything-V2 model

Multiple methods can be used together, e.g. [0,1,2]

## Acknowledgments
This project uses the following models and tools:
1. **yolo_ros**:
  Used for object detection in this project. ROS 2 wrap for YOLO models from Ultralytics to perform object detection and tracking, instance segmentation, human pose estimation and Oriented Bounding Box (OBB). [YOLO ROS GitHub Repository](https://github.com/mgonzs13/yolo_ros)
2. **Depth-Anything-V2:**:
  A model for depth prediction used in this project. [Depth-Anything V2 GitHub Repository](https://github.com/DepthAnything/Depth-Anything-V2)

We also acknowledge the various open-source libraries and contributors that helped make this project possible, including ROS 2, RViz, and the many sensor drivers and packages used for integration.









