# IR-turtlebot3-maze
The package uses a frontier and yolo to map a maze and record the images in the maze using Yolo

## Launch Commands
```
ros2 launch turtlebot3_bringup robot.launch.py
ros2 run image_publisher image_publisher
ros2 run yolo_detection yolo_detection_node 
ros2 launch autonomous_exploration autonomous_exploration.launch.py 
```

To start the exploration run this command:
```
ros2 run autonomous_exploration control
```
