# IR-turtlebot3-maze
The package uses a frontier and yolo to map a maze and record the images in the maze using Yolo.

## Requirements
- ROS2 Humble
- Slam Toolbox
- Turtlebot3 Package

## Launch Commands

To reduce the computation on the turtlebot, run the only the following in turtlebot:
```
ros2 launch turtlebot3_bringup robot.launch.py
ros2 run image_publisher image_publisher
```
The launch files and image detection are run in external PC using the following code:
```
ros2 run yolo_detection yolo_detection_node 
ros2 launch autonomous_exploration autonomous_exploration.launch.py
```

To start the exploration run this command in turtlebot (running in turtlebot to prevent network lag):
```
ros2 run autonomous_exploration control
```
## Robot Modification

We were using lattepanda insead of raspi, therefore an external powerbank was used. To have better CG we modified the wheel mount directly under the powerbank.
![Alt Text](/photo_2024-12-11_16-36-39.jpg)

## Acknowledgments
This project uses the following models and tools:
