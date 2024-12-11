import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import json
import math


class YoloDetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_detection_node')

        # Initialize the CvBridge to convert ROS Image messages to OpenCV images
        self.bridge = CvBridge()

        # Subscribe to the compressed image topic
        self.image_sub = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',  # Adjust topic name if necessary
            self.image_callback,
            5
        )

        # Subscribe to the robot's odometry topic
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',  # Default odometry topic for TurtleBot3
            self.odom_callback,
            10
        )

        # Load the YOLOv8 model (use 'yolov8n.pt' for nano, update if using custom model)
        self.model = YOLO('yolov8n.pt')  # Lightweight YOLOv8 nano model

        # Define the classes of interest
        self.classes_of_interest = ['car', 'chair', 'cat', 'dog', 'apple', 'stop sign']

        # Initialize robot pose
        self.current_pose = None

        # Initialize the map to store detected object locations
        self.object_locations = []

        # Publisher for RViz markers
        self.marker_pub = self.create_publisher(MarkerArray, '/detected_objects_markers', 10)
        self.marker_array = MarkerArray()

        # Define the bounding box radius (in meters)
        self.bounding_box_radius = 1.5

        self.get_logger().info("YOLO Detection Node has been started.")

    def odom_callback(self, msg):
        # Update the robot's current pose using odometry
        self.current_pose = msg.pose.pose

    def image_callback(self, msg):
        if self.current_pose is None:
            self.get_logger().warn("No robot pose available yet.")
            return

        try:
            # Convert the CompressedImage message to a CV2 image
            img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Resize the image to a smaller size (e.g., 320x240) for faster processing
            img_resized = cv2.resize(img, (320, 240))

            # Perform YOLO detection
            results = self.model(img_resized)

            # Access the first detection result
            detection_result = results[0]

            # Confidence threshold for detections
            confidence_threshold = 0.72  # 69% confidence threshold

            # Filter detections by classes of interest and confidence threshold
            if detection_result.boxes:
                for box in detection_result.boxes:
                    class_id = int(box.cls)
                    class_name = self.model.names[class_id]  # Map class ID to name
                    confidence = box.conf  # Detection confidence score

                    if confidence >= confidence_threshold and class_name in self.classes_of_interest:
                        # Save the object's location only if it's distinct
                        if self.is_new_object(class_name):
                            self.save_object_location(class_name)

        except Exception as e:
            self.get_logger().error(f"Error in image callback: {e}")


    def is_new_object(self, class_name):
        """Check if the detected object is distinct within the bounding box."""
        for obj in self.object_locations:
            # Calculate the distance between the current and previous detections
            dx = self.current_pose.position.x - obj['position']['x']
            dy = self.current_pose.position.y - obj['position']['y']
            distance = math.sqrt(dx**2 + dy**2)

            # Check if the object is within the bounding box and has the same class name
            if distance < self.bounding_box_radius and obj['object'] == class_name:
                return False

        return True

    def save_object_location(self, class_name):
        # Save the detected object's name and the robot's current pose
        object_info = {
            'object': class_name,
            'position': {
                'x': self.current_pose.position.x,
                'y': self.current_pose.position.y,
                'z': self.current_pose.position.z
            },
            'orientation': {
                'x': self.current_pose.orientation.x,
                'y': self.current_pose.orientation.y,
                'z': self.current_pose.orientation.z,
                'w': self.current_pose.orientation.w
            }
        }
        self.object_locations.append(object_info)

        # Optionally save to a file in real-time
        with open('detected_objects.json', 'w') as f:
            json.dump(self.object_locations, f, indent=4)

        # Generate and publish a marker for this object
        marker_id = len(self.object_locations)  # Use the number of objects as a unique ID
        marker = self.add_marker(object_info, marker_id)
        self.marker_pub.publish(self.marker_array)

        self.get_logger().info(f"Saved location for {class_name}: {object_info}")

    def add_marker(self, object_info, marker_id):
        marker = Marker()
        marker.header.frame_id = "map"  # Adjust to your map frame if necessary
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "detected_objects"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING  # Display text facing the RViz camera
        marker.action = Marker.ADD

        # Set the position of the marker (e.g., slightly above the detected object)
        marker.pose.position.x = object_info['position']['x']
        marker.pose.position.y = object_info['position']['y']
        marker.pose.position.z = object_info['position']['z'] + 0.5  # Slightly above the ground

        # Orientation is not required for TEXT_VIEW_FACING, but can still be set
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # Set the text content to the object's name
        marker.text = object_info['object']

        # Set the scale (affects text size)
        marker.scale.z = 0.2  # The height of the text in meters

        # Set the color of the text
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Fully opaque

        # Add the marker to the array
        self.marker_array.markers.append(marker)
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user.")
    finally:
        # Save all detected object locations to a file before shutting down
        with open('detected_objects_final.json', 'w') as f:
            json.dump(node.object_locations, f, indent=4)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
