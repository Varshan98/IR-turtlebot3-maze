import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header
#from rclpy.qos import QoSDurabilityPolicy
#from rclpy.qos import QoSHistoryPolicy
#from rclpy.qos import QoSProfile
from cv_bridge import CvBridge
import cv2

class CompressedImagePublisher(Node):
    def __init__(self):
        super().__init__('compressed_image_publisher')
        #qos_profile = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST,durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,depth=100)
        self.publisher_ = self.create_publisher(CompressedImage, '/camera/image_raw/compressed', 100)
        self.timer = self.create_timer(0.05, self.timer_callback)  # Capture image every 0.05s (20Hz)
        self.bridge = CvBridge()

        # Try to open the camera with the Video4Linux2 (V4L2) backend
        self.camera = cv2.VideoCapture(0, cv2.CAP_V4L2)
        if not self.camera.isOpened():
            self.get_logger().error("Could not open camera with V4L2 backend. Trying default backend.")
            # Fall back to default if V4L2 fails
            self.camera = cv2.VideoCapture(0)
            if not self.camera.isOpened():
                self.get_logger().error("Could not open camera. Please check the device.")
                rclpy.shutdown()

        # Set camera properties for optimization
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.camera.set(cv2.CAP_PROP_FPS, 30)
        self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 2)

    def timer_callback(self):
        ret, frame = self.camera.read()
        if ret:
            # Convert frame to a compressed image message
            compressed_image_msg = self.bridge.cv2_to_compressed_imgmsg(frame, dst_format="jpg")

            # Add timestamp to the message header
            compressed_image_msg.header = Header()
            compressed_image_msg.header.stamp = self.get_clock().now().to_msg()
            compressed_image_msg.header.frame_id = "camera"  # Optional, you can specify the frame ID

            # Publish the compressed image
            self.publisher_.publish(compressed_image_msg)
        else:
            self.get_logger().error("Failed to read frame from camera.")

    def destroy_node(self):
        if self.camera.isOpened():
            self.camera.release()
            self.get_logger().info("Camera released.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CompressedImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

