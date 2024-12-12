import rclpy
import rclpy.clock
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from yolo_msgs.msg import DetectionArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from robot_initializer_msgs.msg import DetectedObject, ImageRecord, LidarRecord, RobotPosition
from sensor_msgs.msg import CompressedImage
import numpy as np

class FixedSizeArray(list):
    def __init__(self,max_size):
        self.max_size = max_size
        super().__init__()
        
    def append(self,item):
        if len(self)>self.max_size:
            self.pop(0)
        super(FixedSizeArray, self).append(item)
        
class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.reentrant_callback_group = ReentrantCallbackGroup()

        self.robot_position_record = FixedSizeArray(max_size=1000)
        self.lidar_records = FixedSizeArray(max_size=1000)
        self.image_records = FixedSizeArray(max_size=1000)

        qos_profile_scan = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50
        )

        qos_profile_detection = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_ALL,
            depth=10
        )

        # qos_profile_odom = QoSProfile(
        #     reliability=ReliabilityPolicy.RELIABLE,
        #     durability=DurabilityPolicy.VOLATILE,
        #     history=HistoryPolicy.KEEP_LAST,
        #     depth=1
        # )

        qos_profile_image = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            # history=HistoryPolicy.KEEP_ALL,
            # durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=100, #1
        )

        # self.lidar_scan_subscriber = self.create_subscription(
        #     LaserScan,
        #     'scan',
        #     self.lidar_scan_callback,
        #     qos_profile_scan,callback_group=self.reentrant_callback_group)

        self.lidar_scan_subscriber = self.create_subscription(
            LaserScan,
            'scan',
            self.lidar_scan_callback,
            qos_profile_scan)

        self.robot_position_subscriber = self.create_subscription(
            Odometry,
            'odom',
            self.robot_position_callback,
            10,callback_group=self.reentrant_callback_group)
        
        self.detected_object_subscriber = self.create_subscription(
            DetectionArray,
            'yolo/detections',
            self.detected_object_callback,
            qos_profile_detection,callback_group=self.reentrant_callback_group)

        self.image_subscription = self.create_subscription(
            CompressedImage,
            'camera/image_raw/compressed',
            self.raw_image_callback,
            qos_profile_image,callback_group=self.reentrant_callback_group)

        # self.detected_object_publisher = self.create_publisher(DetectedObject,"detected_objects",10)
        self.detected_object_publisher = self.create_publisher(DetectedObject,"detected_objects",qos_profile_detection)
        self.get_logger().info("INITIALIZED.")
        
    def lidar_scan_callback(self,msg):
        new_lidar_record = LidarRecord()
        new_lidar_record.timestamp_sec = msg.header.stamp.sec
        new_lidar_record.timestamp_nanosec = msg.header.stamp.nanosec
        new_lidar_record.ranges = msg.ranges
        self.lidar_records.append(new_lidar_record)

    def robot_position_callback(self,msg):
        new_robot_position = RobotPosition()
        new_robot_position.timestamp_sec = msg.header.stamp.sec
        new_robot_position.timestamp_nanosec = msg.header.stamp.nanosec
        new_robot_position.x = msg.pose.pose.position.x
        new_robot_position.y = msg.pose.pose.position.y
        new_robot_position.roll ,new_robot_position.pitch, new_robot_position.yaw = self.calculate_angle_from_quaternion(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        self.robot_position_record.append(new_robot_position)

    def calculate_angle_from_quaternion(self,x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = np.arctan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = np.arcsin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = np.arctan2(t3, t4)
        return roll_x,pitch_y,yaw_z
    
    def detected_object_callback(self,msg):
        if len(msg.detections) == 0:
            return
        # print("CV Timestamp: ",msg.header.stamp.sec)
        new_unprocessed_object = DetectedObject()
        new_unprocessed_object.timestamp_sec = msg.header.stamp.sec
        new_unprocessed_object.timestamp_nanosec = msg.header.stamp.nanosec
        new_unprocessed_object.cv_record = msg
        new_unprocessed_object.robot_position = self.get_closest_time_records(new_unprocessed_object.timestamp_sec,new_unprocessed_object.timestamp_nanosec,self.robot_position_record)
        new_unprocessed_object.lidar_record = self.get_closest_time_records(new_unprocessed_object.timestamp_sec,new_unprocessed_object.timestamp_nanosec,self.lidar_records)
        new_unprocessed_object.image_record = self.get_closest_time_records(new_unprocessed_object.timestamp_sec,new_unprocessed_object.timestamp_nanosec,self.image_records)
        self.detected_object_publisher.publish(new_unprocessed_object)
                
    def get_closest_time_records(self,timestamp_sec,timestamp_nanosec,records):
        sorted_records = sorted(records,key=lambda x : x.timestamp_sec + x.timestamp_nanosec*pow(10,-9))
        for index,data in enumerate(sorted_records):
            if data.timestamp_sec >= timestamp_sec and data.timestamp_nanosec >= timestamp_nanosec:
                # return records[index-1:0:] + records[0:index+1:] if index==0 else records[index-1:index+1:]
                if index==0:
                    return records[0] + records[0]
                else: 
                    timetaken_1 = records[index-1].timestamp_sec + records[index-1].timestamp_nanosec * pow(10,-9)
                    timetaken_2 = records[index].timestamp_sec + records[index].timestamp_nanosec * pow(10,-9)
                    if abs(timestamp_sec - timetaken_1) <= abs(timestamp_sec - timetaken_2):
                        return records[index-1:index+1:]
                    else:
                        return records[index:index-2:-1]
        return records[len(records)-2::]
    
    def raw_image_callback(self,msg):
        new_image_record = ImageRecord()
        new_image_record.timestamp_sec = msg.header.stamp.sec
        new_image_record.timestamp_nanosec = msg.header.stamp.nanosec
        new_image_record.raw_image = msg
        self.image_records.append(new_image_record)

def main(args=None):
    # rclpy.init(args=args)
    # minimal_subscriber = MinimalSubscriber()
    # rclpy.spin(minimal_subscriber)
    # minimal_subscriber.destroy_node()
    # rclpy.shutdown()

    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    executor = MultiThreadedExecutor()
    executor.add_node(minimal_subscriber)
    try:
        executor.spin()
    except KeyboardInterrupt:
        minimal_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()