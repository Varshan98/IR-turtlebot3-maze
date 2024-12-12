import rclpy
import rclpy.clock
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA
from std_msgs.msg import String
from robot_initializer_msgs.msg import DetectedObject
from nav_msgs.msg import OccupancyGrid
from PIL import Image as PILImage
from cv_bridge import CvBridge
from transformers import pipeline
import numpy as np

class ProcessedObject:
    def __init__(self):
        self.name = ''
        self.x = 0.0            
        self.y = 0.0
        self.mode = 0
        self.count = 0

class ObjectProcesser(Node):
    def __init__(self,detection_mode=0):
        super().__init__('object_procceser')
        self.camera_horizontal_fov = 1.02974
        # self.image_width = 1920
        self.image_width = 320
        self.detection_mode = detection_mode
        self.cleanup_threshold = 4
        self.object_seperation_threshold = 0.2
        self.object_distance_weight = 0.4
        self.min_detection_threshold = 2
        self.detected_objects = []
        self.disable_flag = False
        self.stop_sign_distance = 0.3

        self.reentrant_callback_group = ReentrantCallbackGroup()

        qos_profile_detection = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_ALL,
            depth=10
        )
        
        self.detected_object_subscriber = self.create_subscription(
            DetectedObject,
            'detected_objects',
            self.extract_object_from_detections,
            qos_profile_detection,
            callback_group=self.reentrant_callback_group)
        
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            'map',
            self.cleanup_objects,
            10,
            callback_group=self.reentrant_callback_group)

        self.marker_publisher = self.create_publisher(MarkerArray, 'visualization_marker', 20)
        marker_timer_period = 0.5
        self.marker_timer = self.create_timer(marker_timer_period, self.marker_publisher_callback)

        self.robot_state = self.create_publisher(String,'robot_state_topic',10)
        robot_state_timer_period = 0.5
        self.robot_state_timer = self.create_timer(robot_state_timer_period,self.set_robot_state_time)

        # self.br = CvBridge()
        # self.pipe = pipeline(task="depth-estimation", model="depth-anything/Depth-Anything-V2-Small-hf")

    def extract_object_from_detections(self,unprocessed_object):
        if not self.disable_flag:
            self.get_logger().info(f"RECIEVED DATA...")
            closest_robot_position_records = unprocessed_object.robot_position
            closest_lidar_records = unprocessed_object.lidar_record
            closest_image_records = unprocessed_object.image_record
            cv_record = unprocessed_object.cv_record

            if len(closest_robot_position_records) == 0:
                self.get_logger().info("Nothing in Robot Position Records")
                return
            if len(closest_lidar_records) == 0:
                self.get_logger().info("Nothing in Lidar Records")
                return
            # if len(closest_image_records) == 0:
            #     self.get_logger().info("Nothing in Image Records")
            #     return
            
            closest_robot_position_record = closest_robot_position_records[0]
            # closest_image_record = closest_image_records[0]

            for detection_mode in self.detection_mode:
                for detection in cv_record.detections:
                    object_angle, object_distance = None, None
                    if detection_mode == 0:
                        object_angle, object_distance = self.bounding_box_detection_mode(detection,closest_lidar_records)
                        timestamp_difference = abs((unprocessed_object.timestamp_sec + unprocessed_object.timestamp_nanosec * pow(10,-9)) - 0.5*((closest_lidar_records[0].timestamp_sec + closest_lidar_records[0].timestamp_nanosec  * pow(10,-9)) + (closest_lidar_records[1].timestamp_sec + closest_lidar_records[1].timestamp_nanosec  * pow(10,-9))))
                    elif detection_mode == 1:
                        object_angle, object_distance = self.center_detection_mode(detection,closest_lidar_records)
                        timestamp_difference = abs((unprocessed_object.timestamp_sec + unprocessed_object.timestamp_nanosec * pow(10,-9)) - (closest_lidar_records[0].timestamp_sec + closest_lidar_records[0].timestamp_nanosec  * pow(10,-9)))
                    elif detection_mode == 2:
                        object_angle, object_distance = self.ml_depth_detection_mode(detection,closest_image_record)
                        timestamp_difference = abs((unprocessed_object.timestamp_sec + unprocessed_object.timestamp_nanosec * pow(10,-9)) - (closest_image_record.timestamp_sec + closest_image_record.timestamp_nanosec  * pow(10,-9)))
                    
                    # object_angle, object_distance = self.center_detection_mode(detection,closest_lidar_records)
                    # timestamp_difference = abs((unprocessed_object.timestamp_sec + unprocessed_object.timestamp_nanosec * pow(10,-9)) - (closest_lidar_records[0].timestamp_sec + closest_lidar_records[0].timestamp_nanosec  * pow(10,-9)))
                    # print("Timestamp Difference Lidar - Image",timestamp_difference / pow(10,9))
                    # timestamp_difference_R = abs((unprocessed_object.timestamp_sec + unprocessed_object.timestamp_nanosec * pow(10,-9)) - (closest_robot_position_record.timestamp_sec + closest_robot_position_record.timestamp_nanosec  * pow(10,-9)))
                    # print("Timestamp Difference Robot - Image",timestamp_difference_R / pow(10,9))

                    if object_angle == None or object_distance == None or np.isnan(object_angle) or np.isnan(object_distance):
                        self.get_logger().info("Object Angle or Object Distance not found")
                        continue

                    if detection.class_name == 'stop sign' and object_distance < self.stop_sign_distance:
                        self.disable_flag = True
                    
                    object_x_position = closest_robot_position_record.x + object_distance * np.cos(object_angle + closest_robot_position_record.yaw) * np.cos(closest_robot_position_record.pitch)
                    object_y_position = closest_robot_position_record.y + object_distance * np.sin(object_angle + closest_robot_position_record.yaw) * np.cos(closest_robot_position_record.roll) + object_distance * np.cos(object_angle + closest_robot_position_record.yaw) * np.sin(closest_robot_position_record.pitch) * np.sin(closest_robot_position_record.roll)

                    duplicate_flag = False
                    for detected_object in self.detected_objects:
                        if np.linalg.norm(np.array([object_x_position,object_y_position])-np.array([detected_object.x,detected_object.y])) < self.object_seperation_threshold and \
                                detected_object.name == detection.class_name and detected_object.mode == detection_mode:
                                alpha = self.object_distance_weight if object_distance > self.object_distance_weight else object_distance
                                detected_object.x = alpha * detected_object.x + (1-alpha) * object_x_position
                                detected_object.y = alpha * detected_object.y + (1-alpha) * object_y_position
                                detected_object.count += 1
                                duplicate_flag = True
                                self.get_logger().info("Duplicate Object found. Weighted Averaging")
                                break
                    # for detected_object in self.detected_objects:
                    #     if np.linalg.norm(np.array([object_x_position,object_y_position])-np.array([detected_object.x,detected_object.y])) < self.object_seperation_threshold and \
                    #             detected_object.mode == detection_mode:
                    #             alpha = self.object_distance_weight if object_distance > self.object_distance_weight else object_distance
                    #             detected_object.x = alpha * detected_object.x + (1-alpha) * object_x_position
                    #             detected_object.y = alpha * detected_object.y + (1-alpha) * object_y_position
                    #             detected_object.count += 1
                    #             duplicate_flag = True
                    #             self.get_logger().info("Duplicate Object found. Weighted Averaging")
                    #             break

                    if duplicate_flag == False:
                        new_object = ProcessedObject()
                        new_object.name = detection.class_name
                        new_object.x = object_x_position            
                        new_object.y = object_y_position
                        new_object.mode = detection_mode
                        new_object.count = 1
                        self.detected_objects.append(new_object)
                        self.get_logger().info(f"New object [{detection.class_name}] discovered at ({object_x_position},{object_y_position}) by Detection mode {detection_mode}")
                        self.get_logger().info(f"Timestamp Difference: {timestamp_difference}")
        else:
            self.get_logger().info("Stopped.")
            self.get_logger().info("Found Objects:")
            objects, counts = np.unique([i.name for i in self.detected_objects], return_counts=True)
            for i in range(len(objects)):
                self.get_logger().info(f"Found {counts[i]} {objects[i]}")

    
    def bounding_box_detection_mode(self,detection,closest_lidar_records):
        max_angle_in_camera_frame = - (self.camera_horizontal_fov * (detection.bbox.center.position.x - detection.bbox.size.x/2)/self.image_width - 0.5 * self.camera_horizontal_fov)
        if max_angle_in_camera_frame < 0: max_angle_in_camera_frame += 2*np.pi
        if max_angle_in_camera_frame > 2 * np.pi: max_angle_in_camera_frame -= 2 * np.pi
        frame_max_index = int(len(closest_lidar_records[0].ranges) * max_angle_in_camera_frame/(2*np.pi))
        
        min_angle_in_camera_frame = - (self.camera_horizontal_fov * (detection.bbox.center.position.x + detection.bbox.size.x/2)/self.image_width - 0.5 * self.camera_horizontal_fov)
        if min_angle_in_camera_frame < 0: min_angle_in_camera_frame += 2*np.pi
        if min_angle_in_camera_frame > 2 * np.pi: min_angle_in_camera_frame -= 2*np.pi
        frame_min_index = int(len(closest_lidar_records[0].ranges) * min_angle_in_camera_frame/(2*np.pi))
        
        frame_slice_2 = []
        if frame_max_index > frame_min_index:
            frame_slice = closest_lidar_records[0].ranges[frame_min_index:frame_max_index+1:]
            frame_slice = frame_slice[2::] + frame_slice[:len(frame_slice)-2:]
            if len(closest_lidar_records) > 1:
                frame_slice_2 = closest_lidar_records[1].ranges[frame_min_index:frame_max_index+1:]
        else:
            frame_slice = closest_lidar_records[0].ranges[frame_min_index::]+closest_lidar_records[0].ranges[:frame_max_index+1:]
            frame_slice = frame_slice[2::] + frame_slice[:len(frame_slice)-2:]
            if len(closest_lidar_records) > 1:
                frame_slice_2 = closest_lidar_records[1].ranges[frame_min_index::]+closest_lidar_records[1].ranges[:frame_max_index+1:]
        
        frame_slice += frame_slice_2
        
        condition = [np.isfinite(i) for i in frame_slice]
        non_inf_lidar_values = np.array(frame_slice)[condition]

        if len(non_inf_lidar_values) == 0:
            self.get_logger().info(f"Unable to get object distance. :(")
            return None, None
        object_distance = np.mean(non_inf_lidar_values)

        sin_sum = np.sin(min_angle_in_camera_frame) + np.sin(max_angle_in_camera_frame)
        cos_sum = np.cos(min_angle_in_camera_frame) + np.cos(max_angle_in_camera_frame)
        object_angle = np.arctan2(sin_sum, cos_sum)
        return object_angle, object_distance
    
    def center_detection_mode(self,detection,closest_lidar_records):
        angle_in_camera_frame = - (self.camera_horizontal_fov * (detection.bbox.center.position.x)/self.image_width - 0.5 * self.camera_horizontal_fov)
        if angle_in_camera_frame < 0: angle_in_camera_frame += 2*np.pi
        if angle_in_camera_frame > 2 * np.pi: angle_in_camera_frame -= 2 * np.pi
        frame_index = int(len(closest_lidar_records[0].ranges) * angle_in_camera_frame/(2*np.pi))
        frame_slice = closest_lidar_records[0].ranges[frame_index]
        if frame_slice == np.inf:
            self.get_logger().info(f"Unable to get object distance. :(")
            return None, None
        return angle_in_camera_frame, frame_slice
    
    def ml_depth_detection_mode(self,detection,closest_image_record):
        angle_in_camera_frame = - (self.camera_horizontal_fov * (detection.bbox.center.position.x)/self.image_width - 0.5 * self.camera_horizontal_fov)
        current_frame = self.br.imgmsg_to_cv2(closest_image_record.raw_image)
        pil_image = PILImage.fromarray(current_frame) 
        depth_arr = self.pipe(pil_image)['predicted_depth']
        depth = np.float(depth_arr[int(detection.bbox.center.position.y),int(detection.bbox.center.position.x)])
        return angle_in_camera_frame, depth

    def marker_publisher_callback(self):
        marker_array = MarkerArray()
        for index,detected_object in enumerate(self.detected_objects):
            # if detected_object.count < self.min_detection_threshold:
            #     continue
            new_marker = Marker()
            new_marker.header.frame_id = "map"
            new_marker.header.stamp = self.get_clock().now().to_msg()
            new_marker.ns = "Unknown Mode"
            new_marker.color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0)
            if detected_object.mode == 0:
                new_marker.ns = "Mean Boundary Box (Red)"
                new_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
            elif detected_object.mode == 1:
                new_marker.ns = "Center Boundary Box (Green)"
                new_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
            elif detected_object.mode == 2:
                new_marker.ns = "ML Depth (Blue)"
                new_marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
            new_marker.id = index
            new_marker.type = Marker.TEXT_VIEW_FACING
            new_marker.action = Marker.ADD
            new_marker.pose.position.x = detected_object.x
            new_marker.pose.position.y = detected_object.y
            new_marker.pose.position.z = 0.0
            new_marker.scale.x = 0.4
            new_marker.scale.y = 0.4
            new_marker.scale.z = 0.4
            new_marker.text = detected_object.name
            marker_array.markers.append(new_marker)
        self.marker_publisher.publish(marker_array)

    def cleanup_objects(self,msg):
        map_resolution = msg.info.resolution
        map_origin_x = msg.info.origin.position.x
        map_origin_y = msg.info.origin.position.y
        map_width = msg.info.width
        map_height = msg.info.height
        map_data = np.array(msg.data).reshape(map_height,map_width)

        if map_origin_x == None or map_origin_y == None:
            print("None found in map origin")

        for processed_object in self.detected_objects[::-1]:
            object_map_x_index = int((processed_object.x - map_origin_x) / map_resolution)
            object_map_y_index = int((processed_object.y - map_origin_y) / map_resolution)
            x_indexes = [i for i in range(object_map_x_index-self.cleanup_threshold,object_map_x_index+self.cleanup_threshold,1) if 0<=i<map_width]
            y_indexes = [i for i in range(object_map_y_index-self.cleanup_threshold,object_map_y_index+self.cleanup_threshold,1) if 0<=i<map_height]
            if len([map_data[row,col] for col in x_indexes for row in y_indexes if map_data[row,col]==100]) == 0:
                self.detected_objects.remove(processed_object)
                self.get_logger().info(f"Removing Invalid Object")

        # break_flag = False
        # for detected_object_1 in self.detected_objects:
        #     for detected_object_2 in self.detected_objects:
        #         abs_dist = np.sqrt(pow(detected_object_1.x - detected_object_2.x,2) + pow(detected_object_1.y - detected_object_2.y,2))
        #         if abs_dist < self.object_seperation_threshold and detected_object_1.mode == detected_object_2.mode:
        #             if detected_object_1.count> detected_object_2.count:
        #                 detected_object_1.x = ((detected_object_1.x/detected_object_1.count) + (detected_object_2.x/detected_object_2.count))*(detected_object_1.count + detected_object_2.count)
        #                 detected_object_1.y = ((detected_object_1.y/detected_object_1.count) + (detected_object_2.y/detected_object_2.count))*(detected_object_1.count + detected_object_2.count)
        #                 detected_object_1.count = detected_object_1.count + detected_object_2.count
        #                 self.detected_objects.remove(detected_object_2)
        #             else:
        #                 detected_object_2.x = ((detected_object_1.x/detected_object_1.count) + (detected_object_2.x/detected_object_2.count))*(detected_object_1.count + detected_object_2.count)
        #                 detected_object_2.y = ((detected_object_1.y/detected_object_1.count) + (detected_object_2.y/detected_object_2.count))*(detected_object_1.count + detected_object_2.count)
        #                 detected_object_2.count = detected_object_1.count + detected_object_2.count
        #                 self.detected_objects.remove(detected_object_1)
        #             self.get_logger().info(f"Removing Close Object: {abs_dist}")
        #             break_flag = True
        #             break
        #     if break_flag:
        #         break

    def set_robot_state_time(self):
        new_state = String()
        if self.disable_flag:
            new_state.data= "shutdown"
        else:
            new_state.data = "running"
        self.robot_state.publish(new_state)

def main(args=None):
    rclpy.init(args=args)
    #Mean Bounding Box - 0, Center Bounding Box - 1, ML Depth Estimate - 2
    detection_mode = [1]
    object_procceser = ObjectProcesser(detection_mode)
    executor = MultiThreadedExecutor()
    executor.add_node(object_procceser)
    try:
        executor.spin()
    except KeyboardInterrupt:
        object_procceser.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
