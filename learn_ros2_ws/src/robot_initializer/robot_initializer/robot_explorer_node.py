import rclpy
import rclpy.clock
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist
import numpy as np
import random
import csv

class RobotExplorer(Node):
    def __init__(self):
        super().__init__('simple_explorer')
        self.robot_x_position = 0.0
        self.robot_y_position = 0.0
        self.robot_yaw = 0.0

        self.param_expansion_size = 3
        self.white_space_expansion_size = 5
        self.angular_velocity = 1.0
        self.linear_velocity = 0.2

        self.reentrant_callback_group = ReentrantCallbackGroup()
        
        self.robot_position_subscriber = self.create_subscription(
            Odometry,
            'odom',
            self.robot_position_callback,
            1,
            callback_group = self.reentrant_callback_group)
        
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            'map',
            self.get_new_waypoint,
            1)

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            1)
        
        self.map_subscriber
        self.robot_position_subscriber

    def robot_position_callback(self,msg):
        self.robot_x_position = msg.pose.pose.position.x
        self.robot_y_position = msg.pose.pose.position.y
        self.robot_yaw = self.calculate_yaw_from_quaternion(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)

    def calculate_yaw_from_quaternion(self,x, y, z, w):
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = np.arctan2(t3, t4) 
        if yaw_z < 0:
            yaw_z += 2 * np.pi
        return np.arctan2(t3, t4) 

    def get_new_waypoint(self,msg):
        new_cmd_vel = Twist()
        self.cmd_vel_publisher.publish(new_cmd_vel)
        
        map_resolution = msg.info.resolution
        map_origin_x = msg.info.origin.position.x
        map_origin_y = msg.info.origin.position.y
        map_width = msg.info.width
        map_height = msg.info.height
        # map_data = np.flipud(np.array(msg.data).reshape(map_height,map_width))
        map_data = np.array(msg.data).reshape(map_height,map_width)

        wall = np.where(map_data == 100)
        for i in range(-self.param_expansion_size, self.param_expansion_size+1):
            for j in range(-self.param_expansion_size, self.param_expansion_size+1):
                if i == 0 and j == 0:
                    continue
                x = wall[0]+i
                y = wall[1]+j
                x = np.clip(x, 0, map_height-1)
                y = np.clip(y, 0, map_width-1)
                map_data[x, y] = 100

        wall = np.where(map_data == 0)
        for i in range(-self.white_space_expansion_size, self.white_space_expansion_size+1):
            for j in range(-self.white_space_expansion_size, self.white_space_expansion_size+1):
                if i == 0 and j == 0:
                    continue
                x = wall[0]+i
                y = wall[1]+j
                x = np.clip(x, 0, map_height-1)
                y = np.clip(y, 0, map_width-1)
                #check not 100
                map_data[x,y] = 0 

        # with open('data_next.csv', 'w', newline='') as csvfile:
        #     writer = csv.writer(csvfile)
        #     writer.writerows(map_data)  
        
        robot_map_x_index = int((self.robot_x_position - map_origin_y) / map_resolution)
        robot_map_y_index = map_height - int((self.robot_y_position - map_origin_x) / map_resolution)
        print("Robot Position",robot_map_x_index,robot_map_y_index)

        path,grid = self.bfs(map_data,(robot_map_x_index,robot_map_y_index))
        self.draw_path(grid,[(robot_map_x_index,robot_map_y_index)] + path)
        if len(path) > 2:
            simplified_path = self.simplify_path(path[::-1])[1::]
        else:
            simplified_path = path
        print("Path",simplified_path)

        for waypoint in simplified_path[::-1]:
            waypoint_angle = np.arctan2((waypoint[1] - robot_map_y_index),(waypoint[0] - robot_map_x_index))
            waypoint_distance = np.linalg.norm(np.array([robot_map_x_index,robot_map_y_index]) - np.array([waypoint[0],waypoint[1]])) * map_resolution

            #Rotate
            print("Angle to move to",np.rad2deg(waypoint_angle))
            print("Current Yaw",np.rad2deg(self.robot_yaw))

            while abs(waypoint_angle  - self.robot_yaw) > 0.01:
                new_cmd_vel.angular.z = waypoint_angle - self.robot_yaw
                self.cmd_vel_publisher.publish(new_cmd_vel)

            new_cmd_vel.angular.z = 0.0
            self.cmd_vel_publisher.publish(new_cmd_vel)

            #Move Forward
            new_cmd_vel.linear.x = self.linear_velocity
            print("Distance to move",waypoint_distance)
            initial_point = (robot_map_x_index,robot_map_y_index)
            waypoint_distance = np.linalg.norm(np.array([robot_map_x_index,robot_map_y_index]) - np.array([waypoint[0],waypoint[1]]))
            initial_point_distance = np.linalg.norm(np.array([initial_point[0],initial_point[1]]) - np.array([int((self.robot_x_position - map_origin_y) / map_resolution),map_height - int((self.robot_y_position - map_origin_x) / map_resolution)]))
            while np.greater(waypoint_distance,initial_point_distance):
                print("Initial Dist",initial_point_distance)
                print("Waypoint Dist",waypoint_distance)
                initial_point_distance = np.linalg.norm(np.array([initial_point[0],initial_point[1]]) - np.array([int((self.robot_x_position - map_origin_y) / map_resolution),map_height - int((self.robot_y_position - map_origin_x) / map_resolution)]))
                new_cmd_vel.linear.x = self.linear_velocity * waypoint_distance/initial_point_distance
                self.cmd_vel_publisher.publish(new_cmd_vel)
            new_cmd_vel.linear.x = 0.0
            self.cmd_vel_publisher.publish(new_cmd_vel)

    def get_robot_path(self,grid,robot_location,target_position):
        return_value = []
        current_value = grid[target_position[0],target_position[1]]
        if current_value == 1:
                return_value = [(target_position[0],target_position[1])]
        
        neighbours = [(i,j) for i in range(target_position[0]-1,target_position[0]+2,1) for j in range(target_position[1]-1,target_position[1]+2,1) if (i != target_position[0] or j != target_position[1])]
        for i in neighbours:
            if grid[i[0],i[1]] == current_value-1:
                return_value = [(target_position[0],target_position[1])] + self.get_robot_path(grid,robot_location,i)
                break
        return return_value

    def bfs(self,grid,robot_location):
        grid[robot_location[0],robot_location[1]] = 1
        width = grid.shape[0]
        height = grid.shape[1]
        cutoff_flag = False
        lookup_radius = 1
        robot_path = []
        while not cutoff_flag:
            x_index = [i for i in range(robot_location[0]-lookup_radius,robot_location[0]+lookup_radius+1,1) if -1<i<width]
            y_index = [i for i in range(robot_location[1]-lookup_radius,robot_location[1]+lookup_radius+1,1) if -1<i<height]
            
            lookout_indexes = [(i,j) for i in x_index for j in y_index]
            
            cutoff_flag = True
            for i in lookout_indexes:
                if grid[i[0],i[1]] == lookup_radius - 1 and \
                ((i[0] != 0 and grid[i[0]-1,i[1]] == -1) or (i[0] != width -1 and grid[i[0]+1,i[1]] == -1) or (i[1] != 0 and grid[i[0],i[1]-1] == -1) or (i[1] != height - 1 and grid[i[0],i[1]+1] == -1)):
                    robot_path = self.get_robot_path(grid,robot_location,(i[0],i[1]))
                    cutoff_flag = True
                    break
                if grid[i[0],i[1]] == 0:
                    if i[0] != 0 and grid[i[0]-1,i[1]] == lookup_radius:
                        grid[i[0],i[1]] = lookup_radius + 1
                    elif i[0] != width -1 and grid[i[0]+1,i[1]] == lookup_radius:
                        grid[i[0],i[1]] = lookup_radius + 1
                    elif i[1] != 0 and grid[i[0],i[1]-1] == lookup_radius:
                        grid[i[0],i[1]] = lookup_radius + 1
                    elif i[1] != height - 1 and grid[i[0],i[1]+1] == lookup_radius:
                        grid[i[0],i[1]] = lookup_radius + 1
                    cutoff_flag = False
            lookup_radius+= 1  

        # with open('data_bfs.csv', 'w', newline='') as csvfile:
        #     writer = csv.writer(csvfile)
        #     writer.writerows(grid)   
        return robot_path,grid
    
    def draw_path(self,grid,path):
        for i in path:
            grid[i[0],i[1]] = -100
        print("Drawing Path")
        with open('data_modified.csv', 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerows(grid)   
    
    def simplify_path(self,path):
        direction_array = []
        if len(path)%2==0:
            path.append(path[-1])

        for i in range(2,len(path),2):
            if path[i][0] == path[i-2][0] and path[i][1] == path[i-2][1] - 1:
                direction = "t"
            elif path[i][0] == path[i-2][0]  + 1 and path[i][1] == path[i-2][1] - 1:
                direction = "tr"
            elif path[i][0] == path[i-2][0] + 1 and path[i][1] == path[i-2][1]:
                direction = "r"
            elif path[i][0] == path[i-2][0] + 1 and path[i][1] == path[i-2][1] + 1:
                direction = "br"
            elif path[i][0] == path[i-2][0] and path[i][1] == path[i-2][1] + 1:
                direction = "b"
            elif path[i][0] == path[i-2][0] - 1 and path[i][1] == path[i-2][1] + 1:
                direction = "bl"
            elif path[i][0] == path[i-2][0] - 1 and path[i][1] == path[i-2][1]:
                direction = "l"
            else:
                if path[i-1][0] == path[i-2][0] and path[i-1][1] == path[i-2][1] - 1:
                    direction = "t"
                elif path[i-1][0] == path[i-2][0]  + 1 and path[i-1][1] == path[i-2][1] - 1:
                    direction = "tr"
                elif path[i-1][0] == path[i-2][0] + 1 and path[i-1][1] == path[i-2][1]:
                    direction = "r"
                elif path[i-1][0] == path[i-2][0] + 1 and path[i-1][1] == path[i-2][1] + 1:
                    direction = "br"
                elif path[i-1][0] == path[i-2][0] and path[i-1][1] == path[i-2][1] + 1:
                    direction = "b"
                elif path[i-1][0] == path[i-2][0] - 1 and path[i-1][1] == path[i-2][1] + 1:
                    direction = "bl"
                elif path[i-1][0] == path[i-2][0] - 1 and path[i-1][1] == path[i-2][1]:
                    direction = "l"
                    
                direction_array.append(direction)
                    
                if path[i][0] == path[i-1][0] and path[i][1] == path[i-1][1] - 1:
                    direction = "t"
                if path[i][0] == path[i-1][0]  + 1 and path[i][1] == path[i-1][1] - 1:
                    direction = "tr"
                elif path[i][0] == path[i-1][0] + 1 and path[i][1] == path[i-1][1]:
                    direction = "r"
                elif path[i][0] == path[i-1][0] + 1 and path[i][1] == path[i-1][1] + 1:
                    direction = "br"
                elif path[i][0] == path[i-1][0] and path[i][1] == path[i-1][1] + 1:
                    direction = "b"
                elif path[i][0] == path[i-1][0] - 1 and path[i][1] == path[i-1][1] + 1:
                    direction = "bl"
                elif path[i][0] == path[i-1][0] - 1 and path[i][1] == path[i-1][1]:
                    direction = "l"
            direction_array.append(direction)
            
        simplified_path = []
        current_state = -1
        for i in range(len(direction_array)):
            if direction_array[i] != current_state:
                simplified_path.append(path[i])
            current_state = direction_array[i]
            
        if path[-1] not in simplified_path:
            simplified_path.append(path[-1])
        return simplified_path

def main(args=None):
    rclpy.init(args=args)
    robot_explorer = RobotExplorer()
    executor = MultiThreadedExecutor()
    executor.add_node(robot_explorer)
    try:
        executor.spin()
    except KeyboardInterrupt:
        robot_explorer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()