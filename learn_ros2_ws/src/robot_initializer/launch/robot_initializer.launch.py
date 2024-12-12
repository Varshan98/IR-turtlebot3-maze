import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription,DeclareLaunchArgument, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnExecutionComplete

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
    pkg_turtlebot_bringup = get_package_share_directory('turtlebot3_bringup')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_yolo_bringup = get_package_share_directory('yolo_bringup')
   
    urdf_path = os.path.join(
        get_package_share_directory('robot_initializer'),
        'models',
        'model.sdf'
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    declare_sim_time_cmd = DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true')

    #For Gazebo
    world = os.path.join(
        get_package_share_directory('robot_initializer'),
        'worlds',
        'cv_room.world'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # For Spawning Model in Gazebo
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Specify namespace of the robot')
    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Specify namespace of the robot')
    start_gazebo_ros_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'burger_cv',
            '-file', urdf_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01',
            '-timeout', '60.0'
        ],
    )
    
    #Get URDF
    original_burger_urdf = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf',
        'turtlebot3_burger.urdf')
    with open(original_burger_urdf, 'r') as infp:
        robot_desc = infp.read()
    rsp_params = {'robot_description': robot_desc}
    robot_state_publisher_cmd = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[rsp_params, {'use_sim_time': use_sim_time}]
    )

    delay_for_state = RegisterEventHandler(
            event_handler=OnExecutionComplete(
                target_action=start_gazebo_ros_spawner_cmd,
                on_completion=[robot_state_publisher_cmd],
            )
        )
    
    #For rviz2
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(
            get_package_share_directory('robot_initializer'),
            'rviz',
            'exploration_default_view.rviz')]
    )
    
    # #For Slam
    start_slam_cmd =  IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
        ), launch_arguments={
                'use_sim_time': use_sim_time
            }.items(),
    )
    
    # #For Nav2
    params_file = LaunchConfiguration('params_file')
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_nav2_bringup, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )
    start_navigation_cmd =  IncludeLaunchDescription(
       PythonLaunchDescriptionSource(
           os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
      ),
              launch_arguments={
                   'params_file': params_file,
                  'container_name': 'nav2_container',
               }.items(),
    )

    # #For Yolo Model
    start_yolo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_yolo_bringup,
                "launch",
                "yolov11_burger.launch.py",
            )
        )
    )

    # #For recording node part
    start_record_data_cmd = Node(
        package='robot_initializer',
        executable='robot_initializer_node',
        output='screen'
    )

    # #For recording node part
    start_output_markers_cmd = Node(
        package='robot_initializer',
        executable='detected_object_processer_node',
        output='screen'
    )


    ld = LaunchDescription()
    # ld.add_action(delay_for_state)
    # ld.add_action(declare_x_position_cmd)
    # ld.add_action(declare_y_position_cmd)
    # ld.add_action(declare_params_file_cmd)
    # ld.add_action(declare_sim_time_cmd)

    # #Gazebo
    # ld.add_action(gzclient_cmd)
    # ld.add_action(gzserver_cmd)
    # ld.add_action(start_gazebo_ros_spawner_cmd)
    
    ld.add_action(start_slam_cmd)
    ld.add_action(start_rviz_cmd)
    ld.add_action(start_navigation_cmd)
    ld.add_action(start_yolo_cmd)
    ld.add_action(start_record_data_cmd)
    # ld.add_action(start_output_markers_cmd)
    # ld.add_action(start_explore_cmd)

    return ld
