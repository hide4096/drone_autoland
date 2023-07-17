import os
import math
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    aruco_params = os.path.join(
        get_package_share_directory('ros2_aruco'),
        'config',
        'aruco_parameters.yaml'
        )

    aruco_node = Node(
        package='drone_autoland',
        executable='aruco_detect',
    )
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node'
    )
    camera_pos_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0', '-0.1', '0.0', '0.0', str(math.radians(45)),'0.0', 'base_link', 'camera_pos']
    )

    controller = Node(
        package='joy',
        executable='joy_node',
    )
    
    guide = Node(
        package='drone_autoland',
        executable='read_marker',
        output='screen'
    )
    
    tello = Node(
        package='tello_driver',
        executable='tello_driver_main',
    )

    return LaunchDescription([
        camera_pos_node,
        camera_node,
        aruco_node,
        tello,
        controller,
        guide
    ])
