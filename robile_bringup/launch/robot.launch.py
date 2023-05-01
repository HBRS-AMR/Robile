#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    lms1xx = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('sick_scan'), 'launch'),
            '/sick_lms_1xx.launch.py']),
        launch_arguments={'hostname': '192.168.1.35',
                          'frame_id': 'base_laser',
                          'min_angle': '-1.9',
                          'max_angle': '1.9'}.items()
    )

    hukoyu_urg_04 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('urg_node2'), 'launch'),
            '/urg_node2.launch.py']),
    )   

    smart_wheel_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory("kelo_tulip"), 'launch'),
            '/example_joypad.launch.py'])
    )

    static_transform_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_link_node',
        output='screen',
        arguments=["0.45", "0", "0.22", "0", "0",
                    "0", "base_link", "base_laser"]
    )

    joint_state_publisher = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'rate': 10}]
        )

    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'publish_frequency': 10}]
        )  

    rviz_cmd = Node(package='rviz2',
                    namespace='',
                    executable='rviz2',
                    name='rviz2',
                    output='screen',
                    )
    
    static_transform = Node(package="tf2_ros",
                                executable="static_transform_publisher",
                                output="screen",
                                arguments=["0", "0", "0", "0", "0",
                                           "0", "base_footprint", "base_link"]
                                )
    
    return LaunchDescription([
        lms1xx,
        hukoyu_urg_04,
        smart_wheel_driver,
        static_transform_cmd,
        joint_state_publisher,
        robot_state_publisher,
        rviz_cmd,
        static_transform
    ])
