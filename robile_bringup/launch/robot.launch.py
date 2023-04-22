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

    # laser_filter_config = os.path.join(
    #     get_package_share_directory('robile_bringup'),
    #     'config',
    #     'laser_sick_config.yaml'
    #     )

    # laser_filter = Node(
    #     package="laser_filters",
    #     executable="scan_to_scan_filter_chain",
    #     parameters=[laser_filter_config],
    # )

    smart_wheel_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory("kelo_tulip"), 'launch'),
            '/example_joypad.launch.py'])
    )

    tf2_ros = Node(
        package='tf2_ros',
        node_executable='static_transform_publisher',
        node_name='laser_link_node',
        output='screen',
        parameters=[{'args': '0.45 0 0.22 0 0 0 /base_link /base_laser 30'}]
    )

    joint_state_publisher = Node(
            package='joint_state_publisher',
            node_executable='joint_state_publisher',
            node_name='joint_state_publisher',
            output='screen',
            parameters=[{'rate': 10}]
        )

    robot_state_publisher = Node(
            package='robot_state_publisher',
            node_executable='robot_state_publisher',
            node_name='robot_state_publisher',
            output='screen',
            parameters=[{'publish_frequency': 10}]
        )  

    rviz_cmd = Node(package='rviz2',
                    namespace='',
                    executable='rviz2',
                    name='rviz2',
                    output='screen',
                    )
    
    static_transform_cmd = Node(package="tf2_ros",
                                executable="static_transform_publisher",
                                output="screen",
                                arguments=["0", "0", "0", "0", "0",
                                           "0", "base_footprint", "base_link"]
                                )
    
    return LaunchDescription([
        lms1xx,
        # laser_filter,
        smart_wheel_driver,
        tf2_ros,
        joint_state_publisher,
        robot_state_publisher,
        rviz_cmd,
        static_transform_cmd
    ])
