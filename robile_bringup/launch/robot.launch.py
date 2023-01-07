#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    lms1xx = Node(
            package='lms1xx',
            node_executable='LMS1xx_node',
            node_name='lms1xx_laser', 
            output='screen',
            parameters=[
                {'host': '192.168.1.35'},
                {'frame_id': 'base_laser'}, 
            ]
        )

    laser_filters = Node(
            package='laser_filters',
            node_executable='scan_to_scan_filter_chain',
            node_name='laser_filter',
            output='screen',
            parameters=[{'command': 'load', 'file': '$(find robile_navigation_demo)/ros/config/laser_sick_config.yaml'}]
        )

    smart_wheel_driver = Node(
            package='smart_wheel_driver',
            node_executable='narko.launch',
            node_name='smart_wheel_driver',
            output='screen'
        )

    joy = Node(
            package='joy',
            node_executable='joy_node',
            node_name='joy',
            output='screen'
        )

    tf2_ros = Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            node_name='laser_link',
            output='screen',
            parameters=[{'args': '0.45 0 0.22 0 0 0 /base_link /base_laser 30'}]
        )

    kelo_tulip = Node(
            package='kelo_tulip',
            node_executable='example_joypad.launch.py',
            node_name='kelo_tulip',
            output='screen',
        ) 

    return LaunchDescription([
        lms1xx,
        laser_filters,
        smart_wheel_driver,
        joy,
        tf2_ros       
    ])