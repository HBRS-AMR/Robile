#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():

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

    robot_description = SetEnvironmentVariable(
            'robot_description',
            "$(find robile_description)/robots/robile1.urdf.xacro"
        )

    run_param_robot_description = ExecuteProcess(
            cmd=['rosrun', 'xacro', 'xacro', '$(env robot_description)'],
            name='xacro_process'
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

    return LaunchDescription([

        smart_wheel_driver,
        joy,
        lms1xx,
        laser_filters,
        robot_description,
        run_param_robot_description,
        joint_state_publisher,
        robot_state_publisher

    ])
