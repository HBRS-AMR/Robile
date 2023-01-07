#!/usr/bin/env python3

import os

from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    controller_frequency = LaunchConfiguration('controller_frequency', default='5.0')
    costmap_params_dir = LaunchConfiguration('costmap_params_dir', default=os.path.join(get_package_share_directory('robile_navigation'), 'ros', 'config', 'move_base_config'))
    costmap_common_params = os.path.join(costmap_params_dir, 'costmap_common_params.yaml')
    global_costmap_params = os.path.join(costmap_params_dir, 'global_costmap_params.yaml')
    local_costmap_params = os.path.join(costmap_params_dir, 'local_costmap_params.yaml')
    base_local_planner_params = os.path.join(costmap_params_dir, 'base_local_planner_params.yaml')

    move_base = Node(
            package='move_base',
            node_executable='move_base',
            node_name='move_base',
            output='screen',
            parameters=[{'base_local_planner': 'base_local_planner/TebLocalPlannerROS'},
                        {'controller_frequency': controller_frequency}],
            remappings=[
                ('cmd_vel', '/cmd_vel'),
                ('odom', '/odom'),                        
                ('/move_base_simple/goal', '/move_base_simple/goal'),
                ('/move_base/cancel', '/move_base/cancel'),
                ('/move_base/feedback', '/move_base/feedback'),
                ('/move_base/goal', '/move_base/goal'),
                ('/move_base/result', '/move_base/result'),
                ('/move_base/status', '/move_base/status')
                ],
            arguments=['global_costmap', costmap_common_params, 'load'],
            arguments=['local_costmap', costmap_common_params, 'load'],
            arguments=[global_costmap_params, 'load'],
            arguments=[local_costmap_params, 'load'],
            arguments=[base_local_planner_params, 'load']
        )

    return LaunchDescription([
        move_base,
    ])