#!/usr/bin/env python3
 
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    robot_env = LaunchConfiguration(
        'robot_env', default=os.environ.get('ROBOT_ENV', '!!NO_ROBOT_ENV_SET!!')
    )
    map_file = os.path.join(
        '$(find robile_default_env_config)', 'ros', f'{robot_env}.yaml'
    )
    map_server = ExecuteProcess(
        cmd=['map_server', '--map_file', map_file],
        name='map_server',
        output='screen',
        emulate_tty=True,
    )
    use_map_topic = LaunchConfiguration(
        'use_map_topic', default='true'
    )

    # Transform laser scan data to base_laser
    laser_link = ExecuteProcess(
        cmd=[
            'tf2_static_transform_publisher',
            '0.45',
            '0',
            '0.22',
            '0',
            '0',
            '0',
            '/base_link',
            '/base_laser',
            '30',
        ],
        name='laser_link',
        output='screen',
        emulate_tty=True,
    )

    amcl = ExecuteProcess(
        cmd=['amcl'],
        name='amcl',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'use_map_topic': {'value': use_map_topic}},
            {'odom_model_type': {'value': 'omni'}},
            {'odom_alpha5': {'value': '0.1'}},
            {'gui_publish_rate': {'value': '10.0'}},
            {'laser_max_beams': {'value': '60'}},
            {'laser_max_range': {'value': '12.0'}},
            {'min_particles': {'value': '500'}},
            {'max_particles': {'value': '2000'}},
            {'kld_err': {'value': '0.05'}},
            {'kld_z': {'value': '0.99'}},
            {'odom_alpha1': {'value': '0.2'}},
            {'odom_alpha2': {'value': '0.2'}},
            {'odom_alpha3': {'value': '0.2'}},
            {'odom_alpha4': {'value': '0.2'}},
            {'laser_z_hit': {'value': '0.5'}},
            {'laser_z_short': {'value="0.05'}},
            {'laser_z_max': {'value': '0.05'}},
            {'laser_z_rand': {'value': '0.5'}},
            {'laser_sigma_hit': {'value': '0.2'}},
            {'laser_lambda_short': {'value': '0.1'}},
            {'laser_model_type': {'value': 'likelihood_field'}},
            {'laser_likelihood_max_dist': {'value': '2.0'}},
            {'update_min_d': {'value': '0.25'}},
            {'update_min_a': {'value': '0.2'}},
            {'odom_frame_id': {'value': 'odom'}},
            {'resample_interval': {'value': '1'}},
            {'transform_tolerance': {'value': '1.0'}},
            {'recovery_alpha_slow': {'value': '0.0'}},
            {'recovery_alpha_fast': {'value': '0.0'}},
        ],
        remappings=[('scan', 'scan_filtered')],
    )

    return LaunchDescription([
        map_server,
        laser_link,
        amcl
    ])
