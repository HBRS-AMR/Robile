#!/usr/bin/env python3
 
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
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

    gmapping = ExecuteProcess(
        cmd=['gmapping', '--slam_gmapping'],
        name='slam_gmapping',
        output='screen',
        emulate_tty=True,
        remappings=[('scan', '/scan_filtered')],
        parameters=[
            {'delta': {'value': '0.05'}},
            {'xmin': {'value': '-5'}},
            {'xmax': {'value': '5'}},
            {'ymin': {'value': '-5'}},
            {'ymax': {'value': '5'}},
            {'llsamplerange': {'value': '0.01'}},
            {'llsamplestep': {'value': '0.01'}},
            {'lasamplerange': {'value': '0.005'}},
            {'lasamplestep': {'value': '0.005'}},
            {'base_frame': {'value': 'base_link'}},
            {'odom_frame': {'value': 'odom'}},
            {'map_update_interval': {'value': '5.0'}},
            {'maxUrange': {'value': '6.0'}},
            {'maxRange': {'value': '8.0'}},
            {'sigma': {'value': '0.05'}},
            {'kernalSize': {'value': '1'}},
            {'lstep': {'value': '0.05'}},
            {'astep': {'value': '0.05'}},
            {'iterations': {'value': '5'}},
            {'lsigma': {'value': '0.075'}},
            {'ogain': {'value': '3.0'}},
            {'lskip': {'value': '0'}},
            {'minimumScore': {'value': '200'}},
            {'srr': {'value': '0.01'}},
            {'srt': {'value': '0.02'}},
            {'str': {'value': '0.01'}},
            {'stt': {'value': '0.02'}},
            {'linearUpdate': {'value': '0.5'}},
            {'linearUpdate': {'value': '0.05'}},
            {'angularUpdate': {'value': '0.436'}},
            {'temporalUpdate': {'value': '-1.0'}},
            {'resampleThreshold': {'value': '0.5'}},
            {'particles': {'value': '80'}},
        ],
    )

    return LaunchDescription([laser_link, gmapping])