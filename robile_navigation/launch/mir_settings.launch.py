#!/usr/bin/env python3

import launch
from launch_ros.actions import Node

def generate_launch_description():
    # run scan unifier
    unifier_node = Node(
        package='cob_scan_unifier',
        node_executable='scan_unifier_node',
        node_name='scan_unifier',
        output='log',
        parameters=[{
            'input_scans': ['scan_front', 'scan_rear']
        }, {
            'loop_rate': 20.0
        }],
        remappings=[('scan_unified', '/scan_unified')]
    )

    gmapping_node = Node(
        package='gmapping',
        node_executable='slam_gmapping',
        node_name='slam_gmapping',
        remappings=[('scan', '/scan_front')],
        parameters=[{
            'base_frame': 'base_footprint',
            'map_frame': 'map',
            'odom_frame': 'odom',
            'map_update_interval': 5.0, # How long (in seconds) between updates to the map. Lowering this number updates the occupancy grid more often, at the expense of greater computational load
            'maxUrange': 4.5,       # The maximum usable range of the laser. A beam is cropped to this value 
            'sigma': 0.05,          # The sigma used by the greedy endpoint matching
            'kernelSize': 1,        # The kernel in which to look for a correspondence
            'lstep': 0.05,          # The optimization step in translation
            'astep': 0.05,          # The optimization step in rotation
            'iterations': 5,        # The number of iterations of the scanmatcher
            'lsigma': 0.075,        # The sigma of a beam used for likelihood computation
            'ogain': 3.0,           # Gain to be used while evaluating the likelihood, for smoothing the resampling effects
            'lskip': 0,             # Number of beams to skip in each scan.
            'srr': 0.1,             # Odometry error in translation as a function of translation (rho/rho)
            'srt': 0.2,             # Odometry error in translation as a function of rotation (rho/theta)
            'str': 0.1,             # Odometry error in rotation as a function of translation (theta/rho)
            'stt': 0.2,             # Odometry error in rotation as a function of rotation (theta/theta)
            'linearUpdate': 1.0,    # Process a scan each time the robot translates this far
            'angularUpdate': 0.5,   # Process a scan each time the robot rotates this far
            'temporalUpdate': -1.0, # Process a scan if the last scan proccessed is older than the update time in seconds. A value less than zero will turn time based updates off
            'resampleThreshold': 0.5, # The neff based resampling threshold
            'particles': 30,        # Number of particles in the filter
            'xmin': -5.0,           # Initial map size
            'ymin': -5.0,           # Initial map size
            'xmax': 5.0,            # Initial map size
            'ymax': 5.0,            # Initial map size
            'delta': 0.05,          # Processing parameters (resolution of the map)
            'llsamplerange': 0.01,  # Translational sampling range for the likelihood
            'llsamplestep': 0.01,   # Translational sampling step for the likelihood
            'lasamplerange': 0.005, # Angular sampling range for the likelihood
            'lasamplestep': 0.005,  # Angular sampling step for the likelihood
            'transform_publish_period': 0.05, # How long (in seconds) between transform publications
            'occ_thresh': 0.25,     # Threshold on gmapping's occupancy values. Cells with greater occupancy are considered occupied (i.e., set to 100 in the resulting sensor_msgs/LaserScan). New in 1.1.0.
            'maxRange': 8.0         # The maximum range of the sensor. If regions with no obstacles within the range of the sensor should appear as free space in the map, set maxUrange < maximum range of the real sensor <= maxRange 
        }]
    )

    nav_view = Node(
    package='nav_view',
    node_executable='nav_view',
    node_name='nav_view',
    output='log',
    parameters=[{'input_map': '/static_map'}],
    remappings=[('/dynamic_map', '/static_map')]
    )

    return launch.LaunchDescription([unifier_node, gmapping_node, nav_view])