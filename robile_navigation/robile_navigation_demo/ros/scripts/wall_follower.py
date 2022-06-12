#! /usr/bin/env python

import math
import time
import rospy
import numpy as np
import random
import tf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt


class wall_follower:
    def __init__(self):
        print("Node initiated..............")
        self.vel_command_publisher = rospy.Publisher(
            '/cmd_vel', Twist, queue_size=1)
        self.max_vel = 0.1
        self.max_ang_vel = 0.1
        self.threshold_wall_dist = 1.0
        self.command_running = False
        self.angle_aligned = False
        self.collison_detected = False
        self.close_to_wall = False
        self.robot_center_wrt_laser = np.array([-0.45, 0])
        self.parallel_to_wall = False
        self.ready_to_publish = True
        self.command_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/scan_filtered', LaserScan, self.laser_callback)
        self.loop_rate = rospy.Rate(rospy.get_param("~loop_rate", 0.2))

    def laser_callback(self, msg):
        """
        This function is called when the subscriber subscribes to a topic
        :param msg: msg received from the subscribed topic
        :type msg: depends on the subscribed topic
        """
        range_data = msg.ranges
        max_angle = msg.angle_max
        min_angle = msg.angle_min
        max_range = msg.range_max
        min_range = msg.range_min
        rospy.loginfo('Points: ' + str(len(range_data)) + '  Max-angle: ' + str(
            max_angle) + '  Min-angle: ' + str(min_angle) + '  Max-range: ' + str(max_range))

        laser_sub_cartesian = self.process_data(
            np.array(range_data), max_angle, min_angle, max_range, min_range)

        if self.ready_to_publish:
            self.publish_command_velocity(laser_sub_cartesian)
            self.ready_to_publish = False
            
            rospy.loginfo(
                'one iteration of publish_command_velocity completed now')

    def process_data(self, range_data: np.ndarray, max_angle: int, min_angle: int, max_range: int, min_range: int):
        """
        Function to pre-process the laser range data
        :param range_data: 1D numpy array of laser range data
        :return processed_data: 2D array of cartesian coordinates of points along the column
        :rtype processed_data: numpy.ndarray
        """
        processed_data = None
        # Your CODE here

        return processed_data

    def RANSAC(self, points: list, dist_thresh: int, iterations: int, thresh_count: int):
        """
        A simple RANSAC algorithm to fit a line to a set of 2D points
        :param points: The list of points to fit a line where each point
                    is an numpy array in the form array of form [x,y].
        :type points: numpy.ndarray
        :param dist_thresh: The parameter used to determine the furthest a point
                            can be from the line and still be considered an inlier
        :type dist_thresh: int
        :param k: The number of iterations the RANSAC algorithm will run
        :type k: int
        :param thresh_count: number of minimum points required to form a line
        :type thresh_count: int
        :return: A tuple of two numpy arrays which are the two points which produce
                a line with the corresponding inliers
        :rtype: tuple
        """
        best_point_1 = None
        best_point_2 = None
        inliers = dict()
        best_pair = None
        # YOUR CODE HERE

        return (best_point_1, best_point_2, inliers[best_pair])

    def np_polar2rect(self, polar_points):
        """
        Function to convert polar coordinates to cartesian form
        :param polar_points: 2D array of shape (n,2), where n: no. of points. 
        First column-range values in meters; Second column: angles in radians

        :return: A 2D array of shape (n,2), representing points in cartesian coordinates 
        """
        laser_cart_coord = None
        # YOUR CODE HERE

        return laser_cart_coord

    def publish_zero_twist(self):
        """
        Function to publish zero linear and angular velocity
        """
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.angular.z = 0
        rospy.loginfo('publishing zero velocity')
        self.command_pub.publish(msg)
        rospy.sleep(5)

    def rotate(self, angle_rad):
        """
        Function to publish angular velocity to achieve required angle of rotation
        :param angle_rad: angle of rotation in radians
        :param type: float
        """
        # YOUR CODE HERE
        return 0

    def move(self, direction, distance):
        """
        Function to publish direction and distance of travel 
        :param direction: 1D array representing the x and y coordinates of direction of motion
        :param type: np.ndarray
        :param distance: angle of rotation in radians
        :param type: float    
        """
        msg = Twist()

        # YOUR CODE HERE
        return 0

    def verify_collison(self, laser_sub_cartesian):
        """
        Function to determine if robot is going to collide with any obstacle
        :param direction: 1D array representing the x and y coordinates of direction of motion
        :param type: np.ndarray
        :param distance: angle of rotation in radians
        :param type: float    
        """

        # YOUR CODE HERE
        return 0

    def get_line_params(self, laser_sub_cartesian):
        """
        Function to extract line parameters from laser scan data
        :param laser_sub_cartesian: 2D array representing laser scan data in cartesian coordinates
        :param type: np.ndarray
        :return m_c_start_end: tuple of slope, constant of line equation, start and end points of line as 1D arrays 
        :param type: tuple    
        """
        points = [point for point in laser_sub_cartesian]
        m_c_start_end = []

        # YOUR CODE HERE

        return m_c_start_end

    def reset_flags(self):
        self.angle_aligned = False
        self.parallel_to_wall = False
        self.close_to_wall = False

    def publish_command_velocity(self, laser_sub_cartesian):
        """
        Function to determine linear and angular velocities to be published by infering from line parameters  
        :param laser_sub_cartesian: 2D array representing laser scan data in cartesian coordinates
        :param type: np.ndarray   
        """

        # YOUR CODE HERE
        return 0


class Line:
    """
    An object used to create a line from two points
    :param start: First point used to generate the line. It's an
                  array of form [x,y].
    :type start: numpy.ndarray
    :param end: Second point used to generate the line. It's an
                array of form [x,y].
    :type end: numpy.ndarray
    """

    def __init__(self, start: np.ndarray, end: np.ndarray):
        if np.shape(start) != (2,):
            raise ValueError("Start point must have the shape (2,)", start)
        if np.shape(end) != (2,):
            raise ValueError("End point must have the shape (2,)")
        if (start == end).all():
            raise ValueError("Start and end points must be different")

        # Calculate useful properties of the line
        self.start = start
        self.line = end - start
        self.length = np.linalg.norm(self.line)
        self.unit_line = self.line / self.length

    def point_dist(self, point: np.ndarray):
        if np.shape(point) != (2,):
            raise ValueError("Start point must have the shape (2,)")
        return np.linalg.norm(np.cross(self.line, self.start - point)) / self.length

    def equation(self):
        m = self.line[1]/self.line[0]
        c = self.start[1] - m*self.start[0]
        return (m, c)


if __name__ == '__main__':
    rospy.init_node("wall_follower")
    OBJ = wall_follower()
