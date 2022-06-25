#! /usr/bin/env python

import math as m
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
        self.max_vel = 0.1
        self.max_ang_vel = 0.1
        self.threshold_wall_dist = 1.0
        # Vector from base_laser to base_link
        self.center_wrt_laser = np.array([-0.45, 0])
        self.ready_to_publish = True
        self.command_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.laser_sub_cartesian = None
        self.laser_scan_processed = False
        rospy.Subscriber('/scan_filtered', LaserScan, self.laser_callback)
        self.loop_rate = rospy.Rate(1)
        self.loop_rate.sleep()

    def main(self):
        """
        Starts the node
        """

        rospy.loginfo("Ready to start now...")
        while not rospy.is_shutdown():
            if self.ready_to_publish and self.laser_scan_processed:
                self.ready_to_publish = False
                self.avoid_collison()
                self.publish_command_velocity()

                rospy.loginfo(
                    'one iteration of publish_command_velocity completed now')
            self.loop_rate.sleep()

    def laser_callback(self, msg):
        """
        This function is called when the subscriber subscribes to a topic (here, topic:/scan_filtered)
        :param msg: msg received from the subscribed topic
        :type msg: depends on the subscribed topic
        """

        range_data = msg.ranges
        max_angle = msg.angle_max
        min_angle = msg.angle_min
        max_range = msg.range_max
        min_range = msg.range_min
        ang_inc = msg.angle_increment #added new line to get angle increment data from message header

        self.laser_sub_cartesian = self.process_data(
            np.array(range_data), max_angle, min_angle, max_range, min_range, ang_inc)

    def process_data(self, range_data: np.ndarray, max_angle: float, min_angle: float, max_range: float, min_range: float, ang_inc: float):
        """
        Function to pre-process the laser range data
        :param range_data: 1D numpy array of laser range data
        :type range_data: numpy.ndarray
        :param max_angle: upper limit of the angle in radians corresponding to the range measurement
        :type max_angle: float
        :param min_angle: lower limit of the angle in radians corresponding to the range measurement
        :type min_angle: float
        :param max_range: upper limit of the range that can be measured (in meters)
        :type max_range: float
        :param min_range: lower limit of the range that can be measured (in meters)
        :type min_range: float
        :return processed_data: 2D array of cartesian coordinates of points along the column
        :rtype processed_data: numpy.ndarray
        """

        processed_data = None
        angle = []

        for i in range(min_angle, max_angle, ang_inc):
            angle.append(i)
        angle = np.array(angle)

        x = []
        y = []


        for i, j in zip(range_data, angle):
            x.append(i * m.cos(np.rad2deg(j)))
            y.append(i * m.sin(np.rad2deg(j)))

        x = np.array(x).T
        y = np.array(y).T

        processed_data = np.vstack((x,y)).T
        self.laser_scan_processed = True
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
        :param iterations: The number of iterations the RANSAC algorithm will run
        :type iterations: int
        :param thresh_count: number of minimum points required to form a line
        :type thresh_count: int
        :return: A tuple of all points for the line fitted using the best_pair of points
        :rtype: tuple
        """

        best_point_1 = None
        best_point_2 = None
        inliers = dict()
        best_pair = None
   
        best_inliers_count = 0
        for i in range(0,iterations):
            random_number = np.random.randint(0, len(points), size = 2)
            random_points = points[random_number]
            
            line = Line(random_points[0], random_points[1])
            
            inliers_count = 0

            for point in points:
                distance = line.point_dist(point)
                if distance <= dist_thresh:
                    inliers_count = inliers_count + 1 
            
            if  best_inliers_count < inliers_count:
                best_inliers_count = inliers_count
                best_point_1 = random_points[0]
                best_point_2 = random_points[1]
                

        #return (best_point_1, best_point_2, inliers[best_pair])
        return (best_point_1, best_point_2) #removed inliers from given return 


    def np_polar2rect(self, polar_points):
        """
        Function to convert polar coordinates to cartesian form
        :param polar_points: 2D array of shape (n,2), where n: no. of points. 
        First column: range values in meters; Second column: angles in radians

        :return: A 2D array of shape (n,2), representing points in cartesian coordinates 
        """

        laser_cart_coord = None
        # center = np.array([0,0])
        # r = np_array.T[0,]
        # theta = np_array.T[1,]
        # x = r*np.sin(np.deg2rad(theta))
        # y = r*np.cos(np.deg2rad(theta))
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
        self.loop_rate.sleep()

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

    def avoid_collison(self):
        """
        Function to determine if robot is going to collide with any obstacle
        """


        return 0

    def get_line_params(self):
        """
        Function to extract line parameters from laser scan data
        :return m_c_start_end: tuple of slope, constant of line equation, start and end points of line as 1D arrays 
        :param type: tuple    
        """

        points = [point for point in self.laser_sub_cartesian]
        m_c_start_end = []

        BestFitLinePoints = RANSAC(points, self.threshold_wall_dist, 100, 2)
        wall = Line(BestFitLinePoints[0], BestFitLinePoints[1])
        m,c = wall.equation()
        m_c_start_end = (m,c, BestFitLinePoints[0], BestFitLinePoints[1])

        return m_c_start_end



    def publish_command_velocity(self):
        """
        Function to determine linear and angular velocities to be published by infering from line parameters    
        """
        get_line_params()

        
        return 0

    def __del__(self):
        self.publish_zero_twist()
        rospy.loginfo('Destructor called.... Published zero velocity')

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
        # if np.shape(start) != (2,):
        #     raise ValueError("Start point must have the shape (2,)", start)
        # if np.shape(end) != (2,):
        #     raise ValueError("End point must have the shape (2,)")
        # if (start == end).all():
        #     raise ValueError("Start and end points must be different")

        # Calculate useful properties of the line
        self.start = start
        self.line = end - start
        self.length = np.linalg.norm(self.line)
        self.unit_line = self.line / self.length

    def point_dist(self, point: np.ndarray):
        # if np.shape(point) != (2,):
        #     raise ValueError("Start point must have the shape (2,)")
        return np.linalg.norm(np.cross(self.line, self.start - point)) / self.length

    def equation(self):
        m = self.line[1]/self.line[0]
        c = self.start[1] - m*self.start[0]
        return (m, c)


if __name__ == '__main__':
    rospy.init_node("wall_follower")
    OBJ = wall_follower()
    OBJ.main()
