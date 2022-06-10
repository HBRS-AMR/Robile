#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import random
import pandas as pd
import time
from tf import transformations

max_vel = 0.1
max_ang_vel = 0.1
command_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

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
        if np.shape(start)!= (2,):
            raise ValueError("Start point must have the shape (2,)", start)
        if np.shape(end) != (2,):
            raise ValueError("End point must have the shape (2,)")
        if (start==end).all():
            raise ValueError("Start and end points must be different")
        
        self.start = start
        self.line = end - start
        self.length = np.linalg.norm(self.line)
        self.unit_line = self.line / self.length
        
    def point_dist(self, point: np.ndarray):
        if np.shape(point)!= (2,):
            raise ValueError("Start point must have the shape (2,)")
        return np.linalg.norm(np.cross(self.line, self.start - point)) /self.length
    
    def equation(self):
        m = self.line[1]/self.line[0]
        c = self.start[1] - m*self.start[0]
        return (m, c)

def laser_callback(msg):
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
	rospy.loginfo('Points: ' + str(len(range_data)) + '  Max-angle: ' + str(max_angle) + '  Min-angle: ' + str(min_angle) + '  Max-range: ' + str(max_range))
	laser_sub_cartesian = process_data(np.array(range_data), max_angle, min_angle, max_range, min_range)

	rate = rospy.Rate(1)
	
	while not rospy.is_shutdown():
		msg = Twist()
		publish_command_velocity(laser_sub_cartesian)
		rate.sleep()

def process_data(range_data: np.ndarray, max_angle: int, min_angle: int, max_range: int, min_range: int):
    """
	Function to pre-process the laser range data
    :param range_data: 1D numpy array of laser range data
    :return processed_data: 2D array of cartesian coordinates of points along the column
    :rtype processed_data: numpy.ndarray
	"""
    processed_data = None
    ### Your CODE here
	
	
    return processed_data

def RANSAC(points: list, dist_thresh: int, k: int, thresh_count: int):
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
    ### YOUR CODE HERE

	return (best_point_1, best_point_2, inliers[best_pair])

def np_polar2rect(polar_points: np.ndarray):
    """
    Function to convert polar coordinates to cartesian form

    :param polar_points: 2D array of shape (n,2), where n: no. of points. 
    First column-range values in meters; Second column: angles in radians
    
    :return: A 2D array of shape (n,2), representing points in cartesian coordinates 
    """
    ### YOUR CODE HERE

    return None

def publish_zero_twist():
	"""
	Function to publish zero linear and angular velocity
	"""
	msg = Twist()
	msg.linear.x = 0
	msg.linear.y = 0
	msg.angular.z = 0	

def rotate(angle_rad):
	"""
	Function to publish angular velocity to achieve required angle of rotation

    :param angle_rad: angle of rotation in radians
    :param type: float
	"""    
	### YOUR CODE HERE
	return 0

def move(direction, distance):
	"""
	Function to publish direction and distance of travel 

    :param direction: 1D array representing the x and y coordinates of direction of motion
    :param type: np.ndarray

    :param distance: angle of rotation in radians
    :param type: float    
	"""    
	### YOUR CODE HERE
	return 0    

def get_line_params(laser_sub_cartesian):
	"""
	Function to extract line parameters from laser scan data

    :param laser_sub_cartesian: 2D array representing laser scan data in cartesian coordinates
    :param type: np.ndarray

    :return m_c_start_end: tuple of slope, constant of line equation, start and end points of line as 1D arrays 
    :param type: tuple    
	"""     
	points = [point for point in laser_sub_cartesian]
	m_c_start_end = []
	
    ### YOUR CODE HERE

	return m_c_start_end
		
def publish_command_velocity(laser_sub_cartesian):
	"""
	Function to determine linear and angular velocities to be published by infering from line parameters  

    :param laser_sub_cartesian: 2D array representing laser scan data in cartesian coordinates
    :param type: np.ndarray   
	"""      
	msg = Twist()
	msg.linear.x = 0
	msg.linear.y = 0
	msg.angular.z = 0.05
	### YOUR CODE HERE
    
	
def main():
	rospy.init_node('laserscan_reader')
	rospy.Subscriber('/scan_filtered', LaserScan, laser_callback)
	rospy.spin()

if __name__ == '__main__':
	main()