#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from tf import transformations

max_vel = 0.01

def laser_callback(msg):
	range_data = msg.ranges
	rospy.loginfo(len(range_data))
	processed_data = process_data(range_data)
	return processed_data

def process_data(range_data):
	processed_data = range_data
	## Your CODE here
	
	
	return processed_data
	
def main():
	rospy.init_node('laserscan_reader')
	laser_sub = rospy.Subscriber('/scan_filtered', LaserScan, laser_callback)
	command_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
	msg = Twist()
	
	msg = command_velocity(laser_sub)
	rate = rospy.Rate(20)
	
	while not rospy.is_shutdown():
		msg = Twist()
		msg = command_velocity(laser_sub)
		command_pub.publish(msg)
		rate.sleep()
	
def command_velocity(laser_sub):
	msg = Twist()
	msg.linear.x = 0
	msg.linear.y = 0
	msg.angular.z = 0
	## Your CODE here
	
		
	assert msg.linear.x <= max_vel
	assert msg.linear.y <= max_vel
	assert msg.linear.z <= max_vel		
	return msg	
	

if __name__ == '__main__':
	main()

