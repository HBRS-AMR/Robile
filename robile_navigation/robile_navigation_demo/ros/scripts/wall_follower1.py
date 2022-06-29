import math as m
import time
import rospy
import numpy as np
import random
import tf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
#import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
#import scipy

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
                self.listener()
                print(self.pos_values)

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

        self.laser_sub_cartesian = self.process_data(np.array(range_data), max_angle, min_angle, 
                                                     max_range, min_range, ang_inc, new_max_range = 10)

    def odom_callback(self, msg):
        x_pos = msg.pose.pose.position.x
        y_pos = msg.pose.pose.position.y
        z_ort = msg.pose.pose.orientation.z

        self.cur_pos = (x_pos, y_pos, z_ort)

    def listener(self):

        # run simultaneously.
        rospy.init_node('odom_listener', anonymous=True)

        rospy.Subscriber('odom', Odometry, self.odom_callback)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    def process_data(self, range_data: np.ndarray, max_angle: float, min_angle: float, max_range: float, min_range: float, ang_inc: float, new_max_range: float):
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
        :rtype: numpy.ndarray
        """

        processed_data = None
        #N = int((max_angle - min_angle)/ang_inc)
        angle = np.arange(min_angle,max_angle+ang_inc, ang_inc)
        polar_points = np.vstack((range_data,angle)).T
        
        filtered_polar_points = []

        for i in range(len(polar_points)):
             if polar_points[i][0] < new_max_range:
                 filtered_polar_points.append(polar_points[i])
                 
        processed_data = self.np_polar2rect(filtered_polar_points)
        self.laser_scan_processed = True
        return processed_data
    
    def np_polar2rect(self, polar_points):
        """
        Function to convert polar coordinates to cartesian form
        :param polar_points: 2D array of shape (n,2), where n: no. of points; First column: range values in meters; Second column: angles in radians
        :type polar_points: np.ndarray
        :return laser_cart_coord: A 2D array of shape (n,2), representing points in cartesian coordinates  
        :rtype: np.ndarray
        """
        robot_coord = []
        for i in polar_points:
            d = i[0]
            a =np.deg2rad(i[1])
            x= d*np.cos(a)
            y= d*np.sin(a)
            robot_coord.append(np.array([x,y]))
        return robot_coord

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
        best_point_1 = []
        best_point_2 = []
        best_inliers = []
        best_inliers_len = 0
        for i in range(iterations):
            inliers = []
            rand_num = random.sample(range(0, len(points)), thresh_count)
            start = points[rand_num[0]]
            end = points[rand_num[1]]
            line = Line(start,end)
            inliers_len = 0
            try:
                for point in points:
                    print(point)
                    dist = line.point_dist(point)
                    if dist <= dist_thresh:
                        inliers_len = inliers_len + 1
                        inliers.append(point)
            except:
                pass
            if best_inliers_len < inliers_len:
                best_inliers_len = inliers_len
                best_point_1 = start
                best_point_2 = end
                best_inliers = inliers
        return (best_point_1, best_point_2, best_inliers)
    
    #def filter_data(self, points, kernel_size):
      #  fitered = scipy.medfilt2d(points, kernel_size)
      #  return fitered
    
    def find_all_lines(self, points: list, dist_thresh: int, iterations: int, thresh_count: int, no_line: int):
        """
        Algorithm to find all lines in a set of 2D points
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
        :param no_line: max no. of lines to be found
        :type no_line: int
        :return lines: A tuple of all points for the line fitted using the best_pair of points
        :rtype: tuple
        """
        lines = []
        data_2 = points.copy() #self.filter_data(points, kernel_size=3)
        while True:
            (best_point_1, best_point_2, best_inliers) = self.RANSAC(data_2, dist_thresh, iterations, thresh_count)
            lines.append((best_point_1, best_point_2, best_inliers))
            best_inliers.append(best_point_1)
            best_inliers.append(best_point_2)
            for i in best_inliers:
                index = np.argwhere(data_2==i)
                data_2 = np.delete(data_2, index)
            if len(lines)>=no_line or len(data_2)<10:
                break
        return lines
    
    def get_close_line(self, robot_cord, lines, thresh):
        """
        Algorithm to find clossest line to robot
        :param robot_cord: Coordinate of robot as array of form [x,y].
        :type robot_cord: numpy.ndarray
        :param lines: array containing tuple of all points for the line fitted using 
                      the best_pair of points
        :type lines: numpy.ndarray
        :param thresh: The min number of points to be considered a line
        :type thresh: int
        :return closest: A tuple of best_pair of points
        :rtype: tuple
        """
        closest = []
        distance = 100000
        for i in lines:
            wall = Line(i[0],i[1])
            dist = wall.point_dist(robot_cord)
            if dist < distance and len(i[2]) > thresh:
                distance = dist
                closest = (i[0],i[1])
        return closest
    
    def get_line_params(self,line):
        """
        Function to extract line parameters from laser scan data
        :param line: tuple of best_pair of points
        :type line: tupple
        :return m_c_start_end: tuple of slope, constant of line equation, start and end points of line as 1D arrays 
        :rtype: tuple    
        """
        wall = Line(line[0],line[1])
        slope, intercept = wall.equation()
        m_c_start_end = (slope, intercept, line[0], line[1])
        return m_c_start_end
    
    def find_wall(self, robot_cord, dist_thresh: int, iterations: int, thresh_count: int, thresh: int, no_line: int):
        """
        Function to find the clossest wall
        :param robot_cord: Coordinate of robot as array of form [x,y].
        :type robot_cord: numpy.ndarray
        :param dist_thresh: The parameter used to determine the furthest a point
                            can be from the line and still be considered an inlier
        :type dist_thresh: int
        :param iterations: The number of iterations the RANSAC algorithm will run
        :type iterations: int
        :param thresh_count: number of minimum points required to form a line
        :type thresh_count: int
        :param no_line: max no. of lines to be found
        :type no_line: int
        :return closest_wall_params: tuple of slope, constant of line equation, start and end points of line as 1D arrays 
        :rtype: tuple    
        """
        points = [point for point in self.laser_sub_cartesian]
        lines = self.find_all_lines(points, dist_thresh, iterations, thresh_count, no_line)
        closest_wall = self.get_close_line(self, robot_cord, lines, thresh=thresh)
        closest_wall_params = self.get_line_params(self,closest_wall)
        return closest_wall_params
        
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
        x_int, y_int, ang_int = self.cur_pos
        ang_des = ang_int+angle_rad
        msg = Twist()
        msg.angular.z = max_ang_vel
        rospy.loginfo('publishing rotate robot')
        self.command_pub.publish(msg)
        self.loop_rate.sleep()
        while True:
            x_cur, y_cur, ang_cur = self.cur_pos
            if ang_cur == ang_des:
                self.publish_zero_twist()
        # YOUR CODE HERE
        return 0      
    
    def move_direc(self, direction, destination):
        msg = Twist()
        if direction == "x":
            msg.linear.x = self.max_vel
            msg.linear.y = 0
            msg.angular.z = 0
        else:
            msg.linear.x = 0
            msg.linear.y = self.max_vel
            msg.angular.z = 0
        self.command_pub.publish(msg)
        self.loop_rate.sleep()
        if direction == "x":
            while True:
                x_cur, y_cur, ang_cur = self.cur_pos
                if x_cur == destination:
                    self.publish_zero_twist()
        else:
            while True:
                x_cur, y_cur, ang_cur = self.cur_pos
                if y_cur == destination:
                    self.publish_zero_twist()

    def move(self, direction, distance):
        """
        Function to publish direction and distance of travel 
        :param direction: 1D array representing the x and y coordinates of direction of motion
        :param type: np.ndarray
        :param distance: distance to be travelled in input 'direction' in meters
        :param type: float    
        """
        x_int, y_int, ang_int = self.cur_pos
        msg = Twist()
        deg = np.arctan2(direction[0],direction[1])
        x = distance*m.sin(deg)
        y = distance*m.cos(deg)
        x_des, y_des = x_int+x, y_int+y
        # YOUR CODE HERE
        self.move_direc(self, "x", x_des)
        self.move_direc(self, "y", y_des)
        return 0

    def avoid_collison(self):
        """
        Function to determine if robot is going to collide with any obstacle
        """
        return 0

    def publish_command_velocity(self):
        """
        Function to determine linear and angular velocities to be published by infering from line parameters    
        """
        (slope, c, start, end) = self.find_wall(self.center_wrt_laser, 0.1, 100, 2, 100, 10)
        return 0

    def plotting(self, params):
        return True

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
        """
        Function to find least distance between a point and the line
        :param point: 1D array representing a point in cartesian coordinates
        :type point: numpy.ndarray
        :return: shortest distance from the point to the line in meters
        :rtype: float
        """
        if np.shape(point) != (2,):
            raise ValueError("Start point must have the shape (2,)")
        return np.linalg.norm(np.cross(self.line, self.start - point)) / self.length

    def equation(self):
        """
        Function to calculate the parameters (m,c) of equation of line "y=mx+c"
        :return (m,c): m-> slope of line; c-> 'y' intercept of the line
        :rtype: (float, float)
        """
        slope = self.line[1]/self.line[0]
        c = self.start[1] - slope*self.start[0]
        return (slope, c)

if __name__ == '__main__':
    rospy.init_node("wall_follower")
    OBJ = wall_follower()
    OBJ.main()