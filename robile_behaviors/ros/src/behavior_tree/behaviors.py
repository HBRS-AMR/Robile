#! /usr/bin/env python3

import py_trees as pt
import py_trees_ros as ptr

import rospy
import roslaunch
import rospkg
from tf.transformations import euler_from_quaternion

import std_msgs.msg
from geometry_msgs.msg import Twist, Point32
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan, Joy, PointCloud
from nav_msgs.msg import Odometry

import os
import time
import numpy as np

from behavior_tree.utils import *


class publishLines(pt.behaviour.Behaviour):

    """
    publishes line parameters (to plot lines in rviz)
    """

    def __init__(self, name="publish lines", topic_name="line_end_pts"):
        rospy.loginfo(
            "[PUBLISH LINES] __init__")

        self.topic_name = topic_name
        self.blackboard = pt.blackboard.Blackboard()

        super(publishLines, self).__init__(name)

    def setup(self, timeout):
        """
        Set up things that should be setup only for one time and which generally might 
        require time to prevent delay in tree initialisation
        """
        rospy.loginfo(
            "[PUBLISH LINES] setup")
        self.line_param_pub = rospy.Publisher(
            self.topic_name, PointCloud, queue_size=10)
        self.feedback_message = "setup"
        return True

    def update(self):
        """
        Primary function of the behavior is implemented in this method

        Publishing start and end points of all detected lines in order
        """
        rospy.loginfo(
            "[PUBLISH LINES] update")
        self.logger.debug("%s.update()" % self.__class__.__name__)

        line_parameters = self.blackboard.line_parameters
        point_cld_msg = PointCloud()
        header = std_msgs.msg.Header()
        header.seq = 0
        header.stamp = rospy.Time.now()
        header.frame_id = 'base_laser'  # frame in which the points are measured
        point_cld_msg.header = header

        # appending all end points of all detected lines
        for i in range(len(line_parameters)):
            point_cld_msg.points.append(
                Point32(line_parameters[i][-1][0][0], line_parameters[i][-1][0][1], 0.0))
            point_cld_msg.points.append(
                Point32(line_parameters[i][-1][1][0], line_parameters[i][-1][1][1], 0.0))
        self.line_param_pub.publish(point_cld_msg)
        rospy.loginfo(
            "[PUBLISH LINES] update: SUCCESS, published %0.3f lines" % len(line_parameters))

        return pt.common.Status.SUCCESS


class rotate(pt.behaviour.Behaviour):

    """
    Rotates the robot about z-axis 
    """

    def __init__(self, name="rotate platform", topic_name="/cmd_vel", to_align=False,
                 max_ang_vel=0.1, align_threshold=0.2, wall_priority_thresh=0.6):
        rospy.loginfo("[ROTATE] __init__")

        self.topic_name = topic_name
        self.blackboard = pt.blackboard.Blackboard()
        self.max_ang_vel = max_ang_vel  # units: rad/sec
        self.to_align = to_align
        self.log_start_time = True
        self.align_threshold = align_threshold
        # wall found within 'wall_priority_thresh' distance is prioritised to align with
        self.wall_priority_thresh = wall_priority_thresh
        self.alignment_status = False

        super(rotate, self).__init__(name)

    def setup(self, timeout):
        """
        Set up things that should be setup only for one time and which generally might 
        require time to prevent delay in tree initialisation
        """
        rospy.loginfo("[ROTATE] setup")
        self.cmd_vel_pub = rospy.Publisher(
            self.topic_name, Twist, queue_size=2)
        self.feedback_message = "setup"
        return True

    def initialise(self):
        """
        It will be called the first time your behaviour is ticked and anytime the status is 
        not RUNNING thereafter. 
        """
        rospy.loginfo("[ROTATE] initialise")
        if self.to_align:
            line_parameters = self.blackboard.line_parameters
            # line_parameters: [per_dist, slope, const, end_pts]
            line_parameters.sort(key=lambda x: x[0])

            angle_rad = np.arctan(line_parameters[0][1])
            if abs(line_parameters[0][1]) > 2 and self.blackboard.parent_wall_location == "right":
                if np.all(line_parameters[0][-1][:, 1] < 0):
                    # rotate right
                    angle_rad = -abs(angle_rad)
                else:
                    # rotate left
                    angle_rad = abs(angle_rad)
            if abs(line_parameters[0][1]) > 2 and self.blackboard.parent_wall_location == "left":
                if np.all(line_parameters[0][-1][:, 1] > 0):
                    # rotate left
                    angle_rad = abs(angle_rad)
                else:
                    # rotate right
                    angle_rad = -abs(angle_rad)
            self.angle_to_rotate = angle_rad
            if abs(angle_rad) < self.align_threshold:
                self.alignment_status = True
                self.blackboard.rotation_log = "[initialise] already aligned to wall"
            else:
                self.alignment_status = False
                orientation = self.blackboard.odom_data.orientation
                _, _, self.yaw_at_begining = euler_from_quaternion(
                    [orientation.x, orientation.y, orientation.z, orientation.w])
                self.blackboard.rotation_log = [
                    self.angle_to_rotate, self.yaw_at_begining]

    def update(self):
        """
        Primary function of the behavior is implemented in this method

        Rotating the robot at maximum allowed angular velocity in a given direction, 
        where if _direction_ is +1, it implies clockwise rotation, and if it is -1, it implies
        counter-clockwise rotation
        """
        rospy.loginfo("[ROTATE] update")
        self.logger.debug("%s.update()" % self.__class__.__name__)

        # send the goal
        twist_msg = Twist()
        if self.to_align:
            if self.alignment_status:
                rospy.loginfo("[ROTATE] update: SUCCESS, already aligned")
                self.blackboard.rotation_log = "[update] already aligned to wall"
                return pt.common.Status.SUCCESS

            orientation = self.blackboard.odom_data.orientation
            _, _, current_yaw = euler_from_quaternion(
                [orientation.x, orientation.y, orientation.z, orientation.w])
            twist_msg.angular.z = float(np.sign(
                self.angle_to_rotate)*self.max_ang_vel)

            # as yaw has range b/w +/-2pi, considering the edge cases
            angle_rotated = min(abs(self.yaw_at_begining-current_yaw), -abs(self.yaw_at_begining-current_yaw)%(2*np.pi))
            remaining_angle = abs(self.angle_to_rotate) - angle_rotated            

            if abs(remaining_angle) <= self.align_threshold:
                self.alignment_status = True
                self.blackboard.rotation_log = "[update] aligned successfully"
                return pt.common.Status.SUCCESS
            else:
                self.alignment_status = False
                rospy.loginfo("[ROTATE] update: RUNNING, rotating for [%0.3f radians]" %
                              remaining_angle)

        else:
            rospy.loginfo("[ROTATE] update: RUNNING, rotating for low battery:  [%0.3f]" %
                          self.blackboard.battery)
            twist_msg.angular.z = self.max_ang_vel

        self.cmd_vel_pub.publish(twist_msg)
        return pt.common.Status.RUNNING

    def terminate(self, new_status):
        """
        terminate() is trigerred once the execution of the behavior finishes, 
        i.e. when the status changes from RUNNING to SUCCESS or FAILURE
        """
        rospy.loginfo("[ROTATE] terminate: publishing zero angular velocity")
        twist_msg = Twist()
        twist_msg.angular.z = 0
        self.cmd_vel_pub.publish(twist_msg)
        return super().terminate(new_status)


class move(pt.behaviour.Behaviour):

    """
    To move the robot in a specified direction
    """

    def __init__(self, name="move platform", topic_name="/cmd_vel", towards_wall=False, along_wall=False,
                 avoid_collison=False, direction=[-1.0, 0.0], max_linear_vel=0.1, safe_dist_thresh=0.8, allowance=0.2):
        rospy.loginfo("[MOVE] __init__")

        self.topic_name = topic_name
        self.blackboard = pt.blackboard.Blackboard()
        self.blackboard.parent_wall_location = None
        self.max_linear_vel = max_linear_vel  # units: rad/sec
        self.towards_wall = towards_wall
        self.along_wall = along_wall
        self.avoid_collison = avoid_collison
        self.to_move_bool = False
        self.direction = direction
        self.safe_dist_thresh = safe_dist_thresh
        # allowance: distance allowed to deviate from wall while moving along the wall
        self.allowance = allowance

        super(move, self).__init__(name)

    def setup(self, timeout):
        """
        Set up things that should be setup only for one time and which generally might 
        require time to prevent delay in tree initialisation
        """
        rospy.loginfo("[MOVE] setup")
        self.cmd_vel_pub = rospy.Publisher(
            self.topic_name, Twist, queue_size=2)
        self.feedback_message = "setup"
        return True

    def initialise(self):
        """
        It will be called the first time your behaviour is ticked and anytime the status is 
        not RUNNING thereafter. 
        """
        rospy.loginfo("[MOVE] initialise")

        if self.towards_wall:
            # line_parameters: [per_dist, slope, const, end_pts]
            line_parameters = self.blackboard.line_parameters
            # sorting based on perpendicular distance
            line_parameters.sort(key=lambda x: x[0])

            angle_rad = np.arctan(-1/line_parameters[0][1])
            perpendicular_vector = [np.cos(angle_rad), np.sin(angle_rad)]
            self.direction = [np.sign(-line_parameters[0][2]/line_parameters[0][1])
                              * x for x in perpendicular_vector]  # to specify direction

            # move_log: [distance to move, current location]
            x_at_begining, y_at_begining = self.blackboard.odom_data.position.x, self.blackboard.odom_data.position.y
            self.dist_to_move = line_parameters[0][0]-self.safe_dist_thresh
            self.start_coordinates = np.array([x_at_begining, y_at_begining])
            self.blackboard.move_log = [
                self.dist_to_move, self.start_coordinates]
            rospy.loginfo(
                "[MOVE] initialise(towards_wall): initialised")
            if self.safe_dist_thresh > line_parameters[0][0]:
                self.to_move_bool = False
                self.blackboard.move_log = "[initialise](towards_wall) already closer to wall"
                rospy.loginfo(
                    "[MOVE] initialise(towards_wall): already closer to wall")
            else:
                self.to_move_bool = True

        if self.along_wall:
            # line_parameters: [per_dist, slope, const, end_pts]
            line_parameters = self.blackboard.line_parameters
            # sorting based on perpendicular distance
            line_parameters.sort(key=lambda x: x[0])
            # as it is called after aligning with the wall
            self.direction = [1.0, 0.0]
            if line_parameters[0][2] > 0:
                self.blackboard.parent_wall_location = "left"
            else:
                self.blackboard.parent_wall_location = "right"
            x_at_begining, y_at_begining = self.blackboard.odom_data.position.x, self.blackboard.odom_data.position.y
            self.dist_to_move = (np.linalg.norm(
                line_parameters[0][-1][0]-line_parameters[0][-1][1]))
            self.start_coordinates = np.array([x_at_begining, y_at_begining])
            self.blackboard.move_log = [
                self.dist_to_move, self.start_coordinates]
            rospy.loginfo(
                "[MOVE] initialise(along_wall): initialised")
            self.to_move_bool = True

    def update(self):
        """
        Primary function of the behavior is implemented in update method

        """
        rospy.loginfo("[MOVE] update")
        self.logger.debug("%s.update()" % self.__class__.__name__)

        # send the goal
        twist_msg = Twist()
        if self.along_wall or self.towards_wall:
            if self.blackboard.corner_detected:
                return pt.common.Status.SUCCESS
            elif self.to_move_bool:
                x_curr, y_curr = self.blackboard.odom_data.position.x, self.blackboard.odom_data.position.y
                current_coordinates = np.array([x_curr, y_curr])
                remaining_dist = self.dist_to_move - np.linalg.norm(current_coordinates-self.blackboard.move_log[1])

                if remaining_dist < 0.:
                    self.to_move_bool = False
                    rospy.loginfo(
                        "[MOVE] update: SUCCESS")
                    self.blackboard.move_log = "[update] moved successfully"
                    return pt.common.Status.SUCCESS

                if self.along_wall and self.blackboard.point_at_min_dist>self.safe_dist_thresh+self.allowance:
                    return pt.common.Status.SUCCESS

                twist_msg.linear.x = self.direction[0]*self.max_linear_vel
                twist_msg.linear.y = self.direction[1]*self.max_linear_vel
                rospy.loginfo("[MOVE] update: RUNNING, moving for [%0.3f m]" %
                              remaining_dist)

                self.cmd_vel_pub.publish(twist_msg)
                return pt.common.Status.RUNNING
            rospy.loginfo(
                "[MOVE] update: SUCCESS, already moved")

        elif self.avoid_collison:
            twist_msg.linear.x = self.blackboard.dir_to_avoid_collison[0] * \
                self.max_linear_vel
            twist_msg.linear.y = self.blackboard.dir_to_avoid_collison[1] * \
                self.max_linear_vel
            self.cmd_vel_pub.publish(twist_msg)
            rospy.loginfo("[MOVE] update: RUNNING, moving away from obstacle")
            return pt.common.Status.RUNNING
        return pt.common.Status.SUCCESS

    def terminate(self, new_status):
        """
        terminate() is trigerred once the execution of the behavior finishes, 
        i.e. when the status changes from RUNNING to SUCCESS or FAILURE
        """
        rospy.loginfo("[MOVE] terminate: publishing zero linear velocity")
        twist_msg = Twist()
        if not self.along_wall:
            twist_msg.linear.x = 0
            twist_msg.linear.y = 0
            self.cmd_vel_pub.publish(twist_msg)
        return super().terminate(new_status)


class stopMotion(pt.behaviour.Behaviour):

    """
    Stops the robot when it is controlled using joystick or by cmd_vel command
    """

    def __init__(self, name="stop platform", topic_name1="/cmd_vel", topic_name2="/joy"):
        rospy.loginfo("[STOP MOTION] __init__")

        self.cmd_vel_topic = topic_name1
        self.joy_topic = topic_name2

        super(stopMotion, self).__init__(name)

    def setup(self, timeout):
        """
        Set up things that should be setup only for one time and which generally might 
        require time to prevent delay in tree initialisation
        """
        rospy.loginfo("[STOP MOTION] setup")
        self.cmd_vel_pub = rospy.Publisher(
            self.cmd_vel_topic, Twist, queue_size=2)
        self.joy_pub = rospy.Publisher(self.joy_topic, Joy, queue_size=2)
        self.feedback_message = "setup"
        return True

    def update(self):
        """
        Primary function of the behavior is implemented in this method

        """
        rospy.loginfo("[STOP MOTION] update")
        self.logger.debug("%s.update()" % self.__class__.__name__)

        # send the goal
        twist_msg = Twist()
        twist_msg.linear.x = 0
        twist_msg.linear.y = 0
        self.cmd_vel_pub.publish(twist_msg)

        joy_msg = Joy()
        joy_msg.header.stamp = rospy.Time.now()
        joy_msg.header.frame_id = "/dev/input/js0"
        joy_msg.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        joy_msg.buttons = [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]
        self.joy_pub.publish(joy_msg)

        rospy.loginfo(
            "[STOP MOTION] update: FAILURE, publishing zero cmd_vel and dead-man's switch in joy command")
        return pt.common.Status.FAILURE

    def terminate(self, new_status):
        """
        terminate() is trigerred once the execution of the behavior finishes, 
        i.e. when the status changes from RUNNING to SUCCESS or FAILURE
        """
        rospy.loginfo(
            "[STOP MOTION] terminate: publishing zero angular velocity")
        twist_msg = Twist()
        twist_msg.linear.x = 0
        twist_msg.linear.y = 0
        twist_msg.angular.z = 0

        self.cmd_vel_pub.publish(twist_msg)
        self.sent_goal = False
        return super().terminate(new_status)


class launchNodes(pt.behaviour.Behaviour):
    """
    to run launch files
    """
    def __init__(self, name="LaunchFile", pkg="robile_navigation_demo", launch_file=None, mapping_time_out=300, mapping_dist_thresh=0.2):
        self.name = name
        info = "[LAUNCH "+name+"] __init__"
        rospy.loginfo(info)
        self.blackboard = pt.blackboard.Blackboard()
        self.pkg = pkg
        self.launch_file= launch_file
        self.mapping_time_out = mapping_time_out
        self.mapping_dist_thresh = mapping_dist_thresh
        super(launchNodes, self).__init__(name)

    def setup(self, timeout):
        """
        Set up things that should be setup only for one time and which generally might 
        require time to prevent delay in tree initialisation
        """
        info = "[LAUNCH "+self.name+"] setup"
        rospy.loginfo(info)
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        self.rospack = rospkg.RosPack()
        pkg_path = self.rospack.get_path(self.pkg)
        roslaunch.configure_logging(self.uuid)
        self.path = pkg_path+"/ros/launch/"+self.launch_file
        self.launch = roslaunch.parent.ROSLaunchParent(self.uuid, [self.path])
        self.feedback_message = "setup"
        return True

    def update(self):
        """
        Primary function of the behavior is implemented in this method
        """
        info = "[LAUNCH "+self.name+"] update"
        rospy.loginfo(info)
        if self.launch_file=="gmapping.launch":
            if self.blackboard.gmapping_status=="START":
                self.launch.start()
                self.mapping_start_time = time.time()
                self.blackboard.gmapping_status = "RUN"
                info = "[LAUNCH "+self.name+"] update: started"
                rospy.loginfo(info)
                return pt.common.Status.RUNNING
            elif self.blackboard.gmapping_status=="RUN":
                # current_coordinate = np.array([self.blackboard.odom_data.position.x, self.blackboard.odom_data.position.y])
                # dist_from_start_coord = np.linalg.norm(self.blackboard.start_coordinate-current_coordinate)
                current_time = time.time()
                time_passed = current_time-self.mapping_start_time
                if time_passed > self.mapping_time_out:
                    self.blackboard.gmapping_status = "SAVE"
                    return pt.common.Status.SUCCESS
                return pt.common.Status.RUNNING

        if self.launch_file=="amcl.launch":
            if self.blackboard.amcl_status=="START":
                self.launch.start()
                self.blackboard.amcl_status = "RUN"
                info = "[LAUNCH "+self.name+"] update: started"
                rospy.loginfo(info)

        if self.launch_file=="move_base_dwa.launch":
            if self.blackboard.move_base_dwa_status=="START":
                self.launch.start()
                self.blackboard.move_base_dwa_status = "RUN"
                info = "[LAUNCH "+self.name+"] update: started"
                rospy.loginfo(info)
                return pt.common.Status.RUNNING
            elif self.blackboard.move_base_dwa_status == "RUN":
                return pt.common.Status.RUNNING
        return pt.common.Status.SUCCESS


class saveMap(pt.behaviour.Behaviour):
    """
    save and load the saved map after 
    """
    def __init__(self, name="SaveMap", map_name="sim_env"):
        rospy.loginfo("[SAVE MAP] __init__")
        self.map_saved = False
        self.map_name = map_name
        self.blackboard = pt.blackboard.Blackboard()
        self.rospack = rospkg.RosPack()
        self.map_path = self.rospack.get_path('robile_default_env_config')+'/ros/maps/'+self.map_name
        super(saveMap, self).__init__(name)

    def update(self):
        """
        Primary function of the behavior is implemented in this method
        """
        rospy.loginfo("[SAVE MAP] update")
        if (not self.map_saved) and self.blackboard.gmapping_status=='SAVE':
            node_description = "rosrun map_server map_saver -f "+self.map_path+" --free 20 --occ 80"
            os.system(node_description)
            os.system('rosnode kill /slam_gmapping')
            os.environ['ROBOT_ENV'] = "maps/" + self.map_name
            self.map_saved = True
            self.blackboard.amcl_status = 'START' 
            self.blackboard.move_base_dwa_status = 'START'
        return pt.common.Status.SUCCESS 


class batteryStatus2bb(ptr.subscribers.ToBlackboard):

    """
    Checking battery status
    """

    def __init__(self,  name, topic_name="/mileage", threshold=30.0):
        rospy.loginfo("[BATTERY] __init__")
        super(batteryStatus2bb, self).__init__(name=name,
                                                topic_name=topic_name,
                                                topic_type=Float32,
                                                blackboard_variables={
                                                    'battery': 'data'},
                                                initialise_variables={
                                                    'battery': 0.0},
                                                clearing_policy=pt.common.ClearingPolicy.NEVER
                                                )
        self.blackboard = pt.blackboard.Blackboard()
        self.blackboard.battery_low_warning = False
        self.threshold = threshold
       

    def update(self):
        """
        Primary function of the behavior is implemented in this method

        Call the parent to write the raw data to the blackboard and then check against the
        threshold to determine if the low warning flag should also be updated.
        """
        rospy.loginfo('[BATTERY] update')
        self.logger.debug("%s.update()" % self.__class__.__name__)
        status = super(batteryStatus2bb, self).update()

        if status != pt.common.Status.RUNNING:
            if self.blackboard.battery < self.threshold:
                rospy.loginfo('[BATTERY] update: lesser than threshold')
                self.blackboard.battery_low_warning = True
                rospy.logwarn_throttle(
                    30, "%s: battery level is low!" % self.name)
            else:
                rospy.loginfo('[BATTERY] update: greater than threshold')
                self.blackboard.battery_low_warning = False

            self.feedback_message = "Battery level is low" if self.blackboard.battery_low_warning else "Battery level is ok"
        return status


class laserScanFiltered2bb(ptr.subscribers.ToBlackboard):

    """
    Checking filtered laser_scan to avoid possible collison
    """

    def __init__(self, name, topic_name="/scan_filtered", safe_range=0.4):
        rospy.loginfo("[LASER SCAN] __init__")
        super(laserScanFiltered2bb, self).__init__(name=name,
                                                     topic_name=topic_name,
                                                     topic_type=LaserScan,
                                                     blackboard_variables={
                                                         'laser_scan': 'ranges', 'max_angle': 'angle_max', 'min_angle': 'angle_min',
                                                         'max_range': 'range_max', 'min_range': 'range_min'},
                                                     # to dictate when data should be cleared/reset.
                                                     clearing_policy=pt.common.ClearingPolicy.NEVER
                                                     )
        self.blackboard = pt.blackboard.Blackboard()
        self.blackboard.collison_warning = False
        self.safe_min_range = safe_range
        self.blackboard.point_at_min_dist = 0.0

    def update(self):
        """
        Primary function of the behavior is implemented in this method

        Call the parent to write the raw data to the blackboard and then check against the
        threshold to set the warning if the robot is close to any obstacle.
        """
        rospy.loginfo(
            "[LASER SCAN] update")
        self.logger.debug("%s.update()" % self.__class__.__name__)
        status = super(laserScanFiltered2bb, self).update()

        if status != pt.common.Status.RUNNING:
            # splitting data into segments to handle possible noises
            segment_size = 20
            scan_data_np_array = np.array(self.blackboard.laser_scan)
            self.blackboard.laser_scan = "Processed(laserScanFiltered2bb)"
            # laser scanner points which are outside the min and max range are assigned zero distance
            scan_data_np_array[scan_data_np_array == 0.0] = 20.0
            scan_data_np_array = scan_data_np_array.reshape(segment_size, -1)
            scan_data_segment_mean = np.mean(scan_data_np_array, axis=1)
            self.blackboard.point_at_min_dist = np.min(scan_data_segment_mean)

            if self.blackboard.point_at_min_dist < self.safe_min_range:
                rospy.loginfo("[LASER SCAN] update: possible collison detected at [%0.3f meters]" %
                              self.blackboard.point_at_min_dist)
                self.blackboard.collison_warning = True
                segment_ang_step = (
                    self.blackboard.max_angle-self.blackboard.min_angle)/segment_size
                dir_ang_rad = np.pi+self.blackboard.min_angle + \
                    ((np.argmin(scan_data_segment_mean)+0.5))*(segment_ang_step)
                self.blackboard.dir_to_avoid_collison = [
                    np.cos(dir_ang_rad), np.sin(dir_ang_rad)]

                rospy.logwarn_throttle(
                    30, "%s: possible collison detected!" % self.name)
            else:
                rospy.loginfo("[LASER SCAN] update: no collison detected")
                self.blackboard.collison_warning = False

            self.feedback_message = "Possible collison detected!" if self.blackboard.collison_warning else "Collison status: free to move"

        return status


class odom2bb(ptr.subscribers.ToBlackboard):

    """
    Saving odometry data in blackboard
    """

    def __init__(self, name, topic_name="/odom"):
        rospy.loginfo("[ODOM] __init__")
        super(odom2bb, self).__init__(name=name,
                                      topic_name=topic_name,
                                      topic_type=Odometry,
                                      blackboard_variables={
                                          'odom_data': 'pose.pose'},
                                      clearing_policy=pt.common.ClearingPolicy.NEVER
                                      )

        self.blackboard = pt.blackboard.Blackboard()
        self.save_initial_pose = True
        # initialising navigation launch file status
        self.blackboard.gmapping_status = 'STOP'
        self.blackboard.amcl_status = 'STOP'
        self.blackboard.move_base_dwa_status = 'STOP'        

    def update(self):
        """
        Collecting odometry data of the robot
        """
        rospy.loginfo("[ODOM] update")
        self.logger.debug("%s.update()" % self.__class__.__name__)
        status_subscribe = super(odom2bb, self).update()
        if self.save_initial_pose:
            # self.blackboard.start_coordinate = np.array([self.blackboard.odom_data.position.x, self.blackboard.odom_data.position.y])
            self.blackboard.gmapping_status = 'START'
            self.save_initial_pose = False
        return status_subscribe


class gridOccupancyStatus2bb(ptr.subscribers.ToBlackboard):

    """
    Determining whether the grid consists of sufficient laser scan points
    """

    def __init__(self, name, topic_name="/scan_filtered", min_points=100, grid_size_x=1, grid_size_y=1.4):
        rospy.loginfo("[GRID STATUS] __init__")
        super(gridOccupancyStatus2bb, self).__init__(name=name,
                                                       topic_name=topic_name,
                                                       topic_type=LaserScan,
                                                       blackboard_variables={
                                                           'laser_scan_grid': 'ranges', 'max_angle': 'angle_max', 'min_angle': 'angle_min'},
                                                       clearing_policy=pt.common.ClearingPolicy.NEVER
                                                       )
        self.blackboard = pt.blackboard.Blackboard()
        self.blackboard.grid_occupied = False
        self.blackboard.grid_occupancy_count = 0
        # minimum points to consider the grid to be filled for wall detection
        self.min_points = min_points
        self.grid_size_x = grid_size_x
        self.grid_size_y = grid_size_y

    def update(self):
        """
        Collecting parameters of line associated with each wall
        """
        rospy.loginfo(
            "[GRID STATUS] update")
        self.logger.debug("%s.update()" % self.__class__.__name__)
        status = super(gridOccupancyStatus2bb, self).update()

        if status != pt.common.Status.RUNNING:
            laser_scan_array = np.array(self.blackboard.laser_scan_grid)
            self.blackboard.laser_scan_grid = "Processed(gridOccupancyStatus2bb)"
            angles = np.linspace(
                self.blackboard.min_angle, self.blackboard.max_angle, laser_scan_array.shape[0])
            laser_sub_cartesian = np_polar2cart(
                np.array([laser_scan_array, angles]).T)
            inliers = abs(laser_sub_cartesian) < np.array(
                [self.grid_size_x, self.grid_size_y/2])
            inlier_count = inliers[~np.any(
                inliers == False, axis=1), :].shape[0]

            self.blackboard.grid_occupancy_count = inlier_count
            if inlier_count > self.min_points:
                self.blackboard.grid_occupied = True
            else:
                self.blackboard.grid_occupied = False
            rospy.loginfo(
                "[GRID STATUS] update: number of points found are [%0.3f]" % inlier_count)
            self.feedback_message = "Sufficient points detected in the grid" if inlier_count > self.min_points else "Not sufficient points detected in the grid"

        return status


class wallParamRANSAC2bb(ptr.subscribers.ToBlackboard):

    """
    Collecting parameters of line associated with each wall using RANSAC on scan filtered data
    """

    def __init__(self, name, topic_name="/scan_filtered", dist_thresh=0.1, iterations=15, thresh_count=15, sigma=1, rf_max_pts=5, wall_priority_thresh=0.8):
        rospy.loginfo(
            "[WALL PARAMETERS RANSAC] __init__")
        super(wallParamRANSAC2bb, self).__init__(name=name,
                                                   topic_name=topic_name,
                                                   topic_type=LaserScan,
                                                   blackboard_variables={
                                                       'laser_scan_ransac': 'ranges', 'max_angle': 'angle_max', 'min_angle': 'angle_min',
                                                       'max_range': 'range_max', 'min_range': 'range_min'},
                                                   clearing_policy=pt.common.ClearingPolicy.NEVER
                                                   )
        self.blackboard = pt.blackboard.Blackboard()
        # The parameter used to determine the furthest a point can be from the line and still be considered an inlier
        self.dist_thresh = dist_thresh
        # k: The number of iterations the RANSAC algorithm will run for
        self.iterations = iterations
        # number of minimum points required to form a line
        self.thresh_count = thresh_count
        # maximum number of points allowed in a cluster (used in reduction filter)
        self.rf_max_pts = rf_max_pts
        # maximum distance between two consecutive points to consider them (used in reduction filter)
        self.sigma = sigma
        # wall found within 'wall_priority_thresh' distance is prioritised to align with
        self.wall_priority_thresh = wall_priority_thresh
        self.blackboard.corner_detected = False        

    def update(self):
        """
        Collecting parameters of line associated with each wall
        """
        rospy.loginfo(
            "[WALL PARAMETERS RANSAC] update")
        self.logger.debug("%s.update()" % self.__class__.__name__)
        status_subscribe = super(wallParamRANSAC2bb, self).update()

        if status_subscribe != pt.common.Status.RUNNING:
            if self.blackboard.grid_occupied:
                rospy.loginfo(
                    '[WALL PARAMETERS RANSAC] update: SUCCESS, grids are occupied, grid occupancy status will be used')
                self.blackboard.laser_scan_ransac = "Occupied grid cells are considered"
                return pt.common.Status.FAILURE
                #  return status_subscribe
            else:
                self.blackboard.line_parameters = []
                laser_sub_cartesian = process_data(range_data=np.array(self.blackboard.laser_scan_ransac), max_angle=self.blackboard.max_angle,
                                                   min_angle=self.blackboard.min_angle, max_range=self.blackboard.max_range, min_range=self.blackboard.min_range,
                                                   sigma=self.sigma, rf_max_pts=self.rf_max_pts, reduce_bool=True)
                self.blackboard.laser_scan_ransac = "Processed(wallParamRANSAC2bb)"
                points = [point for point in laser_sub_cartesian]
                self.blackboard.line_parameters = RANSAC_get_line_params(
                    points, self.dist_thresh, self.iterations, self.thresh_count)

                # detecting the corner and discarding parent wall
                line_parameters = self.blackboard.line_parameters
                line_parameters.sort(key=lambda x: x[0])                
                for line_param in line_parameters:
                    if abs(line_param[1]) > 2 and line_param[0] < self.wall_priority_thresh:
                        self.blackboard.line_parameters = [line_param]
                        self.blackboard.corner_detected = True
                        rospy.loginfo(
                            "[ROTATE] initialise: Corner detected")
                        break
                    self.blackboard.corner_detected = False

                self.blackboard.num_walls = len(
                    self.blackboard.line_parameters)

                rospy.loginfo("[WALL PARAMETERS RANSAC] update: SUCCESS,number of walls found are [%0.3f]" % len(
                    self.blackboard.line_parameters))
                self.feedback_message = "Line parameters collection successful (RANSAC)" if self.blackboard.line_parameters else "Line parameters not updated (RANSAC)"

        return status_subscribe


class wallParamGrid2bb(ptr.subscribers.ToBlackboard):

    """
    Collecting parameters of line associated with each wall using occupied cells from grid, by optionally using RANSAC or online line extraction algorithm
    """

    def __init__(self, name, topic_name="/scan_filtered", grid_size_x=1, grid_size_y=1.4, grid_width=0.04,
                 occupancy_thresh=0.1, allowed_deviation=0.45, incr=0.01, max_dist=0.1, min_points=5,
                 sigma=1, rf_max_pts=5, iterations=10, algorithm="ransac", wall_priority_thresh=0.8):
        rospy.loginfo(
            "[WALL PARAMETERS Grid] __init__")
        super(wallParamGrid2bb, self).__init__(name=name,
                                                 topic_name=topic_name,
                                                 topic_type=LaserScan,
                                                 blackboard_variables={
                                                     'laser_scan_grid': 'ranges', 'max_angle': 'angle_max',
                                                     'min_angle': 'angle_min', 'max_range': 'range_max', 'min_range': 'range_min'},
                                                 clearing_policy=pt.common.ClearingPolicy.NEVER
                                                 )

        self.blackboard = pt.blackboard.Blackboard()
        self.grid_size_x = grid_size_x
        self.grid_size_y = grid_size_y
        self.grid_width = grid_width
        self.algorithm = algorithm

        # assigned to an array of probability of occupancy of individual grid cells
        self.occupancy_thresh = occupancy_thresh

        # allowed fraction of deviation of sum of distances between consecutive points and
        # (Online) the total length  of the line
        self.e = allowed_deviation
        # (Online) increment in the error values with the number of points
        self.incr = incr
        # (Online) maximum distance between consecutive points allowed in a line segment
        self.max_dist = max_dist
        # (Online) minimum number of points required in a line segment
        self.k = min_points
        # (RANSAC) The parameter used to determine the furthest a point can be from the line and still be considered an inlier
        self.dist_thresh = 3*grid_width
        # (RANSAC) The number of iterations the RANSAC algorithm will run for
        self.iterations = iterations
        # (RANSAC) number of minimum points required to form a line
        self.thresh_count = min_points*2
        # maximum number of points allowed in a cluster (used in reduction filter)
        self.rf_max_pts = rf_max_pts
        # maximum distance between two consecutive points to consider them (used in reduction filter)
        self.sigma = sigma
        # wall found within 'wall_priority_thresh' distance is prioritised to align with
        self.wall_priority_thresh = wall_priority_thresh
        self.blackboard.corner_detected = False

    def update(self):
        """
        Collecting parameters of line associated with each wall
        """
        rospy.loginfo(
            "[WALL PARAMETERS Grid] update")
        self.logger.debug("%s.update()" % self.__class__.__name__)
        status_subscribe = super(wallParamGrid2bb, self).update()

        if status_subscribe != pt.common.Status.RUNNING:

            if self.blackboard.grid_occupied:
                self.blackboard.line_parameters = []

                # TEST: use raw data in cartesian form instead of using reduced data
                laser_sub_cartesian = process_data(range_data=np.array(self.blackboard.laser_scan_grid), max_angle=self.blackboard.max_angle,
                                                   min_angle=self.blackboard.min_angle, max_range=self.blackboard.max_range, min_range=self.blackboard.min_range,
                                                   sigma=self.sigma, rf_max_pts=self.rf_max_pts, reduce_bool=False)

                # get probability of cell being occupied
                grid_occupancy_probability = np.round(get_occupancy_probability(
                    laser_sub_cartesian, self.grid_size_x, self.grid_size_y, self.grid_width), 2)

                # get coordinates of occupied cells
                array_of_occupied_cells = get_array_of_occupied_cells(
                    grid_occupancy_probability, self.occupancy_thresh, self.grid_size_x, self.grid_size_y, self.grid_width)

                if self.algorithm == 'online':
                    # get line parameters from the occupied cell coordinates
                    self.blackboard.laser_scan_grid = "Processed(wallParamGrid2bb)-Online"
                    self.blackboard.line_parameters = online_get_line_params(
                        array_of_occupied_cells, self.e, self.incr, self.max_dist, self.k)
                if self.algorithm == 'ransac':
                    self.blackboard.laser_scan_grid = "Processed(wallParamGrid2bb)-RANSAC"
                    rospy.loginfo(
                        "[WALL PARAMETERS Grid] update: using RANSAC")
                    # get line parameters from the occupied cell coordinates
                    points = [point for point in array_of_occupied_cells]
                    self.blackboard.line_parameters = RANSAC_get_line_params(
                        points, self.dist_thresh, self.iterations, self.thresh_count)
                    if len(self.blackboard.line_parameters) == 0:
                        self.blackboard.laser_scan_grid = "Processed(wallParamGrid2bb)-Online"
                        rospy.loginfo(
                            "[WALL PARAMETERS Grid] update: using Online line detection")
                        # get line parameters from the occupied cell coordinates
                        self.blackboard.line_parameters = online_get_line_params(
                            array_of_occupied_cells, self.e, self.incr, self.max_dist, self.k)

                # detecting the corner and discarding parent wall
                line_parameters = self.blackboard.line_parameters
                line_parameters.sort(key=lambda x: x[0])                
                for line_param in line_parameters:
                    if abs(line_param[1]) > 2 and line_param[0] < self.wall_priority_thresh:
                        self.blackboard.line_parameters = [line_param]
                        self.blackboard.corner_detected = True
                        break
                    self.blackboard.corner_detected = False

                self.blackboard.num_walls = len(
                    self.blackboard.line_parameters)

                rospy.loginfo("[WALL PARAMETERS Grid] update: SUCCESS, number of walls found are [%0.3f]" % len(
                    self.blackboard.line_parameters))
                self.feedback_message = "Line parameters collection successful" if self.blackboard.line_parameters else "Line parameters not updated"
                return pt.common.Status.SUCCESS
            else:
                self.blackboard.laser_scan_grid = "RANSAC is being used"
                rospy.loginfo(
                    '[WALL PARAMETERS Grid] update: SUCCESS, grids NOT occupied, using RANSAC')
                return pt.common.Status.FAILURE

        else:
            return status_subscribe
