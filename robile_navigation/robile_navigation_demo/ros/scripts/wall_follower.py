#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
class wall_follower:
    def __init__(self):
        print("Node initiated..............")
        self.regions_ = {'r': 0,
                         'fr': 0,
                         'f': 0,
                         'fl': 0,
                         'l': 0}
        self.state_ = 0
        self.state_dict_ = {0: 'find the wall',
                            1: 'turn',
                            2: 'follow the wall',
                            3: 'stop'}
        self.wside = None
        self.nwside = None
        self.d = 0.9 #threshold_wall_dist
        self.min_point_thresh = 20
        self.wall_found = False
        self.command_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/scan_filtered', LaserScan, self.clbk_laser)
        self.loop_rate = rospy.Rate(1)
        self.loop_rate.sleep()
        
    def return_thres(self, array):
        within_range = []
        x=0
        for i in array:
            if i<self.d:
                within_range.append(i)
                x=x+1
        #print(x)
        if x>self.min_point_thresh:
            return [1,min(within_range)]
        else:
            return [0,min(within_range)]
        
    def clbk_laser(self,msg):
        self.regions_ = {'lr': self.return_thres(msg.ranges[0:87]),
                         'r': self.return_thres(msg.ranges[0:128]),
                         'fr': self.return_thres(msg.ranges[88:173]),
                         'f': self.return_thres(msg.ranges[129:309]),
                         'fl': self.return_thres(msg.ranges[264:350]),
                         'l': self.return_thres(msg.ranges[309:437]),
                         'll': self.return_thres(msg.ranges[354:437])}
        #print(msg)
        self.take_action()

    def change_state(self,cond,state):
        if state is not self.state_:
            print('Wall follower - [%s] [%s] - %s' % (cond, state, self.state_dict_[state]))
            self.state_ = state

    def take_action(self):
        regions = self.regions_
        if self.wside == None or self.nwside ==None:
            if regions['f'][0] == 0 and regions[self.nwside][0] == 0 and regions[self.wside][0] == 0:
                self.change_state(0,0)
        else:
            if regions['f'][0] == 0 and regions[self.nwside][0] == 0 and regions[self.wside][0] == 0:
                self.change_state(0,0)
            elif regions['f'][0] == 1 and regions[self.nwside][0] == 0 and regions[self.wside][0] == 0:
                self.change_state(1,1)
            elif regions['f'][0] == 0 and regions[self.nwside][0] == 0 and regions[self.wside][0] == 1:
                self.change_state(2,2)
            elif regions['f'][0] == 0 and regions[self.nwside][0] == 1 and regions[self.wside][0] == 0:
                self.change_state(3,0)
            elif regions['f'][0] == 1 and regions[self.nwside][0] == 0 and regions[self.wside][0] == 1:
                self.change_state(4,1)
            elif regions['f'][0] == 1 and regions[self.nwside][0] == 1 and regions[self.wside][0] == 0:
                self.change_state(5,0)
            elif regions['f'][0] == 1 and regions[self.nwside][0] == 1 and regions[self.wside][0] == 1:
                if regions['ll'][0] == 0:
                    self.change_state(6,1)
                elif regions['lr'][0] == 0:
                    self.change_state(6,0)
                else:
                    self.change_state(6,3)
            elif regions['f'][0] == 0 and regions[self.nwside][0] == 1 and regions[self.wside][0] == 1:
                self.change_state(7,2)
            else:
                rospy.loginfo(regions)

    def find_wall(self):
        regions = self.regions_
        distances = np.array([regions['fl'][1],regions['l'][1],regions['r'][1],regions['fr'][1]])
        ind = np.where(distances == distances.min())
        msg = Twist()
        if ind==0 or ind==1:
            if self.wside == None or self.nwside ==None:
                self.wside = "l"
                self.nwside = "r"
            msg.linear.x = 0.2
            msg.angular.z = 0.2
        else:
            if self.wside == None or self.nwside ==None:
                self.wside = "r"
                self.nwside = "l"
            msg.linear.x = 0.2
            msg.angular.z = -0.2
        return msg
    
    def turn(self):
        if self.wside == "l":
            msg = Twist()
            msg.angular.z = -0.5
        else:
            msg = Twist()
            msg.angular.z = 0.5
        return msg

    def follow_the_wall(self):
        msg = Twist()
        msg.linear.x = 0.2
        return msg

    def stop(self):
        msg = Twist()
        msg.linear.x = 0
        msg.angular.z = 0
        return msg

    def main(self):
        rospy.loginfo("Ready to start now...")
        while not rospy.is_shutdown():
            msg = Twist()
            if self.state_ == 0:
                msg = self.find_wall()
            elif self.state_ == 1:
                msg = self.turn()
            elif self.state_ == 2:
                msg = self.follow_the_wall()
            elif self.state_ == 3:
                msg = self.stop()
            else:
                rospy.logerr('Unknown state!')
            self.command_pub.publish(msg)
            self.loop_rate.sleep()

if __name__ == '__main__':
    rospy.init_node("wall_follower")
    OBJ = wall_follower()
    OBJ.main()
