import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations

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
                            1: 'turn left',
                            2: 'follow the wall',
                            3: 'stop'}
        self.d = 0.8 #threshold_wall_dist
        self.min_point_thresh = 20
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
            return min(within_range)
        else:
            return 10
        
    def clbk_laser(self,msg):
        self.regions_ = {
            'r':  min(self.return_thres(msg.ranges[0:87]), 10),
            'fr': min(self.return_thres(msg.ranges[88:175]), 10),
            'f':  min(self.return_thres(msg.ranges[176:264]), 10),
            'fl':  min(self.return_thres(msg.ranges[265:353]), 10),
            'l':   min(self.return_thres(msg.ranges[354:435]), 10),
        }
        #print(msg)
        self.take_action()

    def change_state(self,cond,state):
        if state is not self.state_:
            print('Wall follower - [%s] [%s] - %s' % (cond, state, self.state_dict_[state]))
            self.state_ = state

    def take_action(self):
        regions = self.regions_
        if regions['f'] > self.d and regions['fl'] > self.d and regions['fr'] > self.d:
            self.change_state(0,0)
        elif regions['f'] < self.d and regions['fl'] > self.d and regions['fr'] > self.d:
            self.change_state(1,1)
        elif regions['f'] > self.d and regions['fl'] > self.d and regions['fr'] < self.d:
            self.change_state(2,2)
        elif regions['f'] > self.d and regions['fl'] < self.d and regions['fr'] > self.d:
            self.change_state(3,0)
        elif regions['f'] < self.d and regions['fl'] > self.d and regions['fr'] < self.d:
            self.change_state(4,1)
        elif regions['f'] < self.d and regions['fl'] < self.d and regions['fr'] > self.d:
            self.change_state(5,0)
        elif regions['f'] < self.d and regions['fl'] < self.d and regions['fr'] < self.d:
            if regions['l'] > self.d:
                self.change_state(6,1)
            elif regions['r'] > self.d:
                self.change_state(6,0)
            else:
                self.change_state(6,3)
        elif regions['f'] > self.d and regions['fl'] < self.d and regions['fr'] < self.d:
            self.change_state(7,2)
        else:
            rospy.loginfo(regions)

    def find_wall(self):
        msg = Twist()
        msg.linear.x = 0.2
        msg.angular.z = -0.2
        return msg

    def turn_left(self):
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
                msg = self.turn_left()
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