import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import nunpy as np

class wall_follower:
    def __init__(self):
        print("Node initiated..............")
        self.wside = None
        self.nwside = None
        self.regions_ = {'r': 0,
                         'fr': 0,
                         'f': 0,
                         'fl': 0,
                         'l': 0}
        self.state_ = -1
        self.state_dict_ = {0: 'find the wall',
                            1: 'turn',
                            2: 'follow the wall',
                            3: 'stop',
                            4: "nturn"}
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
            'r':  self.return_thres(msg.ranges[0:87]),
            'fr': self.return_thres(msg.ranges[87:174]),
            'f':  self.return_thres(msg.ranges[174:263]),
            'fl':  self.return_thres(msg.ranges[263:350]),
            'l':   self.return_thres(msg.ranges[350:437])}
        #print(msg)
        self.take_action()

    def change_state(self,cond,state):
        if state is not self.state_:
            print('Wall follower - [%s] [%s] - %s' % (cond, state, self.state_dict_[state]))
            self.state_ = state

    def take_action(self):
        regions = self.regions_
        if self.wside == None or self.nwside ==None:
            if regions['fl'] < regions['fr']:
                self.wside = 'fl'
                self.nwside = 'fr'
            elif regions['fl'] > regions['fr']:
                self.nwside = 'fl'
                self.wside = 'fr'
            else:
                self.nwside = 'fl'
                self.wside = 'fr'
            
            if self.wside == None or self.nwside ==None:
                self.take_action()
            else:
                self.state_ = 0
        else:
            cond = tuple(1*(np.array([regions['f'], regions[self.nwside], regions[self.wside]]) < self.d))
            conds ={(0,0,0): (0,0), #f,nw,w
                    (0,0,1): (1,2),
                    (0,1,0): (2,0),
                    (0,1,1): (3,2),
                    (1,0,0): (4,1),
                    (1,0,1): (5,2),
                    (1,1,0): (6,4),
                    (1,1,1): (7,self.find_escape())}
            if cond in conds:
                con, new_state = conds[cond]
                self.change_state(con,new_state)
            else:
                rospy.loginfo(regions)
                
    def find_escape(self):
        regions = self.regions_
        if regions[self.nwside[-1]] > self.d and regions[self.wside[-1]] > self.d:
            return 1
        elif regions[self.nwside[-1]] > self.d and regions[self.wside[-1]] < self.d:
            return 1
        elif regions[self.nwside[-1]] < self.d and regions[self.wside[-1]] > self.d:
            return 4
        else:
            return 3

    def find_wall(self):
        msg = Twist()
        if self.wside == "fr":
            msg.linear.x = 0.2
            msg.angular.z = -0.2
        else:
            msg.linear.x = 0.2
            msg.angular.z = 0.2
        return msg

    def turn(self):
        msg = Twist()
        if self.wside == "fr":
            msg.angular.z = 0.5
        else:
            msg.angular.z = -0.5
        return msg
    
    def nturn(self):
        msg = Twist()
        if self.wside == "fr":
            msg.angular.z = -0.5
        else:
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
            elif self.state_ == 4:
                msg = self.nturn()
            else:
                rospy.logerr('Unknown state!')
            self.command_pub.publish(msg)
            self.loop_rate.sleep()

if __name__ == '__main__':
    rospy.init_node("wall_follower")
    OBJ = wall_follower()
    OBJ.main()