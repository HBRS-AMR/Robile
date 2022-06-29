import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
import math

#global variables
pub_ = None
regions_ = {
    'r': 0,
    'fr': 0,
    'f': 0,
    'fl': 0,
    'l': 0,
}
state_ = 0
state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
    3: 'stop'
}

def return_thres(array, n=20, d=0.8):
    within_range = []
    x=0
    for i in array:
        if i<d:
            within_range.append(i)
            x=x+1
    #print(x)
    if x>n:
        return min(within_range)
    else:
        return 10
    
def clbk_laser(msg):
    global regions_
    regions_ = {
        'r':  min(return_thres(msg.ranges[0:87]), 10),
        'fr': min(return_thres(msg.ranges[88:175]), 10),
        'f':  min(return_thres(msg.ranges[176:264]), 10),
        'fl':  min(return_thres(msg.ranges[265:353]), 10),
        'l':   min(return_thres(msg.ranges[354:435]), 10),
    }
    #print(msg)
    take_action()


def change_state(cond,state):
    global state_, state_dict_
    if state is not state_:
        print('Wall follower - [%s] [%s] - %s' % (cond, state, state_dict_[state]))
        state_ = state


def take_action():
    global regions_
    regions = regions_
    msg = Twist()
    d = 0.8
    if regions['f'] > d and regions['fl'] > d and regions['fr'] > d:
        change_state(0,0)
    elif regions['f'] < d and regions['fl'] > d and regions['fr'] > d:
        change_state(1,1)
    elif regions['f'] > d and regions['fl'] > d and regions['fr'] < d:
        change_state(2,2)
    elif regions['f'] > d and regions['fl'] < d and regions['fr'] > d:
        change_state(3,0)
    elif regions['f'] < d and regions['fl'] > d and regions['fr'] < d:
        change_state(4,1)
    elif regions['f'] < d and regions['fl'] < d and regions['fr'] > d:
        change_state(5,0)
    elif regions['f'] < d and regions['fl'] < d and regions['fr'] < d:
        if regions['l'] > d:
            change_state(6,1)
        elif regions['r'] > d:
            change_state(6,0)
    elif regions['f'] > d and regions['fl'] < d and regions['fr'] < d:
        change_state(7,2)
    else:
        rospy.loginfo(regions)


def find_wall():
    msg = Twist()
    msg.linear.x = 0.2
    msg.angular.z = -0.2
    return msg

def turn_left():
    msg = Twist()
    msg.angular.z = 0.5
    return msg

def follow_the_wall():
    global regions_
    
    msg = Twist()
    msg.linear.x = 0.2
    return msg

def stop():
    msg = Twist()
    msg.linear.x = 0
    msg.linear.x = 0
    msg.angular.z = 0
    return msg


def main():
    global pub_
    rospy.init_node('reading_laser')
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/scan_filtered', LaserScan, clbk_laser)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        msg = Twist()
        if state_ == 0:
            msg = find_wall()
            #print("Finding The wall")
        elif state_ == 1:
            msg = turn_left()
            #print("Turning Left")
        elif state_ == 2:
            msg = follow_the_wall()
            #print("Following the wall")
        elif state_ == 3:
            #print("Stopping")
            msg = stop()
        else:
            rospy.logerr('Unknown state!')
        pub_.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    main()