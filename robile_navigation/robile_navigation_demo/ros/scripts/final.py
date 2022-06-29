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


def clbk_laser(msg):
    global regions_
    regions_ = {
        'r':  min(min(msg.ranges[0:87]), 10),
        'fr': min(min(msg.ranges[88:175]), 10),
        'f':  min(min(msg.ranges[176:264]), 10),
        'fl':  min(min(msg.ranges[265:353]), 10),
        'l':   min(min(msg.ranges[354:435]), 10),
    }
    print(msg)
    take_action()


def change_state(state):
    global state_, state_dict_
    if state is not state_:
        print('Wall follower - [%s] - %s' % (state, state_dict_[state]))
        state_ = state


def take_action():
    global regions_
    regions = regions_
    d = 0.7
    if regions['f'] > d and regions['fl'] > d and regions['fr'] > d:
        change_state(0)
    elif regions['f'] < d and regions['fl'] > d and regions['fr'] > d:
        change_state(1)
    elif regions['f'] > d and regions['fl'] > d and regions['fr'] < d:
        change_state(2)
    elif regions['f'] > d and regions['fl'] < d and regions['fr'] > d:
        change_state(0)
    elif regions['f'] < d and regions['fl'] > d and regions['fr'] < d:
        change_state(1)
    elif regions['f'] < d and regions['fl'] < d and regions['fr'] > d:
        change_state(0)
    elif regions['f'] < d and regions['fl'] < d and regions['fr'] < d:
        change_state(3)
    elif regions['f'] > d and regions['fl'] < d and regions['fr'] < d:
        change_state(2)
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
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/scan_filtered', LaserScan, clbk_laser)
    rate = rospy.Rate(20)
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
