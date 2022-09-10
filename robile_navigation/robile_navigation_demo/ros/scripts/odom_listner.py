 #!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry

def odom_callback(msg):
    x_pos = msg.pose.pose.position.x
    y_pos = msg.pose.pose.position.y
    z_ort = msg.pose.pose.orientation.z
    global pos_values
    pos_values = (x_pos, y_pos, z_ort)
    
def listener():


    # run simultaneously.
    rospy.init_node('odom_listener', anonymous=True)

    rospy.Subscriber('odom', Odometry, odom_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
     listener()