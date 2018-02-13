#!/usr/bin/python

import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

if __name__=="__main__":
    p = rospy.Publisher('/cmd_vel', Twist)

    rospy.init_node('move')

    stif_enable = rospy.ServiceProxy('/body_stiffness/enable', Empty)
    stif_enable()
    twist = Twist()
    twist.linear.x = 1

    rospy.loginfo("About to be moving forward!")
    for i in range(20):
        p.publish(twist)
        rospy.sleep(0.5)

    rospy.loginfo("Stopping!")
    twist = Twist()
    p.publish(twist)
    
