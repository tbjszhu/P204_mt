#!/usr/bin/python

import rospy
from std_msgs.msg import String

def dialog():
    pub = rospy.Publisher('/speech', String, queue_size=10)
    rospy.init_node('dialog', anonymous=True)
    check_str = "bonjour, comment allez vous"
    rate = rospy.Rate(0.2)
    while not rospy.is_shutdown():
        rospy.loginfo(check_str)
        pub.publish(check_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        dialog()
    except rospy.ROSInterruptException:
        pass
