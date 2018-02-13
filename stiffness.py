#!/usr/bin/python

import rospy
from std_srvs.srv import Empty

def stiffness():
    rospy.init_node('stiffness', anonymous=True)
    rospy.wait_for_service('/body_stiffness/enable')
    print "after wait for service"
    try:
        stif_enable = rospy.ServiceProxy('/body_stiffness/enable', Empty)
        stif_disable = rospy.ServiceProxy('/body_stiffness/disable', Empty)
        stif_disable()
        s = raw_input("after stiffness disable")
        stif_enable()
        s = raw_input("after stiffness enable")
        
        alife_enable = rospy.ServiceProxy('/nao_alife/solitary', Empty)
        alife_disable = rospy.ServiceProxy('/nao_alife/disabled', Empty)
        
        alife_enable()
        s = raw_input("after alife enable") 
        alife_disable()
        s = raw_input("after alife disable") 
        
    except rospy.ServiceException, e:
        print "Service call failed : %s" %e

if __name__ == '__main__':
    try:
        stiffness()
    except rospy.ROSInterruptException:
        pass
