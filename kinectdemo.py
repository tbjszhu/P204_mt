#!/usr/bin/python

import rospy
import tf
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('tf_lookup_example')
    listener = tf.TransformListener()
    rate = rospy.Rate(0.2)
    print "start"

    while not rospy.is_shutdown():
        try:
            (trans1, rot1) = listener.lookupTransform('/openni_depth_frame', '/neck_1', rospy.Time(0))
            (trans2, rot2) = listener.lookupTransform('/openni_depth_frame', '/left_hip_1', rospy.Time(0))
            (trans3, rot3) = listener.lookupTransform('/openni_depth_frame', '/head_1', rospy.Time(0))

            print "distance z between neck and left hip : "
            print trans1[2] - trans2[2]
            print "distance %f, direction %f, height %f" % (trans3[0] , trans3[1], trans3[2])
            print "=================="
        
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()
