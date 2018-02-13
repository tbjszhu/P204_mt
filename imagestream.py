#!/usr/bin/python
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def callback(data):
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

def imagestream():
    rospy.init_node('image_stream', anonymous=True)
    image_sub = rospy.Subscriber('/nao_vision', Image, callback)
    rospy.spin()
    

if __name__ == '__main__':
    try:
        imagestream();
    except rospy.ROSInterruptException:
        pass
    cv2.destroyAllWindows()
