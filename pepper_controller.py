#!/usr/bin/python

from mail import Mail
from kinect import Kinect
from Pepper import Pepper
from xaalproxy import xAALProxy
import netifaces as ni
import time
import rospy
from std_srvs.srv import Empty
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from naoqi_bridge_msgs.msg import *
import actionlib
from naoqi import (ALBroker, ALProxy, ALModule)
import math


class Controller:
    
    def __init__(self):
        self.nao = Pepper(self) #this cmd (rospy.init_node) makes ctrl+c invalide 

    def run(self):
               
        self.nao.generateProxy()
        self.nao.postureInit()
        self.nao.say("moving moving")
        x = -2
        y = 0
        theta = 0
        direction = 0
        
        
        #self.nao.motion_service.moveToward(0, 0, direction)
        self.nao.motion_service.moveTo(0, 0, direction)
        
        self.nao.motion_service.moveTo(x, y, theta)               

if __name__== '__main__':
    c = Controller()
    #c.testNao()
   

    try:
	c.run()
    except KeyboardInterrupt:
        print "Program Quit"
