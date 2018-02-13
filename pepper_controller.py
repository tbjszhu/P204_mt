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
from sensfloor import Sensfloor
import threading

class Controller:
    
    def __init__(self):
        self.nao = Pepper(self) #this cmd (rospy.init_node) makes ctrl+c invalide 
        self.kinect = Kinect()
        self.xaalproxy = xAALProxy()
        self.sensfloor = Sensfloor()
        self.threadlist = []

    def run(self):
               
        self.nao.generateProxy()
        self.nao.postureInit()
        self.nao.say("moving moving")
        distance = 3.606245
        direction = 1
        #self.nao.motion_service.moveToward(0, 0, direction)
      #  self.nao.moveToPerson(0, 0, direction)
        self.nao.moveToPerson(distance, 0, 0)               
        if False:        
            self.nao.searchFace()
        
    def waitPersonResponse(self, timeout):
        count = 0
        self.nao.resetVerifyState()
        while not self.nao.getVerifyState():
            time.sleep(3)
            count += 1
            if (timeout!=0) and (count > timeout) : 
                print "waitpersonresponse timeout!"
                return False
        print "getpersonresponse"
        return self.nao.getVerifyState()%2
    
    def sendMail(self):
        link = self.webRTCaddress()
        topic = "nao_robot/camera/top/camera/image_raw"
        text = "The assistance robot and smart house has detected a fall. To check, please see the video from this address " + link
        text = text + "/stream_viewer?topic=" + topic
        mail = Mail(text)
        mail.sendMail()

    def webRTCaddress(self):
        ni.ifaddresses('wlan0')
        ip = ni.ifaddresses('wlan0')[2][0]['addr'] + ":8080"
        print "HostIP : ", ip
        return ip

    def scenario(self):
        self.smartDeviceAction("shutterleft", "up")
        self.smartDeviceAction("shutterright", "up")
        self.smartDeviceAction("lamp1", "on")
        self.smartDeviceAction("lamp2", "on")
        self.smartDeviceAction("lamp3", "on")
        print "lampes allumeees"
        self.nao.say("I turn off the stove and open the window shades")
        self.smartDeviceAction("switch", "off")
        self.nao.say("I turn the lights on")
        print "porte ouverte"
        try:
            self.sendMail()
            self.nao.say("I sent an email to your contact list")
        except:
            print "mail send failed"
        self.smartDeviceAction("mobilephone","inform", "msg", "J'ai detecte un probleme. Votre ami a fait un malaise. Venez l'aider.")
        self.nao.say("I am sending a vocal message")
        

    def smartDeviceAction(self, device, action, key=None, value=None):
        self.xaalproxy.sendmsg(device, action, key, value)


    def testNao(self):
        # stif_enable = rospy.ServiceProxy('/body_stiffness/enable', Empty)
        #stif_enable() 
        #self.nao.moveToPerson() #ok
        self.nao.say("m")
        self.nao.say("Hello")
        print "Bonjour"
        self.nao.verifyPersonState(5)
        print self.nao.verifyState
if __name__== '__main__':
    c = Controller()
    #c.testNao()
   

    try:
	c.run()
    except KeyboardInterrupt:
        print "Program Quit"
