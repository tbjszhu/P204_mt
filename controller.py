#!/usr/bin/python

from mail import Mail
from kinect import Kinect
from Pepper import Pepper
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
from xaal.lib import Device,Engine,tools
import config

class Controller:
    
    def __init__(self):
        self.addr = tools.get_random_uuid()
        self.engine = Engine()      
        self.dev = Device("cli.experimental",self.addr)
        self.dev.vendor_id = 'IHSEV'
        self.dev.info = 'chute'
        self.engine.add_device(self.dev)
        self.threadlist = []
        self.fallEvent = threading.Event()        
            
        self.pepper = Pepper(self) #this cmd (rospy.init_node) makes ctrl+c invalide 
        self.kinect = Kinect()
        self.sensfloor = Sensfloor(self.engine)
        
    def engineLoop(self, event):
        print "Sensfloor engine start->"
        while (not self.fallEvent.is_set()):#not self.sensfloor.falldetected:        
            self.engine.loop()
            if self.sensfloor.falldetected:
                if not event.is_set():
                    event.set()
                    print "[Sensfloor] fallEvent set by sensfloor"     
        
    def run(self):
        if False:
            engineRun = threading.Thread(target=self.engineLoop, args=(self.fallEvent,), name="sensfloor")
            engineRun.start()
            self.threadlist.append(engineRun)

        if False:
            self.kinect.setEvent(self.fallEvent)
            self.kinect.start()
            self.threadlist.append(self.kinect)
            
        for thread in self.threadlist:
            thread.join()
            if (self.sensfloor.cpos != None):          
                print "[Sensfloor] x : %f, y : %f distance %f, direction %f" % (self.sensfloor.cpos[0], self.sensfloor.cpos[1], self.sensfloor.spos[0], self.sensfloor.spos[1]) 
            elif (self.kinect.pos_c != None):   
                print "[Kinect] x : %f, y : %f" % (self.kinect.pos_c[0], self.kinect.pos_c[1])
                
                               
                '''self.pepper.generateProxy()
                self.pepper.postureInit()
                self.pepper.say("moving to person")
                distance = self.sensfloor.spos[0]
                direction = -(self.sensfloor.spos[1])
                self.pepper.motion_service.moveToward(0, 0, direction)
                time.sleep(3)
                self.pepper.moveToPerson(distance, 0, 0)
                
                time.sleep(10)
                self.pepper.say("moving back")
                direction = -(self.sensfloor.spos[1])
                self.pepper.motion_service.moveToward(0, 0, 1)
                time.sleep(3)
                self.pepper.motion_service.moveToward(distance, 0, 0)
                time.sleep(3)
                self.pepper.motion_service.moveToward(0, 0, direction)'''  
                        
        print "fall detection finished"
        self.fallEvent.set()    
        if self.fallEvent.is_set():        
            self.pepper.generateProxy()
            self.pepper.postureInit()
            self.pepper.say("I have detected that a person fell down")
            
            self.headposition = self.pepper.searchFace()
            if self.headposition == []:
                print "No position"
            else:
                print "pitch %f, yaw %f" % (self.headposition[0], self.headposition[1])        
            if self.pepper.dialogue():
                self.pepper.speakToPersonDetected("All right")#speaktoperson("str", headpos)
            else:
                self.pepper.speakToPersonDetected("Don't worry")
                self.scenario() 
        
    def waitPersonResponse(self, timeout):
        count = 0
        self.pepper.resetVerifyState()
        while not self.pepper.getVerifyState():
            time.sleep(3)
            count += 1
            if (timeout!=0) and (count > timeout) : 
                print "waitpersonresponse timeout!"
                return False
        print "getpersonresponse"
        return self.pepper.getVerifyState()%2
    
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
        self.pepper.say("I turn off the stove and open the window shades")
        self.smartDeviceAction("switch", "off")
        self.pepper.say("I turn the lights on")
        print "porte ouverte"
        try:
            self.sendMail()
            self.pepper.say("I sent an email to your contact list")
        except:
            print "mail send failed"
        #self.smartDeviceAction("mobilephone","inform", "msg", "J'ai detecte un probleme. Votre ami a fait un malaise. Venez l'aider.")
        self.pepper.say("I am sending a vocal message")
        

    def smartDeviceAction(self, device, action):
        addr = config.getConfigInfo(device, "xaaladdr") #hdrware addr of the equpmt
        print addr         
        self.engine.send_request(self.dev,[addr],action,None);

    def testPepper(self):
        # stif_enable = rospy.ServiceProxy('/body_stiffness/enable', Empty)
        #stif_enable() 
        #self.pepper.moveToPerson() #ok
        self.pepper.say("m")
        self.pepper.say("Hello")
        print "Bonjour"
        self.pepper.verifyPersonState(5)
        print self.pepper.verifyState
if __name__== '__main__':
    c = Controller()
    #c.testPepper()
   

    try:
	c.run()
    except KeyboardInterrupt:
        print "Program Quit"
