#!/usr/bin/python

import rospy
from naoqi import ALProxy
import qi
import argparse
import sys
import almath
import math
import time
from facedetector import HumanGreeter
from speechdetector import Speechdetector

class Pepper:

    def __init__(self, controller):
        #rospy.init_node('pepper', anonymous=True, disable_signals=True) # disable_signals makes the ctrl+c works
        self.controller = controller
        self.check_str = "Are you alright? "
        self.verifyState = 0 # 0 no answer, 1 for good situation, 2 for bad situation
        
        self.x_org = 3.5
        self.y_org = 6.5
        self.dist_person = 1        
        
        self.ip = "10.77.3.19"
        self.port = 9559
        self.session = qi.Session()       
        self.connectPepper(self.ip, self.port)
        self.changeAutonomousLife('disabled')
        self.generateProxy()
        self.postureInit()
        self.say("Detection Start")
        
    def connectPepper(self, ip, port):
        try:
            self.session.connect("tcp://" + str(ip) + ":" + str(port))
        except RuntimeError:
            print ("Can't connect to Naoqi at ip \"" + str(ip) + "\" on port " + str(port) +".\n")
            sys.exit(1)
            
    def generateProxy(self):

        self.tts = self.session.service("ALTextToSpeech")
        self.navigation_service = self.session.service("ALNavigation")
        self.motion_service = self.session.service("ALMotion")
        self.posture_service = self.session.service("ALRobotPosture")
        self.motion_service.wakeUp()
        
    def changeAutonomousLife(self, status, force = False):  
        autolife=ALProxy("ALAutonomousLife",self.ip, self.port)
        print autolife.getState()
        if autolife.getState()!=status or force:
            autolife.setState(status)
            print "ALAutonomousLife status:" + status
                    
    def postureInit(self):
        self.posture_service.goToPosture("StandInit", 0.5)                
        
    def moveToPerson(self, position):        
        #flag = self.motion_service.moveTo(x_distance, y_distance, theta)
        if position[0] >= self.dist_person:
            distance = position[0] - self.dist_person # retain the distance with the person
        else:
            distance = 0
        direction = -(position[1])
        # walkout from init point to avoid the obstacle
        self.motion_service.moveTo(0.5, 0, 0) 
        time.sleep(1)
        
        # reach the falling position
        self.motion_service.moveTo(0, 0, direction)
        # Wait the body of the robot stable
        time.sleep(1)
        self.motion_service.moveTo(distance, 0, 0)
        time.sleep(2)   
    
    def returnToInitPosition(self, position):
        if position[0] >= self.dist_person:
            distance = position[0] - self.dist_person # retain the distance with the person
        else:
            position[0] = 0
        direction = -(position[1])    
        time.sleep(3)
        self.say("Going back")
        print "Going back"
        self.motion_service.moveTo(0, 0, -3.142)
        time.sleep(1)
        self.motion_service.moveTo(distance, 0, 0)
        #self.motion_service.waitUntilMoveIsFinished()
        time.sleep(1)
        self.motion_service.moveTo(0, 0, -direction)
        time.sleep(1)
        self.motion_service.moveTo(0.5, 0, 0)
        time.sleep(1)
        self.say("turning around")
        self.motion_service.moveTo(0, 0, -3.142)          
        
    def headVerPosition(self, angle, speed):
        angle = 0.4
        speed = 0.1
        self.motion_service.setAngles("HeadPitch",angle,speed)

    def headHorPosition(self, angle, speed):
        angle = 0.4
        speed = 0.1
        self.motion_service.setAngles("HeadYaw",angle,speed)
        
    def searchFace(self, time_out):
        names      = ["HeadYaw", "HeadPitch"]
        angleLists = [[1.0, -1.0, 1.0, -1.0, 0.0], [0.3, 1.0]]
        times      = [[1.0,  6.0, 11.0, 16.0, 18.0], [5.0, 11.0]]
        isAbsolute = True
        # With _async=True, angleInterpolation become non-blocking
        self.motion_service.angleInterpolation(names, angleLists, times, isAbsolute, _async=True)  
        try:
            # Initialize qi framework.
            connection_url = "tcp://" + self.ip + ":" + str(self.port)
            app = qi.Application(["HumanGreeter", "--qi-url=" + connection_url])
        except RuntimeError:
            print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) +".\n")
            sys.exit(1)
        self.human_greeter = HumanGreeter(app)
        return self.human_greeter.run(time_out)
        
    def dialogue(self):
        self.speechdetector = Speechdetector(self.session, 5)    
        self.speakToPersonDetected("Are you all right?")
        person_state = self.speechdetector.speechDetect()
        return person_state
        
            
    def speakToPersonDetected(self, words):
        if (self.human_greeter.headposition  != []):
            names = ["HeadYaw", "HeadPitch"]
            angleLists = [[self.human_greeter.headposition[0]], [self.human_greeter.headposition[1]]]
            times = [[1.0], [1.0]]
            isAbsolute = True
            self.motion_service.angleInterpolation(names, angleLists, times, isAbsolute, _async=False)        
            self.tts.say(str(words))
        else:
            self.tts.say(str(words))            

    def say(self, words):
        self.tts.say(str(words))  
            
