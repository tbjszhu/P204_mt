#!/usr/bin/python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from naoqi_bridge_msgs.msg import *
import actionlib
from std_srvs.srv import Empty

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
        self.dist_person = 0.5        
        
        self.ip = "10.77.3.19"
        self.port = 9559
        self.session = qi.Session()       
        self.connectPepper(self.ip, self.port)
        
    def connectPepper(self, ip, port):
        try:
            self.session.connect("tcp://" + str(ip) + ":" + str(port))
        except RuntimeError:
            print ("Can't connect to Naoqi at ip \"" + str(ip) + "\" on port " + str(port) +".\n")
            sys.exit(1)
            
    def generateProxy(self):

        self.tts = ALProxy("ALTextToSpeech", self.ip, self.port)
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
        self.motion_service.moveTo(0.5, 0, 0) #walkout from init point to avoid the obstacle
        time.sleep(1)
        self.motion_service.moveTo(0, 0, direction)
        time.sleep(1)
        self.motion_service.moveTo(distance, 0, 0)  
    
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
        #self.motion_service.setStiffnesses("Head", 0.0)
        #taskList = self.motion_service.getTaskList()
        #self.motion_service.killTask(taskList[0][1])    
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
        
    def verifyPersonState(self, timeout):
        rospy.loginfo("NAO will verify person state")
        self.say(self.check_str)
        self.response(timeout,["no","yes"])
        if self.words[0] =="yes" and self.confidence_values[0] > 0.4 : 
            self.verifyState = 1
        else :
            self.verifyState = 2
            
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
    
    def response(self, timeout, words):
        self.resetVerifyState()
        
        client = actionlib.SimpleActionClient('speech_vocabulary_action', SetSpeechVocabularyAction)
        client.wait_for_server()
        goal = SetSpeechVocabularyGoal(words)
        rospy.sleep(1)

        client.send_goal(goal)
        client.wait_for_result()
        rospy.sleep(1)

        rospy.Subscriber("/word_recognized", WordRecognized, self.hearCallback)
        rospy.wait_for_service('start_recognition')
        startRecognition = rospy.ServiceProxy('start_recognition', Empty)
        startRecognition()
        
        count = 0
        while(not self.verifyState):
            rospy.sleep(5)
            count += 1
            if(timeout != 0) and (count > timeout):
                rospy.loginfo("wait person response timeout")
                break
        
        stopRecognition = rospy.ServiceProxy('stop_recognition',Empty)
        stopRecognition()


    def hearCallback(self, data):
        print "class NAO hearCallback"
        print data.words
        print data.confidence_values
        self.words=data.words
        self.confidence_values=data.confidence_values


    def getVerifyState(self):
        return self.verifyState

    def resetVerifyState(self):
        self.verifyState = 0
            
