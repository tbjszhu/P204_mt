#!/usr/bin/python

import rospy
import tf
import geometry_msgs.msg
import math
import threading

class Kinect(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self)
        rospy.init_node('kinect', anonymous=True, disable_signals=True)
        self.threadName = "kinect" 
        self.listener = tf.TransformListener()
        self.rate = rospy.Rate(0.5)
        self.distance = 1
        self.x_org = 2.63
        self.y_org = 0.80
        self.fallflag = False
        self.pos = []
        self.pos_c = []
        self.fallEvent = []
        
    def setEvent(self, event):
        self.fallEvent = event

    def fallDetection(self):
        self.resetDistance()
        
        while (not self.fallEvent.is_set()):
            try:
                (trans1, rot1) = self.listener.lookupTransform('/openni_depth_frame', '/neck_1', rospy.Time(0))
                (trans2, rot2) = self.listener.lookupTransform('/openni_depth_frame', '/left_hip_1', rospy.Time(0))    
                (headposition, rot3) = self.listener.lookupTransform('/openni_depth_frame', '/head_1', rospy.Time(0))            
                self.headPosition = headposition  
                self.distance = abs(trans1[2]-trans2[2])          
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            self.rate.sleep()
                
            if (self.distance < 0.15):
                self.fallflag = True
                self.fallEvent.set()
                #print "[Kinect INFO] fallEvent set by Kinect"
                #else:
                #print "[Kinect INFO] fallEvent set by other eqp"        
                print "Fall detected from kinect!"
                rospy.loginfo("Fall detected from kinect!")
                return True
        print "[Kinect INFO] fall detection end"                
                        
        '''while self.distance > 0.15:
            #print "distance h of neck and hip ", self.distance
            try:
                (trans1, rot1) = self.listener.lookupTransform('/openni_depth_frame', '/neck_1', rospy.Time(0))
                (trans2, rot2) = self.listener.lookupTransform('/openni_depth_frame', '/left_hip_1', rospy.Time(0))    
                self.distance = abs(trans1[2]-trans2[2])
            
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            self.rate.sleep()'''
           
        

    def getHeadPosition(self): 
        if self.fallflag == True:
            return self.headPosition #distance, left/right, height
        else:
           print "[Kinect INFO]: NO Falling detected for kinect!"

    #angle between optique axe and the indoor map y axe
    def transformImageCrdToCartesian(self, dist, hrz, angle):
        x_bias = hrz
        y_bias = dist #math.sqrt(dist * dist - hrz * hrz)
        y_max = 3.1
        y_pitch = y_max + self.y_org
        factor = 7/y_pitch
        x = self.x_org + x_bias
        y = (self.y_org + y_bias) * factor
        return [x, y]
        
    def resetDistance(self):
        self.distance = 1
    
    def run(self):
        print "Kinect Start-->"
        if self.fallDetection():
            self.pos = self.getHeadPosition()
            self.pos_c = self.transformImageCrdToCartesian(self.pos[0], self.pos[1], 0)
            print "[Kinect INFO] x : %f, y : %f" % (self.pos_c[0], self.pos_c[1])    
             
