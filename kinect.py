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
        self.x_org = 2.75 #x axis from map(sensfloor)
        self.y_org = 1.10 #y axis from map(sensfloor)
        self.fallflag = False
        self.pos = []
        self.pos_c = []
        self.fallEvent = []
        self.headPosition = []
        
        #self.temp = 0
        
    def setEvent(self, event):
        self.fallEvent = event
        
    def fallDetectionPourChaquePersonne(self, pid):
        #print "detect for person_id: " + str(pid)
        if (not self.fallEvent.is_set()):
            try:
                (trans1, rot1) = self.listener.lookupTransform('/openni_depth_frame', '/neck_'+str(pid), rospy.Time(0))
                (trans2, rot2) = self.listener.lookupTransform('/openni_depth_frame', '/left_hip_'+str(pid), rospy.Time(0))    
                (headPosition, rot3) = self.listener.lookupTransform('/openni_depth_frame', '/head_'+str(pid), rospy.Time(0))            
                self.headPosition = headPosition
                #self.pos_c = self.transformImageCrdToCartesian(self.headPosition[0], self.headPosition[1], 0)
                #if self.temp < self.headPosition[1]:
                    #print "[Kinect INFO] x : %f, y : %f" % (self.headPosition[0], self.headPosition[1])
                    #self.temp = self.headPosition[1]  
                self.distance = abs(trans1[2]-trans2[2])          
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass#continue
            #self.rate.sleep()
                
            if (self.distance < 0.15):
                pass
                self.fallflag = True
                return True            

    def fallDetection(self):
        self.resetDistance()
        i = 1
        while True:
            #detectionPersonnel = threading.Thread(target=self.fallDetectionPourChaquePersonne, args=(self.fallEvent,i), name="sensfloor")
            #detectionPersonnel.start()
            self.fallDetectionPourChaquePersonne(i)
            if (not self.fallEvent.is_set()):
                if (self.fallflag):
                    self.fallEvent.set()       
                    print "[Kinect INFO] Fall detected from kinect! person id: " + str(i)
                    rospy.loginfo("Fall detected from kinect!")
                    return True;
            else:
                break;
                
            if i == 5:
                i = 1
            else:
                i = i + 1

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
    def transformImageCrdToCartesian(self, ver, hrz, angle):
        x_bias = hrz
        y_bias = ver #math.sqrt(dist * dist - hrz * hrz)
        
        x_max = 2.2 # max x from kinect deep figure
        x_min = -2.0 # max x from kinect deep figure
        y_max = 3.7 # max y from kinect deep figure
        
        x_pitch = x_max - x_min
        y_self = 0.60 # org point y axis for kinect deep figure
        y_pitch = y_max + y_self
        
        factor_y = 8.0/y_pitch # 8.0 sensfloor max y value
        factor_x = 7.0/x_pitch # 7.0 sensfloor max x value
        
        x = self.x_org - x_bias * factor_x # x_org = 2.63
        y = self.y_org + y_bias * factor_y # y_org = 0.8
        return [x, y]
        
    def resetDistance(self):
        self.distance = 1
    
    def run(self):
        print "Kinect Start-->"
        if self.fallDetection():
            self.pos = self.getHeadPosition()
            self.pos_c = self.transformImageCrdToCartesian(self.pos[0], self.pos[1], 0)
            print "[Kinect INFO] x : %f, y : %f" % (self.pos_c[0], self.pos_c[1])    
             
