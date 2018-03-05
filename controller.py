#!/usr/bin/python

from mail import Mail
from kinect import Kinect
from Pepper import Pepper
import netifaces as ni
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
        self.pepper = Pepper(self)         
        self.kinect = Kinect()
        self.sensfloor = Sensfloor(self.engine)
    
    # thread to receive/send the xaal message    
    def engineLoop(self, event):
        print "Sensfloor engine start->"
        while (not event.is_set()):#not self.sensfloor.falldetected:    
            self.engine.loop()
            if self.sensfloor.falldetected:
                event.set()
                print "[Sensfloor] fallEvent set by sensfloor"   
        
    def run(self):
    
        # start falling detection with sensfloor
        if True:
            engineRun = threading.Thread(target=self.engineLoop, args=(self.fallEvent,), name="sensfloor")
            engineRun.start()
            self.threadlist.append(engineRun)
            
        # start falling detection with kinect
        if True:
            self.kinect.setEvent(self.fallEvent)
            self.kinect.start()
            self.threadlist.append(self.kinect)
        
        # wait the two thread to be finished    
        for thread in self.threadlist:
            thread.join()
            
            if (self.sensfloor.cpos != [] and self.sensfloor.spos != []):
                fallPosition = self.sensfloor.spos          
                print "[Sensfloor] x : %f, y : %f distance %f, direction %f" % (self.sensfloor.cpos[0], self.sensfloor.cpos[1], self.sensfloor.spos[0], self.sensfloor.spos[1]) 
            elif (self.kinect.pos_c != []):
                # mapping from the sensfloor position to robot cmd (distance, direction)
                fallPosition = self.sensfloor.transformCartesianToPolar(3.5, 7, self.kinect.pos_c[0], self.kinect.pos_c[1])  
                print "[Kinect] x : %f, y : %f" % (self.kinect.pos_c[0], self.kinect.pos_c[1])          
                                                   
        print "[Controller INFO] All fall detection finished"
        
        # the robot reach to the position and communicate with the person
        if self.fallEvent.is_set() and True:        
            self.pepper.postureInit()
            self.pepper.say("I have detected that a person fell down")
            
            self.pepper.moveToPerson(fallPosition)
            
            # start searching the face of the person and dialogue face to face            
            self.headposition = self.pepper.searchFace(15) # search face time_out 15s
            if self.headposition == []:
                print "No position"
            else:
                print "[Face Detection INFO] Average pitch %f, yaw %f" % (self.headposition[0], self.headposition[1])
                
            # verify the state of the of the person        
            if self.pepper.dialogue():
                self.pepper.speakToPersonDetected("All right")
            else:
                self.pepper.speakToPersonDetected("Don't worry")
                self.scenario()
            self.pepper.postureInit()    
            self.pepper.returnToInitPosition(fallPosition)                             
    
    def sendMail(self):
        #link = self.webRTCaddress()
        topic = "nao_robot/camera/top/camera/image_raw"
        text = "The assistance robot and smart house has detected a fall. To check, please see the video from this address " #+ link
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
        self.smartDeviceAction("lamp4", "on")
        self.smartDeviceAction("lamp5", "on")
        self.smartDeviceAction("lamp6", "on")
        self.pepper.say("I turn the lights on")
        print "lampes allumeees"
        try:
            self.sendMail()
            self.pepper.say("I sent an email to your contact list")
        except:
            print "mail send failed"
        self.engine.loop()        

    def smartDeviceAction(self, device, action):
        addr = config.getConfigInfo(device, "xaaladdr") #hdrware addr of the equpmt
        print addr         
        self.engine.send_request(self.dev,[addr],action,None)

if __name__== '__main__':
    c = Controller()   

    try:
	c.run()
    except KeyboardInterrupt:
        print "Program Quit"
