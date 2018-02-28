#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

"""Example: Use ALSpeechRecognition Module"""

import qi
import argparse
import sys
import time
from naoqi import ALProxy

class Movedetector:
    def __init__(self, session, timeout):
        self.session = session
        self.timeout = timeout
        self.state = False

    def moveDetect(self):
        """
        This example uses the ALSpeechRecognition module.
        """
        # Get the service ALSpeechRecognition.
        tts = self.session.service("ALTextToSpeech")
        
        # Get the service ALMemory.
        self.memory = self.session.service("ALMemory")
        
        # Connect the event callback.
        subscriber = self.memory.subscriber("MovementDetected")
        self.connection_id = subscriber.signal.connect(self.onMovementDetected)
        
        # Get the services ALMovementDetection
        self.mv_service = self.session.service("ALMovementDetection")
        self.mv_service.subscribe("Movement")
                
        self.mv_service.setDepthSensitivity(0.01)
        self.mv_service.setColorSensitivity(0.02)        
        
        print 'movement Detection started'
        tts.say("started")
        
        print  "depth",self.mv_service.getDepthSensitivity()
        print  "color", self.mv_service.getColorSensitivity()
        
        time.sleep(self.timeout)
        tts.say("end")
        subscriber.signal.disconnect(self.connection_id)
        self.mv_service.unsubscribe("Movement")        
        return self.state
        
    def onMovementDetected(self, value):
        print "Hello"
        tts.say("Hello")
        print value   
                    
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="127.0.0.1",
                        help="Robot IP address. On robot or Local Naoqi: use '127.0.0.1'.")
    parser.add_argument("--port", type=int, default=9559,
                        help="Naoqi port number")

    args = parser.parse_args()
    session = qi.Session()
    args.ip = "10.77.3.19"
    try:
        session.connect("tcp://" + args.ip + ":" + str(args.port))
    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) +".\n"
               "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)
    sd = Movedetector(session, 10)
    sd.moveDetect()
    
