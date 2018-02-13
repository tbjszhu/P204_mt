#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

"""Example: A Simple class to get & read FaceDetected Events"""

import qi
import time
import sys
import argparse
from naoqi import ALProxy

class HumanGreeter(object):
    """
    A simple class to react to face detection events.
    """

    def __init__(self, app):
        """
        Initialisation of qi framework and event detection.
        """
        super(HumanGreeter, self).__init__()
        app.start()
        self.session = app.session
        # Get the service ALMemory.
        self.memory = self.session.service("ALMemory")
        # Connect the event callback.
        self.subscriber = self.memory.subscriber("FaceDetected")
        self.subscriber.signal.connect(self.on_human_tracked)
        # Get the services ALTextToSpeech and ALFaceDetection.
        self.face_detection = self.session.service("ALFaceDetection")
        self.face_detection.subscribe("HumanGreeter")
        self.got_face = False
        self.tts = ALProxy("ALTextToSpeech", "10.77.3.19", 9559)
        self.motion_service = self.session.service("ALMotion")
        self.got_pos = False
        self.headposition = []
        
        
        self.tracking_enabled = True

        # Get the service ALFaceDetection.

        self.face_service = self.session.service("ALFaceDetection")

        print "Will set tracking to '%s' on the robot ..." % self.tracking_enabled

        # Enable or disable tracking.
        self.face_service.enableTracking(self.tracking_enabled)

        # Just to make sure correct option is set.
        print "Is tracking now enabled on the robot?", self.face_service.isTrackingEnabled()
        

    def on_human_tracked(self, value):
        """
        Callback for event FaceDetected.
        """
        if value == []:  # empty value when the face disappears
            self.got_face = False
        elif not self.got_face:  # only speak the first time a face appears
            if not self.got_pos:
                self.tts.say("Face detected")              
            print "Face detected!"
            
            self.got_face = True
                        
            #self.motion_service.setStiffnesses("Head", 0.0)
            # First Field = TimeStamp.
            timeStamp = value[0]
            #print "TimeStamp is: " + str(timeStamp)

            # Second Field = array of face_Info's.
            faceInfoArray = value[1]
            if not self.got_pos:
                self.got_pos = True
                CameraPose_InTorsoFrame = value[2]
                CameraPose_InRobotFrame = value[3]
                CameraID = value[4]
                self.headposition = [ CameraPose_InTorsoFrame[5], CameraPose_InTorsoFrame[4] ]
                print "pitch %f, yaw %f" % (CameraPose_InTorsoFrame[4], CameraPose_InTorsoFrame[5])
                names      = ["HeadYaw", "HeadPitch"]
                angleLists = [[CameraPose_InTorsoFrame[5]], [CameraPose_InTorsoFrame[4]]]
                times      = [[1.0], [ 1.0]]
                isAbsolute = True
                self.motion_service.angleInterpolation(names, angleLists, times, isAbsolute, _async=False)

    def run(self):
        """
        Loop on, wait for events until manual interruption.
        """
        self.tts.say("Face detection Start")
        print "Starting HumanGreeter"
        wait_count = 0
        wait_time = 15
        try:
            while wait_count < wait_time:
                time.sleep(1)
                wait_count += 1
            self.tts.say("Face detection End")
            return self.headposition
        except KeyboardInterrupt:
            print "Interrupted by user, stopping HumanGreeter"
            self.face_detection.unsubscribe("HumanGreeter")
            #self.motion_service.setStiffnesses("Head", 1.0)
            #stop
            sys.exit(0)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="127.0.0.1",
                        help="Robot IP address. On robot or Local Naoqi: use '127.0.0.1'.")
    parser.add_argument("--port", type=int, default=9559,
                        help="Naoqi port number")

    args = parser.parse_args()
    args.ip = "10.77.3.19"
    try:
        # Initialize qi framework.
        connection_url = "tcp://" + args.ip + ":" + str(args.port)
        app = qi.Application(["HumanGreeter", "--qi-url=" + connection_url])
    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) +".\n"
               "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)

    human_greeter = HumanGreeter(app)
    human_greeter.run()
