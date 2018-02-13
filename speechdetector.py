#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

"""Example: Use ALSpeechRecognition Module"""

import qi
import argparse
import sys
import time
from naoqi import ALProxy

class Speechdetector:
    def __init__(self, session, timeout):
        self.session = session
        self.timeout = timeout
        self.state = False

    def speechDetect(self):
        """
        This example uses the ALSpeechRecognition module.
        """
        # Get the service ALSpeechRecognition.
        #tts = ALProxy("ALTextToSpeech", "10.77.3.19", 9559)
        self.asr_service = self.session.service("ALSpeechRecognition")
        self.memory = self.session.service("ALMemory")

        self.asr_service.setLanguage("English")

        # Example: Adds "yes", "no" and "please" to the vocabulary (without wordspotting)
        vocabulary = ["yes", "no"]
        self.asr_service.pause(True)
        self.asr_service.setVocabulary(vocabulary, False)
        self.asr_service.pause(False)

        # Start the speech recognition engine with user Test_ASR
        self.asr_service.subscribe("Speech")
        subscriber = self.memory.subscriber("WordRecognized")
        subscriber.signal.connect(self.onWordRecognized)
        print 'Speech recognition engine started'
        time.sleep(self.timeout)
        self.asr_service.unsubscribe("Speech")        
        return self.state
        
    def onWordRecognized(self, value):
        if value[0] == "yes" and value[1] > 0.4: 
            print "[Pepper INFO] Got Phrase: yes"
            self.state = True
        elif value[0] == "no" and value[1] > 0.4: 
            print "[Pepper INFO] Got Phrase: no"
            self.state = False
                    
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
    sd = Speechdetector(session, 5)
    sd.speechDetect()
    
