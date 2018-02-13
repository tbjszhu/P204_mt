#!/usr/bin/python

import rospy
from naoqi_bridge_msgs.msg import *
import actionlib
from std_srvs.srv import Empty

def callback(data):
    print "words : ", data.words
    print "confidence_values ", data.confidence_values
    
def recognize():
    rospy.init_node('recognize', anonymous=True)
    client = actionlib.SimpleActionClient('speech_vocabulary_action', SetSpeechVocabularyAction)
    print "waiting for action server to start"
    client.wait_for_server()
    print "action server started, sending goal"
    goal = SetSpeechVocabularyGoal(words = ["ca va", "salut", "comment va tu"])
    
    client.send_goal(goal)
    print "client send goal"
    client.wait_for_result()
    print "set vocabulary ", client.get_result()

    rospy.Subscriber("/word_recognized", WordRecognized, callback)
    print "subscribe word_recognized"
    rospy.wait_for_service('start_recognition')
    print "after wait for service"
    start = rospy.ServiceProxy('start_recognition', Empty)
    stop =  rospy.ServiceProxy('stop_recognition', Empty)
    start()
    rospy.sleep(10)
    stop()

if __name__ == '__main__':
    try:
        recognize()
    except rospy.ROSInterruptException:
        pass
