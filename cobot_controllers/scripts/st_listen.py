#!/usr/bin/env python3

import os
import time
import rospy
#import sounddevice as sd
from scipy.io.wavfile import write
from std_msgs.msg import Int16
from std_msgs.msg import String
from cobot_msgs.srv import *
from vision_msgs.msg import ObjectHypothesis

import rospy
import moveit_commander 
from cobot_controllers.moveit_controller import MoveitArm
from franka_msgs.srv import SetForceTorqueCollisionBehavior, SetForceTorqueCollisionBehaviorRequest


#import librosa
import numpy as np
#import torch as t
from std_msgs.msg import Empty
#from opendr.engine.data import Timeseries
#from opendr.perception.speech_recognition.matchboxnet.matchboxnet_learner import MatchboxNetLearner



def callback(data):

    
    if data.id == 38 and data.score > 0.5:
        print("wow")
        whatever_publisher.publish(Empty())





def listener():
    rospy.init_node('opendrsa', anonymous=True)


    #whatever_publisher = rospy.Publisher('/whatever', Empty, queue_size=10)

    
    #rospy.Subscriber("/opendr/skeleton_based_action_recognition", ObjectHypothesis)
    rospy.Subscriber("/opendr/skeleton_based_action_recognition", ObjectHypothesis, callback)
#        rospy.Subscriber("chatter", String, callback)

    print("Listening") 
    rospy.spin()



if __name__ == '__main__':

    whatever_publisher = rospy.Publisher('/whatever', Empty, queue_size=10)
    listener()
