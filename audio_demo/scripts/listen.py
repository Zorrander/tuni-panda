#!/usr/bin/env python3

import os
import time
import rospy
import sounddevice as sd
from scipy.io.wavfile import write
from std_msgs.msg import Int16

import librosa
import numpy as np
import torch as t
from std_msgs.msg import Empty
from opendr.engine.data import Timeseries
from opendr.perception.speech_recognition.matchboxnet.matchboxnet_learner import MatchboxNetLearner


TEST_CLASSES_N = 20
TEST_EPOCHS = 5
PATH  = "/home/alex/ros_ws/opendr/MatchboxNet"

fs = 16000  # Sample rate
seconds = 2 # Duration of recording

def record(time, sample_rate):
    myrecording = sd.rec(int(time * sample_rate), samplerate=sample_rate, channels=1)
    sd.wait()  # Wait until recording is finished
    print("writing file...")
    write('output.wav', fs, myrecording)  # Save as WAV file

def analyse(learner, filename):
    commands = []
    stream = librosa.stream(filename,
                            block_length=256,
                            frame_length=2048,
                            hop_length=2048)
    for y_block in stream:
        values = np.expand_dims(y_block, axis=0)
        test_timeseries = Timeseries(values)
        command = learner.infer(test_timeseries)
        print(command)
        #if (command.data in [4, 10, 14, 15, 16]) and command.confidence > 0.7:
        #    commands.append(command.data)
        if (command.data in [4, 10, 14, 15, 16]) and command.confidence > 0.85:
            commands.append(command.data)
        return commands

def listener():
    rospy.init_node('opendr_sr', anonymous=True)

    learner = MatchboxNetLearner(device="cpu", output_classes_n=TEST_CLASSES_N, iters=TEST_EPOCHS)
    learner.load(PATH)

    request_publisher = rospy.Publisher('/request_detection', Int16, queue_size=10)
    go_pub = rospy.Publisher('go', Empty, queue_size=10)
    stop_pub = rospy.Publisher('stop', Empty, queue_size=10)

    while not rospy.is_shutdown():
        print("Say something")
        record(seconds, fs)
        cmds = analyse(learner, os.getcwd() + "/output.wav")
        if 4 in cmds:  # Command = Go
            go_pub.publish(Empty())
        elif 10 in cmds:  # Command = One
            msg = Int16()
            msg.data = 1
            request_publisher.publish(msg)
            time.sleep(20)
        elif 14 in cmds:  # Command = Stop
            stop_pub.publish(Empty())
        elif 16 in cmds:  # Command = Two
            msg = Int16()
            msg.data = 2
            request_publisher.publish(msg)
            time.sleep(20)
        else:
            print("...")
        '''
        elif 15 in cmds:  # Command = Three
            msg = Int16()
            msg.data = 3
            request_publisher.publish(msg)
            time.sleep(20)
        '''

        '''
        values, _ = librosa.load(os.getcwd() + "/output.wav", sr=fs)
        values = np.expand_dims(values, axis=0)
        test_timeseries = Timeseries(values)
        command = learner.infer(test_timeseries)
        print(command)
        if command.data == 0 and command.confidence > 0.75:
            go_pub.publish(Empty())
        elif command.data == 1 and command.confidence > 0.75:
            stop_pub.publish(Empty())
        time.sleep(2)
        '''


if __name__ == '__main__':
    listener()
