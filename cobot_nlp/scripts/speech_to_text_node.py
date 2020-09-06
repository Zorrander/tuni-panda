#!/usr/bin/env python
import time
import rospy
import speech_recognition as sr
from std_msgs.msg import String
from os import path

publisher = rospy.Publisher('/command', String, queue_size=10)

def callbackAudio(recognizer, audio):
    # received audio data, now we'll recognize it using Google Speech Recognition
    try:
        print("Google Speech Recognition thinks you said ")
        # for testing purposes, we're just using the default API key
        # to use another API key, use `r.recognize_google(audio, key="GOOGLE_SPEECH_RECOGNITION_API_KEY")`
        # instead of `r.recognize_google(audio)`
        cmd = recognizer.recognize_google(audio)
        print(cmd)
        publisher.publish(cmd)
    except sr.UnknownValueError:
        print("Google Speech Recognition could not understand audio")
    except sr.RequestError as e:
        print("Could not request results from Google Speech Recognition service; {0}".format(e))

def sphinx_callback(recognizer, audio):
    # recognize speech using Sphinx
    try:
        print("Think")
        # print("Sphinx thinks you said " + recognizer.recognize_sphinx(audio, grammar=path.join(path.dirname(path.realpath(__file__)), 'counting.gram')))
        cmd = recognizer.recognize_sphinx(audio, keyword_entries=[("pass", 1.0), ("reach", 1.0), ("grasp", 1.0), ("open", 1.0), ("peg", 1.0)])
        bow = cmd.strip().split(' ')
        print(bow)
        print(type(bow))
        if type(bow) == list:
            print(bow[2], bow[0])
        publisher.publish(bow[2] + ' ' + bow[0])
        # print("Sphinx thinks you said " + recognizer.recognize_sphinx(audio))
    except sr.UnknownValueError:
        print("Sphinx could not understand audio")
    except sr.RequestError as e:
        print("Sphinx error; {0}".format(e))
    print("Say something")


if __name__ == '__main__':
    rospy.init_node('speech_to_text_input')
    try:
        r = sr.Recognizer()
        m = sr.Microphone()
        with m as source:
            r.adjust_for_ambient_noise(source, duration=2)
            while not rospy.is_shutdown():
                print("say something!")
                audio = r.listen(m)
                sphinx_callback(r, audio)
    except rospy.ROSInterruptException:
        stop_listening(wait_for_stop=False)
