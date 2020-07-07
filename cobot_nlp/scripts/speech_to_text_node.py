#!/usr/bin/env python

import rospy
import speech_recognition as sr
from std_msgs.msg import String


def recordAudio():
    r = sr.Recognizer()
    with sr.Microphone() as source:
        r.adjust_for_ambient_noise(source)
        #audio = r.listen(source, phrase_time_limit=10)
        audio = r.listen(source)
    print("Listening")
    data = ""
    try:
        # Uses the default API key
        # To use another API key: `r.recognize_google(audio, key="GOOGLE_SPEECH_RECOGNITION_API_KEY")`
        data = r.recognize_google(audio)
    except sr.UnknownValueError:
        print("Google Speech Recognition could not understand audio")
    except sr.RequestError as e:
        print("Could not request results from Google Speech Recognition service; {0}".format(e))
    return data

if __name__ == '__main__':
    try:
        rospy.init_node('speech_to_text_input')
        publisher = rospy.Publisher('/command', String, queue_size=10)
        while not rospy.is_shutdown():

            data = recordAudio()
            if data:
                print("I heard: {}".format(data))
                publisher.publish(data)
    except rospy.ROSInterruptException:
        pass
