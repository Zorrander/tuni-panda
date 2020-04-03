#!/usr/bin/env python

import rospy
import speech_recognition as sr
from cobot_nlp.nlp import NLP
from std_msgs.msg import String

from cobot_tuni_msgs.msg import Command

publishers = dict()

nlp_processor = NLP()

def transmit(audio):
    if audio:
        print audio
        sentence_type, msg = nlp_processor.run(audio)
        print(sentence_type)
        print(msg)
        print(publishers[sentence_type])
        if not msg:
            print("I don't understand your request")
        else:
            publishers[sentence_type].publish(msg)

def recordAudio():
    r = sr.Recognizer()
    with sr.Microphone() as source:
        r.adjust_for_ambient_noise(source)
        print("Say something!")
        audio = r.listen(source, phrase_time_limit=10)

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
        publishers['cmd'] = rospy.Publisher('/nlp_command', Command, queue_size=10)
        while not rospy.is_shutdown():
            data = recordAudio()
            transmit(data)
    except rospy.ROSInterruptException:
        pass
