#!/usr/bin/env python

import rospy
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import String


def speak(msg, soundhandle):
    print("Saying: {}".format(msg))
    s3 = soundhandle.voiceSound(msg.data)
    s3.play()


if __name__ == '__main__':
    try:
        rospy.init_node('text_to_speech')
        soundhandle = SoundClient()
        rospy.Subscriber("/speech_output", String, speak, (soundhandle))
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
