#!/usr/bin/env python

import rospy
from cobot_vision.semantic_vision import SemanticVision
from sem_server_ros.msg import Object


def detection_callback(obj_msg, sem_vision):
    sem_vision.new_object(obj_msg.type)


if __name__ == '__main__':
    try:
        rospy.init_node('vision')
        sem_vision = SemanticVision()
        rospy.Subscriber("object", Object, detection_callback, (sem_vision))
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
