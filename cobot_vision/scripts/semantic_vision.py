#!/usr/bin/env python

import rospy
from cobot_vision.semantic_vision import SemanticVision
from cobot_tuni_msgs.msg import Object, Command

def detection_callback(obj_msg, sem_vision):
    sem_vision.new_object(obj_msg.type, obj_msg.width)

def command_callback(cmd_msg, sem_vision):
    sem_vision.anchor_object(cmd_msg.action, cmd_msg.target)

if __name__ == '__main__':
    try:
        rospy.init_node('vision')
        cmd_update = rospy.Publisher('/semantic_command', Command, queue_size=10)
        host = rospy.get_param('/host', 'localhost:3030')
        dataset = rospy.get_param('/dataset', 'Panda')
        sem_vision = SemanticVision(cmd_update, host, dataset)
        rospy.Subscriber("object", Object, detection_callback, (sem_vision))
        rospy.Subscriber("/anchoring", Command, command_callback, (sem_vision))
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
