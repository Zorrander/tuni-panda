#!/usr/bin/env python

import rospy
from cobot_vision.semantic_vision import SemanticVision
from cobot_tuni_msgs.msg import Object


def detection_callback(obj_msg, sem_vision):
    sem_vision.new_object(obj_msg.type)


if __name__ == '__main__':
    try:
        rospy.init_node('vision')
        host = rospy.get_param('/host', 'localhost:3030')
        dataset = rospy.get_param('/dataset', 'Panda')
        sem_vision = SemanticVision(host, dataset)
        rospy.Subscriber("object", Object, detection_callback, (sem_vision))
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
