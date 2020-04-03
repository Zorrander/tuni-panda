#!/usr/bin/env python

import rospy
from cobot_controllers.semantic_controller import SemanticController
from cobot_tuni_msgs.msg import Command

def action_callback(action_msg, sem_controller):
    sem_controller.interpret(action_msg.action, action_msg.target)


if __name__ == '__main__':
    try:
        rospy.init_node('controller_node')
        host = rospy.get_param('/host', 'localhost:3030')
        dataset = rospy.get_param('/dataset', 'Panda')
        sem_controller = SemanticController(host, dataset)
        rospy.Subscriber("/semantic_action", Command, action_callback, (sem_controller))
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
