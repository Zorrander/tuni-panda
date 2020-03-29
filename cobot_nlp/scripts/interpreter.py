#!/usr/bin/env python

import rospy
from cobot_nlp.semantic_interactions import SemanticInterpreter
from cobot_tuni_msgs.msg import Command, Collection


def command_callback(cmd_msg, sem_interpreter):
    sem_interpreter.new_command(cmd_msg.action, cmd_msg.target)


if __name__ == '__main__':
    try:
        rospy.init_node('interpreter_node')
        cmd_update = rospy.Publisher('/plan', Collection, queue_size=10)
        sem_interpreter = SemanticInterpreter(cmd_update)
        rospy.Subscriber("/command", Command, command_callback, (sem_interpreter))
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
