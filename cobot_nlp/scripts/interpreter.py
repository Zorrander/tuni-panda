#!/usr/bin/env python

import rospy
from cobot_nlp.semantic_interactions import SemanticInterpreter
from cobot_tuni_msgs.msg import Command, Collection

def command_callback(cmd_msg, sem_interpreter):
    sem_interpreter.new_command(cmd_msg.action, cmd_msg.target)


if __name__ == '__main__':
    try:
        rospy.init_node('interpreter_node')
        cmd_update = rospy.Publisher('/anchoring', Command, queue_size=10)
        host = rospy.get_param('/host', 'localhost:3030')
        dataset = rospy.get_param('/dataset', 'Panda')
        sem_interpreter = SemanticInterpreter(cmd_update, host, dataset)
        rospy.Subscriber("/nlp_command", Command, command_callback, (sem_interpreter))
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
