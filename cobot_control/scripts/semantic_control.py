#!/usr/bin/env python

import rospy
from cobot_control.planner import JenaSempyPlanner
from cobot_control.dispatcher import Dispatcher
from sem_server_ros.server_com import ROSFusekiServer
from sem_server_ros.semantic_controller import SemanticController
from sem_server_ros.semantic_vision import SemanticVision
from sem_server_ros.semantic_interactions import SemanticInterpreter
from sem_server_ros.msg import Command
from std_msgs.msg import String

speak = rospy.Publisher('/speech_output', String, queue_size=10)

planner = JenaSempyPlanner()
sem_server = ROSFusekiServer()
interpreter = SemanticInterpreter()
sem_vision = SemanticVision()
sem_controller = SemanticController()

def cmd_received(cmd_msg):
    try:
        print("Action: {} --- Target:{}".format(cmd_msg.action, cmd_msg.target))
        # Check that the manipulation is known
        action_sem = interpreter.check_action(cmd_msg.action)
        # Verify if the object is valid
        object_sem = interpreter.check_manipulation_preconditions(cmd_msg.target)
        # establish a plan
        plan = planner.create_plan(action_sem, object_sem)
        # Dispatch it
        dispatcher = Dispatcher(plan)
        for task in dispatcher.dispatch()
            sem_controller.interpret(task)
        print("{} OVER".format(cmd_msg))
    except:
        pass

if __name__ == "__main__":
    rospy.init_node('semantic_control')
    rospy.Subscriber("/command", Command, cmd_received)
    print "Robot ready"
    rospy.spin()
