#!/usr/bin/env python

import rospy

from sem_server_ros.planner import JenaSempyPlanner
from sem_server_ros.server_com import ROSFusekiServer
from cobot_controllers.semantic_controller import SemanticController

from sem_server_ros.msg import Command, Triple
from std_msgs.msg import String

speak = rospy.Publisher('/speech_output', String, queue_size=10)

def check_syntax(symbol, sem_server):
    triple = Triple()
    triple.subject = symbol
    symbol_sem = sem_server.test_data(action_triple)
    if not symbol_sem:
        speak.publish("I cannot understand {}".format(symbol))
        raise Exception("I don't know what {} means".format(symbol))

def check_manipulation_preconditions(object_symbol, sem_server):
    check_syntax(object_symbol, sem_server)
    object_seen = False
    target_sem = sem_server.read_data(object_symbol)
    for triple in target_sem:
        if triple.predicate == "rdf:type" and triple.object == cmd_msg.target:
            object_seen = True
    if not object_seen:
        speak.publish("I cannot see a {}".format(cmd_msg.target))
        raise ("I can't see a {}".format(object_symbol))
    return target_sem

def cmd_received(cmd_msg, planner, sem_server, sem_controller):
    try:
        print("Action: {} --- Target:{}".format(cmd_msg.action, cmd_msg.target))
        # Check that the manipulation is known
        check_syntax(cmd_msg.action, sem_server)
        # Verify if the object is valid
        target_sem = check_manipulation_preconditions(cmd_msg.target, sem_server):
        # establish a plan
        planner.init_time()
        planner.create_plan("Cranfield_Assembly")
        print("Dispatching")
        available_steps = planner.find_available_steps()
        while available_steps:
            # Pick an event to be performed
            action_symbol = planner.find_next_action()
            action_sem = sem_server.read_data(action_symbol)
            sem_controller.interpret(action_sem, target_sem)
            # Apply a timestamp to it
            planner.apply_timestamp(action_symbol)
            # Update available steps
            available_steps = planner.find_available_steps()
        print("{} OVER".format(cmd_msg))
    except:
        pass

if __name__ == "__main__":
    rospy.init_node('basic_control')

    planner = JenaSempyPlanner()
    sem_server = ROSFusekiServer()
    sem_controller = SemanticController()

    rospy.Subscriber("/command", Command, cmd_received, (planner, sem_server, sem_controller))

    print "Robot ready"
    # subscribe cmd topic
    rospy.spin()
