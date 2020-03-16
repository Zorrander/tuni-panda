#!/usr/bin/env python

import rospy
from sem_server_ros.ontology import JenaSempy
from sem_server_ros.planner import JenaSempyPlanner
from sem_server_ros.srv import CreateSem, ReadSem, UpdateSem, DeleteSem
from sem_server_ros.srv import CreateSemResponse, ReadSemResponse, UpdateSemResponse, DeleteSemResponse
from sem_server_ros.msg import Triple, URI
from sem_server_ros.msg import Command
from std_msgs.msg import String

speak = rospy.Publisher('/speech_output', String, queue_size=10)
find_symbol = rospy.ServiceProxy('read_sem', ReadSem)

def check_manipulation(symbol):
    symbol_sem = find_symbol(symbol)
    if not symbol_sem.result:
        speak.publish("I cannot understand {}".format(symbol))

def check_manipulation_preconditions(object_symbol):
    symbol_syntax_checking(object_symbol)
    object_seen = False
    for triple in target_sem:
        if triple.predicate == "rdf:type" and triple.object == cmd_msg.target:
            object_seen = True
    if not object_seen:
        speak.publish("I cannot see a {}".format(cmd_msg.target))

def cmd_received(cmd_msg, planner):
    try:
        print("Action: {} --- Target:{}".format(cmd_msg.action, cmd_msg.target))
        # Check that the manipulation is known
        #check_manipulation(cmd_msg.action)
        # Verify if the object is valid
        #check_manipulation_preconditions(cmd_msg.target):
        # establish a plan
        dispatchable = planner.create_plan("Cranfield_Assembly")
        # dispatch it
        if dispatchable:
            print("Dispatching")
            # Initialize variables
            available_steps = planner.find_available_steps()
            while available_steps:
                # Pick an event to be performed
                action = planner.find_next_action()
                # Wait for it to be performed

                # Apply a timestamp to it
                planner.apply_timestamp(action)
                # Update available steps
                available_steps = planner.find_available_steps()

    except:
        pass

if __name__ == "__main__":
    rospy.init_node('reasoner')
    planner = JenaSempyPlanner()
    # Wait for everything to turn on
    rospy.wait_for_service('read_sem')
    print "Robot ready"
    rospy.Subscriber("/command", Command, cmd_received, (planner))

    # subscribe cmd topic
    rospy.spin()
