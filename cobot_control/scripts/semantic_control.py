#!/usr/bin/env python

import rospy
from sem_server_ros.planner import JenaSempyPlanner
from sem_server_ros.server_com import ROSFusekiServer
from cobot_controllers.arm import Arm
from cobot_controllers.gripper import Gripper
from sem_server_ros.triple_collection import Triple, Collection
from sem_server_ros.msg import Command
from std_msgs.msg import String

speak = rospy.Publisher('/speech_output', String, queue_size=10)

planner = JenaSempyPlanner()
sem_server = ROSFusekiServer()
arm = Arm()
gripper = Gripper()

def check_syntax(symbol):
    triple = Triple()
    triple.subject = symbol
    symbol_sem = sem_server.test_data(symbol)
    if not symbol_sem:
        speak.publish("I cannot understand {}".format(symbol))
        raise Exception("I don't know what {} means".format(symbol))

def check_manipulation_preconditions(object_symbol):
    check_syntax(object_symbol, sem_server)
    object_seen = False
    object_sem = sem_server.read_data(object_symbol)
    for triple in object_sem:
        if triple.predicate == "rdf:type" and triple.object == cmd_msg.target:
            object_seen = True
    if not object_seen:
        speak.publish("I cannot see a {}".format(cmd_msg.target))
        raise ("I can't see a {}".format(object_symbol))
    return object_sem

def choose_step(available_steps):
    ''' Implement different collaboration policies '''
    return available_steps[0]

def interpret(action_sem, object_sem):
    ''' decide what to do '''
    if action_sem.has_type() == 'arm_action':
        pass
    elif action_sem.has_type() == 'gripper_action':
        pass
    else:
        raise Exception


def cmd_received(cmd_msg):
    try:
        print("Action: {} --- Target:{}".format(cmd_msg.action, cmd_msg.target))
        # Check that the manipulation is known
        check_syntax(cmd_msg.action, sem_server)
        # Verify if the object is valid
        object_sem = check_manipulation_preconditions(cmd_msg.target, sem_server):
        # establish a plan
        planner.init_time()
        planner.create_plan("Cranfield_Assembly")
        print("Dispatching")
        available_steps = planner.find_available_steps()
        while available_steps:
            # Pick an event to be performed
            next_step = choose_step(available_steps)
            next_action = planner.find_next_action(next_step)
            while next_action:
                action_sem = sem_server.read_data(next_action)
                interpret(Collection(action_sem), Collection(object_sem))
                next_action = planner.find_next_action(next_step)
            # Apply a timestamp to it
            planner.apply_timestamp(action_symbol)
            # Update available steps
            available_steps = planner.find_available_steps()
        print("{} OVER".format(cmd_msg))
    except:
        pass

if __name__ == "__main__":
    rospy.init_node('semantic_control')
    rospy.Subscriber("/command", Command, cmd_received)
    print "Robot ready"
    rospy.spin()
