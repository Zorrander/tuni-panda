#!/usr/bin/env python

import rospy
from cobot_control.planner import JenaSempyPlanner
from cobot_control.dispatcher import Dispatcher
from sem_server_ros.server_com import ROSFusekiServer
from sem_server_ros.semantic_controller import SemanticController
from sem_server_ros.semantic_vision import SemanticVision
from sem_server_ros.semantic_interactions import SemanticInterpreter


class SemanticHandoverTutorial(object):

    def __init__(self):
        self.planner = JenaSempyPlanner()
        self.sem_server = ROSFusekiServer()
        self.sem_vision = SemanticVision()
        self.interpreter = SemanticInterpreter()
        self.sem_controller = SemanticController()

def main():
  try:
    print "============ Press `Enter` to begin the tutorial by setting up the interface to the knowledgebase and the planner (press ctrl-d to exit) ..."
    raw_input()
    tutorial = SemanticHandoverTutorial()

    print "============ Press `Enter` to spawn an object on the workspace ..."
    raw_input()
    self.sem_vision.new_object("Cup")

    print "============ Press `Enter` to send a command to the robot to handover the previous object ..."
    raw_input()
    action = "Give"
    target = "Cup"
    self.interpreter.new_command("Give", "Cup")

    print "============ Press `Enter` to make a plan..."
    raw_input()
    # Check that the manipulation is known
    action_sem = self.interpreter.check_action(action)
    # Verify if the object is valid
    object_sem = self.interpreter.check_manipulation_preconditions(target)
    # establish a plan
    plan = self.planner.create_plan(action_sem, object_sem)

    print "============ Press `Enter` to reach for the object  ..."
    raw_input()
    dispatcher = Dispatcher(plan)
    task = dispatcher.dispatch()
    self.sem_controller.interpret(action)
    print "============ Press `Enter` to grasp it ..."
    raw_input()
    tutorial.execute_plan(cartesian_plan)

    print "============ Press `Enter` to give it ..."
    raw_input()
    tutorial.add_box()

    print "============ Press `Enter` to send the signal that the object has been received ..."
    raw_input()
    tutorial.attach_box()

    print "============ Press `Enter` to release the object ..."
    raw_input()
    cartesian_plan, fraction = tutorial.plan_cartesian_path(scale=-1)
    tutorial.execute_plan(cartesian_plan)

    print "============ Press `Enter` to clean the knowledge base ..."
    raw_input()
    tutorial.detach_box()

    print "============ Demo complete!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
