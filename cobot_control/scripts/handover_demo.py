#!/usr/bin/env python

import rospy
from cobot_control.planner import RosPlanner
from cobot_control.dispatcher import Dispatcher
from sem_server_ros.msg import Command, Object


def main():
  try:
    print "============ Press `Enter` to begin the tutorial by printing general info about each node (press ctrl-d to exit) ..."
    raw_input()

    print "============ Press `Enter` to spawn an object on the workspace ..."
    raw_input()
    obj_detector_publisher = rospy.Publisher('/object', Object, queue_size=10)
    obj_detector_publisher.publish(Object("Cup"))

    print "============ Press `Enter` to send a command to the robot to handover the previous object ..."
    raw_input()
    action = "Give"
    target = "Cup"
    cmd_publisher = rospy.Publisher('/command', Command, queue_size=10)
    cmd_publisher.publish(Command("Give", "Cup"))

    print "============ Press `Enter` to make a plan..."
    raw_input()
    # Check that the manipulation is known
    action_sem = tutorial.interpreter.check_action(action)
    # Verify if the object is valid
    object_sem = tutorial.interpreter.check_manipulation_preconditions(target)
    # establish a plan
    planner = RosPlanner()
    plan = planner.create_plan(action_sem, object_sem)

    print "============ Press `Enter` to reach for the object  ..."
    raw_input()
    dispatcher = Dispatcher(plan)
    task = dispatcher.dispatch()
    #tutorial.sem_controller.interpret(action)

    print "============ Press `Enter` to release the object ..."
    raw_input()

    print "============ Press `Enter` to clean the knowledge base ..."
    raw_input()

    print "============ Demo complete!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  rospy.init_node('handover_demo')
  main()
