#!/usr/bin/env python

import rospy
from sem_server_ros.planner import JenaSempyPlanner
from sem_server_ros.server_com import ROSFusekiServer


class SemanticHandoverTutorial(object):

    def __init__(self):
        self.planner = JenaSempyPlanner()
        self.sem_server = ROSFusekiServer()

def main():
  try:
    print "============ Press `Enter` to begin the tutorial by setting up the interface to the knowledgebase and the planner (press ctrl-d to exit) ..."
    raw_input()
    tutorial = SemanticHandoverTutorial()

    print "============ Press `Enter` to spawn an object on the workspace ..."
    raw_input()
    tutorial.go_to_pose_goal()

    print "============ Press `Enter` to send a command to the robot to handover the previous object ..."
    raw_input()
    tutorial.go_to_joint_state()

    print "============ Press `Enter` to make a plan and display it..."
    raw_input()
    cartesian_plan, fraction = tutorial.plan_cartesian_path()

    print "============ Press `Enter` to reach for the object  ..."
    raw_input()
    tutorial.display_trajectory(cartesian_plan)

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
