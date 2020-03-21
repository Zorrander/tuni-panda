#!/usr/bin/env python

import rospy

import moveit_msgs.msg
import moveit_commander


class SemanticController(object):

    def __init__(self):
      moveit_commander.roscpp_initialize(sys.argv)
      group_name = "panda_arm"
      self.robot = moveit_commander.RobotCommander()
      self.scene = moveit_commander.PlanningSceneInterface()
      self.group = moveit_commander.MoveGroupCommander(group_name)

    def interpret(action_sem, target_sem):
        pass
