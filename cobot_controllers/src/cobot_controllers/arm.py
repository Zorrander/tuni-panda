#!/usr/bin/env python

import sys
import moveit_commander

class Arm(object):
    def __init__(self):
      moveit_commander.roscpp_initialize(sys.argv)
      group_name = "panda_arm"
      self.robot = moveit_commander.RobotCommander()
      self.scene = moveit_commander.PlanningSceneInterface()
      self.group = moveit_commander.MoveGroupCommander(group_name)

    def set_speed(self, speed_factor):
        ''' Modify max speed of the arm by a speed factor ranging from 0 to 1.'''
        self.group.set_max_velocity_scaling_factor(speed_factor)

    def store_position(self, name):
        self.group.remember_joint_values(name)

    def move_to(self, target_name):
        ''' Send robot to a predefined position'''
        try:
            self.group.set_named_target(target_name)
            self.group.go()
            self.group.stop()
            self.group.clear_pose_targets()
        except:
            pass

    def go_to_cartesian_goal(self, pose):
        try:
            next_point = self.group.get_current_pose().pose
            next_point.position.x = pose.position.x
            next_point.position.y = pose.position.y
            next_point.position.z = pose.position.z
            self.group.set_pose_target(next_point)
            plan = self.group.go(wait=True)
            self.group.stop()
            self.group.clear_pose_targets()
        except:
            print("Error in go_to_cartesian_goal")

    def go_to_joint_space_goal(self, joints_goal):
        arm_joints = self.group.get_current_joint_values()
        for i in range(0, len(joints_goal)):
            arm_joints[i] = joints_goal[i]
        self.group.go(arm_joints, wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
