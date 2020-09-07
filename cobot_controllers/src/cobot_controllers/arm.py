#!/usr/bin/env python
from tf.transformations import *
import sys
import moveit_commander
import math
import yaml
import os.path
from os.path import expanduser

home = expanduser("~")

RESOURCE_PATH  = os.path.join(home, "ros_ws", "robot_catkin_ws", "src", "tuni-panda", "cobot_controllers", "config")

class Arm(object):
    def __init__(self):
      moveit_commander.roscpp_initialize(sys.argv)
      group_name = "panda_arm"
      self.robot = moveit_commander.RobotCommander()
      self.scene = moveit_commander.PlanningSceneInterface()
      self.group = moveit_commander.MoveGroupCommander(group_name)
      with open(os.path.join(RESOURCE_PATH, "pose-configuration.yaml"), 'r') as file:
          conf = yaml.load(file) or {}
      for key, value in conf.items():
          if not key.endswith("_ee"):
              self.group.remember_joint_values(key, value)


    def set_speed(self, speed_factor):
        ''' Modify max speed of the arm by a speed factor ranging from 0 to 1.'''
        self.group.set_max_velocity_scaling_factor(speed_factor)

    def list_targets(self):
        return self.group.get_remembered_joint_values()

    def store_position(self, name):
        joint_conf = {name : self.group.get_current_joint_values()}
        ee_conf = {name + "_ee" : {'position': self.group.get_current_pose().pose.position, 'orientation': self.group.get_current_pose().pose.orientation}}
        with open(os.path.join(RESOURCE_PATH, "pose-configuration.yaml"), 'r') as file:
            conf = yaml.load(file) or {}
        conf.update(joint_conf)
        conf.update(ee_conf)
        with open(os.path.join(RESOURCE_PATH, "pose-configuration.yaml"), "w") as file:
            documents = yaml.dump(conf, file)

    def move_to(self, target_name):
        ''' Send robot to a predefined position'''
        try:
            self.group.set_named_target(target_name)
            self.group.go()
            self.group.stop()
            self.group.clear_pose_targets()
        except Exception as e:
            print(e)

    def go_to_cartesian_goal(self, point):
        print("go_to_cartesian_goal")
        try:
            next_point = self.group.get_current_pose().pose
            if next_point.position.z > point.z: # We need to lower the arm
                next_point.position.x = point.x
                next_point.position.y = point.y
                self.group.set_pose_target(next_point)
                plan = self.group.go(wait=True)
                self.group.stop()
                self.group.clear_pose_targets()
                # ---
                next_point.position.z = point.z
                self.group.set_pose_target(next_point)
                plan = self.group.go(wait=True)
                self.group.stop()
                self.group.clear_pose_targets()
            else:  # We need to move the arm up
                next_point.position.z = point.z
                self.group.set_pose_target(next_point)
                plan = self.group.go(wait=True)
                self.group.stop()
                self.group.clear_pose_targets()
                # ---
                next_point.position.x = point.x
                next_point.position.y = point.y
                self.group.set_pose_target(next_point)
                plan = self.group.go(wait=True)
                self.group.stop()
                self.group.clear_pose_targets()
        except Exception as e:
            print(e)

    def go_to_joint_space_goal(self, joints_goal):
        arm_joints = self.group.get_current_joint_values()
        for i in range(0, len(joints_goal)):
            arm_joints[i] = joints_goal[i]
        self.group.go(arm_joints, wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
