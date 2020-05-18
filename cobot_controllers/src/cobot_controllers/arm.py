#!/usr/bin/env python
from tf.transformations import *
import sys
import moveit_commander
import math

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
        except Exception as e:
            print(e)

    def go_to_cartesian_goal(self, pose):
        try:
            next_point = self.group.get_current_pose().pose

            next_point.position.x = pose.position.x
            next_point.position.y = pose.position.y
            # quaternion = pose.orientation
            # explicit_quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
            # gripper_rotation = quaternion_from_euler(0, 0, math.pi/2)
            # new_orientation = quaternion_multiply(gripper_rotation, explicit_quat)
            self.group.set_pose_target(next_point)
            plan = self.group.go(wait=True)
            self.group.stop()
            self.group.clear_pose_targets()
            '''
            arm_joints = self.group.get_current_joint_values()

            print(arm_joints[6])
            print(pose.orientation.z)
            print(arm_joints[6] - pose.orientation.z)
            if (arm_joints[6] - pose.orientation.z) < 0.0:
                arm_joints[6] -= math.pi/2 + (arm_joints[6] - pose.orientation.z)
            else:
                arm_joints[6] += math.pi/2 - (arm_joints[6] - pose.orientation.z)
            print(arm_joints[6])

            arm_joints[6] = pose.orientation.z
            self.group.go(arm_joints, wait=True)
            self.group.stop()
            self.group.clear_pose_targets()
            '''
            if pose.position.z > 0.1:
                next_point.position.z = pose.position.z
            else:
                next_point.position.z = pose.position.z + 0.10
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
