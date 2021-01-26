#!/usr/bin/env python
from tf.transformations import *
import rospy
import sys
import moveit_commander
import math
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
import yaml
import time
import os.path
from os.path import expanduser

home = expanduser("~")

RESOURCE_PATH  = os.path.join(home, "ros_ws", "robot_catkin_ws", "src", "tuni-panda", "cobot_controllers", "config")

class Arm(object):
    def __init__(self, pub_controller):
      moveit_commander.roscpp_initialize(sys.argv)
      group_name = "panda_arm"
      self.robot = moveit_commander.RobotCommander()
      self.scene = moveit_commander.PlanningSceneInterface()
      self.group = moveit_commander.MoveGroupCommander(group_name)
      self.pub_controller = pub_controller
      self.controller_switcher = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)

      self.switchToArmNavigationControl("")

      with open(os.path.join(RESOURCE_PATH, "pose-configuration.yaml"), 'r') as file:
          conf = yaml.load(file) or {}
      for key, value in conf.items():
          if not key.endswith("_ee"):
              self.group.remember_joint_values(key, value)

    def switchToArmNavigationControl(self, msg):
        rospy.loginfo('Switching to arm navigation control')
        switch_msg = SwitchControllerRequest()
        switch_msg.start_controllers = ["cartesian_pose_controller"]
        switch_msg.stop_controllers = ["cartesian_impedance_controller"]
        switch =  self.controller_switcher(switch_msg)
        print(switch.ok)
        return switch.ok

    def switchToForceImpedanceControl(self, msg):
        rospy.loginfo('Switching to force/impedance control on arm')
        switch_msg = SwitchControllerRequest()
        switch_msg.start_controllers = ["cartesian_impedance_controller"]
        switch_msg.stop_controllers = ["cartesian_pose_controller"]
        switch =  self.controller_switcher(switch_msg)
        print(switch.ok)
        return switch.ok

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
        self.set_speed(0.75)
        try:
            '''
            self.group.set_named_target("part_approach")
            self.group.go()
            self.group.stop()
            self.group.clear_pose_targets()
            '''
            self.group.set_named_target(target_name)
            self.group.go()
            self.group.stop()
            self.group.clear_pose_targets()
        except Exception as e:
            print(e)

    def go_to_cartesian_goal(self, pose):
        print("go_to_cartesian_goal")

        try:
            point = pose.position
            print("z --- {}".format(point.z))
            next_point = self.group.get_current_pose().pose
            print("current z --- {}".format(next_point.position.z))

            if next_point.position.z-0.10 > point.z: # We need to lower the arm
                goal = Pose()
                #next_point.position.x = point.x
                #next_point.position.y = point.y
                goal.position.x = point.x
                goal.position.y = point.y
                goal.position.z = next_point.position.z
                goal.orientation = pose.orientation
                self.pub_controller.publish(goal)

                # self.group.set_pose_target(next_point)
                # plan = self.group.go(wait=True)
                # self.group.stop()
                #self.group.clear_pose_targets()
                # ---
                time.sleep(2)
                print("z --- {}".format(point.z))
                goal.position.z = point.z
                self.pub_controller.publish(goal)

                # self.group.set_pose_target(next_point)
                # plan = self.group.go(wait=True)
                # self.group.stop()
                # self.group.clear_pose_targets()
            else:  # We need to move the arm up
                goal = Pose()
                # next_point.position.z = point.z
                goal.position.x = next_point.position.x
                goal.position.y = next_point.position.y
                goal.position.z = point.z
                goal.orientation = pose.orientation
                self.pub_controller.publish(goal)

                # self.group.set_pose_target(next_point)
                # plan = self.group.go(wait=True)
                # self.group.stop()
                # self.group.clear_pose_targets()
                # ---
                time.sleep(2.5)
                goal.position.x = point.x
                goal.position.y = point.y
                self.pub_controller.publish(goal)
                # next_point.position.x = point.x
                # next_point.position.y = point.y
                # self.group.set_pose_target(next_point)
                # plan = self.group.go(wait=True)
                # self.group.stop()
                # self.group.clear_pose_targets()

        except Exception as e:
            print(e)
        '''
        self.set_speed(0.75)
        self.move_to('part_approach')
        self.move_to('box')
        '''

    def go_to_joint_space_goal(self, joints_goal):
        '''
        arm_joints = self.group.get_current_joint_values()
        for i in range(0, len(joints_goal)):
            arm_joints[i] = joints_goal[i]
        self.group.go(arm_joints, wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
        '''
        self.set_speed(0.75)
        self.move_to('part_approach')
        self.move_to('part')
        '''
        goal = Float64MultiArray()
        for i in range(len(joints_goal)):
            goal.data.append(joints_goal[i])
        print(goal)
        self.pub_controller.publish(goal)
        '''
