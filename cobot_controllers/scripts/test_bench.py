#!/usr/bin/env python

import sys
import copy
import time
import rospy
import actionlib

from math import pi
import moveit_msgs.msg
import moveit_commander

from cobot_controllers.msg import Test
from std_msgs.msg import String, Empty, Float32
from moveit_commander.conversions import pose_to_list

from control_msgs.msg import GripperCommandActionGoal, GripperCommandGoal, GripperCommand
from franka_gripper.msg import HomingAction, HomingActionGoal, GraspAction, GraspActionGoal, GraspGoal, MoveActionGoal, MoveGoal

from franka_control.msg import ErrorRecoveryActionGoal

class TestBench(object):
    def __init__(self):
      moveit_commander.roscpp_initialize(sys.argv)

      group_name = "panda_arm"
      self.robot = moveit_commander.RobotCommander()
      self.scene = moveit_commander.PlanningSceneInterface()
      self.group = moveit_commander.MoveGroupCommander(group_name)

      self.reset_sub = rospy.Subscriber("/reset_cmd", Empty, self.reset)
      self.x_test_sub = rospy.Subscriber("/x_test", Float32, self.test_x)
      self.start_sub = rospy.Subscriber("/test_bench", Test, self.routine)
      self.homing_sub = rospy.Subscriber("/homing_cmd", Empty, self.homing)
      self.stop_sub = rospy.Subscriber("/stop_cmd", Empty, self.stop_routine)
      self.approach_sub = rospy.Subscriber("/approach_cmd", Empty, self.approach)
      self.height_test_sub = rospy.Subscriber("/height_test", Float32, self.test_height)

      self.open_gripper_pub = rospy.Publisher("/franka_gripper/move/goal", MoveActionGoal, queue_size=10)
      self.gripper_grasp_client = rospy.Publisher('/franka_gripper/grasp/goal', GraspActionGoal, queue_size=10)
      self.recovery_pub = rospy.Publisher("/franka_control/error_recovery/goal", ErrorRecoveryActionGoal, queue_size=10)
      self.generic_grasp_client =  rospy.Publisher('/franka_gripper/gripper_action/goal', GripperCommandActionGoal, queue_size=10)

      self.gripper_homing_client = actionlib.SimpleActionClient('/franka_gripper/homing', HomingAction)
      self.gripper_homing_client.wait_for_server()

      self.carry_on_routine = True

    def homing(self, msg):
        goal = HomingActionGoal(goal={})
        self.gripper_homing_client.send_goal(goal)
        self.gripper_homing_client.wait_for_result()
        self.gripper_homing_client.publish(goal)

    def release(self):
        gripper_goal = MoveGoal()
        gripper_goal.speed = 20.0
        gripper_goal.width = 0.08
        self.open_gripper_pub.publish(MoveActionGoal(goal=gripper_goal))
        if not self.carry_on_routine:
            raise ValueError

    def reset(self, msg):
        self.carry_on_routine = True
        recovery_goal = ErrorRecoveryActionGoal()
        self.recovery_pub.publish(recovery_goal)

    def grasp(self, width, force):
        ''' width, epsilon_inner, epsilon_outer, speed, force '''
        if self.carry_on_routine:
            grasp_goal = GraspGoal()
            grasp_goal.width = float(width)
            grasp_goal.epsilon.inner = 0.005
            grasp_goal.epsilon.outer = 0.005
            grasp_goal.speed = 20.0
            grasp_goal.force = float(force)
            grasp_msg = GraspActionGoal(goal=grasp_goal)
            self.gripper_grasp_client.publish(grasp_msg)
        else:
            raise ValueError

    def grasp2(self, width, force):
        ''' width, epsilon_inner, epsilon_outer, speed, force '''
        grasp_goal = GripperCommandGoal()
        grasp_goal.command.position = float(width)
        grasp_goal.command.max_effort = float(force)
        grasp_msg = GripperCommandActionGoal(goal=grasp_goal)
        self.generic_grasp_client.publish(grasp_msg)

    def test_height(self, msg):
        value = -msg.data
        self.vertical_move(value)

    def test_x(self, msg):
        value = msg.data
        self.horizontal_move(value)

    def horizontal_move(self, value):
        next_point = self.group.get_current_pose().pose
        next_point.position.x += float(value)/100
        self.move_group(next_point)
        self.clear_group()

    def horizontal_move2(self, first_pose, value, rep):
        next_point = first_pose
        next_point.position.x += (float(value)/100)*rep
        self.move_group(next_point)
        self.clear_group()

    def vertical_move(self, value):
        if self.carry_on_routine:
            next_point = self.group.get_current_pose().pose
            next_point.position.z += value/100
            self.move_group(next_point)
            self.clear_group()
        else:
            raise ValueError

    def vertical_move2(self, first_pose, value, set):
        if self.carry_on_routine:
            next_point = first_pose
            next_point.position.z += (value/100)*set
            self.move_group(next_point)
            self.clear_group()
        else:
            raise ValueError

    def place(self):
        if self.carry_on_routine:
            self.group.set_named_target("low_pose")
            self.group.go()
            self.group.stop()
        else:
            raise ValueError

    def approach(self, msg):
        self.group.set_max_velocity_scaling_factor(0.75)
        joint_goal = self.group.get_current_joint_values()
        joint_goal[0] = -0.000190902556141577
        joint_goal[1] = -0.6355551061881216
        joint_goal[2] = 0.0005800443025828713
        joint_goal[3] = -2.6969856774848804
        joint_goal[4] = 7.176671650806756e-05
        joint_goal[5] = 2.0608427823384603
        joint_goal[6] = 0.7847883398630692
        self.group.go(joint_goal, wait=True)
        self.clear_group()

    def move_group(self, target):
        self.group.set_pose_target(target)
        self.group.go(wait=True)

    def clear_group(self):
        self.group.stop()
        self.group.clear_pose_targets()

    def stop_routine(self, msg):
        self.carry_on_routine = False

    def routine(self, msg):
        '''
        '''
        print("=============================================")
        print()
        print("*****            TEST BENCH             *****")
        print()
        width = msg.width/100  # cm
        force = msg.force  # N
        reps = msg.reps
        sets = msg.sets
        h_interval = msg.h_interval
        v_interval = msg.v_interval
        print("Desired width: {}m".format(width))
        print("Desired force: {}N".format(force))
        print("Desired repetitions: {}".format(reps))
        print("Desired sets: {}".format(sets))
        print("Desired h_interval: {}m".format(h_interval))
        print("Desired v_interval: {}m".format(v_interval))
        print()
        print("=============================================")
        try:
            first_pose = self.group.get_current_pose().pose
            for set in range(0, sets):
                print("Horizontal_move")
                for rep in range(0, reps):
                    self.group.remember_joint_values("low_pose")
                    self.grasp2(width, force)
                    time.sleep(1)
                    self.approach({})
                    time.sleep(1)
                    #self.vertical_move(-0.13)
                    self.place()
                    self.release()
                    time.sleep(0.5)
                    #self.approach({})
                    #self.place()
                    if (rep<reps-1):
                        self.horizontal_move2(first_pose, h_interval, rep)
                if (set<sets-1):
                    self.horizontal_move2(first_pose, -(reps-1)*h_interval, set)
                    self.vertical_move2(first_pose, v_interval, rep)
                else:
                    self.approach({})
        except:
            print("Stopped")


if __name__ == '__main__':
    rospy.init_node('cobot_control')
    test = TestBench()
    rospy.spin()
