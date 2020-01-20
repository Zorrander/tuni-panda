#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import PoseArray


class MoveGroupPythonIntefaceTutorial(object):
    """MoveGroupPythonIntefaceTutorial"""
    def __init__(self):
      super(MoveGroupPythonIntefaceTutorial, self).__init__()

      ## BEGIN_SUB_TUTORIAL setup
      ##
      ## First initialize `moveit_commander`_ and a `rospy`_ node:
      moveit_commander.roscpp_initialize(sys.argv)

      ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
      ## the robot:
      robot = moveit_commander.RobotCommander()

      ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
      ## to the world surrounding the robot:
      scene = moveit_commander.PlanningSceneInterface()

      ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
      ## to one group of joints.  In this case the group is the joints in the Panda
      ## arm so we set ``group_name = panda_arm``. If you are using a different robot,
      ## you should change this value to the name of your robot arm planning group.
      ## This interface can be used to plan and execute motions on the Panda:
      group_name = "panda_arm"
      group = moveit_commander.MoveGroupCommander(group_name)

      ## We create a `DisplayTrajectory`_ publisher which is used later to publish
      ## trajectories for RViz to visualize:
      display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                     moveit_msgs.msg.DisplayTrajectory,
                                                     queue_size=20)

      ## END_SUB_TUTORIAL

      ## BEGIN_SUB_TUTORIAL basic_info
      ##
      ## Getting Basic Information
      ## ^^^^^^^^^^^^^^^^^^^^^^^^^
      # We can get the name of the reference frame for this robot:
      planning_frame = group.get_planning_frame()
      print "============ Reference frame: %s" % planning_frame

      # We can also print the name of the end-effector link for this group:
      eef_link = group.get_end_effector_link()
      print "============ End effector: %s" % eef_link

      # We can get a list of all the groups in the robot:
      group_names = robot.get_group_names()
      print "============ Robot Groups:", robot.get_group_names()

      # Sometimes for debugging it is useful to print the entire state of the
      # robot:
      print "============ Printing robot state"
      print robot.get_current_state()
      print ""
      ## END_SUB_TUTORIAL

      # Misc variables
      self.box_name = ''
      self.robot = robot
      self.scene = scene
      self.group = group
      self.display_trajectory_publisher = display_trajectory_publisher
      self.planning_frame = planning_frame
      self.eef_link = eef_link
      self.group_names = group_names


    def lift(self):
        wpose = self.group.get_current_pose().pose
        next_point = wpose
        next_point.position.z += 0.30
        self.group.set_pose_target(next_point)
        plan = self.group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.group.stop()
        self.group.clear_pose_targets()

    def go_down(self):
        wpose = self.group.get_current_pose().pose
        # Gather poses from ROS message
        next_point = wpose
        next_point.position.z -= 0.27
        self.group.set_pose_target(next_point)
        plan = self.group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.group.stop()
        self.group.clear_pose_targets()

    def approach(self):
        #wpose = self.group.get_current_pose().pose
        joint_goal = self.group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = 0
        joint_goal[2] = 0
        joint_goal[3] = -pi/1.5
        joint_goal[4] = 0
        joint_goal[5] = pi/1.5
        joint_goal[6] = pi/4
        self.group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        self.group.stop()


    def routine(self):
        '''
        Make a chopstick go through for points corresponding to the 4 spaces between one's fingers.
        '''
        self.approach()
        print "============ Press `Enter` to go do down ..."
        raw_input()
        self.go_down()
        print "============ Press `Enter` to lift ..."
        raw_input()
        self.lift()

if __name__ == '__main__':
    rospy.init_node('cobot_control')
    test = MoveGroupPythonIntefaceTutorial()
    test.routine()
    rospy.spin()


'''
rostopic pub /franka_grippegrasp/goal franka_gripper/GraspngActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  width: 0.02
  epsilon:
    inner: 0.01
    outer: 0.01
  speed: 50.0
  force: 10.0"
'''
