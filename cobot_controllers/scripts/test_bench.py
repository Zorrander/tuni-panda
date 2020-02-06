#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from math import pi
from std_msgs.msg import String, Empty
from moveit_commander.conversions import pose_to_list
from cobot_controllers.msg import Test
import actionlib
from franka_gripper.msg import HomingAction, HomingActionGoal, GraspAction, GraspActionGoal, GraspGoal


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

      self.start_sub = rospy.Subscriber("/test_bench", Test, self.routine)
      self.homing_sub = rospy.Subscriber("/homing_cmd", Empty, self.release)
      self.gripper_homing_client = actionlib.SimpleActionClient('/franka_gripper/homing', HomingAction)
      self.gripper_homing_client.wait_for_server()
      self.gripper_grasp_client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)
      self.gripper_grasp_client.wait_for_server()
      print("Gripper connected")

    def release(self, msg):
        goal = HomingActionGoal(goal={})
        # Sends the goal to the action server.
        self.gripper_homing_client.send_goal(goal)
        # Waits for the server to finish performing the action.
        self.gripper_homing_client.wait_for_result()
        # Prints out the result of executing the action
        return self.gripper_homing_client.get_result()

    def grasp(self, width, force):
        ''' width, epsilon_inner, epsilon_outer, speed, force '''
        goal_msg = GraspGoal()
        goal_msg.epsilon.inner = 0.01
        goal_msg.epsilon.outer = 0.01
        goal_msg.speed = 20.0
        goal_msg.width = float(width)
        goal_msg.force = float(force)
        goal = GraspActionGoal()
        #goal.goal = goal_msg
        print(goal)
        self.gripper_grasp_client.send_goal(goal)
        self.gripper_grasp_client.wait_for_result()
        return self.gripper_grasp_client.get_result()

    def vertical_move(self, value):
        wpose = self.group.get_current_pose().pose
        next_point = wpose
        next_point.position.z += value
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


    #def routine(self, msg):
    def routine(self):
        '''
        Make a chopstick go through for points corresponding to the 4 spaces between one's fingers.
        '''
        print("=============================================")
        print()
        print("*****            TEST BENCH             *****")
        print()
        #width = msg.width
        #force = msg.force
        #reps = msg.reps
        width = 0.02
        force = 20.0
        reps = 1

        print("Desired width: {}m".format(width))
        print("Desired force: {}N".format(force))
        print("Desired repetitions: {}".format(reps))
        print()
        print("=============================================")
        self.approach()
        print "============ Press `Enter` to go do down ..."
        raw_input()
        self.vertical_move(-0.21)
        while not rospy.is_shutdown():
            print "============ Press `Enter` to go up..."
            raw_input()
            #self.grasp(width, force)
            self.vertical_move(0.21)
            print "============ Press `Enter` to release..."
            raw_input()
            self.vertical_move(-0.21)
            self.release({})

if __name__ == '__main__':
    rospy.init_node('cobot_control')
    test = MoveGroupPythonIntefaceTutorial()
    test.routine()



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
