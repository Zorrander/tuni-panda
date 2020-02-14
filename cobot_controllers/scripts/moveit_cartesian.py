#!/usr/bin/env python

import sys
import copy
import rospy
import tf
import moveit_commander
import moveit_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import *
from std_msgs.msg import Int8
from cobot_vision.srv import TagPose

class MoveGroupPythonIntefaceTutorial(object):
    """MoveGroupPythonIntefaceTutorial"""
    def __init__(self):
      super(MoveGroupPythonIntefaceTutorial, self).__init__()

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

      scene = moveit_commander.PlanningSceneInterface()
      robot = moveit_commander.RobotCommander()

      rospy.sleep(2)

      ## Add the table
      p = PoseStamped()
      p.header.frame_id = robot.get_planning_frame()
      p.pose.position.x = 0.
      p.pose.position.y = 0.
      p.pose.position.z = -.05
      scene.add_box("table", p, (3.0, 1.0, 0.1))

      ## Add the windows
      # RPY to convert: 90deg, 0, -90deg
      #q = quaternion_from_euler(1.5707, 0, -1.5707)
      p = PoseStamped()
      p.header.frame_id = robot.get_planning_frame()
      p.pose.position.x = -0.30
      p.pose.position.y = 0.
      p.pose.position.z = -.05
      p.pose.orientation.x = 0.0
      p.pose.orientation.y = 1.0
      p.pose.orientation.z = 0.0
      p.pose.orientation.w = 1.0
      scene.add_box("windows", p, (3.0, 1.0, 0.1))

      # Misc variables
      self.robot = robot
      self.scene = scene
      self.group = group
      self.display_trajectory_publisher = display_trajectory_publisher
      self.tf_listener = tf.TransformListener()

      rospy.wait_for_service('calculate_tag_pose')
      self.tag_detector = rospy.ServiceProxy('calculate_tag_pose', TagPose)

      rospy.Subscriber("/tag_goal", Int8, self.go_to_goal)
      print("READY")

    def go_to_goal(self, msg):
        tag_pose_resp = self.tag_detector(msg.data)
        ee_pose = self.group.get_current_pose().pose
        next_point = ee_pose
        next_point.position.x = tag_pose_resp.tag_pose.position.x
        next_point.position.y = tag_pose_resp.tag_pose.position.y
        next_point.position.z = tag_pose_resp.tag_pose.position.z + 0.20
        print(next_point)
        print "============ Press `Enter` to execute the motion ..."
        raw_input()
        self.group.set_pose_target(next_point)
        plan = self.group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.group.stop()
        self.group.clear_pose_targets()


if __name__ == '__main__':
    rospy.init_node('cobot_control')
    test = MoveGroupPythonIntefaceTutorial()
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
