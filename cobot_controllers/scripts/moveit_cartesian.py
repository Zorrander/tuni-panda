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

      # Misc variables
      self.robot = robot
      self.scene = scene
      self.group = group
      self.display_trajectory_publisher = display_trajectory_publisher
      self.tf_listener = tf.TransformListener()

      rospy.Subscriber("/moveit_cartesian_goal", Point, self.go_to_goal)
      print("READY")

    def transform_point(self, msg):
        point = PointStamped()
        point.header.frame_id = "camera_color_optical_frame"
        point.header.stamp = self.tf_listener.getLatestCommonTime("/camera_color_optical_frame", "/panda_link0")
        point.point.x = msg.x
        point.point.y = msg.y
        point.point.z = msg.z
        return self.tf_listener.transformPoint("panda_link0", point)

    def go_to_goal(self, msg):
        wpose = self.group.get_current_pose().pose
        next_point = wpose
        tag_point_r = self.transform_point(msg)
        next_point.position.x = tag_point_r.point.x
        next_point.position.y = tag_point_r.point.y
        next_point.position.z = tag_point_r.point.z + 20
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
