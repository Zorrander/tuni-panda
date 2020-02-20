#!/usr/bin/env python

import tf
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import *
from cobot_vision.srv import TagPose
from cobot_controllers.srv import ReachTag,ReachTagResponse

class MoveitCartesianController(object):
    def __init__(self):
      moveit_commander.roscpp_initialize(sys.argv)

      group_name = "panda_arm"
      self.robot = moveit_commander.RobotCommander()
      self.scene = moveit_commander.PlanningSceneInterface()
      self.group = moveit_commander.MoveGroupCommander(group_name)

      ## Add the table
      p = PoseStamped()
      p.header.frame_id = self.robot.get_planning_frame()
      p.pose.position.x = 0.
      p.pose.position.y = 0.
      p.pose.position.z = -.05
      self.scene.add_box("table", p, (3.0, 1.0, 0.1))

      ## Add the windows
      # RPY to convert: 90deg, 0, -90deg
      #q = quaternion_from_euler(1.5707, 0, -1.5707)
      p = PoseStamped()
      p.header.frame_id = self.robot.get_planning_frame()
      p.pose.position.x = -0.30
      p.pose.position.y = 0.
      p.pose.position.z = -.05
      p.pose.orientation.x = 0.0
      p.pose.orientation.y = 1.0
      p.pose.orientation.z = 0.0
      p.pose.orientation.w = 1.0
      self.scene.add_box("windows", p, (3.0, 1.0, 0.1))

      rospy.wait_for_service('calculate_tag_pose')
      self.tag_detector = rospy.ServiceProxy('calculate_tag_pose', TagPose)

      s = rospy.Service('reach_tag', ReachTag, self.go_to_goal)
      s = rospy.Service("move_start", Trigger, self.move_to_start)
      print("READY")

    def move_to_start(self, empty_msg):
        print("Reset robot")
        response = TriggerResponse()
        try:
            self.group.set_named_target('ready')
            self.group.go()
            response.success = True
        except:
            response.success = False
        return response

    def go_to_goal(self, msg):
        response = ReachTagResponsei()
        try:
            tag_pose_resp = self.tag_detector(msg.tag_id)
            ee_pose = self.group.get_current_pose().pose
            next_point = ee_pose
            next_point.position.x = tag_pose_resp.tag_pose.position.x
            next_point.position.y = tag_pose_resp.tag_pose.position.y
            next_point.position.z = tag_pose_resp.tag_pose.position.z + 0.20
            print(next_point)
            print("============ Press `Enter` to go to tag {} ...".format(msg.tag_id))
            raw_input()
            self.group.set_pose_target(next_point)
            plan = self.group.go(wait=True)
            # Calling `stop()` ensures that there is no residual movement
            self.group.stop()
            self.group.clear_pose_targets()
            response.success = True
        except:
            response.success = False
        return response


if __name__ == '__main__':
    rospy.init_node('cobot_cartesian_control')
    test = MoveitCartesianController()
    rospy.spin()
