#!/usr/bin/env python3

import rospy
from cobot_msgs.srv import *
from sensor_msgs.msg import JointState
import time
import roslib
import math
import tf
from geometry_msgs.msg import *


class Values(object):
	def __init__(self, listener):
		self.listener = listener
		self.joints_state = []

	def transform_ee_pose(self):
	    pose = PoseStamped()
	    #pose.header.frame_id = "panda_leftfinger"
	    pose.header.frame_id = "panda_EE"
	    pose.header.stamp = rospy.Time(0)
	    # t = listener.getLatestCommonTime("/panda_leftfinger", "/panda_link0")
	    # pose.header.stamp = t
	    pose.pose.position.x = 0
	    pose.pose.position.y = 0
	    pose.pose.position.z = 0
	    pose.pose.orientation.w = 1.0
	    transformed_pose = self.listener.transformPose("panda_link0", pose)

	    ee_position = transformed_pose.pose.position
	    ee_orientation = transformed_pose.pose.orientation
	    return ([ee_position.x, ee_position.y, ee_position.z], [ee_orientation.x, ee_orientation.y, ee_orientation.z, ee_orientation.w], self.joints_state[:-2])


	def handle_joint(self, msg):
	    self.joints_state = msg.position

	def handle_read_request(self, req):
	    ee_pose, ee_oriention, joints_state = self.transform_ee_pose()
	    return ReadValuesResponse(ee_pose, ee_oriention, joints_state)




if __name__ == '__main__':
    rospy.init_node('read_values_service', anonymous=True)


    listener = tf.TransformListener()
    v = Values(listener)

    reset_service = rospy.Service('/read_values', ReadValues, v.handle_read_request)
    rospy.wait_for_service('/read_values')
    print("/read_values ready")

    rospy.Subscriber('/joint_states', JointState, v.handle_joint)

    rospy.spin()
