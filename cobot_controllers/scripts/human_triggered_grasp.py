#!/usr/bin/env python

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from cobot_controllers.arm import Arm
from cobot_msgs.srv import *
from geometry_msgs.msg import Pose
from franka_control.msg import ErrorRecoveryActionGoal


if __name__ == '__main__':
    rospy.init_node('human_trigger_grasp')

    pub_controller = rospy.Publisher('/new_target', Pose, queue_size=10)

    test_pose = Pose()
    test_pose.position.x = 0.50
    test_pose.position.y = 0.0
    test_pose.position.z = 0.20

    pub_controller.publish(test_pose)

    rospy.spin()
