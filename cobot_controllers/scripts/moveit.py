#!/usr/bin/env python

import rospy
from cobot_controllers.arm import Arm


def go_to_joint_space_goal(request, robot):
    robot.go_to_joint_space_goal()

def go_to_cartesian_goal(request, robot):
    robot.go_to_cartesian_goal()

def move_to(request, robot):
    robot.move_to()

def store_position(request, robot):
    robot.store_position()

def set_speed(request, robot):
    robot.set_speed()

if __name__ == '__main__':
    rospy.init_node('panda_arm_control')
    panda_arm = Arm()

    s = rospy.Service('go_to_joint_space_goal', ReachJointPose, go_to_goal(msg, panda_arm))
    s = rospy.Service("go_to_cartesian_goal", ReachCartesianPose, panda_arm.move_to_start)
    s = rospy.Service('move_to', ReachNamedTarget, panda_arm.go_to_goal)
    s = rospy.Service("store_position", Trigger, panda_arm.move_to_start)
    s = rospy.Service('set_speed', Trigger, panda_arm.go_to_goal)
    print("Arm ready")
    rospy.spin()
