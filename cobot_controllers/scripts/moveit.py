#!/usr/bin/env python

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from cobot_controllers.arm import Arm
from cobot_msgs.srv import *
from franka_control.msg import ErrorRecoveryActionGoal

test_joint_positions_array = [-0.000190902556141577, -0.6355551061881216, 0.0005800443025828713,
                    -2.6969856774848804, 7.176671650806756e-05, 2.0608427823384603, 0.7847883398630692]

test_cartesian_postion = {
    'x': 0.34,
    'y': 0.0,
    'z': 0.32
}

def go_to_joint_space_goal(request, robot):
    robot.go_to_joint_space_goal(request.position)
    return ReachJointPoseResponse(True)

def go_to_cartesian_goal(request, robot):
    print("RECEIVED REQUEST")
    print(request)
    robot.go_to_cartesian_goal(request.point)
    return ReachCartesianPoseResponse(True)

def move_to(request, robot):
    robot.move_to(request.name)
    return NamedTargetResponse(True)

def store_position(request, robot):
    robot.store_position(request.name)
    return NamedTargetResponse(True)

def set_speed(request, robot):
    robot.set_speed(request.speed_factor)
    return RobotSpeedResponse(True)

def list_targets(request, robot):
    print("Received request")
    return ListTargetsResponse(robot.list_targets())

def reset(request, pub):
    print("Received reset request")
    pub.publish(ErrorRecoveryActionGoal())
    return TriggerResponse(True, "Robot recovery")

if __name__ == '__main__':
    rospy.init_node('panda_arm_control')
    panda_arm = Arm()

    pub = rospy.Publisher('/franka_control/error_recovery/goal', ErrorRecoveryActionGoal, queue_size=10)

    s1 = rospy.Service('go_to_joint_space_goal', ReachJointPose, lambda msg: go_to_joint_space_goal(msg,panda_arm))
    s2 = rospy.Service("go_to_cartesian_goal", ReachCartesianPose, lambda msg: go_to_cartesian_goal(msg,panda_arm))
    s3 = rospy.Service('move_to', NamedTarget, lambda msg: move_to(msg,panda_arm) )
    s4 = rospy.Service("store_position", NamedTarget, lambda msg: store_position(msg,panda_arm) )
    s5 = rospy.Service('set_speed', RobotSpeed, lambda msg: set_speed(msg,panda_arm) )
    s6 = rospy.Service('get_targets', ListTargets, lambda msg: list_targets(msg,panda_arm) )
    s7 = rospy.Service('reset', Trigger, lambda msg: reset(msg, pub) )

    print("Arm ready")
    rospy.spin()
