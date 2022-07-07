#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty, Float64MultiArray
from std_srvs.srv import Trigger, TriggerResponse
from cobot_controllers.arm import Arm
from cobot_msgs.srv import *
from geometry_msgs.msg import Pose
from franka_control.msg import ErrorRecoveryActionGoal

test_joint_positions_array = [-0.000190902556141577, -0.6355551061881216, 0.0005800443025828713,
                    -2.6969856774848804, 7.176671650806756e-05, 2.0608427823384603, 0.7847883398630692]

test_cartesian_postion = {
    'x': 0.34,
    'y': 0.0,
    'z': 0.32
}

def go_to_joint_space_goal(request, robot):
    robot.go_to_joint_space_goal(request.position.data)
    return ReachJointPoseResponse(True)

def go_to_cartesian_goal(request, robot, pub, action_performed_pub):
    print("RECEIVED REQUEST")
    print(request)
    robot.switchToArmNavigationControl("")
    reset(request, pub)
    robot.go_to_cartesian_goal(request.pose)
    action_performed_pub.publish(Empty())
    return ReachCartesianPoseResponse(True)

def move_to(request, robot, action_performed_pub):
    robot.move_to(request.name)
    action_performed_pub.publish(Empty())
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

def reset(request, pub, action_performed_pub):
    print("Received reset request")
    pub.publish(ErrorRecoveryActionGoal())
    action_performed_pub.publish(Empty())
    return TriggerResponse(True, "Robot recovery")

def idle(request, pub, action_performed_pub):
    action_performed_pub.publish(Empty())
    return TriggerResponse(True, "Robot idling")

def communicate(request, pub, action_performed_pub):
    action_performed_pub.publish(Empty())
    return TriggerResponse(True, "Robot communicate")

if __name__ == '__main__':
    rospy.init_node('panda_arm_control')

    cartesian_pub_controller = rospy.Publisher('/new_target', Pose, queue_size=10)
    # joint_pub_controller = rospy.Publisher('/new_target', Float64MultiArray, queue_size=10)
    panda_arm = Arm(cartesian_pub_controller)

    sub = rospy.Subscriber("/target_reached", Empty, panda_arm.switchToForceImpedanceControl)

    pub = rospy.Publisher('/franka_control/error_recovery/goal', ErrorRecoveryActionGoal, queue_size=10)
    action_performed_pub = rospy.Publisher('/action_performed', Empty, queue_size=10)

    s1 = rospy.Service('go_to_joint_space_goal', ReachJointPose, lambda msg: go_to_joint_space_goal(msg,panda_arm))
    s2 = rospy.Service("go_to_cartesian_goal", ReachCartesianPose, lambda msg: go_to_cartesian_goal(msg,panda_arm, pub, action_performed_pub))
    s3 = rospy.Service('move_to', NamedTarget, lambda msg: move_to(msg,panda_arm, action_performed_pub) )
    s4 = rospy.Service("store_position", NamedTarget, lambda msg: store_position(msg,panda_arm) )
    s5 = rospy.Service('set_speed', RobotSpeed, lambda msg: set_speed(msg,panda_arm) )
    s6 = rospy.Service('get_targets', ListTargets, lambda msg: list_targets(msg,panda_arm) )
    s7 = rospy.Service('reset', Trigger, lambda msg: reset(msg, pub, action_performed_pub) )
    s8 = rospy.Service('idle', Trigger, lambda msg: idle(msg, pub, action_performed_pub) )
    s9 = rospy.Service('communicate', Trigger, lambda msg: idle(msg, pub, action_performed_pub) )

    print("Arm ready")

    rospy.spin()
