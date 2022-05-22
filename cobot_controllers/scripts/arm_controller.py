#!/usr/bin/env python

import time
import sys
import rospy
import moveit_commander
from cobot_controllers.srv import *
from cobot_controllers.moveit_controller import MoveitArm
from cobot_msgs.srv import *
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Empty
from franka_control.msg import ErrorRecoveryActionGoal
# from franka_msgs.srv import SetForceTorqueCollisionBehavior, SetForceTorqueCollisionBehaviorRequest

'''
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
'''

def idle(request, pub, action_performed_pub):
    action_performed_pub.publish(Empty())
    return TriggerResponse(True, "Robot idling")

'''
def communicate(request, pub, action_performed_pub):
    action_performed_pub.publish(Empty())
    return TriggerResponse(True, "Robot communicate")
'''

if __name__ == '__main__':
    rospy.init_node('moveit_arm_control', anonymous=True)

    moveit_commander.roscpp_initialize(sys.argv)
    group_name = "panda_arm"

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander(group_name)

    arm = MoveitArm(group)

    #sub = rospy.Subscriber("/target_reached", Empty, panda_arm.switchToForceImpedanceControl)

    pub = rospy.Publisher('/franka_control/error_recovery/goal', ErrorRecoveryActionGoal, queue_size=10)
    action_performed_pub = rospy.Publisher('/action_performed', Empty, queue_size=10)

    reset_service = rospy.Service('/reset_env', ResetEnv, arm.handle_reset)
    rospy.wait_for_service('/reset_env')
    print("/reset_env ready")
    action_service = rospy.Service('/take_action', TakeAction, arm.handle_move_to_target)
    rospy.wait_for_service('/take_action')
    print("/take_action ready")
    cartesian_action_service = rospy.Service('/take_cartesian_action', TakeCartesianAction, arm.handle_move_to_cartesian_target)
    rospy.wait_for_service('/take_cartesian_action')
    print("/take_cartesian_action ready")
    cartesian_action_service = rospy.Service('/stop_actions', StopAction, arm.handle_stop)
    rospy.wait_for_service('/stop_actions')
    print("/stop_actions ready")
    cartesian_action_service = rospy.Service('/read_values', ReadValues, arm.handle_read_bot_info)
    rospy.wait_for_service('/read_values')
    print("/read_values ready")
    
    #s1 = rospy.Service('go_to_joint_space_goal', ReachJointPose, lambda msg: go_to_joint_space_goal(msg,panda_arm))
    #s2 = rospy.Service("go_to_cartesian_goal", ReachCartesianPose, lambda msg: go_to_cartesian_goal(msg,panda_arm, pub, action_performed_pub))
    #s3 = rospy.Service('move_to', NamedTarget, lambda msg: move_to(msg,panda_arm, action_performed_pub) )
    #s4 = rospy.Service("store_position", NamedTarget, lambda msg: store_position(msg,panda_arm) )
    #s5 = rospy.Service('set_speed', RobotSpeed, lambda msg: set_speed(msg,panda_arm) )
    #s6 = rospy.Service('get_targets', ListTargets, lambda msg: list_targets(msg,panda_arm) )
    #s7 = rospy.Service('reset', Trigger, lambda msg: reset(msg, pub, action_performed_pub) )
    s8 = rospy.Service('idle', Trigger, lambda msg: idle(msg, pub, action_performed_pub) )
    #s9 = rospy.Service('communicate', Trigger, lambda msg: idle(msg, pub, action_performed_pub) )

    #arm_threshold_srv = rospy.ServiceProxy('/franka_control/set_force_torque_collision_behavior', SetForceTorqueCollisionBehavior)
    #rospy.wait_for_service('/franka_control/set_force_torque_collision_behavior')
    #print("/franka_control/set_force_torque_collision_behavior ready")

    #req = SetForceTorqueCollisionBehaviorRequest()
    #print(req)
    #req.lower_torque_thresholds_nominal = [36.0, 36.0, 32.0, 32.0, 28.0, 24.0, 20.0]
    #req.upper_torque_thresholds_nominal = [36.0, 36.0, 32.0, 32.0, 28.0, 24.0, 20.0]
    #req.lower_force_thresholds_nominal = [30.0, 30.0, 30.0, 40.0, 40.0, 40.0]
    #req.upper_force_thresholds_nominal = [30.0, 30.0, 30.0, 40.0, 40.0, 40.0]
    #print(req)
    #resp = arm_threshold_srv(req)
    #print(resp)
    print("ARM READY")

    rospy.spin()


