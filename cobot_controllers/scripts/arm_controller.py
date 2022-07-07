#!/usr/bin/env python3

import time
import sys
import rospy
import moveit_commander
from cobot_controllers.moveit_controller import MoveitArm
from cobot_msgs.srv import *
from franka_msgs.srv import SetForceTorqueCollisionBehavior, SetForceTorqueCollisionBehaviorRequest
from std_srvs.srv import Trigger
from control_msgs.msg import FollowJointTrajectoryActionResult

if __name__ == '__main__':
    rospy.init_node('moveit_arm_control', anonymous=True)

    moveit_commander.roscpp_initialize(sys.argv)
    group_name = "panda_arm"

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander(group_name)

    arm = MoveitArm(group)
    #reset_service = rospy.Service('/reset_env', ResetEnv, arm.handle_reset)
    #rospy.wait_for_service('/reset_env')
    #print("/reset_env ready")
    action_service = rospy.Service('/take_action', TakeAction, arm.handle_move_to_target)
    rospy.wait_for_service('/take_action')
    print("/take_action ready")
    cartesian_action_service = rospy.Service('/take_cartesian_action', TakeCartesianAction, arm.handle_move_to_cartesian_target)
    rospy.wait_for_service('/take_cartesian_action')
    print("/take_cartesian_action ready")
    cartesian_action_service = rospy.Service('/stop_actions', StopAction, arm.handle_stop)
    rospy.wait_for_service('/stop_actions')
    print("/stop_actions ready")
    #cartesian_action_service = rospy.Service('/read_values', ReadValues, arm.handle_read_bot_info)
    #rospy.wait_for_service('/read_values')
    #print("/read_values ready")
    
    cartesian_action_service_2d = rospy.Service('/take_2D_cartesian_action', Take2DCartesianAction, arm.handle_move_to_2D_cartesian_target)
    rospy.wait_for_service('/take_2D_cartesian_action')
    print("/take_2D_cartesian_action")
    cartesian_action_service_1d = rospy.Service('/take_1D_cartesian_action', Take1DCartesianAction, arm.handle_move_to_1D_cartesian_target)
    rospy.wait_for_service('/take_1D_cartesian_action')
    print("/take_1D_cartesian_action")

    start_engine_service = rospy.Service('/move_above_engine', Trigger, arm.handle_move_above_engine)
    rospy.wait_for_service('/move_above_engine')
    print("/move_above_engine")

    arm_threshold_srv = rospy.ServiceProxy('/franka_control/set_force_torque_collision_behavior', SetForceTorqueCollisionBehavior)
    rospy.wait_for_service('/franka_control/set_force_torque_collision_behavior')
    print("/franka_control/set_force_torque_collision_behavior ready")

    rospy.Subscriber("/position_joint_trajectory_controller/follow_joint_trajectory/result", FollowJointTrajectoryActionResult, arm.monitor_trajectory)
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


