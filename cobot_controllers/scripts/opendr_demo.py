#!/usr/bin/env python3

import time
import rospy
from cobot_msgs.srv import *

engine_hover_pose = [2.2735651273908486, -0.5375771900478162, 0.6629416082554471, -2.270488261072275,0.360277423871888,1.8296079346338907, 0.3163624463594622]

bolt_pose = [2.167535255593166, -0.7421887756565161, -1.7394938050654896, -1.993554066674753, -0.800695950574345, 2.015025856835834, 0.058869495841154154]

if __name__ == '__main__':
    rospy.init_node('recordings')
    
    move_to_target_service = rospy.ServiceProxy("/take_action", TakeAction)
    cartesian_action_service_1D = rospy.ServiceProxy('/take_1D_cartesian_action', Take1DCartesianAction)
    grasping_service = rospy.ServiceProxy("grasp", Grasp)
    move_gripper_service = rospy.ServiceProxy("move_gripper", MoveGripper)

    # Make sure the gripper is working properly
    try:
        move_gripper_service(20.0, 0.02) 
        move_gripper_service(20.0, 0.08) 
    except rospy.ServiceException as exc:
        print("Gripper not operational")
    else:
        move_to_target_service(bolt_pose)
        time.sleep(3)
        grasping_service(width=0.017, force=50.0)
        time.sleep(3)
        cartesian_action_service_1D(z_pose=0.4)
        move_to_target_service(engine_hover_pose)