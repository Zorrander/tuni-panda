#!/usr/bin/env python

import rospy
from std_srvs.srv import Trigger
from cobot_controllers.gripper import Gripper
from cobot_msgs.srv import *
from std_msgs.msg import Empty

def homing(request, robot):
    robot.homing()
    return TriggerResponse(True)

def stop(request, robot):
    robot.stop()
    return TriggerResponse(True)

def move(request, robot):
    speed = request.speed if request.speed > 0 else 20.0
    width = request.width/100 if request.width > 0 else 0.08
    robot.move(speed, width)
    return MoveGripperResponse(True)

def grasp(request, robot):
    print("""grasp""")
    force = request.force if request.force > 0 else 70.0
    #width = request.width/100 if request.width > 0 else 0.004
    width = 0.03
    robot.grasp(force, width)
    return GraspResponse(True)

if __name__ == '__main__':
    rospy.init_node('panda_gripper_control')

    panda_gripper = Gripper()

    sub = rospy.Subscriber("/human_ready", Empty, panda_gripper.grasp_triggered)

    homing_service = rospy.Service("home_gripper", Trigger, lambda msg: homing(msg,panda_gripper))
    stop_gripper_service = rospy.Service("stop_gripper", Trigger, lambda msg: stop(msg,panda_gripper))
    move_gripper_service = rospy.Service("move_gripper", MoveGripper, lambda msg: move(msg,panda_gripper))
    grasping_service = rospy.Service("grasp", Grasp, lambda msg: grasp(msg,panda_gripper))
    print("Gripper Ready")
    rospy.spin()
