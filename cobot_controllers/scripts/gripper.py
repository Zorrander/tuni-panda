#!/usr/bin/env python

import rospy
from std_srvs.srv import Trigger
from cobot_controllers.gripper import Gripper
from cobot_controllers.srv import Grasp, MoveGripper


def homing(request, robot):
    robot.homing()

def stop(request, robot):
    robot.stop()

def move(request, robot):
    speed = request.speed if request.speed > 0 else 20.0
    width = request.width/100 if request.width > 0 else 0.08
    robot.move(speed, width)

def grasp(request, robot):
    force = request.force if request.force > 0 else 20.0
    width = request.width/100 if request.width > 0 else 0.004
    robot.grasp(force, width)

if __name__ == '__main__':
    rospy.init_node('panda_gripper_control')

    panda_gripper = Gripper()

    homing_service = rospy.Service("home_gripper", Trigger, lambda msg: homing(msg,panda_gripper))
    stop_gripper_service = rospy.Service("stop_gripper", Trigger, lambda msg: stop(msg,panda_gripper))
    move_gripper_service = rospy.Service("move_gripper", MoveGripper, lambda msg: move(msg,panda_gripper))
    grasping_service = rospy.Service("grasp", Grasp, lambda msg: grasp(msg,panda_gripper))

    rospy.spin()
