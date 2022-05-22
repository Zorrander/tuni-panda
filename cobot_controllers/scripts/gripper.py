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

def move(request, robot, action_performed_pub):
    print("MOVE GRIPPER")
    speed = request.speed if request.speed > 0 else 20.0
    # width = request.width/100 if request.width > 0 else 0.05
    width = request.width
    robot.move(speed, width)
    print("action performed")
    action_performed_pub.publish(Empty())
    return MoveGripperResponse(True)

def grasp(request, robot, action_performed_pub):
    print("""grasp""")
    force = request.force if request.force > 0 else 70.0
    #width = request.width/100 if request.width > 0 else 0.004
    width = request.width # 0.008
    robot.grasp(force, width)
    # robot.grasp2(width, force)
    print("action performed")
    action_performed_pub.publish(Empty())
    return GraspResponse(True)

if __name__ == '__main__':
    rospy.init_node('panda_gripper_control')

    panda_gripper = Gripper()

    # sub = rospy.Subscriber("/human_ready", Empty, panda_gripper.grasp_triggered)

    action_performed_pub = rospy.Publisher('/action_performed', Empty, queue_size=10)

    homing_service = rospy.Service("home_gripper", Trigger, lambda msg: homing(msg,panda_gripper))
    stop_gripper_service = rospy.Service("stop_gripper", Trigger, lambda msg: stop(msg,panda_gripper))
    move_gripper_service = rospy.Service("move_gripper", MoveGripper, lambda msg: move(msg,panda_gripper, action_performed_pub))
    grasping_service = rospy.Service("grasp", Grasp, lambda msg: grasp(msg,panda_gripper, action_performed_pub))
    print("Gripper Ready")
    rospy.spin()
