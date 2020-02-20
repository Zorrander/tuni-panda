#!/usr/bin/env python

from cobot_controllers.gripper import Gripper

if __name__ == '__main__':
    rospy.init_node('panda_gripper_control')
    panda_gripper = Gripper()

    homing_service = rospy.Service("home_gripper", Trigger, panda_gripper.homing)
    stop_gripper_service = rospy.Service("stop_gripper", Trigger, panda_gripper.stop)
    move_gripper_service = rospy.Service("move_gripper", MoveGripper, panda_gripper.move)
    grasping_service = rospy.Service("grasp", Grasp, panda_gripper.grasp)

    rospy.spin()
