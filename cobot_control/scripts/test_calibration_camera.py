#!/usr/bin/env python

import time
import rospy
from cobot_controllers.srv import NamedTarget, ReachCartesianPose
from cobot_vision.srv import TagPose

def test_calibration_camera():
    rospy.wait_for_service('go_to_cartesian_goal')
    rospy.wait_for_service('move_to')
    rospy.wait_for_service('calculate_tag_pose')
    tag_detector = rospy.ServiceProxy('calculate_tag_pose', TagPose)
    move_to_pose = rospy.ServiceProxy('go_to_cartesian_goal', ReachCartesianPose)
    move_to_named_target = rospy.ServiceProxy('move_to', NamedTarget)
    tags = [18, 20, 22, 16, 24]
    calibrated = True
    try:
        print("============ Press `Enter` to go through the test tags ========== ")
        raw_input()
        move_to_named_target("ready")
        time.sleep(1)
        for tag in tags:
            print("going to tag {} ".format(tag))
            tag_pose_resp = tag_detector(tag)
            tag_pose_resp.tag_pose.position.z += 0.20
            print("Pose: {} ".format(tag_pose_resp))
            calibrated = move_to_pose(tag_pose_resp.tag_pose)
            print("Result: {}".format(calibrated.success))
            time.sleep(1)
        move_to_named_target("ready")
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == '__main__':
    rospy.init_node('test_calibration_camera')
    test_calibration_camera()
    rospy.spin()
