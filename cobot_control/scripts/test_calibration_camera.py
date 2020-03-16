#!/usr/bin/env python

import rospy
from std_msgs.msg import Int8, Empty
from std_srvs.srv import Trigger
from cobot_controllers.srv import ReachTag

def test_calibration_camera():
    rospy.wait_for_service('reach_tag')
    rospy.wait_for_service('move_start')
    go_to_tag = rospy.ServiceProxy('reach_tag', ReachTag)
    reset_order = rospy.ServiceProxy('move_start', Trigger)
    tags = [18, 20, 22, 16, 24]
    calibrated = True
    try:
        reset_order()
        for tag in tags:
            calibrated = go_to_tag(tag).success
        reset_order()
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == '__main__':
    rospy.init_node('test_calibration_camera')
    test_calibration_camera()
    rospy.spin()
