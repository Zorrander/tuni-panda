#!/usr/bin/env python

import rospy
from std_msgs.msg import Int8, Empty
from std_srvs.srv import Trigger
from cobot_controllers.srv import ReachTag

"""
## Add the table
p = PoseStamped()
p.header.frame_id = self.robot.get_planning_frame()
p.pose.position.x = 0.
p.pose.position.y = 0.
p.pose.position.z = -.05
self.scene.add_box("table", p, (3.0, 1.0, 0.1))

## Add the windows
# RPY to convert: 90deg, 0, -90deg
#q = quaternion_from_euler(1.5707, 0, -1.5707)
p = PoseStamped()
p.header.frame_id = self.robot.get_planning_frame()
p.pose.position.x = -0.30
p.pose.position.y = 0.
p.pose.position.z = -.05
p.pose.orientation.x = 0.0
p.pose.orientation.y = 1.0
p.pose.orientation.z = 0.0
p.pose.orientation.w = 1.0
self.scene.add_box("windows", p, (3.0, 1.0, 0.1))
"""

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

    rospy.wait_for_service('calculate_tag_pose')
    self.tag_detector = rospy.ServiceProxy('calculate_tag_pose', TagPose)
    rospy.spin()
