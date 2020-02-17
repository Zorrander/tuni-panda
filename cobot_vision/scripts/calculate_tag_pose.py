#!/usr/bin/env python

import rospy
from cobot_vision.srv import TagPose, TagPoseResponse
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped
import tf

class TagPoseServer(object):
    def __init__(self):
        self.tags = {
            1: None,
            2: None,
            3: None,
            4: None,
            5: None,
            6: None,
            7: None,
            8: None,
            9: None,
            10: None,
            11: None
        }
        self.listener = tf.TransformListener()
        rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.callback)
        self.s = rospy.Service('calculate_tag_pose', TagPose, self.calculate_tag_pose)

    def callback(self, msg_array):
        for tag in msg_array.detections:
            pose = PoseStamped()
            pose.header.frame_id = "camera_color_optical_frame"
            pose.pose.position.x = tag.pose.pose.pose.position.x
            pose.pose.position.y = tag.pose.pose.pose.position.y
            pose.pose.position.z = tag.pose.pose.pose.position.z
            pose.pose.orientation.x = tag.pose.pose.pose.orientation.x
            pose.pose.orientation.y = tag.pose.pose.pose.orientation.y
            pose.pose.orientation.z = tag.pose.pose.pose.orientation.z
            pose.pose.orientation.w = tag.pose.pose.pose.orientation.w
            t = self.listener.getLatestCommonTime("/camera_color_optical_frame", "/panda_link0")
            pose.header.stamp = t
            pose_in_robot = self.listener.transformPose("panda_link0", pose)
            id = "".join(_ for _ in str(tag.id) if _ in ".1234567890")
            self.tags[int(id)] = pose_in_robot

    def calculate_tag_pose(self, req):
        return TagPoseResponse(self.tags[req.tag_id].pose)



if __name__ == "__main__":
    rospy.init_node('tag_detector')
    TagPoseServer()
    print "Ready to send tag poses."
    rospy.spin()
