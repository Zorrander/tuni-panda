#!/usr/bin/env python

import rospy
from cobot_msgs.srv import TagPose, TagPoseResponse
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, Pose, Point
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
        self.target_pub = rospy.Publisher('target_pose', Point, queue_size=10)
        self.obstacle_pub = rospy.Publisher('obstacle_pose', Point, queue_size=10)
        self.s = rospy.Service('/calculate_tag_pose', TagPose, self.calculate_tag_pose)

    def tranform(self, pose):
        stamped_pose = PoseStamped()
        stamped_pose.header.frame_id = "camera_color_optical_frame"
        stamped_pose.pose.position.x = pose.position.x
        stamped_pose.pose.position.y = pose.position.y
        stamped_pose.pose.position.z = pose.position.z
        stamped_pose.pose.orientation.x = pose.orientation.x
        stamped_pose.pose.orientation.y = pose.orientation.y
        stamped_pose.pose.orientation.z = pose.orientation.z
        stamped_pose.pose.orientation.w = pose.orientation.w
        t = self.listener.getLatestCommonTime("/camera_color_optical_frame", "/panda_link0")
        stamped_pose.header.stamp = t
        pose_in_robot = self.listener.transformPose("panda_link0", stamped_pose)
        return pose_in_robot

    def callback(self, msg_array):
        for tag in msg_array.detections:
            print(tag.id[0])
            if tag.id[0] in [0,1,2,3]:
                print("obstacle")
                transformed_pose = self.tranform(tag.pose.pose.pose)
                self.obstacle_pub.publish(transformed_pose.pose.position)
            elif tag.id[0] in [4, 5]:
                print("target")
                transformed_pose = self.tranform(tag.pose.pose.pose)
                self.target_pub.publish(transformed_pose.pose.position)

        '''
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
            self.tags[int(id)] = pose_in_robot.pose
        '''

    def calculate_tag_pose(self, req):
        pose = self.tags[req.tag_id]
        response = TagPoseResponse()
        response.tag_pose = pose
        print("Sending tag pose: {}".format(response))
        return response



if __name__ == "__main__":
    rospy.init_node('tag_detector')
    TagPoseServer()
    print "Ready to send tag poses."
    rospy.spin()
