#!/usr/bin/env python2
import roslib
import rospy
import math
import tf
from geometry_msgs.msg import *

if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')

    pub = rospy.Publisher('/ee_pose', Pose, queue_size=10)

    listener = tf.TransformListener()

    rate = rospy.Rate(20.0)
    rospy.sleep(1.0)
    rate.sleep()
    while not rospy.is_shutdown():
        '''
        Transformation between the tag and the base of the robot
        '''
        pose = PoseStamped()
        #pose.header.frame_id = "panda_leftfinger"
        pose.header.frame_id = "panda_EE"
        pose.header.stamp = rospy.Time(0)
        # t = listener.getLatestCommonTime("/panda_leftfinger", "/panda_link0")
        # pose.header.stamp = t
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = 0
        pose.pose.orientation.w = 1.0
        transformed_pose = listener.transformPose("panda_link0", pose)
        print(transformed_pose.pose)
        pub.publish(transformed_pose.pose)
        rate.sleep()

    '''
    Calculate transform between the camera and the base of the robot
    '''

    '''
    print(listener.lookupTransform('camera_color_optical_frame', 'panda_link0', rospy.Time(0)))


    pose = PoseStamped()
    pose.header.frame_id = "camera_color_optical_frame"
        t = listener.getLatestCommonTime("/camera_color_optical_frame", "/panda_link0")
    pose.header.stamp = t
        pose.pose.position.x = -0.0635371342288
        pose.pose.position.y = -0.00542627724101
        pose.pose.position.z = 0.403605806572
        pose.pose.orientation.x = 0.834286878956
        pose.pose.orientation.y = -0.436089611012
        pose.pose.orientation.z = -0.115006942269
        pose.pose.orientation.w = -0.317119311932
    pose_in_robot = listener.transformPose("panda_link0", pose)
    print("Pose tag 2:")
    print(pose_in_robot)
    '''
