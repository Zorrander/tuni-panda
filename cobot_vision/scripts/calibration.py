#!/usr/bin/env python
import roslib
import rospy
import math
import tf
from geometry_msgs.msg import *
from apriltag_ros.msg import AprilTagDetectionArray
'''
Transformation between the camera and the tag
'''

def callback(msg_array):
    try:
        for tag in msg_array.detections:
            print(tag.pose.pose.pose.position)
            pose = PoseStamped()
            pose.header.frame_id = "camera_color_optical_frame"
            pose.pose.position.x = tag.pose.pose.pose.position.x
            pose.pose.position.y = tag.pose.pose.pose.position.y
            pose.pose.position.z = tag.pose.pose.pose.position0.z
            pose.pose.orientation.x = tag.pose.pose.pose.orientation.x
            pose.pose.orientation.y = tag.pose.pose.pose.orientation.y
            pose.pose.orientation.z = tag.pose.pose.pose.orientation.z
            pose.pose.orientation.w = tag.pose.pose.pose.orientation.w
            t = listener.getLatestCommonTime("/camera_color_optical_frame", "/panda_link0")
            pose.header.stamp = t
            pose_in_robot = listener.transformPose("panda_link0", pose)
            print("Pose tag {}".format(tag.id))
            print(pose_in_robot)
    except:
        print("...")

if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')

    ee_br = tf.TransformBroadcaster()
    camera_br = tf.TransformBroadcaster()
    listener = tf.TransformListener()

    rate = rospy.Rate(1.0)
    rate.sleep()

    #rospy.Subscriber("/tag_detections", AprilTagDetectionArray, callback)


    pose = PoseStamped()
    pose.header.frame_id = "camera"
    pose.header.stamp = rospy.Time(0)
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 0
    pose.pose.orientation.w = 1.0
    transformed_pose = listener.transformPose("/tag_22", pose)  # Pose of the camera from the tag
    while not rospy.is_shutdown():
        '''
        Send transform between the camera and the tag
    	'''
        # Send transform from the tag camera to the camera
        camera_br.sendTransform((transformed_pose.pose.position.x, transformed_pose.pose.position.y,transformed_pose.pose.position.z),
                            (transformed_pose.pose.orientation.x, transformed_pose.pose.orientation.y, transformed_pose.pose.orientation.z,transformed_pose.pose.orientation.w),
                            rospy.Time.now(), "camera_color_optical_frame", "tag")
        rate.sleep()

        '''
	    Transformation between the tag and the base of the robot
	    '''
    	pose = PoseStamped()
    	pose.header.frame_id = "panda_EE"
    	pose.header.stamp = rospy.Time(0)
    	pose.pose.position.x = 0
    	pose.pose.position.y = 0
    	pose.pose.position.z = 0
    	pose.pose.orientation.w = 1.0
        transformed_pose = listener.transformPose("panda_link0", pose)
    	ee_br.sendTransform((transformed_pose.pose.position.x, transformed_pose.pose.position.y, transformed_pose.pose.position.z),
                            (transformed_pose.pose.orientation.x, transformed_pose.pose.orientation.y, transformed_pose.pose.orientation.z, transformed_pose.pose.orientation.w),
                            rospy.Time.now(), "tag", "panda_link0")
        rate.sleep()

    	'''
    	Calculate transform between the camera and the base of the robot
    	'''
        try:
    	    print(listener.lookupTransform('camera_color_optical_frame', 'panda_link0', rospy.Time(0)))
        except:
            print("Cannot yet look up the transform: {}")
