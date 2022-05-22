#!/usr/bin/env python2
import roslib
import rospy
import math
import sys

import tf
from geometry_msgs.msg import *
from std_msgs.msg import Float32




obstacle_pose = [0.0, 0.0, 0.0]

def create_frame(link):
    pose = PoseStamped()
    pose.header.frame_id = link
    pose.header.stamp = rospy.Time(0)
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 0
    pose.pose.orientation.w = 1.0
    return pose

def calculate_distance(link_pose, target_pose):
    return math.sqrt((link_pose[0]-target_pose[0]) ** 2 + (link_pose[1]-target_pose[1]) ** 2 + (link_pose[2]-target_pose[2]) ** 2)

def update_obstacle_pose(msg):
    obstacle_pose[0] = msg.x
    obstacle_pose[1] = msg.y
    obstacle_pose[2] = 0.20


if __name__ == '__main__':
    rospy.init_node('tf_broadcaster2')

    pub = rospy.Publisher('/min_dist', Float32, queue_size=10)
    rospy.Subscriber('/obstacle_pose', Point, update_obstacle_pose)

    links = ["panda_link1", "panda_link2", "panda_link3", "panda_link4", "panda_link5", "panda_link6", "panda_link7",  "panda_EE", "panda_leftfinger", "panda_rightfinger"]

    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    rate.sleep()
    while not rospy.is_shutdown():
        '''
        Transformation between the tag and the base of the robot
        '''
        min_dist = 100.0
        for link in links:
            transformed_pose = listener.transformPose("panda_link1", create_frame(link))
            transformed_pose = [transformed_pose.pose.position.x, transformed_pose.pose.position.y, transformed_pose.pose.position.z]
            dist = calculate_distance(transformed_pose, obstacle_pose)
            if dist < min_dist:
                min_dist = dist
        #print("Min dist is {}".format(min_dist))
        pub.publish(min_dist)
        rate.sleep()
