#!/usr/bin/env python

# Copyright 2020-2022 OpenDR European Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import sys
import copy
import rospy
import numpy as np
import time
import tf
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import random
from math import pi
from moveit_commander.conversions import pose_to_list
from std_msgs.msg import Int16, Float32MultiArray
from geometry_msgs.msg import PoseStamped
from cobot_msgs.srv import GraspPoseDetection
from cobot_msgs.msg import Detection


# executing a sequence of actions to demonstrate a grasping action based on
# single demo grasp model
def main():
    try:
        print("============ Initializing panda control ...")
        Commander = SingleDemoGraspAction()

        # reaching home position above objects with grippers opened
        print("============ Press `Enter` to reach home pose ...")
        raw_input()
        Commander.home_pose()

        # send a request detection to detection server and waits until it receives a reply
        print("press enter to send a detection request to detector node")
        raw_input()
        detection = Commander.request_detection()
        if (detection.obj_class == 0):
            print("Grasping a rocker arm")
        elif (detection.obj_class == 1):
            print("Grasping a pushrod")
        # after receiving the detections, checks if there is an object found in the
        # image frame, and brings the robot's camera view above the object and correct
        # the object's orientation (in case some objects might be placed in the edges
        # and could be hardly visible) to have better predictions.
        if detection.angle != 1e10:
            Commander.reach_hover(detection.x, detection.y)
            Commander.fix_angle(detection.angle)

        # another request, to receive the exact location of grasp followed by translating
        # the 2D coordinate (position in image frame) to 3D (corresponding position
        # in world frame) and executing the grasping action.
        print("============ Press `Enter` to find and reach grasp location")
        #raw_input()
        #detection = Commander.request_detection()
        #if (detection.obj_class == 0):
        #    print("Grasping a rocker arm")
        #elif (detection.obj_class == 1):
        #    print("Grasping a pushrod")
        
        if detection.x != 1e10 or detection.y != 1e10:
            Commander.reach_grasp_hover_kps(detection.kps_x, detection.kps_y)

        # lifting the object
        print("give input to close the gripper and lift the object")
        raw_input()
        Commander.close_hand()
        cartesian_plan, fraction = Commander.plan_linear_z(0.45)
        Commander.execute_plan(cartesian_plan)
        print("============ Python Commander demo complete!")

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


def convert_detection_pose(x ,y):
    listener_tf = tf.TransformListener()
    camera_focal = 550
    (trans1, rot1) = listener_tf.lookupTransform('/panda_link0', '/camera_color_frame', rospy.Time(0))
    z_to_surface = trans1[2]
    to_world_scale = z_to_surface / camera_focal

    x_dist = x * to_world_scale
    y_dist = y * to_world_scale

    my_point = PoseStamped()
    my_point.header.frame_id = "camera_color_frame"
    my_point.header.stamp = rospy.Time(0)
    my_point.pose.position.x = 0
    my_point.pose.position.y = -x_dist
    my_point.pose.position.z = y_dist
    theta = 0
    quat = tf.transformations.quaternion_from_euler(0, 0, theta)
    my_point.pose.orientation.x = quat[0]
    my_point.pose.orientation.y = quat[1]
    my_point.pose.orientation.z = quat[2]
    my_point.pose.orientation.w = quat[3]
    ps = listener_tf.transformPose("/panda_link0", my_point)

    (trans, rot) = listener_tf.lookupTransform('/panda_link0', '/camera_color_frame', rospy.Time(0))
    data = (ps.pose.position.x - trans[0], ps.pose.position.y - trans[1])

    return [data[0], data[1]]

def detection_callback(msg):
    ctr_X = int((msg.bounding_box[0]+msg.bounding_box[2])/2)
    ctr_Y = int((msg.bounding_box[1]+msg.bounding_box[3])/2)
    ref_x = 640/2
    ref_y = 480/2
    dist = [ctr_X - ref_x, ref_y - ctr_Y]
    x_robot, y_robot = convert_detection_pose(dist[0], dist[1])
    print("""


    """)
    print(x_robot)
    print(y_robot)
    print("""

        """)

if __name__ == '__main__':

    rospy.init_node('SingleDemoGraspAction', anonymous=True)
    # listener_tf = tf.TransformListener()
    #main()
    #sys.exit()
    detection_sub = rospy.Subscriber("/objects_detected", Detection, detection_callback)
    print("rospy.spin()")
    rospy.spin()