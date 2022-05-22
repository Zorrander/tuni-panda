#!/usr/bin/env python

import sys
import time
import rospy
import moveit_commander

# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

def move_to_start(group):
    arm_joints = group.get_current_joint_values()
    arm_joints[0] = 0.00018762090386758654 
    arm_joints[1] = -0.7771156373465307
    arm_joints[2] = 0.0005407026658047056
    arm_joints[3] = -2.365469873129632
    arm_joints[4] = -0.00020200796294576732
    arm_joints[5] = 1.5704722326730955
    arm_joints[6] = 0.7845368039521746
    group.set_max_velocity_scaling_factor(0.5)
    group.allow_replanning(True)
    group.go(arm_joints, wait=True)
    group.stop()
    group.clear_pose_targets()
    return True

def move_to_cartesian_target(group, delta_x = 0, delta_y = 0):
    wpose = group.get_current_pose().pose
    wpose.position.x += delta_x  # 
    wpose.position.y += delta_y  #  
    group.set_pose_target(wpose);
    group.set_max_velocity_scaling_factor(0.5)
    group.allow_replanning(True)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    return True

def rotate(group, clockwise):
    arm_joints = group.get_current_joint_values()
    arm_joints[6] += 1.0 if clockwise else -1.0
    group.set_max_velocity_scaling_factor(0.5)
    group.allow_replanning(True)
    group.go(arm_joints, wait=True)
    group.stop()
    group.clear_pose_targets()
    return True

class ImageRecorder:

    def __init__(self):
        self.bridge = CvBridge()
        
    def save_image(self, name):
        cv2.imwrite(name, self.cv2_img)

    def image_callback(self, msg):
        try:
            # Convert your ROS Image message to OpenCV2
            self.cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError, e:
            print(e)


def collect_object_data(group, object_id):
    move_to_start(group)
    recorder.save_image("image" + str(7*object_id+1) + ".jpeg")
    time.sleep(1)
    move_to_cartesian_target(group, 0.1)
    recorder.save_image("image" + str(7*object_id+2) + ".jpeg")
    time.sleep(1)
    rotate(group, clockwise=False)
    recorder.save_image("image" + str(7*object_id+3) + ".jpeg")
    time.sleep(1)
    rotate(group, clockwise=True)
    move_to_cartesian_target(group, -0.05, 0.2)
    recorder.save_image("image" + str(7*object_id+4) + ".jpeg")
    time.sleep(1)
    move_to_cartesian_target(group, 0.1, -0.2)
    rotate(group, clockwise=False)
    recorder.save_image("image" + str(7*object_id+5) + ".jpeg")
    time.sleep(1)
    move_to_cartesian_target(group, -0.15)
    recorder.save_image("image" + str(7*object_id+6) + ".jpeg")
    rotate(group, clockwise=True)
    rotate(group, clockwise=True)
    move_to_cartesian_target(group, 0.15, 0.15)
    recorder.save_image("image" + str(7*object_id+7) + ".jpeg")
    time.sleep(1)
    move_to_start(group)

if __name__ == '__main__':
    rospy.init_node('collect_data', anonymous=True)

    moveit_commander.roscpp_initialize(sys.argv)
    group_name = "panda_arm"

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander(group_name)

    recorder = ImageRecorder()
    image_topic = "/camera/color/image_raw"
    rospy.Subscriber(image_topic, Image, recorder.image_callback)

    object_classes = rospy.get_param("/collect_data/object_classes")

    time.sleep(2)
    
    for i in range(object_classes):
        collect_object_data(group, i)
        raw_input("Press any key when objects are ready.")



