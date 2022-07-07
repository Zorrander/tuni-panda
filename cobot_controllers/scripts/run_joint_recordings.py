#!/usr/bin/env python3
import sys
sys.path.append('../')

import os
import time
import numpy as np
import signal
import subprocess
import glob


import tf_conversions
from scipy.spatial.transform import Rotation as R
from cobot_msgs.srv import *
import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose, PoseStamped
import moveit_commander
import torch

from franka_msgs.msg import FrankaState
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
from sensor_msgs.msg import JointState


def create_joint_msg(joint_config):
    msg = JointState()
    msg.position = joint_config
    return msg



def verify_end_effector_data():

    records = glob.glob('/home/alex/catkin_ws/src/imitation_learning_ros/src/transformer_il/data/experts/*')
    pub = rospy.Publisher('/new_joint_target', JointState, queue_size=1)
    

    for pth in records:
        joints_arr = np.load(pth, allow_pickle=True)[1]
        rate = rospy.Rate(1.0)
        length = 2
        sub_ratio = int(len(joints_arr)/length)
        for aid, action in enumerate(joints_arr[::sub_ratio]):#traj[1:][::2]:
            # ros c++ impedance control
            msg = create_joint_msg(action)
            pub.publish(msg)
            rate.sleep()
        msg = create_joint_msg(joints_arr[-1])
        pub.publish(msg)
        time.sleep(3)

if __name__ == '__main__':


    try:
        rospy.init_node('recordings')
        
        print("""
    ***************************
    REPLICATE EXPERT TRAJECTORY
    ***************************

    """)

        time.sleep(5)


        impedance_pub = rospy.Publisher("/new_stiffness_target",Float32MultiArray, queue_size=10) 
        msg = Float32MultiArray()
        msg.data = [95.0, 60.0]

        input("Press enter to change the stiffness")
        impedance_pub.publish(msg)

        print("The robot will move in")
        for i in range(5):
            print(str(5-i))
            time.sleep(1)

        verify_end_effector_data()

    except rospy.ROSInterruptException:
        pass