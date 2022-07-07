#!/usr/bin/env python3

#https://github.com/MorvanZhou/train-robot-arm-from-scratch/blob/master/final/main.py

#this is used for testing presaved velocity data

import time
import rospy
import pickle
import numpy as np
from numpy import exp
#import tensorflow as tf
import math
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import *
from collections import deque
from std_msgs.msg import Float32

""" import custom defined deep learning libraries"""
#from cobot_controllers.rl import DDPG
#from cobot_controllers.td3 import TD3
from cobot_controllers.controller import Controller

evalnum = 1
ON_TRAIN = False  # make it false if you want to test

a_dim = 7
s_dim_avoid = 20
a_bound = [-0.1, 0.1]
thresh = 0.25

joint_positions = [0.0, 0.0, 0.0,0.0, 0.0, 0.0,0.0]
joint_velocities = [0.0, 0.0, 0.0,0.0, 0.0, 0.0,0.0]
ee_pose = [0.0, 0.0, 0.0, 0., 0., 0.]

#file_name = "TD3_20state"
#directory="/home/atakan/catkin_ws/src/cobot_controllers/resources/hybrid_avoidtd3"
name3 = 'main3random'
name2 = 'avoid'
name = 'reach'

avoid_dir = 'sim2real/sim2real_avoid/'
reach_dir = 'sim2real/sim2real_reach/'
monolithic_dir = 'sim2real/mono/mono10cm/'
hybrid = True

def update_joint_values(msg):
    for x in range(7):
        joint_positions[x] = msg.position[x]
        joint_velocities[x] =  msg.velocity[x]

def calculate_ee_pose(msg):
    ee_pose[0] = msg.position.x
    ee_pose[1] = msg.position.y
    ee_pose[2] = msg.position.z
    ee_pose[0] = msg.orientation.x
    ee_pose[1] = msg.orientation.y
    ee_pose[2] = msg.orientation.z
    #print("ee_updated {}".format(msg))

# we need have calculate target_pose function here which calculates position of the target

def get_state(avoid):
    vector_target = [ee_pose[0] - target_pose[0], ee_pose[1] - target_pose[1], ee_pose[2] - target_pose[2]]
    vector_obstacle = [ee_pose[0] - obstacle_pose[0], ee_pose[1] - obstacle_pose[1], ee_pose[2] - obstacle_pose[2]]
    if avoid:
        return (joint_positions + joint_velocities + vector_target + vector_obstacle)
    else:
        return (joint_positions + joint_velocities + vector_target)
    #return(joint_positions + joint_velocities + ee_pose + target_pose + obstacle_pose)
    #return (joint_positions + joint_velocities + ee_pose + vector_target + vector_obstacle)

def get_state_mono():
    vector_target = [ee_pose[0] - target_pose[0], ee_pose[1] - target_pose[1], ee_pose[2] - target_pose[2]]
    vector_obstacle = [ee_pose[0] - obstacle_pose[0], ee_pose[1] - obstacle_pose[1], ee_pose[2] - obstacle_pose[2]]
    return (joint_positions + joint_velocities + vector_target + vector_obstacle)


def train():
    pass


def run_joint_velocity(controller, data_to_send, publisher):
    delta_joint_v = controller.run_inference(joint_positions)
    #delta_joint_v = (new_joint_positions - joint_positions)
    print('jp', joint_positions)
    print('njp', delta_joint_v)
    #print('dv', delta_joint_v)
    
    if not (np.array(joint_positions)==0).any():
        data_to_send.data = delta_joint_v * 0.1
        publisher.publish(data_to_send)

    gap_time = 0
    start = time.time()
    while gap_time < 0.21:
        # read the external force during the movement command executing
        gap_time = time.time() - start

    print(gap_time)

def run_ee_velocity(controller, data_to_send, publisher):
    """
    #s = get_state() # read the external foce, velocities(cartesian or joint space)
    a = np.zeros(6)+0.02#rl.choose_action(s)
    a[-1] = -0.09
    """
    delta_joint_v = controller.run_inference(ee_pose) - ee_pose
    delta_joint_v[3:] = 0


    if not (np.array(joint_positions)==0).any():
        data_to_send.data = delta_joint_v * 0.1
        publisher.publish(data_to_send)
    
    gap_time = 0
    start = time.time()
    while gap_time < 0.2:
        # read the external force during the movement command executing
        gap_time = time.time() - start

    print(delta_joint_v, gap_time)



def main():
    controller = Controller()
    controller.training()
    #controller.load_pkl()

    rospy.init_node('rospyNode', anonymous=True)
    rospy.Subscriber('/joint_states', JointState, update_joint_values)
    rospy.Subscriber('/ee_pose', Pose, calculate_ee_pose)
    #rospy.Subscriber('/target_pose', Point, update_target_pose)
    #rospy.Subscriber('/obstacle_pose', Point, update_obstacle_pose)
    #rospy.Subscriber('/min_dist',Float32, update_min_dist)
    print("Subscriber...done")
    pub = rospy.Publisher('/test_controller/jointVelocities', Float32MultiArray, queue_size=10)
    print("Publisher...done")


    data_to_send = Float32MultiArray()
    rate = rospy.Rate(20.0)

    while not rospy.is_shutdown():
        #run_joint_velocity(controller, data_to_send, pub)
        run_ee_velocity(controller, data_to_send, pub)
        rate.sleep()

if __name__ == '__main__':
    main()
