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
#from cobot_controllers.rl import DDPG
#from cobot_controllers.td3 import TD3
from collections import deque
from std_msgs.msg import Float32

evalnum = 1
ON_TRAIN = False  # make it false if you want to test

a_dim = 7
s_dim_avoid = 20
a_bound = [-0.1, 0.1]
thresh = 0.25

joint_positions = [0.0, 0.0, 0.0,0.0, 0.0, 0.0,0.0]
joint_velocities = [0.0, 0.0, 0.0,0.0, 0.0, 0.0,0.0]
ee_pose = [0.0, 0.0, 0.0]
obstacle_pose = [0.0, 0.0, 0.0]
target_pose = [0.0, 0.0, 0.0]
min_dist = [0.100]

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
    print(msg)
    for x in range(7):
        joint_positions[x] = msg.position[x]
        joint_velocities[x] =  msg.velocity[x]

def calculate_ee_pose(msg):
    ee_pose[0] = msg.position.x
    ee_pose[1] = msg.position.y
    ee_pose[2] = msg.position.z
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

def get_joint_state():
    print(joint_positions)
    #return(joint_positions + joint_velocities + ee_pose + target_pose + obstacle_pose)
    #return (joint_positions + joint_velocities + ee_pose + vector_target + vector_obstacle)

def train():
    pass

def eval(rl_reach,rl_avoid, data_to_send, publisher, avoid):
    s = get_state(avoid)
    if avoid:
        a = rl_avoid.choose_action(s)
    else:
        a = rl_reach.choose_action(s)

    data_to_send.data = a
    publisher.publish(data_to_send)
    #time.sleep(0.025)

def eval_mono(rl, data_to_send, publisher):

    s = get_state_mono()
    a = np.array([10,2,3,4,5,6,7])#rl.choose_action(s)

    data_to_send.data = a
    publisher.publish(data_to_send)
    #time.sleep(0.025)


def eval_test(data_to_send, publisher):
    #s = get_state() # read the external foce, velocities(cartesian or joint space)
    a = np.zeros(7)+0.02#rl.choose_action(s)
    a[-1] = -0.09
    data_to_send.data = a
    publisher.publish(data_to_send)

    gap_time = 0
    start = time.time()
    while gap_time < 1:
        # read the external force during the movement command executing
        gap_time = time.time() - start
    print(gap_time)


def run_ee_velocity( data_to_send, publisher):
    get_joint_state()

    #s = get_state() # read the external foce, velocities(cartesian or joint space)
    a = np.zeros(6)+0.02#rl.choose_action(s)
    a[-1] = -0.09
    data_to_send.data = a
    publisher.publish(data_to_send)

    gap_time = 0
    start = time.time()
    while gap_time < 1:
        # read the external force during the movement command executing
        gap_time = time.time() - start

    print(a, gap_time)

def update_target_pose(msg):
    #target_pose[0] = msg.x
    #target_pose[1] = msg.y
    target_pose[0] = 0.30
    target_pose[1] = 0.45
    target_pose[2] = 0.08

def update_obstacle_pose(msg):
    #obstacle_pose[0] = msg.x
    #obstacle_pose[1] = msg.y
    obstacle_pose[0] = 0.45
    obstacle_pose[1] = 0.30
    obstacle_pose[2] = 0.20

def update_min_dist(msg):

    #min_dist[0] = msg.data
    min_dist[0] = 0.50

def calculate_distance(link_pose, target_pose):
    return math.sqrt((link_pose[0]-target_pose[0]) ** 2 + (link_pose[1]-target_pose[1]) ** 2 + (link_pose[2]-target_pose[2]) ** 2)


def main():
    rospy.init_node('rospyNode', anonymous=True)
    rospy.Subscriber('/joint_states', JointState, update_joint_values)
    rospy.Subscriber('/ee_pose', Pose, calculate_ee_pose)
    rospy.Subscriber('/target_pose', Point, update_target_pose)
    rospy.Subscriber('/obstacle_pose', Point, update_obstacle_pose)
    rospy.Subscriber('/min_dist',Float32, update_min_dist)
    print("Subscriber...done")
    pub = rospy.Publisher('/test_controller/jointVelocities', Float32MultiArray, queue_size=10)
    print("Publisher...done")


    data_to_send = Float32MultiArray()
    rate = rospy.Rate(20.0)
    while not rospy.is_shutdown():
        run_ee_velocity(data_to_send, pub)
        rate.sleep()

if __name__ == '__main__':
    main()
