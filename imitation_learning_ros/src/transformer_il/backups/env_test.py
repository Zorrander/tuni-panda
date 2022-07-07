import sys
sys.path.append('franka-pybullet/src/')
from panda import Panda

import time
from math import sin
import numpy as np
import pickle
import transformation
import glob

duration = 3000
stepsize = 1e-3

robot = Panda(stepsize)
robot.setControlMode("position")

def simple_tesst():
    cat_trajs = np.load('franka-pybullet/franka_panda_insertion_logs/experts/expert_cartesian_poses.npy', allow_pickle=True)
    j_trajs = np.load('franka-pybullet/franka_panda_insertion_logs/experts/expert_joints_poses.npy', allow_pickle=True)

    euler_trajs = []
    #"""
    for tid, jt in enumerate(j_trajs):
        et = []
        for jid, j_pos in enumerate(jt[::4]):
            #j_pos = np.append(j_pos, [0,0])
            #print(robot.joints)
            #print(j_pos)
            euler_pose = robot.step(j_pos)
            et.append(euler_pose)
            time.sleep(0.1)

            print('real', robot.getEEStates())
            print('logs', cat_trajs[tid][jid])
        euler_trajs.append(et)

    euler_trajs = np.array(euler_trajs)

explore_trajs = []
explore_trajs = np.load('data/insert_records/explore/expert_explore_joints_poses.npy', allow_pickle=True)

for path in explore_trajs:
    print(len(path))
    for j in path[::2]:
        euler_pose = robot.step(j)
        time.sleep(0.1)
a()
dataset_path = f'data/insert-expert.pkl' #f'data/{env_name}-{dataset}-v2.pkl'
with open(dataset_path, 'rb') as f:
    trajectories = pickle.load(f)

for path in trajectories:
    jt = path['observations']
    #actions.append(path['actions'])
    for j in jt:
        euler_pose = robot.step(j)
        time.sleep(0.1)