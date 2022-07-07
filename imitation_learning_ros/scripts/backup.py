#!/usr/bin/env python3

import sys

sys.path.append('/home/alex/catkin_ws/src/imitation_learning_ros/src/franka_insertion_prediction/')
sys.path.append('/home/alex/catkin_ws/src/imitation_learning_ros/src/imitation_learning_ros_v2/')

import os
import time
import glob
import torch
import rospy
import pickle
import argparse
import numpy as np
import threading as th
from itertools import count
from datetime import datetime
from time import gmtime, strftime
from tensorboardX import SummaryWriter

import tf 
from std_msgs.msg import Empty, Float32MultiArray
from franka_msgs.msg import FrankaState
from geometry_msgs.msg import Pose, WrenchStamped
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest

import imitation_learning_ros.utils as utils
import imitation_learning_ros.utils.transformation as transformation
from imitation_learning_ros.models import Actor, Critic
from imitation_learning_ros.trainer import SILCRTrainer
from imitation_learning_ros.prior_info import PriorInfo
from imitation_learning_ros.util import Memory, PriorityMemory
from imitation_learning_ros.panda_insert_env import PandaInsertEnv
# from imitation_learning_ros.franka_insertion_prediction.online_trainer import OnlineLearner
from imitation_learning_ros.online_trainer import OnlineLearner
from imitation_learning_ros.utils.transformation import quaternion_from_euler, euler_from_quaternion

from imitation_learning_ros_v2.imitation_transformer import ImitationTransformer, parse_xy

from differentiable_robot_model.robot_model import DifferentiableRobotModel


COLOR_YELLOW = '\033[93m'
COLOR_CYON = '\033[96m'


keep_going = True
moving = False



class Runner():
    """docstring for ClassName"""
    def __init__(self):
        #super(ClassName, self).__init__()
        # create paths for storing training informations
        # dt = datetime.today().strftime('%Y-%m-%d-%H:%M:%S')
        #device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")


        self.horizon = 30
        self.moveit = False
        self.traj_type = 'joint' # cartesian

        #"""
        self.controller_switcher = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
        switch_msg = SwitchControllerRequest()
        switch_msg.stop_controllers = ["insertion_controller"]    
        switch_msg.start_controllers = ["position_joint_trajectory_controller"]                    
        switch =  self.controller_switcher(switch_msg)
        #""" 
        config_pth = '/home/alex/catkin_ws/src/imitation_learning_ros/src/imitation_learning_ros_v2/configs/config.yaml'
        self.predictor = ImitationTransformer(config_pth)
        self.learner = OnlineLearner(patch_dim=18, args_path='/home/alex/catkin_ws/src/imitation_learning_ros/src/franka_insertion_prediction/config/online_sequence_config_resume.yaml')
        
        if not self.predictor.imitation_config['resume']:
            # load pretrained models
            self.predictor.load(load_pretrain=True)
        else:
            self.predictor.load(load_pretrain=False)
        
        #predictor.encoder.train()
        # finetune a little bit
        #learner.train_iteration(num_steps=100, iter_num=1, print_logs=True)

        # initiate the expert prior class, which contains the expert state-action-next_state in joint angle space
        self.explore_prior = PriorInfo(sample_ratio=2,dataset_path=self.predictor.imitation_config['explore_prior_path'])
        self.explore_prior.load_demonstration()

        urdf_path = '/home/alex/catkin_ws/src/imitation_learning_ros/src/imitation_learning_ros_v2/franka-pybullet/models/panda/panda_gripper_differentiable.urdf'#panda_differentiable.urdf'
        self.panda_bot = DifferentiableRobotModel(urdf_path, device='cpu')

        self.explore_joints = np.load('/home/alex/catkin_ws/src/imitation_learning_ros/src/imitation_learning_ros_v2/data/insert_records/explore/expert_explore_joints_poses.npy', 
                                        allow_pickle=True)[3][::6]

        #ee_poses, ee_quats = panda_bot.compute_forward_kinematics(joints, link_name='panda_virtual_ee_link')

        

    def key_capture_thread(self):
        global keep_going
        x = input()
        print(x)
        if x=='s':
            keep_going = False


    '''
    def check_path(self, path):
        try:
            os.makedirs(path)
        except FileExistsError:
            # directory already exists
            pass
    '''

class Runner_v1(Runner):
    """docstring for ClassName"""
    def __init__(self):
        super().__init__()

    def run(self):  
        # start training after expert demos generated 
        for epoch in count():
            #env.switchToMoveItControl()
            
            state = env.reset()
            states = []
            next_states = []
            actions = []
            dones = []
            infos = []
            encoder_obs = []

            time_steps = 0
            explore_prior_steps = 10

            
            while time_steps<self.horizon:
                try:
                    state = state['observation']
                except:
                    state = state
                
                
                curr_joints = np.array([env.read_obs_srv().joints])


                if time_steps < explore_prior_steps:
                    # using human prior for exploration,
                    prior_target_joints = self.explore_joints[time_steps]
                    #  1 step with 3 actions
                    if len(encoder_obs)>2:
                        print(COLOR_YELLOW+"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                        print(COLOR_YELLOW+'predicting:',self.predictor.encoder.predict(np.array(encoder_obs)))
                        #self.predictor.encoder.train_iteration(num_steps=1, iter_num=1, print_logs=True)

                    action = prior_target_joints

                    next_state, reward, done, info = env.step(action, direct_control=True)

                    """
                    for _ in range(1000):
                        predictor.sim_robot.step(action)
                    """

                    try:
                        next_state = next_state['observation']
                    except:
                        next_state = next_state

                    states.append(state)
                    next_states.append(next_state)
                    actions.append(action)
                    dones.append(done)
                    infos.append(info)
                    encoder_obs.append(parse_xy(info))
                    time_steps+=1
                else:
                    # parse data from the env returned infos: previous_joint angles, previous actions, and the joint torque forces                    

                    '''
                    Switch to impedance control
                    '''
                    
                    if not self.moveit:
                        switch_msg = SwitchControllerRequest()
                        switch_msg.stop_controllers = ["position_joint_trajectory_controller"]    
                        switch_msg.start_controllers = ["insertion_controller"]                    
                        switch =  self.controller_switcher(switch_msg)
                    
                    # use supervised training for target prediction, then retrive human priors
                    #prior_target_joints = learner.choose_action(x, curr_joints, prior_type='action', nsteps=5)
                    #prior_traj = learner.choose_action(x, curr_joints, prior_type='whole_traj', nsteps=5, traj_type=traj_type)
                    
                    target_pos = self.predictor.encoder.predict(np.array(encoder_obs))
                    target_pos[0][0] = target_pos[0][0] * 1000
                    target_pos[0][1] = target_pos[0][1] * 1000
                    prior_traj = self.learner.human_priors.retrieve_prior_whole_trajectory(target_pos, traj_type=self.traj_type)
                    
                    env.step(env.init_ee_pose, direct_control=True)
                    for prior_target_joints in prior_traj:
                        #  1 step with 3 actions
                        print(COLOR_CYON+'predicting:',self.predictor.encoder.predict(np.array(encoder_obs)))
                        #predictor.encoder.train_iteration(num_steps=1, iter_num=1, print_logs=True)
                        #predictor.policy.train_iteration(num_steps=1, iter_num=1, print_logs=True)

                        action = prior_target_joints

                        policy_s =  env.get_joint_angles()
                        policy_s = np.expand_dims(policy_s, axis=0)                   
                        print(COLOR_CYON+'predicted goal:', target_pos)
                        test_a = self.predictor.policy.predict(policy_s, target_pos)
                        print('predicted:', test_a)
                        print('real:', policy_s)
                        print('human:', prior_target_joints)
                        """
                        for _ in range(20):
                            predictor.sim_robot.step(test_a)
                        test_a = np.array(predictor.sim_robot.getJointStates()[0])
                        """
                        print(COLOR_CYON+'predicted action:', test_a / np.pi * 180)
                        print(COLOR_CYON+'prior action:',  action / np.pi * 180)

                        #next_state, reward, done, info = env.cartesian_step(action, moveit=False)
                        if self.traj_type=='joint':
                            next_state, reward, done, info = env.step(prior_target_joints, direct_control=True)
                        elif self.traj_type=='cartesian':
                            env.cartesian_step(action, moveit=self.moveit)
                        try:
                            next_state = next_state['observation']
                        except:
                            next_state = next_state

                        states.append(state)
                        next_states.append(next_state)
                        actions.append(action)
                        dones.append(done)
                        infos.append(info)
                        encoder_obs.append(parse_xy(info))
                        time_steps+=1
                    done = True

                    '''
                    Switch back to moveit
                    '''
                    if not self.moveit:
                        switch_msg = SwitchControllerRequest()
                        switch_msg.stop_controllers = ["insertion_controller"]    
                        switch_msg.start_controllers = ["position_joint_trajectory_controller"]                    
                        switch =  self.controller_switcher(switch_msg)
                    
                state = next_state
              
                if done or time_steps==self.horizon-1:

                    start_expert = True
                    global keep_going 

                    
                    keep_going = True
                    correction_reset = True
                    print('done', done, 'start expert correction... (if no expert correction, press \'s\' key to skip)')

            
                    while start_expert:
                        if correction_reset:
                            print('reset for pose correction:')
                            env.reset()
                            correction_reset = False

                        joint_traj = []
                        cartesian_traj = []

                        steps = 0

                        th.Thread(target=self.key_capture_thread, args=(), name='key_capture_thread', daemon=True).start()

                        while keep_going:
                            joint_velocity = rospy.wait_for_message('/franka_state_controller/franka_states', FrankaState).dq 
                            is_moving = (np.round(np.abs(joint_velocity),3) > 0.01).any()
                            if is_moving:
                                joints = np.array(env.get_bot_info().joints)
                                cartesians = np.array(env.get_ee_cartesians())
                                #print(joints)
                                joint_traj.append(joints)
                                cartesian_traj.append(cartesians)
                                #print(cartesian_traj)

                                print('moving', cartesians[:3], np.array(euler_from_quaternion(cartesians[3:]))/np.pi*180)

                                steps += 1

                        # add one more step just in case
                        joint_velocity = rospy.wait_for_message('/franka_state_controller/franka_states', FrankaState).dq 
                        joints = np.array(env.get_bot_info().joints)
                        cartesians = np.array(env.get_ee_cartesians())
                        joint_traj.append(joints)
                        cartesian_traj.append(cartesians)
                        print('target pose:', cartesians[:3], euler_from_quaternion(cartesians[3:]))
                        start_expert = False

                    good_indication = env.wait_expert_op()
                    break
                else:
                    good_indication = False

            print(COLOR_CYON)
            #self.predictor.encoder.train_iteration(num_steps=10, iter_num=1, print_logs=True)

            if not good_indication:
                print('add this trajectory into dataset')
                pose = cartesian_traj[-1]
                euler = euler_from_quaternion(pose[3:])
                goal = np.array([[pose[0],pose[1],euler[0]/np.pi * 180]])
                enc_traj_goals = np.repeat(goal, len(encoder_obs), axis=0)

                policy_state_traj = joint_traj[:-1]
                policy_action_traj = joint_traj[1:]
                policy_goal_traj = enc_traj_goals

                print(self.predictor.encoder.trajectories_goals[0][0])
                print(enc_traj_goals[0])
                print(COLOR_CYON)

                self.predictor.encoder.update_enc_dataset(np.array(encoder_obs), enc_traj_goals)
                self.predictor.policy.update_mlp_dataset(np.array(policy_state_traj), 
                                                    np.array(policy_action_traj), 
                                                    np.array(policy_goal_traj)) #(np.array(encoder_obs), enc_traj_goals)
                self.predictor.save()

                print('\n after update dataset', COLOR_CYON)
                #predictor.encoder.train_iteration(num_steps=200, iter_num=1, print_logs=True)

                #predictor.encoder.trajectories.append(np.array(encoder_obs))
                #predictor.encoder.trajectories_goals.append(enc_traj_goals)
                #self.predictor.encoder.train_iteration(num_steps=300, iter_num=1, print_logs=True)
                
                self.predictor.load()

                # save all sensor recordings along the trajectory
                record_time = strftime("%Y-%m-%d-%H:%M:%S", gmtime())
                with open('/home/alex/catkin_ws/src/imitation_learning_ros/src/imitation_learning_ros_v2/data/infos/infos'+record_time+'.npy', 'wb') as f:
                    np.save(f, [pose, euler, goal, infos])

 
            # save replays
            #learner.save_model()
            #learner.save_data()
            #learner.save_human_prior()

        print('-----------------training over-------------------------')




class Runner_v1(Runner):
    """docstring for ClassName"""
    def __init__(self):
        super().__init__()

    def run(self):  
        # start training after expert demos generated 
        for epoch in count():
            #env.switchToMoveItControl()
            
            state = env.reset()
            states = []
            next_states = []
            actions = []
            dones = []
            infos = []
            encoder_obs = []

            time_steps = 0
            explore_prior_steps = 10

            
            while time_steps<self.horizon:
                try:
                    state = state['observation']
                except:
                    state = state
                
                
                curr_joints = np.array([env.read_obs_srv().joints])

                explore_prior_traj = self.explore_prior.sample_prior_traj(traj_len=explore_prior_steps)

                if time_steps < explore_prior_steps:
                    # using human prior for exploration,
                    prior_target_joints = explore_prior_traj[time_steps]
                    #  1 step with 3 actions
                    if len(encoder_obs)>2:
                        print(COLOR_YELLOW+"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                        print(COLOR_YELLOW+'predicting:',self.predictor.encoder.predict(np.array(encoder_obs)))
                        #self.predictor.encoder.train_iteration(num_steps=1, iter_num=1, print_logs=True)

                    action = prior_target_joints

                    next_state, reward, done, info = env.step(action, direct_control=True)

                    """
                    for _ in range(1000):
                        predictor.sim_robot.step(action)
                    """

                    try:
                        next_state = next_state['observation']
                    except:
                        next_state = next_state

                    states.append(state)
                    next_states.append(next_state)
                    actions.append(action)
                    dones.append(done)
                    infos.append(info)
                    encoder_obs.append(parse_xy(info))
                    time_steps+=1
                else:
                    # parse data from the env returned infos: previous_joint angles, previous actions, and the joint torque forces                    

                    '''
                    Switch to impedance control
                    '''
                    
                    if not self.moveit:
                        switch_msg = SwitchControllerRequest()
                        switch_msg.stop_controllers = ["position_joint_trajectory_controller"]    
                        switch_msg.start_controllers = ["insertion_controller"]                    
                        switch =  self.controller_switcher(switch_msg)
                    
                    # use supervised training for target prediction, then retrive human priors
                    #prior_target_joints = learner.choose_action(x, curr_joints, prior_type='action', nsteps=5)
                    #prior_traj = learner.choose_action(x, curr_joints, prior_type='whole_traj', nsteps=5, traj_type=traj_type)
                    
                    target_pos = self.predictor.encoder.predict(np.array(encoder_obs))
                    target_pos[0][0] = target_pos[0][0] * 1000
                    target_pos[0][1] = target_pos[0][1] * 1000
                    prior_traj = self.learner.human_priors.retrieve_prior_whole_trajectory(target_pos, traj_type=self.traj_type)
                    
                    env.step(env.init_ee_pose, direct_control=True)
                    for prior_target_joints in prior_traj:
                        #  1 step with 3 actions
                        print(COLOR_CYON+'predicting:',self.predictor.encoder.predict(np.array(encoder_obs)))
                        #predictor.encoder.train_iteration(num_steps=1, iter_num=1, print_logs=True)
                        #predictor.policy.train_iteration(num_steps=1, iter_num=1, print_logs=True)

                        action = prior_target_joints

                        policy_s =  env.get_joint_angles()
                        policy_s = np.expand_dims(policy_s, axis=0)                   
                        print(COLOR_CYON+'predicted goal:', target_pos)
                        test_a = self.predictor.policy.predict(policy_s, target_pos)
                        print('predicted:', test_a)
                        print('real:', policy_s)
                        print('human:', prior_target_joints)
                        """
                        for _ in range(20):
                            predictor.sim_robot.step(test_a)
                        test_a = np.array(predictor.sim_robot.getJointStates()[0])
                        """
                        print(COLOR_CYON+'predicted action:', test_a / np.pi * 180)
                        print(COLOR_CYON+'prior action:',  action / np.pi * 180)

                        #next_state, reward, done, info = env.cartesian_step(action, moveit=False)
                        if self.traj_type=='joint':
                            next_state, reward, done, info = env.step(prior_target_joints, direct_control=True)
                        elif self.traj_type=='cartesian':
                            env.cartesian_step(action, moveit=self.moveit)
                        try:
                            next_state = next_state['observation']
                        except:
                            next_state = next_state

                        states.append(state)
                        next_states.append(next_state)
                        actions.append(action)
                        dones.append(done)
                        infos.append(info)
                        encoder_obs.append(parse_xy(info))
                        time_steps+=1
                    done = True

                    '''
                    Switch back to moveit
                    '''
                    if not self.moveit:
                        switch_msg = SwitchControllerRequest()
                        switch_msg.stop_controllers = ["insertion_controller"]    
                        switch_msg.start_controllers = ["position_joint_trajectory_controller"]                    
                        switch =  self.controller_switcher(switch_msg)
                    
                state = next_state
              
                if done or time_steps==self.horizon-1:

                    start_expert = True
                    global keep_going 

                    
                    keep_going = True
                    correction_reset = True
                    print('done', done, 'start expert correction... (if no expert correction, press \'s\' key to skip)')

            
                    while start_expert:
                        if correction_reset:
                            print('reset for pose correction:')
                            env.reset()
                            correction_reset = False

                        joint_traj = []
                        cartesian_traj = []

                        steps = 0

                        th.Thread(target=self.key_capture_thread, args=(), name='key_capture_thread', daemon=True).start()

                        while keep_going:
                            joint_velocity = rospy.wait_for_message('/franka_state_controller/franka_states', FrankaState).dq 
                            is_moving = (np.round(np.abs(joint_velocity),3) > 0.01).any()
                            if is_moving:
                                joints = np.array(env.get_bot_info().joints)
                                cartesians = np.array(env.get_ee_cartesians())
                                #print(joints)
                                joint_traj.append(joints)
                                cartesian_traj.append(cartesians)
                                #print(cartesian_traj)

                                print('moving', cartesians[:3], np.array(euler_from_quaternion(cartesians[3:]))/np.pi*180)

                                steps += 1

                        # add one more step just in case
                        joint_velocity = rospy.wait_for_message('/franka_state_controller/franka_states', FrankaState).dq 
                        joints = np.array(env.get_bot_info().joints)
                        cartesians = np.array(env.get_ee_cartesians())
                        joint_traj.append(joints)
                        cartesian_traj.append(cartesians)
                        print('target pose:', cartesians[:3], euler_from_quaternion(cartesians[3:]))
                        start_expert = False

                    good_indication = env.wait_expert_op()
                    break
                else:
                    good_indication = False

            print(COLOR_CYON)
            #self.predictor.encoder.train_iteration(num_steps=10, iter_num=1, print_logs=True)

            if not good_indication:
                print('add this trajectory into dataset')
                pose = cartesian_traj[-1]
                euler = euler_from_quaternion(pose[3:])
                goal = np.array([[pose[0],pose[1],euler[0]/np.pi * 180]])
                enc_traj_goals = np.repeat(goal, len(encoder_obs), axis=0)

                policy_state_traj = joint_traj[:-1]
                policy_action_traj = joint_traj[1:]
                policy_goal_traj = enc_traj_goals

                print(self.predictor.encoder.trajectories_goals[0][0])
                print(enc_traj_goals[0])
                print(COLOR_CYON)

                self.predictor.encoder.update_enc_dataset(np.array(encoder_obs), enc_traj_goals)
                self.predictor.policy.update_mlp_dataset(np.array(policy_state_traj), 
                                                    np.array(policy_action_traj), 
                                                    np.array(policy_goal_traj)) #(np.array(encoder_obs), enc_traj_goals)
                self.predictor.save()

                print('\n after update dataset', COLOR_CYON)
                #predictor.encoder.train_iteration(num_steps=200, iter_num=1, print_logs=True)

                #predictor.encoder.trajectories.append(np.array(encoder_obs))
                #predictor.encoder.trajectories_goals.append(enc_traj_goals)
                #self.predictor.encoder.train_iteration(num_steps=300, iter_num=1, print_logs=True)
                
                self.predictor.load()

                # save all sensor recordings along the trajectory
                record_time = strftime("%Y-%m-%d-%H:%M:%S", gmtime())
                with open('/home/alex/catkin_ws/src/imitation_learning_ros/src/imitation_learning_ros_v2/data/infos/infos'+record_time+'.npy', 'wb') as f:
                    np.save(f, [pose, euler, goal, infos])

 
            # save replays
            #learner.save_model()
            #learner.save_data()
            #learner.save_human_prior()

        print('-----------------training over-------------------------')


class Runner_v2(Runner):
    """docstring for ClassName"""
    def __init__(self):
        super().__init__()

    def run(self):  
        # start training after expert demos generated 
        for epoch in count():
            #env.switchToMoveItControl()
            
            state = env.reset()
            states = []
            next_states = []
            actions = []
            dones = []
            infos = []
            encoder_obs = []

            time_steps = 0
            explore_prior_steps = 3

            
            while time_steps<self.horizon:
                try:
                    state = state['observation']
                except:
                    state = state
                
                
                curr_joints = np.array([env.read_obs_srv().joints])

                explore_prior_traj = self.explore_prior.sample_prior_traj(traj_len=explore_prior_steps)

                if time_steps < explore_prior_steps:
                    # using human prior for exploration,
                    prior_target_joints = explore_prior_traj[time_steps]
                    #  1 step with 3 actions
                    if len(encoder_obs)>2:
                        print(COLOR_YELLOW+"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                        print(COLOR_YELLOW+'predicting:',self.predictor.encoder.predict(np.array(encoder_obs)))
                        #self.predictor.encoder.train_iteration(num_steps=1, iter_num=1, print_logs=True)

                    action = prior_target_joints

                    next_state, reward, done, info = env.step(action, direct_control=True)

                    """
                    for _ in range(1000):
                        predictor.sim_robot.step(action)
                    """

                    try:
                        next_state = next_state['observation']
                    except:
                        next_state = next_state

                    states.append(state)
                    next_states.append(next_state)
                    actions.append(action)
                    dones.append(done)
                    infos.append(info)
                    encoder_obs.append(parse_xy(info))
                    time_steps+=1
                else:
                    # parse data from the env returned infos: previous_joint angles, previous actions, and the joint torque forces                    

                    '''
                    Switch to impedance control
                    '''
                    
                    if not self.moveit:
                        switch_msg = SwitchControllerRequest()
                        switch_msg.stop_controllers = ["position_joint_trajectory_controller"]    
                        switch_msg.start_controllers = ["insertion_controller"]                    
                        switch =  self.controller_switcher(switch_msg)
                    
                    # use supervised training for target prediction, then retrive human priors
                    #prior_target_joints = learner.choose_action(x, curr_joints, prior_type='action', nsteps=5)
                    #prior_traj = learner.choose_action(x, curr_joints, prior_type='whole_traj', nsteps=5, traj_type=traj_type)
                    
                    target_pos = self.predictor.encoder.predict(np.array(encoder_obs))

                    target_pos[0][0] = target_pos[0][0] * 1000
                    target_pos[0][1] = target_pos[0][1] * 1000
                    prior_traj = self.learner.human_priors.retrieve_prior_whole_trajectory(target_pos, traj_type=self.traj_type)
                    
                    env.step(env.init_ee_pose, direct_control=True)
                    for prior_target_joints in prior_traj:
                        #  1 step with 3 actions
                        print(COLOR_CYON+'predicting:',self.predictor.encoder.predict(np.array(encoder_obs)))
                        #predictor.encoder.train_iteration(num_steps=1, iter_num=1, print_logs=True)
                        #predictor.policy.train_iteration(num_steps=1, iter_num=1, print_logs=True)

                        action = prior_target_joints
                        goal = self.predictor.encoder.predict(np.array(encoder_obs))
                        policy_s =  env.get_joint_angles()
                        policy_s = np.expand_dims(policy_s, axis=0)                   
                        print(COLOR_CYON+'predicted goal:', goal)
                        test_a = self.predictor.policy.predict(policy_s, goal)
                        print('predicted:', test_a)
                        print('real:', policy_s)
                        print('human:', prior_target_joints)
                        #"""
                        for _ in range(20):
                            predictor.sim_robot.step(test_a)
                        test_a = np.array(predictor.sim_robot.getJointStates()[0])
                        #"""
                        print(COLOR_CYON+'predicted action:', test_a / np.pi * 180)
                        print(COLOR_CYON+'prior action:',  action / np.pi * 180)

                        #next_state, reward, done, info = env.cartesian_step(action, moveit=False)
                        if self.traj_type=='joint':
                            next_state, reward, done, info = env.step(test_a, direct_control=True)
                        elif self.traj_type=='cartesian':
                            env.cartesian_step(action, moveit=self.moveit)
                        try:
                            next_state = next_state['observation']
                        except:
                            next_state = next_state

                        states.append(state)
                        next_states.append(next_state)
                        actions.append(action)
                        dones.append(done)
                        infos.append(info)
                        encoder_obs.append(parse_xy(info))
                        time_steps+=1
                    done = True

                    '''
                    Switch back to moveit
                    '''
                    if not self.moveit:
                        switch_msg = SwitchControllerRequest()
                        switch_msg.stop_controllers = ["insertion_controller"]    
                        switch_msg.start_controllers = ["position_joint_trajectory_controller"]                    
                        switch =  self.controller_switcher(switch_msg)
                    
                state = next_state
              
                if done or time_steps==self.horizon-1:

                    start_expert = True
                    global keep_going 

                    
                    keep_going = True
                    correction_reset = True
                    print('done', done, 'start expert correction... (if no expert correction, press \'s\' key to skip)')

            
                    while start_expert:
                        if correction_reset:
                            print('reset for pose correction:')
                            env.reset()
                            correction_reset = False

                        joint_traj = []
                        cartesian_traj = []

                        steps = 0

                        th.Thread(target=self.key_capture_thread, args=(), name='key_capture_thread', daemon=True).start()

                        while keep_going:
                            joint_velocity = rospy.wait_for_message('/franka_state_controller/franka_states', FrankaState).dq 
                            is_moving = (np.round(np.abs(joint_velocity),3) > 0.01).any()
                            if is_moving:
                                joints = np.array(env.get_bot_info().joints)
                                cartesians = np.array(env.get_ee_cartesians())
                                #print(joints)
                                joint_traj.append(joints)
                                cartesian_traj.append(cartesians)
                                #print(cartesian_traj)

                                print('moving', cartesians[:3], np.array(euler_from_quaternion(cartesians[3:]))/np.pi*180)

                                steps += 1

                        # add one more step just in case
                        joint_velocity = rospy.wait_for_message('/franka_state_controller/franka_states', FrankaState).dq 
                        joints = np.array(env.get_bot_info().joints)
                        cartesians = np.array(env.get_ee_cartesians())
                        joint_traj.append(joints)
                        cartesian_traj.append(cartesians)
                        print('target pose:', cartesians[:3], euler_from_quaternion(cartesians[3:]))
                        start_expert = False

                    good_indication = env.wait_expert_op()
                    break
                else:
                    good_indication = False

            print(COLOR_CYON)
            #self.predictor.encoder.train_iteration(num_steps=10, iter_num=1, print_logs=True)

            if not good_indication:
                print('add this trajectory into dataset')
                pose = cartesian_traj[-1]
                euler = euler_from_quaternion(pose[3:])
                goal = np.array([[pose[0],pose[1],euler[0]/np.pi * 180]])
                enc_traj_goals = np.repeat(goal, len(encoder_obs), axis=0)

                policy_state_traj = joint_traj[:-1]
                policy_action_traj = joint_traj[1:]
                policy_goal_traj = enc_traj_goals

                print(self.predictor.encoder.trajectories_goals[0][0])
                print(enc_traj_goals[0])
                print(COLOR_CYON)

                self.predictor.encoder.update_enc_dataset(np.array(encoder_obs), enc_traj_goals)
                self.predictor.policy.update_mlp_dataset(np.array(policy_state_traj), 
                                                    np.array(policy_action_traj), 
                                                    np.array(policy_goal_traj)) #(np.array(encoder_obs), enc_traj_goals)
                self.predictor.save()

                print('\n after update dataset', COLOR_CYON)
                #predictor.encoder.train_iteration(num_steps=200, iter_num=1, print_logs=True)

                #predictor.encoder.trajectories.append(np.array(encoder_obs))
                #predictor.encoder.trajectories_goals.append(enc_traj_goals)
                #self.predictor.encoder.train_iteration(num_steps=300, iter_num=1, print_logs=True)
                
                self.predictor.load()

                # save all sensor recordings along the trajectory
                record_time = strftime("%Y-%m-%d-%H:%M:%S", gmtime())
                with open('/home/alex/catkin_ws/src/imitation_learning_ros/src/imitation_learning_ros_v2/data/infos/infos'+record_time+'.npy', 'wb') as f:
                    np.save(f, [pose, euler, goal, infos])

 
            # save replays
            #learner.save_model()
            #learner.save_data()
            #learner.save_human_prior()

        print('-----------------training over-------------------------')



class Runner_v3(Runner):
    """docstring for ClassName"""
    def __init__(self):
        super().__init__()

    def run(self):  
        # start training after expert demos generated 
        for epoch in count():
            #env.switchToMoveItControl()
            
            state = env.reset()
            states = []
            next_states = []
            actions = []
            dones = []
            infos = []
            encoder_obs = []

            time_steps = 0
            explore_prior_steps = 10

            
            while time_steps<self.horizon:
                try:
                    state = state['observation']
                except:
                    state = state
                
                
                curr_joints = np.array([env.read_obs_srv().joints])


                if time_steps < explore_prior_steps:
                    # using human prior for exploration,
                    prior_target_joints = self.explore_joints[time_steps]
                    #  1 step with 3 actions
                    if len(encoder_obs)>2:
                        print(COLOR_YELLOW+"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                        print(COLOR_YELLOW+'predicting:',self.predictor.encoder.predict(np.array(encoder_obs)))
                        #self.predictor.encoder.train_iteration(num_steps=1, iter_num=1, print_logs=True)

                    j_action = prior_target_joints
                    j_action = np.array([j_action])
                    j_action = torch.from_numpy(j_action).to(device='cpu', dtype=torch.float32)
                    ee_action = self.panda_bot.compute_forward_kinematics(j_action, link_name='panda_virtual_ee_link')
                    
                    next_state, reward, done, info = env.step(ee_action, cart_impedance=True)



                    """
                    for _ in range(1000):
                        predictor.sim_robot.step(action)
                    """

                    try:
                        next_state = next_state['observation']
                    except:
                        next_state = next_state

                    states.append(state)
                    next_states.append(next_state)
                    actions.append(ee_action)
                    dones.append(done)
                    infos.append(info)
                    encoder_obs.append(parse_xy(info))
                    time_steps+=1
                else:
                    # parse data from the env returned infos: previous_joint angles, previous actions, and the joint torque forces                    

                    '''
                    Switch to impedance control
                    '''
                    
                    if not self.moveit:
                        switch_msg = SwitchControllerRequest()
                        switch_msg.stop_controllers = ["position_joint_trajectory_controller"]    
                        switch_msg.start_controllers = ["insertion_controller"]                    
                        switch =  self.controller_switcher(switch_msg)
                    
                    # use supervised training for target prediction, then retrive human priors
                    #prior_target_joints = learner.choose_action(x, curr_joints, prior_type='action', nsteps=5)
                    #prior_traj = learner.choose_action(x, curr_joints, prior_type='whole_traj', nsteps=5, traj_type=traj_type)
                    
                    target_pos = self.predictor.encoder.predict(np.array(encoder_obs))
                    target_pos[0][0] = target_pos[0][0] * 1000
                    target_pos[0][1] = target_pos[0][1] * 1000
                    prior_traj = self.learner.human_priors.retrieve_prior_whole_trajectory(target_pos, traj_type=self.traj_type)
                    

                    j_action = np.array([env.init_ee_pose])
                    j_action = torch.from_numpy(j_action).to(device='cpu', dtype=torch.float32)
                    ee_action = self.panda_bot.compute_forward_kinematics(j_action, link_name='panda_virtual_ee_link')
                    
                    env.step(ee_action, cart_impedance=True)

                    for prior_target_joints in prior_traj:
                        #  1 step with 3 actions
                        print(COLOR_CYON+'predicting:',self.predictor.encoder.predict(np.array(encoder_obs)))
                        #predictor.encoder.train_iteration(num_steps=1, iter_num=1, print_logs=True)
                        #predictor.policy.train_iteration(num_steps=1, iter_num=1, print_logs=True)

                        action = prior_target_joints

                        policy_s =  env.get_joint_angles()
                        policy_s = np.expand_dims(policy_s, axis=0)                   
                        print(COLOR_CYON+'predicted goal:', target_pos)
                        test_a = self.predictor.policy.predict(policy_s, target_pos)
                        print('predicted:', test_a)
                        print('real:', policy_s)
                        print('human:', prior_target_joints)
                        """
                        for _ in range(20):
                            predictor.sim_robot.step(test_a)
                        test_a = np.array(predictor.sim_robot.getJointStates()[0])
                        """
                        print(COLOR_CYON+'predicted action:', test_a / np.pi * 180)
                        print(COLOR_CYON+'prior action:',  action / np.pi * 180)

                        #next_state, reward, done, info = env.cartesian_step(action, moveit=False)

                        j_action = np.array([prior_target_joints])
                        j_action = torch.from_numpy(j_action).to(device='cpu', dtype=torch.float32)
                        ee_action = self.panda_bot.compute_forward_kinematics(j_action, link_name='panda_virtual_ee_link')
                        next_state, reward, done, info = env.step(ee_action, cart_impedance=True)

                        try:
                            next_state = next_state['observation']
                        except:
                            next_state = next_state

                        states.append(state)
                        next_states.append(next_state)
                        actions.append(ee_action)
                        dones.append(done)
                        infos.append(info)
                        encoder_obs.append(parse_xy(info))
                        time_steps+=1
                    done = True

                    '''
                    Switch back to moveit
                    '''
                    if not self.moveit:
                        switch_msg = SwitchControllerRequest()
                        switch_msg.stop_controllers = ["insertion_controller"]    
                        switch_msg.start_controllers = ["position_joint_trajectory_controller"]                    
                        switch =  self.controller_switcher(switch_msg)
                    
                state = next_state
              
                if done or time_steps==self.horizon-1:

                    start_expert = True
                    global keep_going 

                    
                    keep_going = True
                    correction_reset = True
                    print('done', done, 'start expert correction... (if no expert correction, press \'s\' key to skip)')

            
                    while start_expert:
                        if correction_reset:
                            print('reset for pose correction:')
                            env.reset()
                            correction_reset = False

                        joint_traj = []
                        cartesian_traj = []

                        steps = 0

                        th.Thread(target=self.key_capture_thread, args=(), name='key_capture_thread', daemon=True).start()

                        while keep_going:
                            joint_velocity = rospy.wait_for_message('/franka_state_controller/franka_states', FrankaState).dq 
                            is_moving = (np.round(np.abs(joint_velocity),3) > 0.01).any()
                            if is_moving:
                                joints = np.array(env.get_bot_info().joints)
                                cartesians = np.array(env.get_ee_cartesians())
                                #print(joints)
                                joint_traj.append(joints)
                                cartesian_traj.append(cartesians)
                                #print(cartesian_traj)

                                print('moving', cartesians[:3], np.array(euler_from_quaternion(cartesians[3:]))/np.pi*180)

                                steps += 1

                        # add one more step just in case
                        joint_velocity = rospy.wait_for_message('/franka_state_controller/franka_states', FrankaState).dq 
                        joints = np.array(env.get_bot_info().joints)
                        cartesians = np.array(env.get_ee_cartesians())
                        joint_traj.append(joints)
                        cartesian_traj.append(cartesians)
                        print('target pose:', cartesians[:3], euler_from_quaternion(cartesians[3:]))
                        start_expert = False

                    good_indication = env.wait_expert_op()
                    break
                else:
                    good_indication = False

            print(COLOR_CYON)
            #self.predictor.encoder.train_iteration(num_steps=10, iter_num=1, print_logs=True)

            if not good_indication:
                print('add this trajectory into dataset')
                pose = cartesian_traj[-1]
                euler = euler_from_quaternion(pose[3:])
                goal = np.array([[pose[0],pose[1],euler[0]/np.pi * 180]])
                enc_traj_goals = np.repeat(goal, len(encoder_obs), axis=0)

                policy_state_traj = joint_traj[:-1]
                policy_action_traj = joint_traj[1:]
                policy_goal_traj = enc_traj_goals

                print(self.predictor.encoder.trajectories_goals[0][0])
                print(enc_traj_goals[0])
                print(COLOR_CYON)

                self.predictor.encoder.update_enc_dataset(np.array(encoder_obs), enc_traj_goals)
                self.predictor.policy.update_mlp_dataset(np.array(policy_state_traj), 
                                                    np.array(policy_action_traj), 
                                                    np.array(policy_goal_traj)) #(np.array(encoder_obs), enc_traj_goals)
                self.predictor.save()

                print('\n after update dataset', COLOR_CYON)
                #predictor.encoder.train_iteration(num_steps=200, iter_num=1, print_logs=True)

                #predictor.encoder.trajectories.append(np.array(encoder_obs))
                #predictor.encoder.trajectories_goals.append(enc_traj_goals)
                #self.predictor.encoder.train_iteration(num_steps=300, iter_num=1, print_logs=True)
                
                self.predictor.load()

                # save all sensor recordings along the trajectory
                record_time = strftime("%Y-%m-%d-%H:%M:%S", gmtime())
                with open('/home/alex/catkin_ws/src/imitation_learning_ros/src/imitation_learning_ros_v2/data/infos/infos'+record_time+'.npy', 'wb') as f:
                    np.save(f, [pose, euler, goal, infos])

 
            # save replays
            #learner.save_model()
            #learner.save_data()
            #learner.save_human_prior()

        print('-----------------training over-------------------------')


if __name__ == '__main__':
    rospy.init_node('rl_runner')


    env = PandaInsertEnv()
 
    rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, env.update_franka_state, buff_size=1)
    #rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, env.checking_ext_force, buff_size=1)
    rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, env.checking_collisions, buff_size=1)

    #runner = Runner_v1()
    runner = Runner_v3()

    rate = rospy.Rate(2) # ROS Rate at 10Hz
    
    while not rospy.is_shutdown():
        runner.run()
        rate.sleep()

