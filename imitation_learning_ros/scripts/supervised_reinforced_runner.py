#!/usr/bin/env python3

import sys

#sys.path.append('src/imitation_learning_ros/src/transformer_il/')

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
from scipy.spatial.transform import Rotation as R

import tf 
from std_msgs.msg import Empty, Float32MultiArray
from franka_msgs.msg import FrankaState
from geometry_msgs.msg import Pose, WrenchStamped
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest

from robot_env.panda_insert_env import PandaInsertEnv, set_hard_stiffness, set_zero_stiffness

from transformer_il.imitation_transformer import ImitationTransformer, parse_xy
from transformer_il.utils.prior import Priors
from transformer_il.decision_transformer.transformation import quaternion_from_euler, euler_from_quaternion

from differentiable_robot_model.robot_model import DifferentiableRobotModel
import os
print(os.path.abspath(os.getcwd()))

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
        #self.controller_switcher = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
        #switch_msg = SwitchControllerRequest()
        #switch_msg.stop_controllers = ["insertion_controller"]    
        #switch_msg.start_controllers = ["position_joint_trajectory_controller"]                    
        #switch =  self.controller_switcher(switch_msg)
        #""" 
        config_pth = '/home/alex/catkin_ws/src/imitation_learning_ros/src/transformer_il/configs/config.yaml'
        self.predictor = ImitationTransformer(config_pth)
        self.expert_prior = Priors(config_pth)

        if not self.predictor.imitation_config['resume']:
            # load pretrained models
            self.predictor.load(load_pretrain=True)
        else:
            self.predictor.load(load_pretrain=False)
        
        #predictor.encoder.train()
        # finetune a little bit

        # initiate the expert prior class, which contains the expert state-action-next_state in joint angle space
        #self.explore_prior = PriorInfo(sample_ratio=2,dataset_path=self.predictor.imitation_config['explore_prior_path'])
        #self.explore_prior.load_demonstration()

        urdf_path = '/home/alex/catkin_ws/src/imitation_learning_ros/src/franka-pybullet/models/panda/panda_gripper_differentiable.urdf'#panda_differentiable.urdf'
        self.panda_bot = DifferentiableRobotModel(urdf_path, device='cpu')

        self.explore_joints = np.load('/home/alex/catkin_ws/src/imitation_learning_ros/src/transformer_il/data/explore/explore.npy', 
                                        allow_pickle=True)[1][::3][:10]

        #ee_poses, ee_quats = panda_bot.compute_forward_kinematics(joints, link_name='panda_virtual_ee_link')

        

    def key_capture_thread(self):
        global keep_going
        wait_input = True
        while wait_input:
            x = input()
            if x=='s':            
                print(x)
                keep_going = False
                wait_input = False
            else:
                print('unacceptable key, type \'s\'')

    '''
    def check_path(self, path):
        try:
            os.makedirs(path)
        except FileExistsError:
            # directory already exists
            pass
    '''


class Runner_v3(Runner):
    """docstring for ClassName"""
    def __init__(self):
        super().__init__()

    def run(self, impedance_pub):  
        # start training after expert demos generated 
        
        # Make sure the robot is not in compliant mode
        msg = Float32MultiArray()
        msg.data = [95.0, 60.0]
        impedance_pub.publish(msg)

        for epoch in count():

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
                    
                    next_state, reward, done, info = env.step(ee_action, control_type='cart_impedance')

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

                    parsed_obs, _ = parse_xy(info)
                    for iid in range(len(parsed_obs)):
                        encoder_obs.append(parsed_obs[iid])
                    time_steps+=1
                else:
                    # parse data from the env returned infos: previous_joint angles, previous actions, and the joint torque forces                    
                    # use supervised training for target prediction, then retrive human priors

                    env.back_to_init_state()
                    time.sleep(2)
                    
                    target_pos = self.predictor.encoder.predict(np.array(encoder_obs))
                    target_pos[0][0] = target_pos[0][0] * 1000
                    target_pos[0][1] = target_pos[0][1] * 1000
                    prior_traj = self.expert_prior.find_priors(target_pos, horizon=15)

                    j_action = np.array([env.init_ee_pose])
                    j_action = torch.from_numpy(j_action).to(device='cpu', dtype=torch.float32)
                    ee_action = self.panda_bot.compute_forward_kinematics(j_action, link_name='panda_virtual_ee_link')
                    
                    #env.step(ee_action, cart_impedance=True)

                    for prior_target_joints in prior_traj[::2]:
                        #  1 step with 3 actions
                        print(COLOR_CYON+'predicting:',self.predictor.encoder.predict(np.array(encoder_obs)))
                        #predictor.encoder.train_iteration(num_steps=1, iter_num=1, print_logs=True)
                        #predictor.policy.train_iteration(num_steps=1, iter_num=1, print_logs=True)

                        action = prior_target_joints

                        policy_s =  env.get_joint_angles()
                        policy_s = np.expand_dims(policy_s, axis=0)                   
                        print(COLOR_CYON+'predicted goal:', target_pos)
                        goal_pos = self.predictor.encoder.predict(np.array(encoder_obs))
                        #goal_pos = np.array([603, -43, 135])
                        test_a = self.predictor.policy.predict(policy_s, goal_pos)
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


                        j_action = np.array([prior_target_joints])
                        j_action = torch.from_numpy(j_action).to(device='cpu', dtype=torch.float32)
                        ee_action = self.panda_bot.compute_forward_kinematics(j_action, link_name='panda_virtual_ee_link')
                        next_state, reward, done, info = env.step(prior_target_joints)
                        time.sleep(1)
                        #next_state, reward, done, info = env.step(ee_action, control_type='cart_impedance')

                        try:
                            next_state = next_state['observation']
                        except:
                            next_state = next_state

                        states.append(state)
                        next_states.append(next_state)
                        actions.append(ee_action)
                        dones.append(done)
                        infos.append(info)

                        parsed_obs, _ = parse_xy(info)
                        for iid in range(len(parsed_obs)):
                            encoder_obs.append(parsed_obs[iid])
                        time_steps+=1

                    next_state, reward, done, info = env.step(prior_traj[-1])
                    time.sleep(1)
                    next_state, reward, done, info = env.step(prior_traj[-1])
                    time.sleep(1)
                    done = True

                state = next_state
              
                if done or time_steps==self.horizon-1:

                    start_expert = True
                    global keep_going 
   
                    keep_going = True
                    correction_reset = True
                    print('done', done, 'start expert correction... (if no expert correction, press \'s\' key to skip)')
                
                    while start_expert:
                        if correction_reset:
                            # set to hard stiffness in case the arm drops down
                            set_hard_stiffness()
                            print('reset for pose correction:')
                            env.reset()
                            env.stiffness_reset()
                            correction_reset = False

                        joint_traj = []
                        cartesian_traj = []
                        ee_cartesian_traj = []

                        steps = 0

                        th.Thread(target=self.key_capture_thread, args=(), name='key_capture_thread', daemon=True).start()

                        while keep_going:
                            joint_velocity = rospy.wait_for_message('/franka_state_controller/franka_states', FrankaState).dq 
                            is_moving = (np.round(np.abs(joint_velocity),3) > 0.01).any()
                            if is_moving:
                                joints = np.array(env.get_bot_info().joints)
                                cartesians = np.array(env.get_ee_cartesians())

                                joint_traj.append(joints)
                                cartesian_traj.append(cartesians)
                                
                                # record the true ee pose from ROS
                                EE_POSE = joint_velocity = rospy.wait_for_message('/franka_state_controller/franka_states', FrankaState).O_T_EE  

                                r = R.from_matrix([[EE_POSE[0], EE_POSE[4], EE_POSE[8]],
                                                   [EE_POSE[1], EE_POSE[5], EE_POSE[9]],
                                                   [EE_POSE[2], EE_POSE[6], EE_POSE[10]]])
                                quat = r.as_quat()
                                ee_cartesians = [EE_POSE[12], EE_POSE[13], EE_POSE[14], quat[0], quat[1], quat[2], quat[3]]
                                ee_cartesians = np.array(ee_cartesians)
                                ee_cartesian_traj.append(ee_cartesians)
                                print('ee cart', len(ee_cartesian_traj), quat)
                                print('moving', cartesians[:3], np.array(euler_from_quaternion(cartesians[3:]))/np.pi*180)
                                steps += 1

                        # add one more step just in case
                        joint_velocity = rospy.wait_for_message('/franka_state_controller/franka_states', FrankaState).dq 
                        joints = np.array(env.get_bot_info().joints)

                        # record the true ee pose from ROS
                        EE_POSE = joint_velocity = rospy.wait_for_message('/franka_state_controller/franka_states', FrankaState).O_T_EE  
                        r = R.from_matrix([[EE_POSE[0], EE_POSE[4], EE_POSE[8]],
                                           [EE_POSE[1], EE_POSE[5], EE_POSE[9]],
                                           [EE_POSE[2], EE_POSE[6], EE_POSE[10]]])
                        quat = r.as_quat()
                        ee_cartesians = [EE_POSE[12], EE_POSE[13], EE_POSE[14], quat[0], quat[1], quat[2], quat[3]]
                        ee_cartesians = np.array(ee_cartesians)
                        ee_cartesian_traj.append(ee_cartesians)
                        print('ee cart', len(ee_cartesian_traj))

                        joint_traj.append(joints)

                        cartesians = np.array(env.get_ee_cartesians())
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
                #self.predictor.save()

                print('\n after update dataset', COLOR_CYON)
                #predictor.encoder.train_iteration(num_steps=200, iter_num=1, print_logs=True)

                #predictor.encoder.trajectories.append(np.array(encoder_obs))
                #predictor.encoder.trajectories_goals.append(enc_traj_goals)
                #self.predictor.encoder.train_iteration(num_steps=300, iter_num=1, print_logs=True)
                

                # save all sensor recordings along the trajectory
                record_time = strftime("%Y-%m-%d-%H:%M:%S", gmtime())
                with open('src/imitation_learning_ros/src/transformer_il/data/infos/infos'+record_time+'.npy', 'wb') as f:
                    np.save(f, [pose, euler, goal, infos])


                with open('src/imitation_learning_ros/src/transformer_il/data/experts/joints'+record_time+'.npy', 'wb') as f:
                    np.save(f, [goal, joint_traj])

                print('!', ee_cartesian_traj)
                with open('src/imitation_learning_ros/src/transformer_il/data/experts_ee/cartesian'+record_time+'.npy', 'wb') as f:
                    np.save(f, [goal, ee_cartesian_traj])


            self.predictor.load()
            self.expert_prior.load()

        print('-----------------training over-------------------------')



class Runner_test(Runner):
    """docstring for ClassName"""
    def __init__(self):
        super().__init__()

    def run(self):  
        # start training after expert demos generated 
        for epoch in count():
            #env.switchToMoveItControl()
            
            '''
            Switch back to JOINT
            if not self.moveit:
                switch_msg = SwitchControllerRequest()
                switch_msg.stop_controllers = ["insertion_controller"]    
                switch_msg.start_controllers = ["position_joint_trajectory_controller"]                    
                switch =  self.controller_switcher(switch_msg)
                print('switch control mode')
                time.sleep(2)
            '''

            state = env.reset()
            states = []
            next_states = []
            actions = []
            dones = []
            infos = []
            encoder_obs = []

            time_steps = 0
            explore_prior_steps = 10

            # set to hard stiffness in case the arm drops down
            set_hard_stiffness()

            # find the trajectory
            target_pos = np.array([ 608.29619462,  -40.49821727, -136.92268121])
            prior_traj = self.expert_prior.find_priors(target_pos, horizon=5)

            print('switch to joint')

            for prior_target_joints in prior_traj[:]:
                next_state, reward, done, info = env.step(prior_target_joints, control_type='joint_pos')


            '''
                Switch to impedance control
            
            #set_hard_stiffness()
            if not self.moveit:
                switch_msg = SwitchControllerRequest()
                switch_msg.stop_controllers = ["position_joint_trajectory_controller"]    
                switch_msg.start_controllers = ["insertion_controller"]                    
                switch =  self.controller_switcher(switch_msg)
                print('switch control mode')
                time.sleep(2)

            test_traj = np.load('src/imitation_learning_ros/src/transformer_il/data/experts_ee/cartesian2022-03-07-15:29:19.npy',
                allow_pickle=True)[1]
            print(test_traj)
            for p in test_traj[::3]:
                print(p)
                action = [p[:3]], [p[3:]]
                print(action)
                next_state, reward, done, info = env.step(action, cart_impedance=True)
            '''
            # a(9)
            '''

            print('switch to impedance')
            '''

            '''
                Switch to impedance control
            '''
            #set_hard_stiffness()
            if not self.moveit:
                switch_msg = SwitchControllerRequest()
                switch_msg.stop_controllers = ["position_joint_trajectory_controller"]    
                switch_msg.start_controllers = ["insertion_controller"]                    
                switch =  self.controller_switcher(switch_msg)
                print('switch control mode')
                time.sleep(2)

            for prior_target_joints in prior_traj[:]:

                j_action = np.array([prior_target_joints])
                j_action = torch.from_numpy(j_action).to(device='cpu', dtype=torch.float32)
                ee_action = self.panda_bot.compute_forward_kinematics(j_action, link_name='panda_virtual_ee_link')
                next_state, reward, done, info = env.step(ee_action, control_type='cart_impedance')




        print('-----------------training over-------------------------')


#[ 608.29619462  -40.49821727 -136.92268121]
if __name__ == '__main__':
    try:
        rospy.init_node('rl_runner')

        env = PandaInsertEnv()
     
        rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, env.update_franka_state, buff_size=1)
        #rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, env.checking_ext_force, buff_size=1)
        rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, env.checking_collisions, buff_size=1)
        impedance_pub = rospy.Publisher("/new_impedance_target",Float32MultiArray, queue_size=10) 
        runner = Runner_v3()
        #runner = Runner_test()
     
        runner.run(impedance_pub)
        rospy.spin()
    except Exception as e:
        print("ROS terminated")
        print(e)
