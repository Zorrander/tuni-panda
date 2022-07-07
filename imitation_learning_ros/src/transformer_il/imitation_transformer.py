import gym
import numpy as np
import torch
import wandb
import time
import argparse
import pickle
import random
import sys
import torch_optimizer as optim

from transformer_il.decision_transformer.evaluation.evaluate_episodes import evaluate_episode, evaluate_episode_rtg
from transformer_il.decision_transformer.models.decision_transformer import DecisionTransformer
from transformer_il.decision_transformer.models.mlp_bc import ConditionMLPBCModel
from transformer_il.decision_transformer.training.act_trainer import ActTrainer
from transformer_il.decision_transformer.training.seq_trainer import SequenceTrainer
from transformer_il.decision_transformer.training.enc_trainer import EncoderTrainer
from transformer_il.decision_transformer.models.transformer_encoder.encoder import TransformerEncoder
import transformer_il.decision_transformer.transformation as transformation
from transformer_il.utils.configure import read_yaml


import glob

"""
try:
    #sys.path.append('/home/alex/catkin_ws/src/imitation_learning_ros/src/transformer_il/franka-pybullet/src/')
    from panda import Panda
except:
    sys.path.append('franka-pybullet/src/')
    from panda import Panda
    print('cannot load panda libs')
"""

def parse_xy(info, goal=[0,0,0], digitize=False):
    previous_ee_pose_quat = info['previous_ee_pose_quat']
    previous_ee_pose_euler = transformation.euler_from_quaternion(previous_ee_pose_quat[3:])
    previous_ee_pose = np.append(previous_ee_pose_quat[:3], previous_ee_pose_euler)


    ee_pose_quat = info['ee_pose_quat']
    ee_pose_euler = transformation.euler_from_quaternion(ee_pose_quat[3:])
    ee_pose = np.append(ee_pose_quat[:3], ee_pose_euler)

    delta_ee_pose = ee_pose - previous_ee_pose

    previous_joint_angles = info['previous_joints']
    joint_velocities = np.mean(np.array(info['short_joint_velocities']), axis=0)
    ee_wrench = np.mean(np.array(info['short_ee_cartesian_wrench_sequence']), axis=0)
    joint_accelerations = np.mean(np.array(info['short_joint_accelerations']), axis=0)
    #ee_wrench = np.concatenate(info['ee_cartesian_wrench_sequence'])
    #x = np.concatenate([previous_ee_pose,joint_velocities,ee_wrench]).flatten()
    x = np.concatenate([ee_pose, previous_ee_pose, joint_velocities, joint_accelerations, ee_wrench]).flatten()

     
    if goal is not None:
        y = goal#np.array([position[0]*1000, position[1]*1000, euler[0]]).astype(np.float64)

        if digitize:
            trasn_x_max, trasn_x_min = 615, 590
            trasn_y_max, trasn_y_min = 50, 18
            rot_max, rot_min = 145, 120

            trans_x_bins = np.linspace(trasn_x_min, trasn_x_max, num=30)
            trans_y_bins = np.linspace(trasn_y_min, trasn_y_max, num=30)
            rot_bins = np.linspace(rot_min, rot_max, num=30)

            y[0] = np.digitize(y[0],trans_x_bins,right=False)
            y[1] = np.digitize(y[1],trans_y_bins,right=False)
            y[2] = np.digitize(y[2],rot_bins,right=False)

            # then convert from value to one-hot vecs

    return [x],[y]


def parse_xy_v0(info, goal=[0,0,0], digitize=False):
    previous_ee_pose_quat = info['previous_ee_pose_quat']
    previous_ee_pose_euler = transformation.euler_from_quaternion(previous_ee_pose_quat[3:])
    previous_ee_pose = np.append(previous_ee_pose_quat[:3], previous_ee_pose_euler)


    ee_pose_quat = info['ee_pose_quat']
    ee_pose_euler = transformation.euler_from_quaternion(ee_pose_quat[3:])
    ee_pose = np.append(ee_pose_quat[:3], ee_pose_euler)

    delta_ee_pose = ee_pose - previous_ee_pose

    previous_joint_angles = info['previous_joints']
    
    
    length = np.min([len(np.array(info['short_joint_velocities'])),
                    len(np.array(info['short_ee_cartesian_wrench_sequence'])),
                    len(np.array(info['short_joint_accelerations'])),
                    len(np.array(info['short_joint_positions'])),])
    
    if goal is not None:
        gt = goal#np.array([position[0]*1000, position[1]*1000, euler[0]]).astype(np.float64)

        # then convert from value to one-hot vecs
        if digitize:
            trasn_x_max, trasn_x_min = 615, 590
            trasn_y_max, trasn_y_min = 50, 18
            rot_max, rot_min = 145, 120

            trans_x_bins = np.linspace(trasn_x_min, trasn_x_max, num=30)
            trans_y_bins = np.linspace(trasn_y_min, trasn_y_max, num=30)
            rot_bins = np.linspace(rot_min, rot_max, num=30)

            gt[0] = np.digitize(gt[0],trans_x_bins,right=False)
            gt[1] = np.digitize(gt[1],trans_y_bins,right=False)
            gt[2] = np.digitize(gt[2],rot_bins,right=False)

    xs = []
    ys = []

    window_size = length
    steps = int(length / window_size)
    new_len = np.max([1, steps])

    # smoothing (mean operation) the signals with sliding window, step size 1, window size int(length / 3)
    #print(length, window_size, length-window_size)
    for iid in range(new_len):
        jv = np.mean(info['short_joint_velocities'][iid*window_size:(iid+1)*window_size], axis=0)
        ja = np.mean(info['short_joint_accelerations'][iid*window_size:(iid+1)*window_size], axis=0)
        jp = np.mean(info['short_joint_positions'][iid*window_size:(iid+1)*window_size], axis=0)
        ee_f = np.mean(info['short_ee_cartesian_wrench_sequence'][iid*window_size:(iid+1)*window_size], axis=0)
        x = np.concatenate([jv, ja, jp, ee_f]
                          ).flatten()

        xs.append(x)
        ys.append(gt)

    return xs,ys
 

def minmax_normalize(x, x_mean, x_max):
    return (x-x_mean) / (x_max - x_mean)

def std_normalize(x, x_mean, x_std):
    return (x-x_mean) / x_std


def reverse_max_normalize(x, x_mean, x_max):
    return x * (x_max - x_mean) + x_mean

def reverse_std_normalize(x, x_mean, x_std):
    return x * x_std + x_mean


class ImitationMLP(object):
    """docstring for ImitationMLP"""
    def __init__(self, config):
        super(ImitationMLP, self).__init__()
        self.config = config
        
        self.model = ConditionMLPBCModel(
            state_dim=config['state_dim'],
            act_dim=config['act_dim'],
            goal_dim=config['goal_dim'],
            hidden_size=256,#config['embed_dim'],
            n_layer=config['n_layer'],
        )
       
        self.max_ep_len = config['max_ep_len']
        self.state_dim = config['state_dim']
        self.act_dim = config['act_dim']
        self.goal_dim = config['goal_dim']
        self.max_ep_len = 8#config['max_ep_len']
        self.K = 8#config['K']

        self.batch_size = config['batch_size']*10
        self.log_to_wandb = config['log_to_wandb']
        self.device = config['device']

        self.init_mlp_dataset(config)
        self.update_mlp_dataset_info()

        self.model = self.model.to(device=config['device'])
        self.loss_fn = lambda s_hat, a_hat, r_hat, s, a, r: torch.mean((a_hat - a)**2)

        warmup_steps = config['warmup_steps']

        """
        optimizer = torch.optim.AdamW(
            model.parameters(),
            lr=config['learning_rate'],
            weight_decay=config['weight_decay'],
        )
        """
        self.optimizer = optim.Yogi(
            self.model.parameters(),
            lr= 1e-2,#config['learning_rate'],
            betas=(0.9, 0.999),
            eps=1e-3,
            initial_accumulator=1e-6,
            weight_decay=0#config['weight_decay'],
        )
        
        self.scheduler = torch.optim.lr_scheduler.LambdaLR(
            self.optimizer,
            lambda steps: min((steps+1)/warmup_steps, 1)
        )


    def init_mlp_dataset(self, config):
        # load dataset
        dataset_path = config['dataset_path'] #f'data/{env_name}-{dataset}-v2.pkl'
        with open(dataset_path, 'rb') as f:
            self.mlp_trajectories = pickle.load(f)

        states_traj, actions_traj, traj_lens, returns, goals = [], [], [], [], []
        for path in self.mlp_trajectories:
            states_traj.append(path['observations'])
            actions_traj.append(path['actions'])
            traj_lens.append(len(path['observations']))
            returns.append(0)
            goals.append(path['goals'])

        self.states_traj = states_traj
        self.actions_traj = actions_traj
        self.goals_traj = goals
        
        #traj_lens, returns = np.array(traj_lens), np.array(returns)
        traj_lens = np.array(traj_lens)

        # used for input normalization
        states = np.concatenate(states_traj, axis=0)
        self.mlp_state_mean, self.mlp_state_max = np.mean(states, axis=0), np.max(states, axis=0) + 1e-6
        #self.mlp_state_max = self.mlp_state_max - self.mlp_state_mean


        # used for action normalization
        actions = np.concatenate(actions_traj, axis=0)
        self.mlp_action_mean, self.mlp_action_max = self.mlp_state_mean, self.mlp_state_max#np.mean(actions, axis=0), np.max(actions, axis=0) + 1e-6
        #self.mlp_action_max = self.mlp_action_max - self.mlp_action_mean

        # used for goal normalization
        goals = np.concatenate(goals, axis=0)
        self.mlp_goal_mean, self.mlp_goal_max = np.mean(goals, axis=0), np.max(goals, axis=0) + 1e-6

    def update_mlp_dataset_info(self):
        #self.mlp_num_timesteps = 0
        #for stj in self.states_traj:
        self.mlp_num_timesteps=len(self.states_traj)

        # only train on top pct_traj trajectories (for %BC experiment)
        self.mlp_num_timesteps = max(int(self.mlp_num_timesteps), 1)
    
    def update_mlp_dataset(self, straj, atraj, gtraj):
        self.states_traj.append(straj)
        self.actions_traj.append(atraj)
        self.goals_traj.append(gtraj)
        self.update_mlp_dataset_info()


    def load_resume_dataset(self, path):
        states_traj, actions_traj, goals = np.load(path, allow_pickle=True)
        self.states_traj = states_traj
        self.actions_traj = actions_traj
        self.goals_traj = goals



    def get_mlp_batch(self, batch_size=64):
        device = self.device
        batch_size = batch_size
        max_len = self.K
        batch_inds = np.random.choice(
            np.arange(len(self.mlp_trajectories)),
            size=batch_size,
            replace=True,
        )

        s, a, r, d, rtg, timesteps, mask, gs = [], [], [], [], [], [], [], []
        for i in range(batch_size):
            traj_id = int(batch_inds[i])
            traj = self.states_traj[traj_id]
            si = random.randint(0, traj.shape[0] - max_len)

            # get sequences from dataset
            s.append(self.states_traj[traj_id][si:si + max_len].reshape(1, -1, self.state_dim))
            a.append(self.actions_traj[traj_id][si:si + max_len].reshape(1, -1, self.act_dim))
          
            timesteps.append(np.arange(si, si + s[-1].shape[1]).reshape(1, -1))
            timesteps[-1][timesteps[-1] >= self.max_ep_len] = self.max_ep_len-1  # padding cutoff

            goal = self.goals_traj[traj_id][si]

            goal = np.broadcast_to(goal,(s[-1].shape[0],s[-1].shape[1],self.goal_dim))

            gs.append(goal)

            if gs[-1].shape[1] <= s[-1].shape[1]:
                gs[-1] = np.concatenate([gs[-1], np.zeros((1, 1, self.goal_dim))], axis=1)

            # padding and state + reward normalization
            tlen = s[-1].shape[1]
            #s[-1] = np.concatenate([s[-1]], axis=1)
            s[-1] = minmax_normalize(s[-1], self.mlp_state_mean, self.mlp_state_max)     #(s[-1] - self.mlp_state_mean) / self.mlp_state_max
            #a[-1] = np.concatenate([a[-1]], axis=1)
            a[-1] = minmax_normalize(a[-1], self.mlp_action_mean, self.mlp_action_max)   #(a[-1] - self.mlp_action_mean) / self.mlp_action_max
            timesteps[-1] = np.concatenate([np.zeros((1, max_len - tlen)), timesteps[-1]], axis=1)
            mask.append(np.concatenate([np.zeros((1, max_len - tlen)), np.ones((1, tlen))], axis=1))

            gs[-1] = np.concatenate([gs[-1]], axis=1)
            gs[-1] = minmax_normalize(gs[-1], self.mlp_goal_mean, self.mlp_goal_max) #(gs[-1] - self.mlp_goal_mean) / (self.mlp_goal_max - self.mlp_goal_mean)

        
        s = torch.from_numpy(np.concatenate(s, axis=0)).to(dtype=torch.float32, device=device)
        a = torch.from_numpy(np.concatenate(a, axis=0)).to(dtype=torch.float32, device=device)
        timesteps = torch.from_numpy(np.concatenate(timesteps, axis=0)).to(dtype=torch.long, device=device)
        mask = torch.from_numpy(np.concatenate(mask, axis=0)).to(device=device)
        gs = torch.from_numpy(np.concatenate(gs, axis=0)).to(dtype=torch.float32, device=device)

        return s, a,  gs


    def train(self):
        print('traing policy...')
        for iter in range(self.config['max_iters']):
            print('iter')
            self.train_iteration(num_steps=self.config['num_steps_per_iter'], iter_num=iter+1, print_logs=True)
            self.model.save(PATH='weights/insert_mlp.pth')
            #model.load(PATH='weights/insert_'+config['model_type']+'transformer.pth', device=config['device'])


    def train_iteration(self, num_steps, iter_num=0, print_logs=False):
        train_losses = []

        train_start = time.time()

        self.model.train()
        for _ in range(num_steps):
            train_loss = self.train_step()
            train_losses.append(train_loss)
            if self.scheduler is not None:
                self.scheduler.step()

            if (_%100==0):
                print(iter_num, _, np.mean(train_losses[-100:]))
                self.model.save(PATH='weights/insert_mlp.pth')

        #logs['time/mlp_training'] = time.time() - train_start

        #logs['training/mlp_train_loss_mean'] = np.mean(train_losses)

        return 

    def train_step(self):
        states, actions, goals = self.get_mlp_batch(self.batch_size)
        #states_2nd, actions_2nd, goals_2nd = self.get_mlp_batch(self.batch_size)
        action_target = torch.clone(actions)
        loss = 0

        for step in range(states.shape[1]):
            action_preds = self.model.forward(states[:,step,:], goals[:,step,:])
            action_gt = actions[:,step,:]
            loss+=torch.mean((action_preds-action_gt)**2)
        '''
        for step in range(states.shape[1]):
            if step == 0:
                action_preds = self.model.forward(states[:,step,:], goals[:,step,:])
            else:
                action_preds = self.model.forward(state_preds, goals[:,step,:])

            action_preds = action_preds 
            action_gt = actions[:,step,:]
            loss+=torch.mean((action_preds-action_gt)**2)
            state_preds = action_preds + torch.rand(action_preds.shape).to(device=self.device) * 0.1
        '''

        for step in range(states.shape[1]):
            if step == 0:
                action_preds = self.model.forward(states[:,step,:], goals[:,step,:])
            else:
                action_preds = self.model.forward(state_preds, goals[:,step,:])

            action_preds = action_preds
            action_gt = actions[:,step,:]
            loss+=torch.mean((action_preds-action_gt)**2)
            state_preds = action_preds
      

        self.optimizer.zero_grad()
        loss.backward()
        #torch.nn.utils.clip_grad_norm_(self.model.parameters(), .25)
        self.optimizer.step()

        return loss.detach().cpu().item()

    def save_mlp(self):
        data_dict = {'state_mean': self.mlp_state_mean, 
                     'state_max':  self.mlp_state_max ,
                     'action_mean':  self.mlp_action_mean, 
                     'action_max':  self.mlp_action_max, 
                     'goal_mean':  self.mlp_goal_mean, 
                     'goal_max':   self.mlp_goal_max}

        np.save(self.config['dec_normalizer_path'], data_dict)
        self.model.save(PATH=self.config['weight_path'])

    def load_mlp(self):
        data_dict = np.load(self.config['dec_normalizer_path'], allow_pickle=True)
        self.mlp_state_mean =  data_dict.item().get('state_mean')
        self.mlp_state_std  = data_dict.item().get('state_max')
        self.mlp_action_mean = data_dict.item().get('action_mean') 
        self.mlp_action_max = data_dict.item().get('action_max')
        self.mlp_goal_mean = data_dict.item().get('goal_mean') 
        self.mlp_goal_max = data_dict.item().get('goal_max')

        self.model.load(PATH=self.config['weight_path'], device=self.config['device'])





    def predict(self, state, goal):

        state = np.array(state)
        state = torch.from_numpy(state).reshape(1, self.state_dim).to(device=self.config['device'], dtype=torch.float32)

        goal = np.array(goal)
        goal = torch.from_numpy(goal).reshape(1, self.goal_dim).to(device=self.config['device'], dtype=torch.float32)

        mlp_state_mean = torch.from_numpy(self.mlp_state_mean).to(device=self.config['device'], dtype=torch.float32)
        mlp_state_max = torch.from_numpy(self.mlp_state_max).to(device=self.config['device'], dtype=torch.float32)
        mlp_goal_mean = torch.from_numpy(self.mlp_goal_mean).to(device=self.config['device'], dtype=torch.float32)
        mlp_goal_max = torch.from_numpy(self.mlp_goal_max).to(device=self.config['device'], dtype=torch.float32)
        mlp_action_max = self.mlp_action_max
        mlp_action_mean = self.mlp_action_mean

        action = self.model(
                            minmax_normalize(state.to(dtype=torch.float32), mlp_state_mean, mlp_state_max),
                            minmax_normalize(goal.to(dtype=torch.float32), mlp_goal_mean, mlp_goal_max),
                        )
        action = action.detach().cpu().numpy()
        action = action * (mlp_action_max-mlp_action_mean) + mlp_action_mean
        #print(action)

        '''
        # state shape (1,7), goal shape (1, 3)
        state = minmax_normalize(state, self.mlp_state_mean, self.mlp_state_max)
        goal = minmax_normalize(goal, self.mlp_goal_mean, self.mlp_goal_max)
        #print(state, goal)

        state = torch.from_numpy(state).to(device=self.device, dtype=torch.float32)
        goal = torch.from_numpy(goal).to(device=self.device, dtype=torch.float32)

        print(state, goal)
        action = self.model(state, goal)
        #print(action)
        action = action.cpu().detach().numpy()
        action = action * (self.mlp_action_max-self.mlp_action_mean) + self.mlp_action_mean
        '''
        return action[0]


    def test(self):
        try:
            #sys.path.append('/home/alex/catkin_ws/src/imitation_learning_ros/src/transformer_il/franka-pybullet/src/')
            from panda import Panda
        except:
            sys.path.append('franka-pybullet/src/')
            from panda import Panda
            print('cannot load panda libs')
        duration = 3000
        stepsize = 1e-3
        robot = Panda(stepsize)
        robot.setControlMode("position")
        target_return = 0       
        device = self.device 

        mlp_state_mean = torch.from_numpy(self.mlp_state_mean).to(device=device, dtype=torch.float32)
        mlp_state_max = torch.from_numpy(self.mlp_state_max).to(device=device, dtype=torch.float32)
        mlp_goal_mean = torch.from_numpy(self.mlp_goal_mean).to(device=device, dtype=torch.float32)
        mlp_goal_max = torch.from_numpy(self.mlp_goal_max).to(device=device, dtype=torch.float32)
        mlp_action_max = self.mlp_action_max
        mlp_action_mean = self.mlp_action_mean

        for tid, straj in enumerate(self.states_traj*2):
           
            states, actions, gs = self.get_mlp_batch(batch_size=1)
            tmp = states * (mlp_state_max - mlp_state_mean) + mlp_state_mean
            gs = gs *(mlp_goal_max-mlp_goal_mean)+mlp_goal_mean 

            state = tmp[:,0,:]#torch.from_numpy(jt[0]).reshape(1, self.state_dim).to(device=device, dtype=torch.float32)#torch.from_numpy(self.states_traj[random.randint(0,53)][random.randint(4,7)]).reshape(1, self.state_dim).to(device=device, dtype=torch.float32)
            #state = torch.from_numpy(self.states_traj[random.randint(0,53)][random.randint(4,9)]).reshape(1, self.state_dim).to(device=device, dtype=torch.float32)

            #goal = gs[:1,0,:] #torch.from_numpy(goal).reshape(1, self.goal_dim).to(device=device, dtype=torch.float32)
            goal = (-2 * torch.rand(1, 3) + 1).numpy() * (self.mlp_goal_max - self.mlp_goal_mean) + self.mlp_goal_mean #self.decoder.goals_traj[tid][0]#[random.randint(0,53)][0]
            goal = torch.from_numpy(goal).to(device)
            #print(goal)
            for _ in range(30):
                #print(state)
                init_j = state.clone().cpu().detach().numpy()[0]
                euler_pose = robot.step(init_j)    
                real = np.array([euler_pose[0], euler_pose[1], euler_pose[3]/np.pi*180])
                #print(goal[0].cpu().detach().numpy()-real)
            
            for t in range(states.shape[1]+4):
                action = self.model(
                    minmax_normalize(state.to(dtype=torch.float32), mlp_state_mean, mlp_state_max),
                    minmax_normalize(goal.to(dtype=torch.float32), mlp_goal_mean, mlp_goal_max),
                )
                #action = actions[:,t,:]
                action = action.detach().cpu().numpy()
                action = action * (mlp_action_max-mlp_action_mean) + mlp_action_mean
                print(action)

                j_pose =  action[0]
                #j_pose = tmp[:,t,:].detach().cpu().numpy()[0]
                state = torch.from_numpy(j_pose).reshape(1, self.state_dim).to(device=device, dtype=torch.float32)
                #print(j_pose, jt[t+1])
                ee_pose = robot.step(j_pose)
                time.sleep(0.1)
                print(robot.getJointStates())
                print('----------------------------')

                #state = torch.from_numpy(state).to(device=device, dtype=torch.float32)
                #print(state)

                if False:#t == 4:
                    # random change goal
                    origin_goal = goal
                    goal = self.goals_traj[random.randint(0,53)][0]
                    goal = torch.from_numpy(goal).to(device=device).reshape(1, self.goal_dim)
                #origin_goal = goal
                #torch.cat(
                    #[goal, pred_goal.reshape(1, 1)], dim=1)
                
                
            target = goal[0].cpu().detach().numpy()
            real = np.array([ee_pose[0], ee_pose[1], ee_pose[3]/np.pi*180])
            #print(real)
            #print((goal- mlp_goal_mean) / (mlp_goal_max - mlp_goal_mean))
            print(target-real, target)#, origin_goal[0].cpu().detach().numpy()-real, target-origin_goal[0].cpu().detach().numpy())

class ImitationEncoder(object):
    """docstring for ImitationEncoder"""
    def __init__(self, config):
        super(ImitationEncoder, self).__init__()
        self.config = config
        

        self.max_ep_len = config['max_ep_len']
        self.state_dim = config['state_dim']
        self.goal_dim = config['goal_dim']
        self.K = config['K']
        self.batch_size = config['batch_size']
        self.log_to_wandb = config['log_to_wandb']
        self.device = config['device']


        n_heads = self.config['n_head'] #1
        batch_size = self.config['batch_size'] #64
        max_len = self.config['max_ep_len'] #10
        d_ff = self.config['embed_dim']
        dropout = self.config['dropout'] #0.1
        n_layers = self.config['n_layer'] #3
        enc_lr = self.config['learning_rate']

        self.model = TransformerEncoder(self.state_dim, d_ff, n_heads=n_heads, n_layers=n_layers, dropout=dropout)
        self.model = self.model.to(device=config['device'])

        self.diagnostics = dict()
        self.start_time = time.time()

        warmup_steps = config['warmup_steps']

        """
        optimizer = torch.optim.AdamW(
            model.parameters(),
            lr=config['learning_rate'],
            weight_decay=config['weight_decay'],
        )
        """
        self.optimizer = optim.Yogi(
            self.model.parameters(),
            lr= enc_lr,#config['learning_rate'],
            betas=(0.9, 0.999),
            eps=1e-3,
            initial_accumulator=1e-6,
            weight_decay=0#config['weight_decay'],
        )
        
        self.scheduler = torch.optim.lr_scheduler.LambdaLR(
            self.optimizer,
            lambda steps: min((steps+1)/warmup_steps, 1)
        )


        self.init_enc_dataset()
        self.update_enc_dataset_info()


        

    def init_enc_dataset(self, ratio=1.0, validate_ratio=0.2):
        # load dataset
        """
        dataset_path = config['dataset_path'] #f'data/{env_name}-{dataset}-v2.pkl'
        
        goal_datset = np.load(dataset_path, allow_pickle=True)

        self.trajectories = goal_datset[0].tolist()
        self.trajectories_goals = goal_datset[1].tolist()
        traj_lens = []
        for path in self.trajectories:
            traj_lens.append(len(path))
        self.enc_num_timesteps = sum(traj_lens)
        """
        enc_data_paths = self.config['data_path_v2']+'*'
        all_flist = glob.glob(enc_data_paths) 

        f_num = len(all_flist)
        val_flist = all_flist[:int(f_num*validate_ratio)]
        new_flist = all_flist[:int(f_num*(1-validate_ratio))]
        print(ratio, f_num, len(new_flist), 194*ratio)
        f_num = len(new_flist)
        new_flist = new_flist[:int(float(f_num*ratio))]


        # generate all dataset
        all_trajectories = []
        all_trajectories_goals = []
        for fname in all_flist:
            g_traj = []
            traj = []

            new_enc_data = np.load(fname, allow_pickle=True)
            goal = np.array([new_enc_data[2][0][0]*1, new_enc_data[2][0][1]*1, new_enc_data[2][0][2]])
            #print(goal)
            for info in new_enc_data[3]:
                if len(info['ee_cartesian_wrench_sequence']) > 2:
                    #g_traj.append(goal)
                    #print(info)

                    states, gts = parse_xy(info, goal)

                    for iid in range(len(states)):
                        traj.append(states[iid])
                        g_traj.append(gts[iid])

            #print(len(traj))
            all_trajectories.append(np.array(traj)[:])
            all_trajectories_goals.append(np.array(g_traj)[:])

        # generate valiadtion dataset
        val_trajectories = []
        val_trajectories_goals = []
        for fname in val_flist:
            g_traj = []
            traj = []

            new_enc_data = np.load(fname, allow_pickle=True)
            goal = np.array([new_enc_data[2][0][0]*1, new_enc_data[2][0][1]*1, new_enc_data[2][0][2]])
            #print(goal)
            for info in new_enc_data[3]:
                if len(info['ee_cartesian_wrench_sequence']) > 2:
                    #g_traj.append(goal)
                    #print(info)

                    states, gts = parse_xy(info, goal)

                    for iid in range(len(states)):
                        traj.append(states[iid])
                        g_traj.append(gts[iid])

            #print(len(traj))
            val_trajectories.append(np.array(traj)[:])
            val_trajectories_goals.append(np.array(g_traj)[:])

        # generate training dataset
        trajectories = []
        trajectories_goals = []
        for fname in new_flist:
            g_traj = []
            traj = []

            new_enc_data = np.load(fname, allow_pickle=True)
            goal = np.array([new_enc_data[2][0][0]*1, new_enc_data[2][0][1]*1, new_enc_data[2][0][2]])
            #print(goal)
            for info in new_enc_data[3]:
                if len(info['ee_cartesian_wrench_sequence']) > 2:
                    #g_traj.append(goal)
                    #print(info)

                    states, gts = parse_xy(info, goal)

                    for iid in range(len(states)):
                        traj.append(states[iid])
                        g_traj.append(gts[iid])

            #print(len(traj))
            trajectories.append(np.array(traj)[:])
            trajectories_goals.append(np.array(g_traj)[:])


        self.trajectories = trajectories
        self.trajectories_goals = trajectories_goals

        self.val_trajectories = val_trajectories
        self.val_trajectories_goals = val_trajectories_goals


        self.all_trajectories = all_trajectories
        self.all_trajectories_goals = all_trajectories_goals

        traj_lens = []
        for path in self.trajectories:
            traj_lens.append(len(path))
        self.enc_num_timesteps = sum(traj_lens)

        #try:
        #    self.load_encoder()
        #except:
        #    print('did not find the data for mornalizer!')
        self.reset_normalizer()
        #self.save_encoder()
        self.update_enc_dataset_info()


    def reset_normalizer(self):
        #print(self.trajectories)
        #print(self.trajectories[0].shape, self.trajectories[1].shape)
        print('reset encoder normalizer...')
        states = np.concatenate(self.all_trajectories, axis=0)
        self.enc_state_mean, self.enc_state_std = np.mean(states, axis=0), np.std(states, axis=0) + 1e-6

        # used for goal normalization
        goals = np.concatenate(self.all_trajectories_goals, axis=0)
        self.enc_goal_mean, self.enc_goal_max = np.mean(goals, axis=0), np.max(goals, axis=0) + 1e-6
        print(len(self.all_trajectories_goals))
        print(np.mean(goals, axis=0), np.max(goals, axis=0) + 1e-6, np.min(goals, axis=0))

    
    def update_enc_dataset_info(self):
        self.enc_num_timesteps = 0
        for stj in self.trajectories:
            self.enc_num_timesteps+=1

        # only train on top pct_traj trajectories (for %BC experiment)
        self.enc_num_timesteps = max(int(self.enc_num_timesteps), 1)
    
    def update_enc_dataset(self, straj, gtraj):
        self.trajectories.append(straj)
        self.trajectories_goals.append(gtraj)
        self.update_enc_dataset_info()


    def load_resume_dataset(self, path):
        states_traj, actions_traj, goals = np.load(self.config['online_goal_data_path'], allow_pickle=True)
        self.states_traj = states_traj
        self.actions_traj = actions_traj
        self.goals_traj = goals

    def predict(self, enc_s):
        enc_s = np.array(enc_s)
        enc_s = enc_s[-self.K:]
        tlen = enc_s.shape[0]

        #print(enc_s, '!')

        enc_s = enc_s.reshape(1, -1, self.state_dim)
        enc_s = np.concatenate([np.zeros((1, self.K - enc_s.shape[1], self.state_dim)), enc_s], axis=1)
        #enc_s = std_normalize(enc_s, self.enc_state_mean, self.enc_state_std) #(s[-1] - self.enc_state_mean) / self.enc_state_std
        enc_mask = np.concatenate([np.zeros((1, self.K - tlen)), np.ones((1, tlen))], axis=1)
        enc_mask = np.expand_dims(enc_mask, axis=0)

        #convert data to tensor
        enc_s = torch.from_numpy(enc_s).to(dtype=torch.float32, device=self.device)
        enc_mask = torch.from_numpy(enc_mask).to(dtype=torch.float32, device=self.device)

        self.model.eval()

        goals_preds = self.model(
            enc_s, enc_mask, [tlen]
        )

        goals_preds = reverse_max_normalize(goals_preds.cpu().detach().numpy(), self.enc_goal_mean, self.enc_goal_max)

        return goals_preds

    def get_enc_batch(self, batch_size):
        device = self.device
        max_len = self.K
        batch_inds = np.random.choice(
            np.arange(len(self.trajectories)),
            size=batch_size,
            replace=True,
        )

        s, gs, mask, lengths = [], [], [], []
        for i in range(batch_size):
            s_traj = self.trajectories[int(batch_inds[i])]
            g_traj = self.trajectories_goals[int(batch_inds[i])]
            si = random.randint(0,  int(s_traj.shape[0] / 5))
            si_max = random.randint(3, max_len)

            # get sequences from dataset
            sampled_s = s_traj[si:si + si_max]
            tmp_s = sampled_s.copy()
            order = np.array(range(sampled_s.shape[0]))            
            if np.random.rand() > 0.85:
                np.random.shuffle(order)

            # in-place changing of values
            sampled_s[np.array(range(sampled_s.shape[0]))] = tmp_s[order]
            sampled_s = sampled_s.reshape(1, -1, self.state_dim)
            s.append(sampled_s)
            gs.append(g_traj[si])
          
            # padding and state + reward normalization
            tlen = s[-1].shape[1]
            lengths.append(tlen)

            s[-1] = np.concatenate([np.zeros((1, max_len - tlen, self.state_dim)), s[-1]], axis=1)
            #s[-1] = std_normalize(s[-1], self.enc_state_mean, self.enc_state_std) #(s[-1] - self.enc_state_mean) / self.enc_state_std
            #gs[-1] = np.concatenate([np.zeros((1, max_len - tlen, goal_dim)), gs[-1]], axis=1)
            gs[-1] = minmax_normalize(gs[-1], self.enc_goal_mean, self.enc_goal_max) #(gs[-1] - self.enc_goal_mean) / (self.enc_goal_max - self.enc_goal_mean)

            mask.append(np.concatenate([np.zeros((1, max_len - tlen)), np.ones((1, tlen))], axis=1))


        s = torch.from_numpy(np.concatenate(s, axis=0)).to(dtype=torch.float32, device=device)
        gs = torch.from_numpy(np.array(gs)).to(dtype=torch.float32, device=device)
        #print(gs.shape)
        
        mask = torch.from_numpy(np.concatenate(mask, axis=0)).to(device=device)
        #lengths = torch.from_numpy(lengths).to(device=device)

        return s, gs, mask, lengths

    def get_enc_eval_batch(self):
        device = self.device
        max_len = self.K

        s, gs, mask, lengths = [], [], [], []
        for i in range(len(self.val_trajectories)):
            s_traj = self.val_trajectories[i]
            g_traj = self.val_trajectories_goals[i]


            for si in range(s_traj.shape[0]-10):
                # get sequences from dataset
                sampled_s = s_traj[:si+10][-self.K:]
                
                sampled_s = sampled_s.reshape(1, -1, self.state_dim)
                s.append(sampled_s)
                gs.append(g_traj[si])
              
                # padding and state + reward normalization
                #print(s[-1].shape)
                tlen = s[-1].shape[1]
                lengths.append(tlen)

                s[-1] = np.concatenate([np.zeros((1, max_len - tlen, self.state_dim)), s[-1]], axis=1)
                #s[-1] = std_normalize(s[-1], self.enc_state_mean, self.enc_state_std) #(s[-1] - self.enc_state_mean) / self.enc_state_std
                #gs[-1] = np.concatenate([np.zeros((1, max_len - tlen, goal_dim)), gs[-1]], axis=1)
                gs[-1] = minmax_normalize(gs[-1], self.enc_goal_mean, self.enc_goal_max) #(gs[-1] - self.enc_goal_mean) / (self.enc_goal_max - self.enc_goal_mean)

                mask.append(np.concatenate([np.zeros((1, max_len - tlen)), np.ones((1, tlen))], axis=1))


        s = torch.from_numpy(np.concatenate(s, axis=0)).to(dtype=torch.float32, device=device)
        gs = torch.from_numpy(np.array(gs)).to(dtype=torch.float32, device=device)
        #print(gs.shape)
        
        mask = torch.from_numpy(np.concatenate(mask, axis=0)).to(device=device)
        #lengths = torch.from_numpy(lengths).to(device=device)

        return s, gs, mask, lengths

    def validation_step(self):
        states, goals, attention_mask, lengths = self.get_enc_eval_batch()
        goals_target = torch.clone(goals)
        self.model.eval()
        goals_preds = self.model(
            states, attention_mask, lengths
        )

        # note: currently indexing & masking is not fully correct
        loss = torch.mean((goals_preds-goals_target)**2)

        with torch.no_grad():
            self.diagnostics['validation/goal_error'] = torch.mean((goals_preds-goals_target)**2).detach().cpu().item()

        return loss.detach().cpu().item()

    def enc_evaluation(self):
        val_len = len(self.val_trajectories)
        records = []

        for vid in range(val_len):
            xs = self.val_trajectories[vid]
            ys = self.val_trajectories_goals[vid] 

            traj_len = len(xs)

            if traj_len > 5:
                traj_len = np.min([traj_len, 15])

                for tid in range(traj_len-1):
                    x = xs[:tid+1]
                    pred = self.predict(x)
                    records.append([pred, ys[0],tid+1, traj_len])

        return records


    def train_iteration(self, num_steps, iter_num=0, print_logs=False):

        train_losses = []
        logs = dict()

        train_start = time.time()

        self.model.train()
        for _ in range(num_steps):
            train_loss = self.train_step()
            train_losses.append(train_loss)
            if self.scheduler is not None:
                self.scheduler.step()

            if (_%100==0):
                print(iter_num, _, np.mean(train_losses[-100:]))

            #if (_%200==0):    
            #    self.model.save(PATH=self.config['weight_path'])


        logs['time/training'] = time.time() - train_start

        
        logs['training/train_loss_mean'] = np.mean(train_losses)
        logs['training/train_loss_std'] = np.std(train_losses)

        if print_logs:
            print('=' * 80)
            print(f'Iteration {iter_num}')
            for k, v in logs.items():
                print(f'{k}: {v}')

        return logs

    def train_step(self):
        states, goals, attention_mask, lengths = self.get_enc_batch(self.batch_size)
        goals_target = torch.clone(goals)

        goals_preds = self.model(
            states, attention_mask, lengths
        )

        # note: currently indexing & masking is not fully correct
        loss = torch.mean((goals_preds-goals_target)**2)
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()

        with torch.no_grad():
            self.diagnostics['training/goal_error'] = torch.mean((goals_preds-goals_target)**2).detach().cpu().item()

        return loss.detach().cpu().item()


    def train(self, max_iters=None, num_steps=None):
        if (max_iters is None) and (num_steps is None):
            max_iters = self.config['max_iters']
            num_steps = self.config['num_steps_per_iter']
        for iter in range(max_iters):
            print('iter')
            outputs = self.train_iteration(num_steps=num_steps, iter_num=iter+1, print_logs=True)
            self.model.save(PATH=self.config['weight_path'])

    def save_encoder(self, weight_path=None):
        print('save encoder ...')
        data_dict = {'state_mean': self.enc_state_mean, 
                     'state_max':  self.enc_state_std ,
                     'goal_mean':  self.enc_goal_mean, 
                     'goal_max':   self.enc_goal_max}

        np.save(self.config['enc_normalizer_path'], data_dict)

        if weight_path is None:
            self.model.save(PATH=self.config['weight_path'])
        else:
            self.model.save(PATH=weight_path)

    def load_encoder(self, weight_path=None):
        print('load encoder ...')
        data_dict = np.load(self.config['enc_normalizer_path'], allow_pickle=True)
        self.enc_state_mean =  data_dict.item().get('state_mean')
        self.enc_state_std  = data_dict.item().get('state_max')
        self.enc_goal_mean = data_dict.item().get('goal_mean') 
        self.enc_goal_max = data_dict.item().get('goal_max')
        if weight_path is None:    
            self.model.load(PATH=self.config['weight_path'], device=self.config['device'])
        else:
            self.model.load(PATH=weight_path, device=self.config['device'])



        
class ImitationTransformer(object):
    """docstring for ImitationTransformer"""
    def __init__(self, config_path="configs/config.yaml"):
        super(ImitationTransformer, self).__init__()
        
        pi_config = read_yaml(config_path)['DEC_INFO']
        enc_config = read_yaml(config_path)['ENC_INFO']
        imitation_config = read_yaml(config_path)['IMITATION_INFO']
        #self.policy = ImitationDecoder(dec_config)
        self.policy = ImitationMLP(pi_config)
        self.encoder = ImitationEncoder(enc_config)
        self.pi_config = pi_config
        self.enc_config = enc_config
        self.imitation_config = imitation_config
        
        #self.sim_robot = Panda(1e-3)
        #self.sim_robot.setControlMode("position")


        #self.online_rl_data = None
        #self.online_goal_data = None

        if imitation_config['resume']:
            self.load()
            self.policy.load_resume_dataset(path=imitation_config['online_rl_data_path'])
            self.encoder.load_resume_dataset(path=imitation_config['online_goal_data_path'])
            self.policy.update_mlp_dataset_info()
            self.encoder.update_enc_dataset_info()


    def train(self, max_iters=10):
        for iter in range(max_iters):
            outputs = self.train_iteration(num_steps=self.pi_config['num_steps_per_iter'], iter_num=iter+1, print_logs=True)
            self.save()

    def train_iteration(self, num_steps, iter_num=0, print_logs=False):
        dec_train_losses = []
        enc_train_losses = []

        logs = dict()

        self.policy.model.train()
        self.encoder.model.train()

        for _ in range(num_steps):
            dec_train_loss = self.policy.train_step()
            enc_train_loss = self.encoder.train_step()
            dec_train_losses.append(dec_train_loss)
            enc_train_losses.append(enc_train_loss)

            if self.policy.scheduler is not None:
                self.policy.scheduler.step()

            if self.encoder.scheduler is not None:
                self.encoder.scheduler.step()

            if (_%100==0):
                print(iter_num, _, np.mean(dec_train_losses[-100:]), np.mean(enc_train_losses[-100:]))
                self.save()

        logs['training/dec_train_loss_mean'] = np.mean(dec_train_losses)
        logs['training/enc_train_loss_mean'] = np.mean(enc_train_losses)
        #logs['training/train_loss_std'] = np.std(train_losses)

        #for k in self.diagnostics:
        #    logs[k] = self.diagnostics[k]

        if print_logs:
            print('=' * 80)
            print(f'Iteration {iter_num}')
            for k, v in logs.items():
                print(f'{k}: {v}')

        return logs

    def predict(self, enc_s, pi_s):
        # first do goal pose prediction using transformer encoder
        tlen = enc_s.shape[1]
        enc_s = enc_s[-self.encoder.K:].reshape(1, -1, self.encoder.state_dim)
        enc_s = np.concatenate([np.zeros((1, self.encoder.K - enc_s.shape[1], self.encoder.state_dim)), enc_s], axis=1)
        enc_s = std_normalize(enc_s, self.encoder.enc_state_mean, self.encoder.enc_state_std) #(s[-1] - self.enc_state_mean) / self.enc_state_std
        enc_mask.append(np.concatenate([np.zeros((1, self.encoder.K - tlen)), np.ones((1, tlen))], axis=1))
        enc_len = tlen

        #convert data to tensor
        enc_s = torch.from_numpy(enc_s).to(device)
        goals_preds = self.model(
            enc_s, enc_mask, enc_len
        )

        # Then do goal-conditioned insertion RL
        pi_s = minmax_normalize(pi_s, self.policy.mlp_state_mean, self.policy.mlp_state_max)
        pi_s = torch.from_numpy(pi_s).to(device)
        action = self.model(
                    minmax_normalize(state.to(dtype=torch.float32), mlp_state_mean, mlp_state_max),
                    minmax_normalize(goal.to(dtype=torch.float32), mlp_goal_mean, mlp_goal_max),
                )

        # convert from tensor to numpy
        action = action.detach().numpy()

        # revert the normalization scale
        action = reverse_max_normalize(action, self.policy.mlp_action_mean, self.policy.mlp_action_max)

        return action

    def update_dataset(self, enc_straj=None, enc_gtraj=None, dec_straj=None, dec_atraj=None, dec_gtraj=None):
        if (dec_straj is not None) and (dec_atraj is not None) and (dec_gtraj is not None):
            self.update_mlp_dataset(dec_straj, dec_atraj, dec_gtraj)

        if (enc_straj is not None) and (enc_gtraj is not None):
            self.update_enc_dataset(enc_straj, enc_gtraj)
        return

    def save(self):
        print('saving parameters...')
        """
        with open(self.imitation_config['policy_state_mean_path'], 'wb') as f:
            np.save(f, self.policy.mlp_state_mean)
        with open(self.imitation_config['policy_state_max_path'], 'wb') as f:
            np.save(f, self.policy.mlp_state_max)
        with open(self.imitation_config['policy_action_mean_path'], 'wb') as f:
            np.save(f, self.policy.mlp_action_mean)
        with open(self.imitation_config['policy_action_max_path'], 'wb') as f:
            np.save(f, self.policy.mlp_action_max)
        with open(self.imitation_config['policy_goal_mean_path'], 'wb') as f:
            np.save(f, self.policy.mlp_goal_mean)
        with open(self.imitation_config['policy_goal_max_path'], 'wb') as f:
            np.save(f, self.policy.mlp_goal_max)
        with open(self.imitation_config['encoder_state_mean_path'], 'wb') as f:
            np.save(f, self.encoder.enc_state_mean)
        with open(self.imitation_config['encoder_state_std_path'], 'wb') as f:
            np.save(f, self.encoder.enc_state_std)
        with open(self.imitation_config['encoder_goal_mean_path'], 'wb') as f:
            np.save(f, self.encoder.enc_goal_mean)
        with open(self.imitation_config['encoder_goal_max_path'], 'wb') as f:
            np.save(f, self.encoder.enc_goal_max)

        self.encoder.model.save(PATH=self.imitation_config['encoder_weight_path'])
        self.policy.model.save(PATH=self.imitation_config['policy_weight_path'])
        """
        self.encoder.save_encoder()
        self.policy.save_mlp()

        # save the online data
        with open(self.imitation_config['online_rl_data_path'], 'wb') as f:
            online_policy_data =np.array([self.policy.states_traj,
                                      self.policy.actions_traj,
                                      self.policy.goals_traj])
            np.save(f, online_policy_data)
        
        with open(self.imitation_config['online_goal_data_path'], 'wb') as f:
            online_goal_data =  np.array([self.encoder.trajectories, self.encoder.trajectories_goals])
            np.save(f, online_goal_data)


    def load(self, load_pretrain=False):
        print('loading parameters...')
        """
        self.policy.mlp_state_mean = np.load(self.imitation_config['policy_state_mean_path'], allow_pickle=True)
        self.policy.mlp_state_max = np.load(self.imitation_config['policy_state_max_path'], allow_pickle=True)
        self.policy.mlp_action_mean = np.load(self.imitation_config['policy_action_mean_path'], allow_pickle=True)
        self.policy.mlp_action_max = np.load(self.imitation_config['policy_action_max_path'], allow_pickle=True)
        self.policy.mlp_goal_mean = np.load(self.imitation_config['policy_goal_mean_path'],  allow_pickle=True)
        self.policy.mlp_goal_max = np.load(self.imitation_config['policy_goal_max_path'],  allow_pickle=True)
        self.encoder.enc_state_mean = np.load(self.imitation_config['encoder_state_mean_path'], allow_pickle=True)
        self.encoder.enc_state_std = np.load(self.imitation_config['encoder_state_std_path'], allow_pickle=True)
        self.encoder.enc_goal_mean = np.load(self.imitation_config['encoder_goal_mean_path'], allow_pickle=True)
        self.encoder.enc_goal_max = np.load(self.imitation_config['encoder_goal_max_path'], allow_pickle=True)
        """
        self.encoder.load_encoder()
        self.policy.load_mlp()

        if load_pretrain:
            try:
                self.encoder.model.load(PATH=self.imitation_config['pretrained_encoder_weight_path'], device=self.enc_config['device'])
            except:
                True
            self.policy.model.load(PATH=self.imitation_config['pretrained_policy_weight_path'], device=self.pi_config['device'])
        else:
            try:
                self.encoder.model.load(PATH=self.imitation_config['encoder_weight_path'], device=self.enc_config['device'])
            except:
                True
            self.policy.model.load(PATH=self.imitation_config['policy_weight_path'], device=self.pi_config['device'])

   

def test():
    """
    runner = ImitationTransformer()

    #runner.save()
    runner.load()
    #runner.encoder.train()
    #runner.save()
    runner.policy.train()
    runner.save()
    runner.load()
    #runner.train()
    #runner.save()

    #runner.load()
    #runner.test()
    """
    config = read_yaml("configs/config.yaml")['DEC_INFO']
    print(config['device'])
    mlp = ImitationMLP(config)
    #mlp.train()
    mlp.model.load(PATH='weights/insert_mlp.pth', device=config['device'])
    #mlp.test()
    #mlp.train()
    #test.model.load(PATH='weights/insert_mlp.pth', device=config['device'])
    mlp.save_mlp()

    #mlp.test()

def policy_test():
    try:
        #sys.path.append('/home/alex/catkin_ws/src/imitation_learning_ros/src/transformer_il/franka-pybullet/src/')
        from panda import Panda
    except:
        sys.path.append('franka-pybullet/src/')
        from panda import Panda
        print('cannot load panda libs')
        
    #s = np.array([[ 0.38048496,  0.5307994 , -0.453509  , -2.23032938 , 0.06509738 , 2.99417096 , 2.29140421]])
    #goal = np.array([[ 611.30746972  ,-42.10419219 ,-132.45377647]])
    s = np.array([[  0.3806605 ,   0.53083504 , -0.4530171 ,  -2.2303187 ,   0.06511857 ,  2.99421919 ,  2.29125091]])
    goal = np.array([[ 608.90207655 , -43.48653998 ,-129.65017195]])


    config = read_yaml("configs/config.yaml")['DEC_INFO']
    mlp = ImitationMLP(config)
    mlp.model.load(PATH='weights/insert_mlp.pth', device=config['device'])


    duration = 3000
    stepsize = 1e-3
    
    robot = Panda(stepsize)
    robot.setControlMode("position")


    for _ in range(300):
        robot.step(s[0])
        print(s)

    for _ in range(10):
        robot.step(mlp.predict(s, goal))
        print(mlp.predict(s, goal))

    time.sleep(100)


def visualize_sensor(plot_mean=True):
    import matplotlib.pyplot as plt

    enc_data_paths = 'data/infos/*'
    new_flist = glob.glob(enc_data_paths)[-19:-3] 
    print(new_flist)

    for fname in new_flist:
        g_traj = []
        traj = []
        m_traj = []

        new_enc_data = np.load(fname, allow_pickle=True)
        print(new_enc_data[[2]])
        for info in new_enc_data[3]:
            for forces in info['short_ee_cartesian_wrench_sequence']:
                traj.append(forces)
            m_traj.append(np.mean(info['short_ee_cartesian_wrench_sequence'], axis=0))

        m_traj = np.array(m_traj)
        traj = np.array(np.array(traj))
        if plot_mean:
            plt.plot(m_traj[:,0])
        else:
            plt.plot(traj[:,0])

        print(np.array(traj).shape)

    plt.show()

    for fname in new_flist:
        g_traj = []
        traj = []
        m_traj = []

        new_enc_data = np.load(fname, allow_pickle=True)
        print(new_enc_data[[2]])
        for info in new_enc_data[3]:
            for forces in info['short_ee_cartesian_wrench_sequence']:
                traj.append(forces)
            m_traj.append(np.mean(info['short_ee_cartesian_wrench_sequence'], axis=0))

        m_traj = np.array(m_traj)
        traj = np.array(np.array(traj))
        if plot_mean:
            plt.plot(m_traj[:,1])
        else:
            plt.plot(traj[:,1])
        print(np.array(traj).shape)

    plt.show()

    for fname in new_flist:
        g_traj = []
        traj = []
        m_traj = []

        new_enc_data = np.load(fname, allow_pickle=True)
        print(new_enc_data[[2]])
        for info in new_enc_data[3]:
            for forces in info['short_ee_cartesian_wrench_sequence']:
                traj.append(forces)
            m_traj.append(np.mean(info['short_ee_cartesian_wrench_sequence'], axis=0))

        m_traj = np.array(m_traj)
        traj = np.array(np.array(traj))
        if plot_mean:
            plt.plot(m_traj[:,2])
        else:
            plt.plot(traj[:,2])
        print(np.array(traj).shape)

    plt.show()

    for fname in new_flist:
        g_traj = []
        traj = []
        m_traj = []

        new_enc_data = np.load(fname, allow_pickle=True)
        print(new_enc_data[[2]])
        for info in new_enc_data[3]:
            for forces in info['short_ee_cartesian_wrench_sequence']:
                traj.append(forces)
            m_traj.append(np.mean(info['short_ee_cartesian_wrench_sequence'], axis=0))

        m_traj = np.array(m_traj)
        traj = np.array(np.array(traj))
        if plot_mean:
            plt.plot(m_traj[:,3])
        else:
            plt.plot(traj[:,3])
        print(np.array(traj).shape)

    plt.show()

    for fname in new_flist:
        g_traj = []
        traj = []
        m_traj = []

        new_enc_data = np.load(fname, allow_pickle=True)
        print(new_enc_data[[2]])
        for info in new_enc_data[3]:
            for forces in info['short_ee_cartesian_wrench_sequence']:
                traj.append(forces)
            m_traj.append(np.mean(info['short_ee_cartesian_wrench_sequence'], axis=0))

        m_traj = np.array(m_traj)
        traj = np.array(np.array(traj))
        if plot_mean:
            plt.plot(m_traj[:,4])
        else:
            plt.plot(traj[:,4])
        print(np.array(traj).shape)


    plt.show()

    for fname in new_flist:
        g_traj = []
        traj = []
        m_traj = []

        new_enc_data = np.load(fname, allow_pickle=True)
        print(new_enc_data[[2]])
        for info in new_enc_data[3]:
            for forces in info['short_ee_cartesian_wrench_sequence']:
                traj.append(forces)
            m_traj.append(np.mean(info['short_ee_cartesian_wrench_sequence'], axis=0))

        m_traj = np.array(m_traj)
        traj = np.array(np.array(traj))
        if plot_mean:
            plt.plot(m_traj[:,5])
        else:
            plt.plot(traj[:,5])
        print(np.array(traj).shape)

    plt.show()

#visualize_sensor(plot_mean=False)
#visualize_sensor()
#test()
#encoder_test()
"""
config_pth = '/home/alex/catkin_ws/src/imitation_learning_ros/src/imitation_learning_ros_v2/configs/config.yaml'
predictor = ImitationTransformer(config_pth)

if not predictor.imitation_config['resume']:
    # load pretrained models
    predictor.load(load_pretrain=True)
else:
    predictor.load(load_pretrain=False)

# finetune a little bit
predictor.encoder.train()#(num_steps=1000, iter_num=1, print_logs=True)




runner = ImitationTransformer('configs/config_origin.yaml')
runner.load()
runner.policy.test()
runner.policy.train()
runner.save()
"""





#"""

#"""

'''
for _ in range(200):
    states, goals, attention_mask, lengths = runner.encoder.get_enc_batch(1024)
    goals_target = torch.clone(goals)

    goals_preds = runner.encoder.model(
        states, attention_mask, lengths
    )
    #print(goals_preds, goals_target)

    # note: currently indexing & masking is not fully correct
    loss = torch.mean((goals_preds*0-goals_target)**2)
    print(loss)

    print('---------------')
'''