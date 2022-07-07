
from differentiable_robot_model.robot_model import DifferentiableRobotModel

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

from decision_transformer.evaluation.evaluate_episodes import evaluate_episode, evaluate_episode_rtg
from decision_transformer.models.decision_transformer import DecisionTransformer
from decision_transformer.models.mlp_bc import ConditionMLPBCModel,MLPDynamics
from decision_transformer.training.act_trainer import ActTrainer
from decision_transformer.training.seq_trainer import SequenceTrainer
from decision_transformer.training.enc_trainer import EncoderTrainer
from decision_transformer.models.transformer_encoder.encoder import TransformerEncoder

from configure import read_yaml

try:
    sys.path.append('/home/alex/catkin_ws/src/imitation_learning_ros/src/imitation_learning_ros_v2/franka-pybullet/src/')
    from panda import Panda
except:
    sys.path.append('franka-pybullet/src/')
    from panda import Panda
    print('cannot load panda libs')


def minmax_normalize(x, x_mean, x_max):
    return (x-x_mean) / (x_max - x_mean)

def std_normalize(x, x_mean, x_std):
    return (x-x_mean) / x_std


def reverse_max_normalize(x, x_mean, x_max):
    return x * (x_max - x_mean) + x_mean

def reverse_std_normalize(x, x_mean, x_std):
    return x * x_std + x_mean


urdf_path = 'franka-pybullet/models/panda/panda_gripper_differentiable.urdf'#panda_differentiable.urdf'
#os.path.join(diff_robot_data.__path__[0], "kuka_iiwa/urdf/iiwa7.urdf")
d = torch.device("cuda" if torch.cuda.is_available() else "cpu")
robot = DifferentiableRobotModel(urdf_path, device=d)

        

class FrankaKinematicsNN():
    def __init__(self, state_dim=7, pred_dim=7):

        config_path="configs/config_origin.yaml"
        config = read_yaml(config_path)['DYN_INFO']
        self.config = config

        self.model = MLPDynamics(
            state_dim=config['state_dim'],
            act_dim=config['act_dim'],
            hidden_size=256,#config['embed_dim'],
            n_layer=config['n_layer'],
        )

        self.batch_size = config['batch_size']*10

        self.device = config['device']

        self.init_mlp_dataset(config)

        self.update_mlp_dataset_info()

        self.model = self.model.to(device=config['device'])

        warmup_steps = config['warmup_steps']

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
        
        self.origin_xs = np.concatenate(self.states_traj, axis=0)

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

        # calculate for dynamic nomrlization
        joints = torch.from_numpy(self.origin_xs).to(dtype=torch.float32, device=self.device)
        print('generating grond truths')
        ee_poses, ee_quats = robot.compute_forward_kinematics(joints, link_name='panda_virtual_ee_link')
        ee_targets = torch.hstack((ee_poses, ee_quats)).detach().cpu().numpy()
        print(ee_targets)
        self.mlp_target_mean, self.mlp_target_max = np.mean(ee_targets, axis=0), np.max(ee_targets, axis=0) + 1e-6
        

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



    def get_mlp_batch(self, batch_size=64):
        device = self.device
        batch_size = 256#batch_size
        #print(batch_size)
        si = random.randint(0, self.origin_xs.shape[0]-int(batch_size/2))
        # random sample N joint setups
        xs1 = self.origin_xs[si:si+int(batch_size/2)] 

        # random generate joint values
        xs2 = (np.random.rand(int(batch_size/2),7) - 0.5)*2
        xs2 = reverse_max_normalize(xs2, self.mlp_action_mean, self.mlp_action_max) 

        xs = np.concatenate((xs1, xs2), axis=0)
        xs = torch.from_numpy(xs).to(dtype=torch.float32, device=self.device)
        

        # use joint and forward kinematics to calculate the ee poses
        ee_poses, ee_quats = robot.compute_forward_kinematics(xs, link_name='panda_virtual_ee_link')
        ys = torch.hstack((ee_poses, ee_quats)).detach().cpu().numpy()

        # normalizing the ee poses
        ys = minmax_normalize(ys, self.mlp_target_mean, self.mlp_target_max) 
        #ys[:,1] = ys[:,1]*100

        ys = torch.from_numpy(ys).to(dtype=torch.float32, device=self.device)

        #  normalizing the joint values
        xs = xs.detach().cpu().numpy()
        xs = minmax_normalize(xs, self.mlp_action_mean, self.mlp_action_max) 
        xs = torch.from_numpy(xs).to(dtype=torch.float32, device=self.device)

        #print(ys[0])
        return xs, ys

    def train(self):
        print('traing policy...')
        for iter in range(self.config['max_iters']):
            print('iter')
            self.train_iteration(num_steps=self.config['num_steps_per_iter'], iter_num=iter+1, print_logs=True)
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
                self.save()

        #logs['time/mlp_training'] = time.time() - train_start

        #logs['training/mlp_train_loss_mean'] = np.mean(train_losses)

        return 

    def train_step(self):
        xs, ys = self.get_mlp_batch(self.batch_size)
        #states_2nd, actions_2nd, goals_2nd = self.get_mlp_batch(self.batch_size)
        loss = 0

        preds = self.model.forward(xs)
        loss+=torch.mean((preds-ys)**2)

        self.optimizer.zero_grad()
        loss.backward()
        #torch.nn.utils.clip_grad_norm_(self.model.parameters(), .25)
        self.optimizer.step()

        return loss.detach().cpu().item()

    def save(self):
        self.model.save(PATH=self.config['weight_path'])

    def load(self):
        self.model.load(PATH=self.config['weight_path'], device=self.config['device'])

class ImitationMLP(object):
    """docstring for ImitationMLP"""
    def __init__(self, config, use_dynamics=False):
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

        if use_dynamics:
            self.dyn_model = FrankaKinematicsNN()
            self.dyn_model.load()

        """
        optimizer = torch.optim.AdamW(
            model.parameters(),
            lr=config['learning_rate'],
            weight_decay=config['weight_decay'],
        )
        """
        self.optimizer = optim.Yogi(
            self.model.parameters(),
            lr= 1e-4,#config['learning_rate'],
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
       
        for step in range(states.shape[1]):
            if step == 0:
                action_preds = self.model.forward(states[:,step,:], goals[:,step,:])
            else:
                action_preds = self.model.forward(state_preds, goals[:,step,:])

            action_gt = actions[:,step,:]

            ee_preds = self.dyn_model.model(action_preds)
            ee_gt = self.dyn_model.model(action_gt)
            loss+=torch.mean((ee_preds-ee_gt)**2)
            loss+=torch.mean((action_preds-action_gt)**2)

            state_preds = action_preds
      

        self.optimizer.zero_grad()
        loss.backward()
        #torch.nn.utils.clip_grad_norm_(self.model.parameters(), .25)
        self.optimizer.step()

        return loss.detach().cpu().item()


    def predict(self, state, goal):
        # state shape (1,7), goal shape (1, 3)
        state = minmax_normalize(state, self.mlp_state_mean, self.mlp_state_max)
        goal = minmax_normalize(goal, self.mlp_goal_mean, self.mlp_goal_max)
        #print(state, goal)

        state = torch.from_numpy(state).to(device=self.device, dtype=torch.float32)
        goal = torch.from_numpy(goal).to(device=self.device, dtype=torch.float32)

        action = self.model(state, goal)
        #print(action)
        action = action.cpu().detach().numpy()
        action = action * (self.mlp_action_max-self.mlp_action_mean) + self.mlp_action_mean

        return action[0]


    def test(self):
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

            traj_id = random.randint(0,53)
            state = torch.from_numpy(self.states_traj[traj_id][0]).reshape(1, self.state_dim).to(device=device, dtype=torch.float32)
            #state = tmp[:,0,:]
            goal = gs[:1,0,:] #torch.from_numpy(goal).reshape(1, self.goal_dim).to(device=device, dtype=torch.float32)
            #goal_random = (-2 * torch.rand(1, 3) + 1).numpy() * (self.mlp_goal_max - self.mlp_goal_mean) + self.mlp_goal_mean #self.decoder.goals_traj[tid][0]#[random.randint(0,53)][0]
            #goal_random = torch.from_numpy(goal).to(device)
            #print(goal, goal_random)
            #a()
            #goal[0,-1] = goal_random[0,-1]
            #print(goal)
            for _ in range(30):
                #print(state)
                init_j = state.clone().cpu().detach().numpy()[0]
                euler_pose = robot.step(init_j)    
                real = np.array([euler_pose[0], euler_pose[1], euler_pose[3]/np.pi*180])
                #print(goal[0].cpu().detach().numpy()-real)
            
            for t in range(states.shape[1]+24):
                action = self.model(
                    minmax_normalize(state.to(dtype=torch.float32), mlp_state_mean, mlp_state_max),
                    minmax_normalize(goal.to(dtype=torch.float32), mlp_goal_mean, mlp_goal_max),
                )
                #action = actions[:,t,:]
                action = action.detach().cpu().numpy()
                action = action * (mlp_action_max-mlp_action_mean) + mlp_action_mean
                #print(action)

                j_pose =  action[0]

                j_pose = self.predict(state.detach().numpy(), goal.detach().numpy())
                print(j_pose)
                #j_pose = tmp[:,t,:].detach().cpu().numpy()[0]
                #print(j_pose, jt[t+1])
                ee_pose = robot.step(j_pose)
                time.sleep(0.1)

                state = torch.from_numpy(j_pose).reshape(1, self.state_dim).to(device=device, dtype=torch.float32)
                state = np.array(robot.getJointStates())[0]
                state = torch.from_numpy(state).reshape(1, self.state_dim).to(device=device, dtype=torch.float32)

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


class ImitationEEMLP(object):
    """docstring for ImitationMLP"""
    def __init__(self, config, use_dynamics=False):
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

        if use_dynamics:
            self.dyn_model = FrankaKinematicsNN()
            self.dyn_model.load()

        """
        optimizer = torch.optim.AdamW(
            model.parameters(),
            lr=config['learning_rate'],
            weight_decay=config['weight_decay'],
        )
        """
        self.optimizer = optim.Yogi(
            self.model.parameters(),
            lr= 1e-4,#config['learning_rate'],
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
            if step == 0:
                ee_preds = self.model.forward(states[:,step,:], goals[:,step,:])
            else:
                ee_preds = self.model.forward(state_preds, goals[:,step,:])

            action_gt = actions[:,step,:]
            ee_gt = self.dyn_model.model(action_gt)
            loss+=torch.mean((ee_preds-ee_gt)**2)
            loss+=torch.mean((action_preds-action_gt)**2)

            state_preds = action_preds
      

        self.optimizer.zero_grad()
        loss.backward()
        #torch.nn.utils.clip_grad_norm_(self.model.parameters(), .25)
        self.optimizer.step()

        return loss.detach().cpu().item()


    def predict(self, state, goal):
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

        return action[0]


    def test(self):
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

            traj_id = random.randint(0,53)
            state = torch.from_numpy(self.states_traj[traj_id][0]).reshape(1, self.state_dim).to(device=device, dtype=torch.float32)
            #state = tmp[:,0,:]
            goal = gs[:1,0,:] #torch.from_numpy(goal).reshape(1, self.goal_dim).to(device=device, dtype=torch.float32)
            #goal_random = (-2 * torch.rand(1, 3) + 1).numpy() * (self.mlp_goal_max - self.mlp_goal_mean) + self.mlp_goal_mean #self.decoder.goals_traj[tid][0]#[random.randint(0,53)][0]
            #goal_random = torch.from_numpy(goal).to(device)
            #print(goal, goal_random)
            #a()
            #goal[0,-1] = goal_random[0,-1]
            #print(goal)
            for _ in range(30):
                #print(state)
                init_j = state.clone().cpu().detach().numpy()[0]
                euler_pose = robot.step(init_j)    
                real = np.array([euler_pose[0], euler_pose[1], euler_pose[3]/np.pi*180])
                #print(goal[0].cpu().detach().numpy()-real)
            
            for t in range(states.shape[1]+24):
                action = self.model(
                    minmax_normalize(state.to(dtype=torch.float32), mlp_state_mean, mlp_state_max),
                    minmax_normalize(goal.to(dtype=torch.float32), mlp_goal_mean, mlp_goal_max),
                )
                #action = actions[:,t,:]
                action = action.detach().cpu().numpy()
                action = action * (mlp_action_max-mlp_action_mean) + mlp_action_mean
                #print(action)

                j_pose =  action[0]
                j_pose = self.predict(state.detach().numpy(), goal.detach().numpy())
                #j_pose = tmp[:,t,:].detach().cpu().numpy()[0]
                #print(j_pose, jt[t+1])
                ee_pose = robot.step(j_pose)
                time.sleep(0.1)

                state = torch.from_numpy(j_pose).reshape(1, self.state_dim).to(device=device, dtype=torch.float32)
                state = np.array(robot.getJointStates())[0]
                state = torch.from_numpy(state).reshape(1, self.state_dim).to(device=device, dtype=torch.float32)

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



"""

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

        tensor_act_mean, tensor_act_max = torch.from_numpy(self.mlp_action_mean).to(device=self.device, dtype=torch.float32), \
                                          torch.from_numpy(self.mlp_action_max).to(device=self.device, dtype=torch.float32)

        for step in range(states.shape[1]):
            if step == 0:
                action_preds = self.model.forward(states[:,step,:], goals[:,step,:])
                ee_pos_pred, ee_ang_pred = robot.compute_forward_kinematics(reverse_max_normalize(action_preds, tensor_act_mean, tensor_act_max), "panda_link7")
            else:
                action_preds = self.model.forward(state_preds, goals[:,step,:])
                ee_pos_pred, ee_ang_pred = robot.compute_forward_kinematics(reverse_max_normalize(action_preds, tensor_act_mean, tensor_act_max), "panda_link7")

            action_preds = action_preds
            action_gt = actions[:,step,:]
            loss+=torch.mean((action_preds-action_gt)**2)

            ee_pos_gt, ee_ang_gt = robot.compute_forward_kinematics(reverse_max_normalize(action_gt, tensor_act_mean, tensor_act_max), "panda_link7")
            loss+=torch.mean((ee_pos_pred-ee_pos_gt)**2)
            loss+=torch.mean((ee_ang_pred-ee_ang_gt)**2)
            state_preds = action_preds
      

        self.optimizer.zero_grad()
        loss.backward()
        #torch.nn.utils.clip_grad_norm_(self.model.parameters(), .25)
        self.optimizer.step()

        return loss.detach().cpu().item()

"""
"""
nn_dyn = FrankaKinematicsNN()
nn_dyn.load()
nn_dyn.train()
"""
config_path="configs/config_origin.yaml"
pi_config = read_yaml(config_path)['DEC_INFO']
policy = ImitationMLP(pi_config,use_dynamics=True)
policy.model.load(PATH='weights/insert_mlp.pth')
policy.test()
policy.train()