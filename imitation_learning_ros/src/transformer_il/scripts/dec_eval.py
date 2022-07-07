import gym
import numpy as np
import torch
import wandb

import argparse
import pickle
import random
import sys
import time

from decision_transformer.evaluation.evaluate_episodes import evaluate_episode, evaluate_episode_rtg
from decision_transformer.models.decision_transformer import DecisionTransformer
from decision_transformer.models.mlp_bc import MLPBCModel
from decision_transformer.training.act_trainer import ActTrainer
from decision_transformer.training.seq_trainer import SequenceTrainer

import torch_optimizer as optim

sys.path.append('franka-pybullet/src/')
from panda import Panda
import random


def discount_cumsum(x, gamma):
    discount_cumsum = np.zeros_like(x)
    discount_cumsum[-1] = x[-1]
    for t in reversed(range(x.shape[0]-1)):
        discount_cumsum[t] = x[t] + gamma * discount_cumsum[t+1]
    return discount_cumsum


def experiment(
        exp_prefix,
        variant,
):
    device = variant.get('device', 'cuda')
    log_to_wandb = variant.get('log_to_wandb', False)

    env_name, dataset = variant['env'], variant['dataset']
    model_type = variant['model_type']
    group_name = f'{exp_prefix}-{env_name}-{dataset}'
    exp_prefix = f'{group_name}-{random.randint(int(1e5), int(1e6) - 1)}'

    # load dataset
    dataset_path = f'data/insert-expert.pkl' #f'data/{env_name}-{dataset}-v2.pkl'
    with open(dataset_path, 'rb') as f:
        trajectories = pickle.load(f)
        state_dim = trajectories[0]['observations'][0].shape[0]
        act_dim = trajectories[0]['actions'][0].shape[0]
        goal_dim = trajectories[0]['goals'][0].shape[0]

    max_ep_len = 10
    

    # save all path information into separate lists
    mode = variant.get('mode', 'normal')
    states, actions, traj_lens, returns, goals = [], [], [], [], []
    for path in trajectories:
        if mode == 'delayed':  # delayed: all rewards moved to end of trajectory
            path['rewards'][-1] = path['rewards'].sum()
            path['rewards'][:-1] = 0.
        states.append(path['observations'])
        actions.append(path['actions'])
        traj_lens.append(len(path['observations']))
        returns.append(0)
        goals.append(path['goals'])
    
    #traj_lens, returns = np.array(traj_lens), np.array(returns)
    traj_lens = np.array(traj_lens)

    # used for input normalization
    states = np.concatenate(states, axis=0)
    state_mean, state_std = np.mean(states, axis=0), np.std(states, axis=0) + 1e-6


    # used for action normalization
    actions = np.concatenate(actions, axis=0)
    action_mean, action_max = np.mean(actions, axis=0), np.max(np.abs(actions), axis=0) + 1e-6
    action_max = action_max - action_mean
    print(np.max(np.abs(actions)), action_max)
    print(np.min(np.abs(actions)))
    print(np.mean(actions))

    # used for goal normalization
    goals = np.concatenate(goals, axis=0)
    #goals[:2] = goals[:2] / 1000

    goal_mean, goal_std = np.mean(goals, axis=0), np.std(goals, axis=0) + 1e-6

 
    if model_type == 'dt':
        model = DecisionTransformer(
            state_dim=state_dim,
            act_dim=act_dim,
            goal_dim=goal_dim,
            max_length=max_ep_len,
            max_ep_len=max_ep_len,
            hidden_size=variant['embed_dim'],
            n_layer=variant['n_layer'],
            n_head=variant['n_head'],
            n_inner=4*variant['embed_dim'],
            activation_function=variant['activation_function'],
            n_positions=1024,
            resid_pdrop=variant['dropout'],
            attn_pdrop=variant['dropout'],
        )
    elif model_type == 'bc':
        model = MLPBCModel(
            state_dim=state_dim,
            act_dim=act_dim,
            max_length=max_ep_len,
            hidden_size=variant['embed_dim'],
            n_layer=variant['n_layer'],
        )
    else:
        raise NotImplementedError

    model = model.to(device=device)
    model.load(PATH='weights/insert_decoder.pth')

    duration = 3000
    stepsize = 1e-3

    robot = Panda(stepsize)
    robot.setControlMode("position")
    target_return = 0
    for path in trajectories:
        jt = path['observations']
        gs = path['goals']
        #actions.append(path['actions'])
        robot.step(jt[0])
        state = trajectories[random.randint(0,53)]['observations'][random.randint(3,4)]
        goal = gs[0]
        # we keep all the histories on the device
        # note that the latest action and reward will be "padding"
        states = torch.from_numpy(state).reshape(1, state_dim).to(device=device, dtype=torch.float32)
        actions = torch.zeros((0, act_dim), device=device, dtype=torch.float32)
        rewards = torch.zeros(0, device=device, dtype=torch.float32)

        goal = torch.from_numpy(goal).reshape(1, goal_dim).to(device=device, dtype=torch.float32)
        timesteps = torch.tensor(0, device=device, dtype=torch.long).reshape(1, 1)

        sim_states = []

        episode_return, episode_length = 0, 0

        for t, j in enumerate(jt):
            for _ in range(30):
                euler_pose = robot.step(j)

            # add padding
            actions = torch.cat([actions, torch.zeros((1, act_dim), device=device)], dim=0)
            rewards = torch.cat([rewards, torch.zeros(1, device=device)])

            action = model.get_action(
                (states.to(dtype=torch.float32) - state_mean) / state_std,
                actions.to(dtype=torch.float32),
                rewards.to(dtype=torch.float32),
                goal.to(dtype=torch.float32),
                timesteps.to(dtype=torch.long),
            )
            actions[-1] = action
            action = action.detach().cpu().numpy()
            action = action * action_max + action_mean

            j_pose = state + action

            #print(j_pose, jt[t+1])
            ee_pose = robot.step(j_pose)

            state = j_pose
            cur_state = torch.from_numpy(state).to(device=device).reshape(1, state_dim)
            states = torch.cat([states, cur_state], dim=0)
            rewards[-1] = 0

            pred_goal = goal[-1]
            pred_goal = pred_goal.reshape(1, goal_dim)
            goal = torch.cat([goal, pred_goal], dim=0)
            #torch.cat(
                #[goal, pred_goal.reshape(1, 1)], dim=1)
            timesteps = torch.cat(
                [timesteps,
                 torch.ones((1, 1), device=device, dtype=torch.long) * (t+1)], dim=1)

            if t==8:
                target = pred_goal.cpu().detach().numpy()[-1]
                real = np.array([ee_pose[0], ee_pose[1], ee_pose[3]/np.pi*180])
                print(pred_goal, target-real)
          

           



if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--env', type=str, default='hopper')
    parser.add_argument('--dataset', type=str, default='medium')  # medium, medium-replay, medium-expert, expert
    parser.add_argument('--mode', type=str, default='normal')  # normal for standard setting, delayed for sparse
    parser.add_argument('--K', type=int, default=5)
    parser.add_argument('--pct_traj', type=float, default=1.)
    parser.add_argument('--batch_size', type=int, default=64)
    parser.add_argument('--model_type', type=str, default='dt')  # dt for decision transformer, bc for behavior cloning
    parser.add_argument('--embed_dim', type=int, default=128)
    parser.add_argument('--n_layer', type=int, default=3)
    parser.add_argument('--n_head', type=int, default=1)
    parser.add_argument('--activation_function', type=str, default='relu')
    parser.add_argument('--dropout', type=float, default=0.1)
    parser.add_argument('--learning_rate', '-lr', type=float, default=1e-4)
    parser.add_argument('--weight_decay', '-wd', type=float, default=1e-4)
    parser.add_argument('--warmup_steps', type=int, default=10000)
    parser.add_argument('--num_eval_episodes', type=int, default=100)
    parser.add_argument('--max_iters', type=int, default=10)
    parser.add_argument('--num_steps_per_iter', type=int, default=10000)
    parser.add_argument('--device', type=str, default='cpu')
    parser.add_argument('--log_to_wandb', '-w', type=bool, default=False)
    
    args = parser.parse_args()

   

    experiment('gym-experiment', variant=vars(args))

'''
   def eval_episodes(target_rew):
        def fn(model):
            returns, lengths = [], []
            for _ in range(num_eval_episodes):
                with torch.no_grad():
                    if model_type == 'dt':
                        ret, length = evaluate_episode_rtg(
                            env,
                            state_dim,
                            act_dim,
                            model,
                            max_ep_len=max_ep_len,
                            scale=scale,
                            target_return=target_rew/scale,
                            mode=mode,
                            state_mean=state_mean,
                            state_std=state_std,
                            device=device,
                        )
                    else:
                        ret, length = evaluate_episode(
                            env,
                            state_dim,
                            act_dim,
                            model,
                            max_ep_len=max_ep_len,
                            target_return=target_rew/scale,
                            mode=mode,
                            state_mean=state_mean,
                            state_std=state_std,
                            device=device,
                        )
                returns.append(ret)
                lengths.append(length)
            return {
                f'target_{target_rew}_return_mean': np.mean(returns),
                f'target_{target_rew}_return_std': np.std(returns),
                f'target_{target_rew}_length_mean': np.mean(lengths),
                f'target_{target_rew}_length_std': np.std(lengths),
            }
        return fn
'''