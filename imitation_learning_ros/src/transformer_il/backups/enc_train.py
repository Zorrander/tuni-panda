
import gym
import numpy as np
import torch
import wandb

import argparse
import pickle
import random
import sys

from decision_transformer.evaluation.evaluate_episodes import evaluate_episode, evaluate_episode_rtg
from decision_transformer.models.decision_transformer import DecisionTransformer
from decision_transformer.models.mlp_bc import MLPBCModel
from decision_transformer.training.act_trainer import ActTrainer
from decision_transformer.training.enc_trainer import EncoderTrainer

import torch_optimizer as optim

import numpy as np
import torch
from decision_transformer.training.trainer import Trainer
from decision_transformer.models.transformer_encoder.encoder import TransformerEncoder



def test_encoder():
    d_model = 6
    n_heads = 3
    batch_size = 64
    max_len = 10
    d_ff = 128
    dropout = 0.1
    n_layers = 4

    enc = TransformerEncoder(d_model, d_ff, n_heads=n_heads, n_layers=n_layers, dropout=dropout)
    x = torch.randn(batch_size, max_len, d_model)
    lengths = torch.randint(0,1,(batch_size,))
    print(lengths)
    mask = torch.randn(batch_size, max_len).ge(0)
    print(x.shape, mask.shape, mask)
    out = enc(x, mask, lengths)
    print(out.shape)
    #assert x.size() == out.size()


def eval_encoder():
    d_model = state_dim
    n_heads = 3
    batch_size = 64
    max_len = 10
    d_ff = 128
    dropout = 0.1
    n_layers = 3

    model = TransformerEncoder(d_model, d_ff, n_heads=n_heads, n_layers=n_layers, dropout=dropout)

    model = model.to(device=device)        
    model.load(PATH='weights/insert_goal_transformer.pth')


def discount_cumsum(x, gamma):
    discount_cumsum = np.zeros_like(x)
    discount_cumsum[-1] = x[-1]
    for t in reversed(range(x.shape[0]-1)):
        discount_cumsum[t] = x[t] + gamma * discount_cumsum[t+1]
    return discount_cumsum



def train_goal_predictor(
        variant,
):
    device = variant.get('device', 'cuda')
    log_to_wandb = variant.get('log_to_wandb', False)

    goal_datset = np.load('data/insert_goal.npy', allow_pickle=True)
    trajectories = goal_datset[0]
    trajectories_goals = goal_datset[1]

    state_dim = goal_datset[0][0][0].shape[0]
    goal_dim = goal_datset[1][0][0].shape[0]


    traj_lens = []
    for path in trajectories:
        traj_lens.append(len(path))

    # used for input normalization
    states = np.concatenate(goal_datset[0], axis=0)
    state_mean, state_std = np.mean(states, axis=0), np.std(states, axis=0) + 1e-6

    # used for goal normalization
    goals = np.concatenate(goal_datset[1], axis=0)
    goal_mean, goal_max = np.mean(goals, axis=0), np.max(goals, axis=0) + 1e-6
    num_timesteps = sum(traj_lens)

    print('=' * 50)
    print(f'Starting new experiment: ')
    print(f'{len(traj_lens)} trajectories, {num_timesteps} timesteps found')
    #print(f'Average return: {np.mean(returns):.2f}, std: {np.std(returns):.2f}')
    #print(f'Max return: {np.max(returns):.2f}, min: {np.min(returns):.2f}')
    print('=' * 50)

    batch_size = variant['batch_size']
    max_ep_len = 10
    K = max_ep_len

    # only train on top pct_traj trajectories (for %BC experiment)
    num_timesteps = max(int(num_timesteps), 1)
    num_trajectories = len(trajectories)
    
    def get_batch(batch_size=256, max_len=K):
        batch_inds = np.random.choice(
            np.arange(num_trajectories),
            size=batch_size,
            replace=True,
            #p=p_sample,  # reweights so we sample according to timesteps
        )

        s, gs, mask, lengths = [], [], [], []
        for i in range(batch_size):
            s_traj = trajectories[int(batch_inds[i])]
            g_traj = trajectories_goals[int(batch_inds[i])]
            si = random.randint(0, s_traj.shape[0] - 1)

            # get sequences from dataset
            s.append(s_traj[si:si + max_len].reshape(1, -1, state_dim))
            gs.append(g_traj[si])
          
            # padding and state + reward normalization
            tlen = s[-1].shape[1]
            lengths.append(tlen)

            s[-1] = np.concatenate([np.zeros((1, max_len - tlen, state_dim)), s[-1]], axis=1)
            s[-1] = (s[-1] - state_mean) / state_std
            #gs[-1] = np.concatenate([np.zeros((1, max_len - tlen, goal_dim)), gs[-1]], axis=1)
            gs[-1] = (gs[-1] - goal_mean) / (goal_max - goal_mean)

            mask.append(np.concatenate([np.zeros((1, max_len - tlen)), np.ones((1, tlen))], axis=1))


        s = torch.from_numpy(np.concatenate(s, axis=0)).to(dtype=torch.float32, device=device)
        gs = torch.from_numpy(np.array(gs)).to(dtype=torch.float32, device=device)
        #print(gs.shape)
        
        mask = torch.from_numpy(np.concatenate(mask, axis=0)).to(device=device)
        #lengths = torch.from_numpy(lengths).to(device=device)


        return s, gs, mask, lengths
 
    d_model = state_dim
    n_heads = 3
    batch_size = 64
    max_len = 10
    d_ff = 128
    dropout = 0.1
    n_layers = 3

    model = TransformerEncoder(d_model, d_ff, n_heads=n_heads, n_layers=n_layers, dropout=dropout)
   

    model = model.to(device=device)

    warmup_steps = variant['warmup_steps']

    """
    optimizer = torch.optim.AdamW(
        model.parameters(),
        lr=variant['learning_rate'],
        weight_decay=variant['weight_decay'],
    )
    """
    optimizer = optim.Yogi(
        model.parameters(),
        lr= 1e-2,#variant['learning_rate'],
        betas=(0.9, 0.999),
        eps=1e-3,
        initial_accumulator=1e-6,
        weight_decay=0#variant['weight_decay'],
    )
    
    scheduler = torch.optim.lr_scheduler.LambdaLR(
        optimizer,
        lambda steps: min((steps+1)/warmup_steps, 1)
    )

   
    trainer = EncoderTrainer(
        model=model,
        optimizer=optimizer,
        batch_size=batch_size,
        get_batch=get_batch,
        scheduler=scheduler,
        loss_fn=lambda s_hat, a_hat, r_hat, s, a, r: torch.mean((a_hat - a)**2),
        eval_fns=None#[eval_episodes(tar) for tar in env_targets],
    )
    
    if log_to_wandb:
        wandb.init(
            name=exp_prefix,
            group=group_name,
            project='decision-transformer',
            config=variant
        )
        # wandb.watch(model)  # wandb has some bug
    
    model.load(PATH='weights/insert_goal_transformer.pth')

    for iter in range(variant['max_iters']):
        print('iter')
        outputs = trainer.train_iteration(num_steps=variant['num_steps_per_iter'], iter_num=iter+1, print_logs=True)
        if log_to_wandb:
            wandb.log(outputs)

        model.save(PATH='weights/insert_goal_transformer.pth')
        model.load(PATH='weights/insert_goal_transformer.pth')



if __name__ == '__main__':
    from configure import read_yaml

 
    #Get the password
    info = read_yaml("configs/config.yaml")['ENC_INFO']


   
    train_goal_predictor(variant=info)
