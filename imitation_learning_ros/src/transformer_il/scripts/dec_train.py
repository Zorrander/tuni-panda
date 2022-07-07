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
from decision_transformer.training.seq_trainer import SequenceTrainer

import torch_optimizer as optim

def discount_cumsum(x, gamma):
    discount_cumsum = np.zeros_like(x)
    discount_cumsum[-1] = x[-1]
    for t in reversed(range(x.shape[0]-1)):
        discount_cumsum[t] = x[t] + gamma * discount_cumsum[t+1]
    return discount_cumsum


def train_dec(
        variant,
):
    device = variant['device']
    log_to_wandb = variant['log_to_wandb']

    # load dataset
    dataset_path = variant['dataset_path'] #f'data/{env_name}-{dataset}-v2.pkl'
    with open(dataset_path, 'rb') as f:
        trajectories = pickle.load(f)
        state_dim = trajectories[0]['observations'][0].shape[0]
        act_dim = trajectories[0]['actions'][0].shape[0]
        goal_dim = trajectories[0]['goals'][0].shape[0]

    max_ep_len = variant['K']
    
    #if model_type == 'bc':
    #    env_targets = env_targets[:1]  # since BC ignores target, no need for different evaluations


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

    num_timesteps = sum(traj_lens)

    print('=' * 50)
    print(f'Starting new experiment:')
    print(f'{len(traj_lens)} trajectories, {num_timesteps} timesteps found')
    #print(f'Average return: {np.mean(returns):.2f}, std: {np.std(returns):.2f}')
    #print(f'Max return: {np.max(returns):.2f}, min: {np.min(returns):.2f}')
    print('=' * 50)

    K = max_ep_len
    batch_size = variant['batch_size']
    num_eval_episodes = variant['num_eval_episodes']
    pct_traj = variant.get('pct_traj', 1.)

    # only train on top pct_traj trajectories (for %BC experiment)
    print(num_timesteps, pct_traj)
    num_timesteps = max(int(pct_traj*num_timesteps), 1)
    num_trajectories = len(trajectories)

    def get_batch(batch_size=256, max_len=K):
        batch_inds = np.random.choice(
            np.arange(num_trajectories),
            size=batch_size,
            replace=True,
        )

        s, a, r, d, rtg, timesteps, mask, gs = [], [], [], [], [], [], [], []
        for i in range(batch_size):
            traj = trajectories[int(batch_inds[i])]
            si = random.randint(0, traj['goals'].shape[0] - 1)

            # get sequences from dataset
            s.append(traj['observations'][si:si + max_len].reshape(1, -1, state_dim))
            a.append(traj['actions'][si:si + max_len].reshape(1, -1, act_dim))
            r.append(traj['rewards'][si:si + max_len].reshape(1, -1, 1))
            if 'terminals' in traj:
                d.append(traj['terminals'][si:si + max_len].reshape(1, -1))
            else:
                d.append(traj['dones'][si:si + max_len].reshape(1, -1))
            timesteps.append(np.arange(si, si + s[-1].shape[1]).reshape(1, -1))
            timesteps[-1][timesteps[-1] >= max_ep_len] = max_ep_len-1  # padding cutoff
            #rtg.append(discount_cumsum(traj['rewards'][si:], gamma=1.)[:s[-1].shape[1] + 1].reshape(1, -1, 1))
            goal = traj['goals'][0]
            goal = np.broadcast_to(goal,(s[-1].shape[0],s[-1].shape[1],goal_dim))

            gs.append(goal)

            #if rtg[-1].shape[1] <= s[-1].shape[1]:
            #    rtg[-1] = np.concatenate([rtg[-1], np.zeros((1, 1, 1))], axis=1)

            if gs[-1].shape[1] <= s[-1].shape[1]:
                gs[-1] = np.concatenate([gs[-1], np.zeros((1, 1, goal_dim))], axis=1)
                #print(gs)

            # padding and state + reward normalization
            tlen = s[-1].shape[1]
            s[-1] = np.concatenate([np.zeros((1, max_len - tlen, state_dim)), s[-1]], axis=1)
            s[-1] = (s[-1] - state_mean) / state_std
            a[-1] = np.concatenate([np.ones((1, max_len - tlen, act_dim)) * -10., a[-1]], axis=1)
            a[-1] = (a[-1] - action_mean) / action_max
            r[-1] = np.concatenate([np.zeros((1, max_len - tlen, 1)), r[-1]], axis=1)
            d[-1] = np.concatenate([np.ones((1, max_len - tlen)) * 2, d[-1]], axis=1)
            #rtg[-1] = np.concatenate([np.zeros((1, max_len - tlen, 1)), rtg[-1]], axis=1) / scale
            timesteps[-1] = np.concatenate([np.zeros((1, max_len - tlen)), timesteps[-1]], axis=1)
            mask.append(np.concatenate([np.zeros((1, max_len - tlen)), np.ones((1, tlen))], axis=1))

            gs[-1] =  np.concatenate([np.zeros((1, max_len - tlen, goal_dim)), gs[-1]], axis=1)
            gs[-1][:,:,2] = gs[-1][:,:,2]/1000
            #gs[-1] =  (gs[-1] - goal_mean) / goal_std

        s = torch.from_numpy(np.concatenate(s, axis=0)).to(dtype=torch.float32, device=device)
        a = torch.from_numpy(np.concatenate(a, axis=0)).to(dtype=torch.float32, device=device)
        r = torch.from_numpy(np.concatenate(r, axis=0)).to(dtype=torch.float32, device=device)
        d = torch.from_numpy(np.concatenate(d, axis=0)).to(dtype=torch.long, device=device)
        #rtg = torch.from_numpy(np.concatenate(rtg, axis=0)).to(dtype=torch.float32, device=device)
        timesteps = torch.from_numpy(np.concatenate(timesteps, axis=0)).to(dtype=torch.long, device=device)
        mask = torch.from_numpy(np.concatenate(mask, axis=0)).to(device=device)
        gs = torch.from_numpy(np.concatenate(gs, axis=0)).to(dtype=torch.float32, device=device)

        #print(s[0], a[0], gs[0])
        #a()

        return s, a, r, d, gs, timesteps, mask

 
    model = DecisionTransformer(
        state_dim=state_dim,
        act_dim=act_dim,
        goal_dim=goal_dim,
        max_length=K,
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

    
    trainer = SequenceTrainer(
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

    model.load_state_dict(torch.load('weights/insert_'+variant['model_type']+'transformer.pth', \
                                      map_location=torch.device(variant['device'])))

    #model.load(PATH='weights/insert_'+variant['model_type']+'transformer.pth', device=variant['device'] )
    for iter in range(variant['max_iters']):
        print('iter')
        outputs = trainer.train_iteration(num_steps=variant['num_steps_per_iter'], iter_num=iter+1, print_logs=True)
        if log_to_wandb:
            wandb.log(outputs)

        model.save(PATH='weights/insert_'+variant['model_type']+'transformer.pth')
        model.load(PATH='weights/insert_'+variant['model_type']+'transformer.pth', device=variant['device'])



if __name__ == '__main__':

    

    from configure import read_yaml

 
    #Get the password
    info = read_yaml("configs/config.yaml")['DEC_INFO']

    train_dec(variant=info)
