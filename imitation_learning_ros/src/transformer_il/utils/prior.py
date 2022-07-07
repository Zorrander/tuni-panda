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

from transformer_il.utils.configure import read_yaml


import glob

class Priors(object):
    """docstring for Priors"""
    def __init__(self, config_path):
        super(Priors, self).__init__()
        config = read_yaml(config_path)
        self.config = config
        self.file_paths = glob.glob(self.config['IMITATION_INFO']['expert_prior_path']+'*')
        self.goals = []
        self.trajs = []
        self.load()

    def load(self):
        self.goals = []
        self.trajs = []
        for fp in self.file_paths:
            data = np.load(fp, allow_pickle=True)
            g = data[0][0]
            t = data[1]

            self.goals.append(g)
            self.trajs.append(t)

        self.goals = np.array(self.goals)
        self.trajs = np.array(self.trajs)

    def find_priors(self, preds, horizon=7):
        targets = np.array(self.goals)
        targets[:,0] = targets[:,0]*1000
        targets[:,1] = targets[:,1]*1000
        
        closest_id = np.argmin(np.linalg.norm(targets-preds, axis=1))
        print("pred:", preds)
        print("prior:", targets[closest_id])

        sample_ratio = int(len(self.trajs[closest_id]) / horizon)
        sample_ratio = np.max([sample_ratio, 1])
        return self.trajs[closest_id][::sample_ratio]

            
def test():
    config_path = 'configs/config_origin.yaml'
    prior = Priors(config)
    g = np.array([601,-43,128])
    prior.find_priors(g)
