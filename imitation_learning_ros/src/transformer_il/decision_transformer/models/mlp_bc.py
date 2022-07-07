import numpy as np
import torch
import torch.nn as nn

import torch.nn.functional as F

from transformer_il.decision_transformer.models.model import TrajectoryModel


class MLPBCModel(TrajectoryModel):

    """
    Simple MLP that predicts next action a from past states s.
    """

    def __init__(self, state_dim, act_dim, hidden_size, n_layer, dropout=0.1, max_length=1, **kwargs):
        super().__init__(state_dim, act_dim)

        self.hidden_size = hidden_size
        self.max_length = max_length

        layers = [nn.Linear(max_length*self.state_dim, hidden_size)]
        for _ in range(n_layer-1):
            layers.extend([
                nn.ReLU(),
                nn.Dropout(dropout),
                nn.Linear(hidden_size, hidden_size)
            ])
        layers.extend([
            nn.ReLU(),
            nn.Dropout(dropout),
            nn.Linear(hidden_size, self.act_dim),
            nn.Tanh(),
        ])

        self.model = nn.Sequential(*layers)

    def forward(self, states, actions, rewards, attention_mask=None, target_return=None):

        states = states[:,-self.max_length:].reshape(states.shape[0], -1)  # concat states
        actions = self.model(states).reshape(states.shape[0], 1, self.act_dim)

        return None, actions, None

    def get_action(self, states, actions, rewards, **kwargs):
        states = states.reshape(1, -1, self.state_dim)
        if states.shape[1] < self.max_length:
            states = torch.cat(
                [torch.zeros((1, self.max_length-states.shape[1], self.state_dim),
                             dtype=torch.float32, device=states.device), states], dim=1)
        states = states.to(dtype=torch.float32)
        _, actions, _ = self.forward(states, None, None, **kwargs)
        return actions[0,-1]

    def save(self, PATH):
        torch.save(self.state_dict(), PATH)

    def load(self, PATH):
        self.load_state_dict(torch.load(PATH))


class ConditionMLPBCModel(TrajectoryModel):

    """
    Simple MLP that predicts next action a from condtional states s.
    """
    def __init__(self, state_dim, act_dim, goal_dim, hidden_size, n_layer, dropout=0.1, **kwargs):
        super().__init__(state_dim, act_dim)

        self.l1 = nn.Linear(state_dim+goal_dim, hidden_size)
        self.l2 = nn.Linear(hidden_size+state_dim+goal_dim, hidden_size)
        self.l3 = nn.Linear(hidden_size+state_dim+goal_dim, hidden_size)
        self.l4 = nn.Linear(hidden_size+state_dim+goal_dim, hidden_size)
        self.l5 = nn.Linear(hidden_size, act_dim)


    def forward(self, state, goal):
        x = torch.cat((state, goal), dim=1)
        a = F.relu(self.l1(x))

        a = torch.cat([a, x], 1)
        a = F.relu(self.l2(a))

        a = torch.cat([a, x], 1)
        a = F.relu(self.l3(a))

        a = torch.cat([a, x], 1)
        a = F.relu(self.l4(a))
        
        a = torch.tanh(self.l5(a))
        return a

    def save(self, PATH):
        torch.save(self.state_dict(), PATH)

    def load(self, PATH, device='cpu'):
        self.load_state_dict(torch.load(PATH,  map_location=torch.device(device)))



class MLPDynamics(TrajectoryModel):

    """
    Simple MLP that predicts next action a from condtional states s.
    """
    def __init__(self, state_dim, act_dim, hidden_size, n_layer, dropout=0.1, **kwargs):
        super().__init__(state_dim, act_dim)

        self.l1 = nn.Linear(state_dim, hidden_size)
        self.l2 = nn.Linear(hidden_size+state_dim, hidden_size)
        self.l3 = nn.Linear(hidden_size+state_dim, hidden_size)
        self.l4 = nn.Linear(hidden_size+state_dim, hidden_size)
        self.l5 = nn.Linear(hidden_size, act_dim)


    def forward(self, state):
        x = state
        a = F.relu(self.l1(x))

        a = torch.cat([a, x], 1)
        a = F.relu(self.l2(a))

        a = torch.cat([a, x], 1)
        a = F.relu(self.l3(a))

        a = torch.cat([a, x], 1)
        a = F.relu(self.l4(a))
        
        a = self.l5(a)#torch.tanh(self.l5(a))
        return a

    def save(self, PATH):
        torch.save(self.state_dict(), PATH)

    def load(self, PATH):
        self.load_state_dict(torch.load(PATH))

    """
    def __init__(self, state_dim, act_dim, goal_dim, hidden_size, n_layer, dropout=0.1, **kwargs):
        super().__init__(state_dim, act_dim)

        self.hidden_size = 28#hidden_size

        layers = [nn.Linear(self.state_dim+goal_dim, hidden_size)]
        for _ in range(1):
            layers.extend([
                nn.ReLU(),
                nn.Dropout(dropout),
                nn.Linear(hidden_size, hidden_size)
            ])
        layers.extend([
            nn.ReLU(),
            nn.Dropout(dropout),
            nn.Linear(hidden_size, self.act_dim),
            nn.Tanh(),
        ])

        self.model = nn.Sequential(*layers)

    def forward(self, states, goal):
        x = torch.cat((states, goal), dim=1)

        actions = self.model(x)
        return actions
    """

    def save(self, PATH):
        torch.save(self.state_dict(), PATH)

    def load(self, PATH, device='cpu'):
        print(device)
        self.load_state_dict(torch.load(PATH,  map_location=torch.device(device)))


