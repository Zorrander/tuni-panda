import numpy as np
import torch

import time
from transformer_il.decision_transformer.training.trainer import Trainer

from transformer_il.decision_transformer.models.transformer_encoder.encoder import TransformerEncoder



def test_encoder():
    d_model = 128
    n_heads = 8
    batch_size = 64
    max_len = 100
    d_ff = 2048
    dropout = 0.1
    n_layers = 6

    enc = TransformerEncoder(d_model, d_ff, n_heads=n_heads, n_layers=n_layers, dropout=dropout)
    x = torch.randn(batch_size, max_len, d_model)
    mask = torch.randn(batch_size, max_len).ge(0)
    out = enc(x, mask)
    assert x.size() == out.size()

class EncoderTrainer:

    def __init__(self, model, optimizer, batch_size, get_batch, loss_fn, scheduler=None, eval_fns=None):
        self.model = model
        self.optimizer = optimizer
        self.batch_size = batch_size
        self.get_batch = get_batch
        self.loss_fn = loss_fn
        self.scheduler = scheduler
        self.eval_fns = [] if eval_fns is None else eval_fns
        self.diagnostics = dict()

        self.start_time = time.time()

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

        logs['time/training'] = time.time() - train_start

        eval_start = time.time()

        self.model.eval()
        for eval_fn in self.eval_fns:
            outputs = eval_fn(self.model)
            for k, v in outputs.items():
                logs[f'evaluation/{k}'] = v

        logs['time/total'] = time.time() - self.start_time
        logs['time/evaluation'] = time.time() - eval_start
        logs['training/train_loss_mean'] = np.mean(train_losses)
        logs['training/train_loss_std'] = np.std(train_losses)

        for k in self.diagnostics:
            logs[k] = self.diagnostics[k]

        if print_logs:
            print('=' * 80)
            print(f'Iteration {iter_num}')
            for k, v in logs.items():
                print(f'{k}: {v}')

        return logs

    def train_step(self):
        states, goals, attention_mask, lengths = self.get_batch(self.batch_size)
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
