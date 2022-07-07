
class ImitationDecoder(object):
    """docstring for ImitationDecoder"""
    def __init__(self, config):
        super(ImitationDecoder, self).__init__()

        self.config = config
        
        self.model = DecisionTransformer(
            state_dim=config['state_dim'],
            act_dim=config['act_dim'],
            goal_dim=config['goal_dim'],
            max_length=config['K'],
            max_ep_len=config['K'],
            hidden_size=config['embed_dim'],
            n_layer=config['n_layer'],
            n_head=config['n_head'],
            n_inner=4*config['embed_dim'],
            activation_function=config['activation_function'],
            n_positions=1024,
            resid_pdrop=config['dropout'],
            attn_pdrop=config['dropout'],
        )

        self.max_ep_len = config['max_ep_len']
        self.state_dim = config['state_dim']
        self.act_dim = config['act_dim']
        self.goal_dim = config['goal_dim']
        self.max_ep_len = config['max_ep_len']
        self.K = config['K']

        self.batch_size = config['batch_size']
        self.log_to_wandb = config['log_to_wandb']
        self.device = config['device']

        self.init_dec_dataset(config)
        self.update_dec_dataset_info()


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



    def init_dec_dataset(self, config):
        # load dataset
        dataset_path = config['dataset_path'] #f'data/{env_name}-{dataset}-v2.pkl'
        with open(dataset_path, 'rb') as f:
            self.dec_trajectories = pickle.load(f)


        states_traj, actions_traj, traj_lens, returns, goals = [], [], [], [], []
        for path in self.dec_trajectories:
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
        self.dec_state_mean, self.dec_state_std = np.mean(states, axis=0), np.std(states, axis=0) + 1e-6


        # used for action normalization
        actions = np.concatenate(actions_traj, axis=0)
        self.dec_action_mean, action_max = np.mean(actions, axis=0), np.max(actions, axis=0) + 1e-6
        self.dec_action_max = action_max - self.dec_action_mean

        # used for goal normalization
        goals = np.concatenate(goals, axis=0)
        self.dec_goal_mean, self.dec_goal_max = np.mean(goals, axis=0), np.max(goals, axis=0) + 1e-6

    def load_resume_dataset(self):
        states_traj, actions_traj, goals = np.load(self.config['online_rl_data_path'], allow_pickle=True)
        self.states_traj = states_traj
        self.actions_traj = actions_traj
        self.goals_traj = goals

    def update_dec_dataset_info(self):
        #self.dec_num_timesteps = 0
        #for stj in self.states_traj:
        self.dec_num_timesteps=len(self.states_traj)

        # only train on top pct_traj trajectories (for %BC experiment)
        self.dec_num_timesteps = max(int(self.dec_num_timesteps), 1)
    
    def update_dec_dataset(self, straj, atraj, gtraj):
        self.states_traj.append(straj)
        self.actions_traj.append(atraj)
        self.goals_traj.append(gtraj)
        self.update_dec_dataset_info()

    def get_dec_batch(self, batch_size):
        device = self.device
        batch_size = self.batch_size
        max_len = self.K
        batch_inds = np.random.choice(
            np.arange(len(self.dec_trajectories)),
            size=batch_size,
            replace=True,
        )

        s, a, r, d, rtg, timesteps, mask, gs = [], [], [], [], [], [], [], []
        for i in range(batch_size):
            traj_id = int(batch_inds[i])
            traj = self.states_traj[traj_id]
            si = random.randint(0, traj.shape[0] - 1)

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
                #print(gs)

            # padding and state + reward normalization
            tlen = s[-1].shape[1]
            s[-1] = np.concatenate([np.zeros((1, max_len - tlen, self.state_dim)), s[-1]], axis=1)
            s[-1] = (s[-1] - self.dec_state_mean) / self.dec_state_std
            a[-1] = np.concatenate([np.ones((1, max_len - tlen, self.act_dim)) * -10., a[-1]], axis=1)
            a[-1] = (a[-1] - self.dec_action_mean) / self.dec_action_max
            timesteps[-1] = np.concatenate([np.zeros((1, max_len - tlen)), timesteps[-1]], axis=1)
            mask.append(np.concatenate([np.zeros((1, max_len - tlen)), np.ones((1, tlen))], axis=1))

            gs[-1] =  np.concatenate([np.zeros((1, max_len - tlen, self.goal_dim)), gs[-1]], axis=1)
            gs[-1] = (gs[-1] - self.dec_goal_mean) / (self.dec_goal_max - self.dec_goal_mean)

        s = torch.from_numpy(np.concatenate(s, axis=0)).to(dtype=torch.float32, device=device)
        a = torch.from_numpy(np.concatenate(a, axis=0)).to(dtype=torch.float32, device=device)
        timesteps = torch.from_numpy(np.concatenate(timesteps, axis=0)).to(dtype=torch.long, device=device)
        mask = torch.from_numpy(np.concatenate(mask, axis=0)).to(device=device)
        gs = torch.from_numpy(np.concatenate(gs, axis=0)).to(dtype=torch.float32, device=device)

        return s, a, r, d, gs, timesteps, mask

    def predict(self, states, goals, actions):
        return 

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
        states, actions, rewards, dones, rtg, timesteps, attention_mask = self.get_dec_batch(self.batch_size)
        action_target = torch.clone(actions)

        state_preds, action_preds, reward_preds = self.model.forward(
            #states, actions, rewards, rtg, timesteps, attention_mask=attention_mask,
            states, actions, rewards, rtg[:,:-1], timesteps, attention_mask=attention_mask,
        )

        act_dim = action_preds.shape[2]
        action_preds = action_preds.reshape(-1, act_dim)[attention_mask.reshape(-1) > 0]
        action_target = action_target.reshape(-1, act_dim)[attention_mask.reshape(-1) > 0]

        loss = self.loss_fn(
            None, action_preds, None,
            None, action_target, None,
        )

        self.optimizer.zero_grad()
        loss.backward()
        torch.nn.utils.clip_grad_norm_(self.model.parameters(), .25)
        self.optimizer.step()

        #with torch.no_grad():
        #    self.diagnostics['training/action_error'] = torch.mean((action_preds-action_target)**2).detach().cpu().item()

        return loss.detach().cpu().item()
    
    
    def train(self):
        #model.load_state_dict(torch.load('weights/insert_'+config['model_type']+'transformer.pth', \
                                          #map_location=torch.device(config['device'])))

        #model.load(PATH='weights/insert_'+config['model_type']+'transformer.pth', device=config['device'] )
        for iter in range(self.config['max_iters']):
            print('iter')
            outputs = self.train_iteration(num_steps=self.config['num_steps_per_iter'], iter_num=iter+1, print_logs=True)
            self.model.save(PATH='weights/insert_'+config['model_type']+'transformer.pth')
            #model.load(PATH='weights/insert_'+config['model_type']+'transformer.pth', device=config['device'])

    def test(self):

        duration = 3000
        stepsize = 1e-3
        robot = Panda(stepsize)
        robot.setControlMode("position")
        target_return = 0       
        device = self.decoder.device 

        dec_state_mean = torch.from_numpy(self.decoder.dec_state_mean).to(device=device, dtype=torch.float32)
        dec_state_std = torch.from_numpy(self.decoder.dec_state_std).to(device=device, dtype=torch.float32)
        dec_goal_mean = torch.from_numpy(self.decoder.dec_goal_mean).to(device=device, dtype=torch.float32)
        dec_goal_max = torch.from_numpy(self.decoder.dec_goal_max).to(device=device, dtype=torch.float32)
        dec_action_max = self.decoder.dec_action_max
        dec_action_mean = self.decoder.dec_action_mean

        for tid, straj in enumerate(self.decoder.states_traj):
            jt = straj
            gs = self.decoder.goals_traj[tid]
            #actions.append(path['actions'])
            robot.step(jt[0])
            state = self.decoder.states_traj[tid][0]#[random.randint(0,53)][random.randint(1,4)]
            goal = self.decoder.goals_traj[tid][0]#[random.randint(0,53)][0]
            # we keep all the histories on the device
            # note that the latest action and reward will be "padding"
            states = torch.from_numpy(state).reshape(1, self.decoder.state_dim).to(device=device, dtype=torch.float32)
            actions = torch.zeros((0, self.decoder.act_dim), device=device, dtype=torch.float32)
            rewards = torch.zeros(0, device=device, dtype=torch.float32)

            goal = torch.from_numpy(goal).reshape(1, self.decoder.goal_dim).to(device=device, dtype=torch.float32)
            timesteps = torch.tensor(0, device=device, dtype=torch.long).reshape(1, 1)

            sim_states = []

            episode_return, episode_length = 0, 0

            for _ in range(30):
                euler_pose = robot.step(self.decoder.states_traj[random.randint(0,53)][random.randint(4,7)])
            
            for t in range(10):
                # add padding
                actions = torch.cat([actions, torch.zeros((1, self.decoder.act_dim), device=device)], dim=0)
                rewards = torch.cat([rewards, torch.zeros(1, device=device)])

                action = self.decoder.model.get_action(
                    (states.to(dtype=torch.float32) - dec_state_mean) / dec_state_std,
                    actions.to(dtype=torch.float32),
                    rewards.to(dtype=torch.float32),
                    (goal.to(dtype=torch.float32)- dec_goal_mean) / (dec_goal_max - dec_goal_mean),
                    timesteps.to(dtype=torch.long),
                )
                actions[-1] = action
                action = action.detach().cpu().numpy()
                action = action * dec_action_max + dec_action_mean

                j_pose =  action

                #print(j_pose, jt[t+1])
                ee_pose = robot.step(j_pose)
                time.sleep(0.1)

                state = j_pose
                cur_state = torch.from_numpy(state).to(device=device).reshape(1, self.decoder.state_dim)
                states = torch.cat([states, cur_state], dim=0)
                rewards[-1] = 0

                if False:#t == 4:
                    # random change goal
                    pred_goal = self.decoder.goals_traj[random.randint(0,53)][0]
                    pred_goal = torch.from_numpy(pred_goal).to(device=device)
                else:
                    pred_goal = goal[-1]
                pred_goal = pred_goal.reshape(1, self.decoder.goal_dim)
                goal = torch.cat([goal, pred_goal], dim=0)
                #torch.cat(
                    #[goal, pred_goal.reshape(1, 1)], dim=1)
                timesteps = torch.cat(
                    [timesteps,
                     torch.ones((1, 1), device=device, dtype=torch.long) * (t+1)], dim=1)

                if t==8:
                    target = pred_goal.cpu().detach().numpy()[-1]
                    real = np.array([ee_pose[0], ee_pose[1], ee_pose[3]/np.pi*180])
                    print(target-real, goal[0].cpu().detach().numpy()-real, target-goal[0].cpu().detach().numpy())
            print(ee_pose) 
            print(robot.step(jt[-1]))
