def collect_data_for_online_learner():
    # create paths for storing training informations
    dt = datetime.today().strftime('%Y-%m-%d-%H:%M:%S')
    base_save_path = "/home/alex/Documents/franka_panda_insertion_logs/"
    expert_replay_save_path = base_save_path + 'replay_buffers/'+dt+'/'#expert_replay-' + 
    online_replay_save_path = base_save_path + 'replay_buffers/'+dt+'/'#online_replay-' + 
    model_save_path = base_save_path + 'models/'+dt+'/'
    reward_log_path = base_save_path + 'rewards/'+dt+'/'
    tensorboard_path = base_save_path + 'tb/'+dt+'/logs_sil_pandaInsert_run_'
    predefined_expert_replay_path = base_save_path+'expert_replay.pkl'
    expert_action_path = base_save_path+'expert_action_under_prior.npy'
    expert_cartesian_path = base_save_path+'expert_cartesian_poses.npy'
    expert_joints_path = base_save_path+'expert_joints_poses.npy'

    check_path(expert_replay_save_path)
    check_path(online_replay_save_path)
    check_path(model_save_path)
    check_path(reward_log_path)
    check_path(tensorboard_path)

    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

    # init ROS subscribers
    rospy.init_node('rl_runner')

    env = PandaInsertEnv()
 
    rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, env.update_franka_state, buff_size=1)
    #rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, env.checking_ext_force, buff_size=1)
    rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, env.checking_collisions, buff_size=1)

    ros_running = True

    horizon = 30

    # initiate the expert prior class, which contains the expert state-action-next_state in joint angle space
    prior = PriorInfo(dataset_path='/home/alex/catkin_ws/src/panda-insert/src/cobot_controllers/franka_insertion_prediction/data/')
    # load expert joint trajectory
    demos = prior.load_demonstration()
    print(len(demos), 'expert trajs are loaded...')

    #while not rospy.is_shutdown(): 
    while ros_running:    
        print('first excuting expert demonstrations to collect data...')
        expert_acts = []
        expert_states = []
        expert_next_states = []
        expert_dones = []
        expert_infos = []
        for exp_ep in range(len(demos)):
            episode_reward = 0.

            state = env.reset()
            states = []
            next_states = []
            actions = []
            dones = []
            infos = []

            demo_traj = demos[exp_ep]
            print(len(demo_traj))

            for ts in range(len(demo_traj)-1):
                curr_joints = np.array([env.read_obs_srv().joints])
                print(curr_joints)
                #curr_cartesian = env.get_ee_cartesians()
                _, _, _, prior_target_joints = prior.retrieve_closest_prior(curr_joints, dis_type='joints')

                target_joint = demo_traj[ts+1] 
                next_state, reward, done, info = env.step(target_joint, joint_control=True)

                next_joints = np.array([env.read_obs_srv().joints])

                action = (next_joints - prior_target_joints) 
                
                print('expert act:', action)
                episode_reward += reward
                states.append(state)
                next_states.append(next_state)
                actions.append(action)
                dones.append(done)
                infos.append(info)

                state = next_state
                
            expert_acts.append(actions)
            expert_states.append(states)
            expert_next_states.append(next_states)
            expert_dones.append(dones)
            expert_infos.append(infos)

            env.wait_expert_op()

        with open('/home/alex/catkin_ws/src/panda-insert/src/cobot_controllers/franka_insertion_prediction/data/expert_recordings.pkl', 'wb') as handle:
            pickle.dump((expert_acts,expert_states,expert_next_states,expert_dones,expert_infos), handle)





def expert_data_collection(save_folder='/home/alex/'):
    
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    env = PandaInsertEnv()

    # init ROS subscribers
    rospy.init_node('rl_runner')
 
    rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, env.update_franka_state, buff_size=1)
    # rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, env.checking_ext_force, buff_size=1)
    rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, env.checking_collisions, buff_size=1)
    rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, checking_joint_velocity, buff_size=1)

    ros_running = True
    
    global keep_going
    global moving


    #while not rospy.is_shutdown(): 
    while ros_running:    
        # Run outer loop until the episodes limit is reached or the task is solved
        # run expert trajs to collect good samples
        start_expert = True
        while start_expert:
            env.reset()
            joint_states = []
            cartesian_states = []
            prev_state = np.zeros(7)

            steps = 0

            th.Thread(target=key_capture_thread, args=(), name='key_capture_thread', daemon=True).start()
            #time.sleep(3)
            moving = False


            rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, checking_joint_velocity, buff_size=1)
            while keep_going:
                print(moving)
                if moving:
                    joints= np.array(env.get_bot_info().joints)
                    cartesians = np.append(env.get_bot_info().ee_pose, env.get_bot_info().ee_orientation)
                    print(joints)
                    joint_states.append(joints)
                    cartesian_states.append(cartesians)

                    prev_state = joints
                    steps += 1


                #if np.linalg.norm(joints-prev_state) < 0.001 and  steps > 2000:
                #    record=False
                #    break

            with open(save_folder+strftime("%Y-%m-%d-%H:%M:%S", gmtime())+'.npy', 'wb') as f:
                np.save(f, np.array(joint_states))


            with open(save_folder+strftime("%Y-%m-%d-%H:%M:%S", gmtime())+'-cartesian.npy', 'wb') as f:
                np.save(f, np.array(cartesian_states))

            print('saved', save_folder+strftime("%Y-%m-%d-%H:%M:%S", gmtime())+'.npy')

            start_expert = env.wait_expert_op()
            
            keep_going = start_expert




if __name__ == '__main__':
    expert_data_collection()