def recorded_trajectory_testing(type='cartesian', dataset='insert',sample_ratio=5):
     # init ROS subscribers
    rospy.init_node('rl_runner')
    env = PandaInsertEnv()
 
    rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, env.update_franka_state, buff_size=1)
    rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, env.checking_collisions, buff_size=1)
    rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, checking_joint_velocity, buff_size=1)

    ros_running = True

    if type=='cartesian':
        file_list = sorted(glob.glob('/home/alex/Documents/franka_panda_insertion_logs/experts/raw_'+dataset+'/' + '*.npy'))
        while ros_running:    
            print('first excuting expert demonstrations to collect data...')
            for exp_ep in range(len(file_list)):
                env.reset()
                
                if 'cartesian' in file_list[exp_ep]:
                    demo_traj = np.load(file_list[exp_ep], allow_pickle=True)[::sample_ratio]
                    print(len(demo_traj))

                    cartesian_traj = []
                    for ts in range(len(demo_traj)-1):
                        print(ts)
                        target_cartesian = demo_traj[ts] 
                        #print(target_cartesian)
                        real_cartesian = env.cartesian_step(target_cartesian)
                        target_p = np.append(target_cartesian[:3], euler_from_quaternion(np.array(target_cartesian[3:]))) 
                        real_p = np.append(real_cartesian[:3], euler_from_quaternion(np.array(real_cartesian[3:])))
                        print(target_p - real_p)

                    env.wait_expert_op()
    else:
        file_list = sorted(glob.glob('/home/alex/Documents/franka_panda_insertion_logs/experts/raw_'+dataset+'/' + '*.npy'))

        while ros_running:    
            print('first excuting expert demonstrations to collect data...')
            for exp_ep in range(len(file_list)):
                env.reset()
                
                if 'cartesian' not in file_list[exp_ep]:
                    demo_traj = np.load(file_list[exp_ep], allow_pickle=True)[::sample_ratio]
                    print(len(demo_traj))

                    cartesian_traj = []
                    for ts in range(len(demo_traj)-1):
                        print(ts)
                        target_joint = demo_traj[ts] 
                        next_state, reward, done, info = env.step(target_joint, joint_control=True)
                       

                    env.wait_expert_op()
