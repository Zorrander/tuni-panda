import time 
import rospy
import numpy as np
import roboticstoolbox as rtb

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty, Float32MultiArray
from franka_msgs.msg import ErrorRecoveryActionGoal
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest

from cobot_msgs.srv import *


def set_hard_stiffness():
    impedance_pub = rospy.Publisher("/new_impedance_target",Float32MultiArray, queue_size=10)
    time.sleep(1) 
    msg = Float32MultiArray()
    msg.data = [95.0, 60.0]
    impedance_pub.publish(msg)

def set_zero_stiffness():
    impedance_pub = rospy.Publisher("/new_impedance_target",Float32MultiArray, queue_size=10) 
    time.sleep(1) 
    msg = Float32MultiArray()
    msg.data = [0.0, 0.0]
    impedance_pub.publish(msg)

def create_joint_msg(joint_config):
    msg = JointState()
    msg.position = joint_config
    return msg

def create_ee_msg(translation,quat):
    '''
    msg = Float32MultiArray()
    for i in action:
        msg.data.append(i)
    #print(msg)
    '''
    msg = PoseStamped()
    msg.pose.position.x = translation[0]
    msg.pose.position.y = translation[1]
    msg.pose.position.z = translation[2]
    msg.pose.orientation.x = quat[0]
    msg.pose.orientation.y = quat[1]
    msg.pose.orientation.z = quat[2]
    msg.pose.orientation.w = quat[3]
    return msg

class PandaInsertEnv(object):
    """docstring for PandaInsertEnv"""
    def __init__(self):
        # Variables for indicating listening to ROS nodes
        self.done = False

        self.THREAD_COLLISION = False
        self.COLLISION = False

        # indicating if read sensor data from thread or not
        self.RECORD = False

        # arrays for stroing the sensor data only during interacting with env
        self.SHORT_EE_EXT_SENSORS = []  #.append(external_torque) 
        self.SHORT_J_EXT_SENSORS = []  #.append(ext_joint_torque) 
        self.SHORT_EE_VELOCITY = []  #.append(ee_velocities)
        self.SHORT_EE_ACCELERATION = []  #.append(ee_accelerations)
        self.SHORT_J_VELOCITY = []  #.append(joint_velocity)
        self.SHORT_J_ACCELERATION = []  #.append(joint_accelerations)
        self.SHORT_J_POSITIONS = []

        # arrays for stroing sensor data all the time
        self.EE_EXT_SENSORS = []  #.append(external_torque) 
        self.J_EXT_SENSORS = []  #.append(ext_joint_torque) 
        self.EE_VELOCITY = []  #.append(ee_velocities)
        self.EE_ACCELERATION = []  #.append(ee_accelerations)
        self.J_VELOCITY = []  #.append(joint_velocity)
        self.J_ACCELERATION = []  #.append(joint_accelerations)

        self.episodeScore = 0
        self.episodeScoreList = []
        self.MAX_EXT_F =  50#[20.0, 20.0, 20.0, 25.0, 25.0, 25.0]
        previous_joints = None # if collision happens, then back to previous state
        self.init_ee_pose = [0.3853664 , 0.49662741, -0.44943702, -2.27425249  , 0.06632516, 2.99395411,2.39860492]
        #self.init_ee_pose = [ 0.50523354 , 0.46390782, -0.44010135 , -2.3131034  , 0.10804452  ,3.02839653 , 2.44997405]

        self.WAIT_TIME = 5 # the wait seconds after collision happended
        self.observation_space = 20
        self.action_space = 7

        # These are the official joint angle range
        self.max_joints = np.array([2.8973  ,1.7628 , 2.8973 , -0.0698 , 2.8973, 3.7525 , 2.8973]) * 0.9# in radius
        self.min_joints = np.array([-2.8973 ,-1.7628 ,  -2.8973,   -3.0718  , -2.8973 ,  -0.0175 ,  -2.8973])* 0.9

        # These are the demonstration joint angle range
        #self.max_joints = [ 0.23797947,  0.48965807, -0.46357953, -2.27631996,  0.06344496,  2.55791777, 1.92846105] 
        #self.min_joints = [ 0.38607616,  0.58593274, -0.29890173, -1.96984302,  0.06597182,  3.00036525, 2.63989776]

        self.max_actions = np.array([0.06098087, 0.06150611, 0.06818593 ,0.17891744 ,0.00088122, 0.30414376, 0.43051925])
        self.max_prior_actions = np.array([0.02006371, 0.01366568, 0.03521475, 0.03136403, 0.00257478, 0.03902129, 0.28672958])

        # the panda robot jacobian
        self.jacob0 = rtb.models.ETS.Panda().jacob0([0,0,0,0,0,0,0])

        self.controller_mode = False

        ### ROS 
        self.reset_srv = rospy.ServiceProxy('/reset_env', ResetEnv)
        self.action_srv = rospy.ServiceProxy('/take_action', TakeAction)
        self.cartesian_action_srv = rospy.ServiceProxy('/take_cartesian_action', TakeCartesianAction)
        self.stop_action_srv = rospy.ServiceProxy('/stop_actions', StopAction)
        self.read_obs_srv = rospy.ServiceProxy('/read_values', ReadValues)

        self.impedance_action_pub = rospy.Publisher("/new_target",JointState, queue_size=10) 

        self.error_pub = rospy.Publisher('/franka_control/error_recovery/goal', ErrorRecoveryActionGoal, queue_size=10)

        self.controller_switcher = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
        
        # publisher for impedance control
        self.cart_impe_pub = rospy.Publisher('/equilibrium_pose', PoseStamped, queue_size=1)
        self.joint_pub = rospy.Publisher('/new_joint_target', JointState, queue_size=1)

    def switchToMoveItControl(self):
        self.controller_mode = False
        rospy.loginfo('Switching to arm navigation control')
        switch_msg = SwitchControllerRequest()
        # switch_msg.start_controllers = ["cartesian_pose_controller"]
        # switch_msg.start_controllers = ["position_joint_trajectory_controller"]
        switch_msg.stop_controllers = ["cartesian_impedance_controller"]
        switch =  self.controller_switcher(switch_msg)
        print(switch.ok)
        return switch.ok

    def switchToImpedanceControl(self):
        self.controller_mode = True
        rospy.loginfo('Switching to arm navigation control')
        switch_msg = SwitchControllerRequest()
        switch_msg.stop_controllers = ["position_joint_trajectory_controller"]
        switch =  self.controller_switcher(switch_msg)
        print(switch.ok)
        return switch.ok

    def back_to_init_state(self):
        print('simple reset')
        msg = JointState()
        msg.position = self.init_ee_pose
        self.joint_pub.publish(msg)

    def reset(self):
        self.error_pub.publish(ErrorRecoveryActionGoal())
        print ('Press 1: for full env reset')
        print ('Press any: for simple env reset')
        x = input()
        
        if x=='1':
            resp = self.reset_srv(40)
            # ee_position = resp.ee_pose
            # ee_orientation = resp.ee_orientation
        else:
            self.back_to_init_state()

        self.EE_EXT_SENSORS = [] 
        self.J_EXT_SENSORS = []   
        self.EE_VELOCITY = []
        self.EE_ACCELERATION = []
        self.J_VELOCITY = []
        self.J_ACCELERATION = []

        self.done = False

        self.THREAD_COLLISION = False

        force_observation = np.zeros(6)

        joints = self.get_joint_angles()
        delta_joints = joints - joints

        state = [joints[0], joints[1], 
                 joints[2], joints[3], 
                 joints[4], joints[5], joints[6],
                 delta_joints[0], delta_joints[1], 
                 delta_joints[2], delta_joints[3], 
                 delta_joints[4], delta_joints[5], delta_joints[6],
                 force_observation[0], force_observation[1], 
                 force_observation[2], force_observation[3], 
                 force_observation[4], force_observation[5]]
        state = np.array(state)
        return state

    def stiffness_reset(self):
        self.error_pub.publish(ErrorRecoveryActionGoal())
        print ('Press 1: for hard stiffness reset')
        print ('Press any: for soft stiffness reset')

        x= input()
        self.error_pub.publish(ErrorRecoveryActionGoal())

        
        if x=='1':
            set_hard_stiffness()
        else:
            set_zero_stiffness()

        return 

    def get_bot_info(self):
        return self.read_obs_srv()

    def get_ee_cartesians(self):
        pose = self.read_obs_srv().ee_pose 
        cartesian = self.read_obs_srv().ee_orientation

        return np.array([pose[0], pose[1], pose[2], 
                         cartesian[0], cartesian[1], cartesian[2], cartesian[3]])

    def get_joint_angles(self):
        obs = self.get_bot_info() 
        ee_posi, ee_ori, joints = np.array(obs.ee_pose), np.array(obs.ee_orientation), np.array(obs.joints) 
        return joints

    def get_ee_velocities(self, joint_v):
        """convert joint velocities to end effector velocity"""
        return self.jacob0.dot(np.array(joint_v))

    def clamp_joint_value(self, joint_vals):
        joint_vals[0] = np.clip(joint_vals[0], a_min=self.min_joints[0],  a_max=self.max_joints[0])
        joint_vals[1] = np.clip(joint_vals[1], a_min=self.min_joints[1],  a_max=self.max_joints[1])
        joint_vals[2] = np.clip(joint_vals[2], a_min=self.min_joints[2],  a_max=self.max_joints[2])
        joint_vals[3] = np.clip(joint_vals[3], a_min=self.min_joints[3],  a_max=self.max_joints[3])
        joint_vals[4] = np.clip(joint_vals[4], a_min=self.min_joints[4],  a_max=self.max_joints[4])
        joint_vals[5] = np.clip(joint_vals[5], a_min=self.min_joints[5],  a_max=self.max_joints[5])
        joint_vals[6] = np.clip(joint_vals[6], a_min=self.min_joints[6],  a_max=self.max_joints[6])

        return joint_vals

    def cartesian_step(self, action, moveit=False, fk=False):
        if not moveit and not fk:
            # pub = rospy.Publisher('/new_target', Float32MultiArray, queue_size=10)
            # resp = self.cartesian_action_srv(action)
            msg = PoseStamped()
            #msg = Pose()
            msg.pose.position.x = action[0]
            msg.pose.position.y = action[1]
            msg.pose.position.z = action[2]

            msg.pose.orientation.x = action[3]
            msg.pose.orientation.y = action[4]
            msg.pose.orientation.z = action[5]
            msg.pose.orientation.w = action[6]
            self.cart_impe_pub.publish(msg)
            time.sleep(1)
        elif not moveit and fk:
            joint_goal_pub = rospy.Publisher('/new_joint_target', Float32MultiArray, queue_size=10)
            joint_msg = Float32MultiArray()
            #msg = Pose()
            joint_msg.data = action
            joint_goal_pub.publish(joint_msg)
            time.sleep(1)
        else:
            resp = self.cartesian_action_srv(action)
        ee_pose_quat = self.get_ee_cartesians()
        return ee_pose_quat

    def step(self, action, control_type='joint_pos', direct_control=True):
        '''
        control_type = ['cart_impedance', 'joint_pos', 'moveit']
        '''
        # use a list to record all possible useful informations
        info = []
        print("env.step()")
        # read states before action
        previous_joints = self.read_obs_srv().joints
        previous_ee_pose_quat = self.get_ee_cartesians()
        print("[env.step()] - read actions")
        # conducting action
        self.RECORD = True

        self.SHORT_EE_EXT_SENSORS = []  #.append(external_torque) 
        self.SHORT_J_EXT_SENSORS = []  #.append(ext_joint_torque) 
        self.SHORT_EE_VELOCITY = []  #.append(ee_velocities)
        self.SHORT_EE_ACCELERATION = []  #.append(ee_accelerations)
        self.SHORT_J_VELOCITY = []  #.append(joint_velocity)
        self.SHORT_J_ACCELERATION = []  #.append(joint_accelerations)
        self.SHORT_J_POSITIONS = []

        if control_type=='cart_impedance':
            ee_pose, ee_quat = action[0], action[1]
            msg = create_ee_msg(ee_pose[0], ee_quat[0])

            self.cart_impe_pub.publish(msg)
            time.sleep(1)

        elif control_type=='joint_pos':
            msg = create_joint_msg(action)

            self.joint_pub.publish(msg)
            time.sleep(1)
            print("[env.step()] - joint_pub.publish(msg)")

        else:
            if direct_control:
                target_state = self.clamp_joint_value(action)
            else:
                action = action * self.max_actions
                target_state = self.clamp_joint_value(np.array(previous_joints) + action)
            print(target_state)
            resp = self.action_srv(target_state)
            

        self.RECORD = False

        # read the states after action
        joints = self.read_obs_srv().joints
        print("[env.step()] - self.read_obs_srv().joints")
        ee_pose_quat = self.get_ee_cartesians()

        print("[env.step()] - self.get_ee_cartesians()")
        ee_cartesian_force = self.get_force_data(type='EE_cartesian')
        print("[env.step()] - self.get_force_data(type='EE_cartesian')")
        joint_ext_force = self.get_force_data(type='Joint')
        print("[env.step()] - self.get_force_data(type='Joint')")
        force_observation = ee_cartesian_force

        reward = self.get_reward()
        done = self.done

        # print(np.mean(np.array(self.EE_VELOCITY[-10:]),axis=0))

        if self.THREAD_COLLISION:
            #self.back_to_init_state()
            #time.sleep(self.WAIT_TIME)
            #done = self.THREAD_COLLISION
            self.THREAD_COLLISION = False
            self.error_pub.publish(ErrorRecoveryActionGoal())
            time.sleep(2)

       
        state = [previous_joints[0], previous_joints[1], 
                 previous_joints[2], previous_joints[3], 
                 previous_joints[4], previous_joints[5], previous_joints[6],
                 action,
                 force_observation[0], force_observation[1], 
                 force_observation[2], force_observation[3], 
                 force_observation[4], force_observation[5]]
        state = np.array(state)

        print('\033[93m',len(self.SHORT_EE_EXT_SENSORS))

        info = {'previous_joints': previous_joints, 
                'current_joints': joints, 
                'previous_ee_pose_quat': previous_ee_pose_quat, 
                'ee_pose_quat': ee_pose_quat, 
                'mean_ee_cartesian_wrench': ee_cartesian_force, 
                'mean_ext_joint_torq': joint_ext_force, 
                'ee_cartesian_wrench_sequence': self.EE_EXT_SENSORS[-10:], 
                'ext_joint_torq_sequence': self.J_EXT_SENSORS[-10:],
                'ee_cartesian_velocities': self.EE_VELOCITY[-10:],
                'ee_cartesian_accelerations': self.EE_ACCELERATION[-10:],
                'joint_velocities': self.J_VELOCITY[-10:],
                'joint_accelerations': self.J_ACCELERATION[-10:],
                'short_ee_cartesian_wrench_sequence': self.SHORT_EE_EXT_SENSORS, 
                'short_ext_joint_torq_sequence': self.SHORT_J_EXT_SENSORS,
                'short_ee_cartesian_velocities': self.SHORT_EE_VELOCITY,
                'short_ee_cartesian_accelerations': self.SHORT_EE_ACCELERATION,
                'short_joint_velocities': self.SHORT_J_VELOCITY,
                'short_joint_accelerations': self.SHORT_J_ACCELERATION,
                'short_joint_positions':self.SHORT_J_POSITIONS}

        return state, reward, done, info

    def take_cartesian_action(self, act):
        return 


    def get_force_data(self, 
                       type='EE_cartesian', # EE_cartesian or Joint
                       ):
        # Filter external forces buffer
        if type=='EE_cartesian':
            force_observation = self.EE_EXT_SENSORS[-5:]
        elif type=='Joint':
            force_observation = self.J_EXT_SENSORS[-5:]
        #self.EXTRA_SENSORS = []

        # simply do mean filtering
        force_observation = np.array(force_observation)
        force_observation = force_observation.mean(axis=0)
        #print(force_observation)
        return force_observation  


    def get_reward(self):
        return 0


    def checking_collisions(self, msg):
        mode = msg.robot_mode
        if mode == 4: # Error detected
            self.error_pub.publish(ErrorRecoveryActionGoal())
            #test = self.stop_action_srv()
            self.THREAD_COLLISION = True
            
 

    # The ROS node read function for reading sensor
    def update_franka_state(self, msg):
        external_torque = msg.O_F_ext_hat_K
        ext_joint_torque = msg.tau_ext_hat_filtered
        ee_velocities = msg.O_dP_EE_c
        ee_accelerations = msg.O_ddP_EE_c
        joint_velocity = msg.dq
        joint_accelerations = msg.ddq_d
        joint_accelerations = msg.ddq_d
        joint_positions = msg.q
        #joint_pose = 

        if self.RECORD:
            self.SHORT_EE_EXT_SENSORS.append(external_torque) 
            self.SHORT_J_EXT_SENSORS.append(ext_joint_torque) 
            self.SHORT_EE_VELOCITY.append(ee_velocities)
            self.SHORT_EE_ACCELERATION.append(ee_accelerations)
            self.SHORT_J_VELOCITY.append(joint_velocity)
            self.SHORT_J_ACCELERATION.append(joint_accelerations)
            self.SHORT_J_POSITIONS.append(joint_positions)

            self.EE_EXT_SENSORS.append(external_torque) 
            self.J_EXT_SENSORS.append(ext_joint_torque) 
            self.EE_VELOCITY.append(ee_velocities)
            self.EE_ACCELERATION.append(ee_accelerations)
            self.J_VELOCITY.append(joint_velocity)
            self.J_ACCELERATION.append(joint_accelerations)

            #print('\033[93m',len(self.SHORT_EE_EXT_SENSORS))
        return

    def checking_ext_force(self, msg):
        """
        checking if the external force is greater than RL safety threshold
        """
        external_torque = msg.O_F_ext_hat_K
        ext_f = np.absolute(external_torque)
        #print(ext_f)
        if np.any(ext_f > self.MAX_EXT_F):
            print('collision happened, stop ROS msg thread and make thread wait for 3s ')
            #print(ext_f)
            test = self.stop_action_srv()
            self.THREAD_COLLISION = True
            time.sleep(self.WAIT_TIME*0.05)
        #else:
        #    self.THREAD_COLLISION = False
        return 

    def wait_expert_op(self):
        wait_op = True
        while(wait_op):
            print ('Press 1: indicating task failed, need pose correction')
            print ('Press 2: task is successful, assign constant reward to each step')

            x= input()
            if x=='1':
                """
                print ('left horizontal joystick for left-right (x) control')
                print('left verticle joystick for forwar-backward (y) control')
                print('left bumper-trigger for up-down (z) control')
                print ('Right horizontal joystick for roll (x) control')
                print('Right verticle joystick for pitch (y) control')
                print('Right bumper-trigger for yall (z) control')
                print ('After correcting press A to correct pose\n')
                xbox = XboxController()
                while(False):
                    xbox.read()
                    mov_y = xbox.LeftJoystickY 
                    mov_x = xbox.LeftJoystickX 
                    mov_down = xbox.LeftTrigger 
                    mov_up = xbox.LeftBumper 
                    rot_r = xbox.RightJoystickY 
                    rot_p = xbox.RightJoystickX 
                    rot_y_neg = xbox.RightTrigger 
                    rot_y_pos = xbox.RightBumper 
                    A = xbox.A 

                    # then excuting the comands
                    print(mov_x, mov_y, mov_down, mov_up, rot_r, rot_p, rot_y_neg, rot_y_pos)
                    #

                    # checking if correcting is done
                    if A == 1:
                        wait_op = False
                        break


                    time.sleep(1)
                """
                # return a signal indicating the trajectory is failure
                return False

                break
            elif x=='2':
                print ('!!Task is successful, set final reward value as 10\n')
                wait_op=False

                # return a signal indicating the trajectory is successful
                return True
                break
            else:
                print ('none of the specified options were chosen')
                time.sleep(0.3)

        
        self.error_pub.publish(ErrorRecoveryActionGoal())
                
