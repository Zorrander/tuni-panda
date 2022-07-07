#!/usr/bin/env python3
import sys
sys.path.append('../')

import os
import time
import numpy as np
import signal
import subprocess



import tf_conversions
from scipy.spatial.transform import Rotation as R
from cobot_msgs.srv import *
import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose, PoseStamped
import moveit_commander
import torch

from franka_msgs.msg import FrankaState
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest

EE_POSE = np.zeros(16)
JOINTS = np.zeros(7)

def set_hard_stiffness():
    # The os.setsid() is passed in the argument preexec_fn so
    # it's run after the fork() and before  exec() to run the shell.
    cmd = 'sh /home/alex/catkin_ws/src/imitation_learning_ros/reset_stiffness_hard.sh'
    pro = subprocess.Popen(cmd, stdout=subprocess.PIPE, 
                           shell=True, preexec_fn=os.setsid) 
    time.sleep(2)
    os.killpg(os.getpgid(pro.pid), signal.SIGTERM)  # Send the signal to all the process groups



def read_trajectory(traj):
    #traj = traj[::2]
    for aid, action in enumerate(traj[::]):
        yield (aid, action)

def create_msg(translation,quat):
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

def create_joint_msg(action):
    msg = Float32MultiArray()
    for i in action:
        msg.data.append(i)
    return msg

def create_impedance_msg(translation,rotmat):
    msg = Float32MultiArray()
    for row in rotmat:
        for value in row: 
            msg.data.append(value)
    for i in translation:
        print(i)
        msg.data.append(i)
    return msg   

def record_ros_ee_pose(franka_states_msg):
    global EE_POSE
    EE_POSE = np.array(franka_states_msg.O_T_EE)


def record_ros_joints(franka_states_msg):
    global JOINTS
    JOINTS = np.array(franka_states_msg.q)
    #print(JOINTS)


def generate_end_effector_data(mode="cartesian0",  dataset_path = "../datasets/without_extra_grabber/npys/"):
    """
    mode="cartesian_sim" is to read end-effector's cartesian position and rotations: [x,y,z,row,pitch,yall]
    mode="trajectory" is to read the joint angle trajectories
    """
    #file_pths = glob.glob(dataset_path+"*.npy")
    #for i in range(len(file_pths)):  
    #moveit_commander.roscpp_initialize(sys.argv)
    #group_name = "panda_arm"

    #robot = moveit_commander.RobotCommander()
    #scene = moveit_commander.PlanningSceneInterface()
    #group = moveit_commander.MoveGroupCommander(group_name)
    global EE_POSE 

    pub = rospy.Publisher('/new_target', Float32MultiArray, queue_size=10)
    #pub = rospy.Publisher('/new_target', Pose, queue_size=10)
    #pub = rospy.Publisher('/equilibrium_pose', PoseStamped, queue_size=10)

    cartesian_action_client = rospy.ServiceProxy('take_cartesian_action', TakeCartesianAction)
    ros_ee_pose_sub = rospy.Subscriber("/franka_state_controller/franka_states", FrankaState, record_ros_ee_pose)

    req = TakeCartesianActionRequest()


    dirname = os.path.dirname(__file__)
    dataset_path = os.path.join(dirname, dataset_path)
    for filename in os.listdir(dataset_path):
        if filename.endswith(".npy") and mode in filename:    
            #print(filename)  
            #print(os.path.join(dataset_path, filename))
            # traj = np.load(dataset_path+mode+str(i).zfill(3)+'.npy')
            traj = np.load('/home/alex/catkin_ws/src/panda-insert/src/cobot_controllers/franka_insertion_prediction/data/trajectories/expert_cartesian_poses.npy', 
                allow_pickle=True)[0]

            ros_cartesian_traj = []

            #iterator = read_trajectory(traj[0][1:])
            rate = rospy.Rate(20.0)

            run = True
            while run:#not rospy.is_shutdown(): 
                for action in traj[::20]:#traj[1:][::2]:
                    #aid, action = iterator.next()
                    # if need, convert rotation matrix into quaternion using R.from_rotvec(action[-3:]).as_quat()
                    """
                    r = R.from_quat([action[3], action[4], action[5], action[6]])
                    rotmat = r.as_matrix()
                    msg = create_impedance_msg(action[0:3],rotmat)
                    pub.publish(msg)
                    """
                    """
                    # ros c++ impedance control
                    msg = create_msg(action)
                    pub.publish(msg)
                    """
                    #"""
                    # moveit position control
                    
                   
                    req.pose = action
                    cartesian_action_client(req)

                    cart_pos = EE_POSE

                    ros_cartesian_traj.append(cart_pos)
                    #"""
                    time.sleep(1)
                    #rate.sleep()
                    
                    #print("ACTION: ", action)
                    #print("ACTUAL: ", group.get_current_pose())
                #prev_action = action
                ros_cartesian_traj = np.array(ros_cartesian_traj)
                np.save('/home/alex/catkin_ws/src/panda-insert/src/cobot_controllers/franka_insertion_prediction/data/trajectories/ros_cartesian_traj.npy',
                        ros_cartesian_traj)
                run = False
                

def verify_forward_kinematics_conversion(mode="cartesian0",  dataset_path = "../datasets/without_extra_grabber/npys/"):
    global EE_POSE 
    global JOINTS
    set_hard_stiffness()

    from differentiable_robot_model.robot_model import DifferentiableRobotModel
    try:
        sys.path.append('/home/alex/catkin_ws/src/imitation_learning_ros/src/imitation_learning_ros_v2/franka-pybullet/src/')
        from panda import Panda
    except:
        sys.path.append('franka-pybullet/src/')
        from panda import Panda
        print('cannot load panda libs')


    urdf_path = '/home/alex/catkin_ws/src/imitation_learning_ros/src/imitation_learning_ros_v2/franka-pybullet/models/panda/panda_gripper_differentiable.urdf'#panda_differentiable.urdf'
    #os.path.join(diff_robot_data.__path__[0], "kuka_iiwa/urdf/iiwa7.urdf")
    d = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    panda_bot = DifferentiableRobotModel(urdf_path, device=d)


    pub = rospy.Publisher('/equilibrium_pose', PoseStamped, queue_size=10)

    ros_ee_pose_sub = rospy.Subscriber("/franka_state_controller/franka_states", FrankaState, record_ros_ee_pose)
    ros_joints_sub = rospy.Subscriber("/franka_state_controller/franka_states", FrankaState, record_ros_joints)

    req = TakeCartesianActionRequest()


    #print(filename)  
    #print(os.path.join(dataset_path, filename))
    # traj = np.load(dataset_path+mode+str(i).zfill(3)+'.npy')
    traj = np.load('/home/alex/catkin_ws/src/imitation_learning_ros/src/imitation_learning_ros_v2/data/insert_records/explore/expert_explore_joints_poses.npy', 
        allow_pickle=True)[2][::5][:10]
    traj = np.load('/home/alex/catkin_ws/src/imitation_learning_ros/src/imitation_learning_ros_v2/data/explore/explore.npy', 
                                        allow_pickle=True)[1][::3][:10]

    rate = rospy.Rate(20.0)

    run = True
    while run:#not rospy.is_shutdown(): 
        for aid, action in enumerate(traj):#traj[1:][::2]:
            #print(JOINTS)
            #j = np.array([[0.85089134,  0.87883825, -0.86497734, -2.14459852,  0.7195039,   2.77910913, 2.31605172,]])
            j = np.array([action])
            joints = torch.from_numpy(j).to(device='cpu', dtype=torch.float32)
            ee_poses, ee_quats = panda_bot.compute_forward_kinematics(joints, link_name='panda_virtual_ee_link')
            ee_poses, ee_quats = ee_poses.detach().numpy()[0], ee_quats.detach().numpy()[0]

            msg = create_msg(ee_poses, ee_quats)
            pub.publish(msg)

            time.sleep(1)

            r = R.from_matrix([[EE_POSE[0], EE_POSE[4], EE_POSE[8]],
                               [EE_POSE[1], EE_POSE[5], EE_POSE[9]],
                               [EE_POSE[2], EE_POSE[6], EE_POSE[10]]])
            quat = r.as_quat()
            pose = [EE_POSE[12], EE_POSE[13], EE_POSE[14]]
            #print('ideal:', ee_poses, ee_quats)
            #print('real:', pose, quat)
            #print('diffs:', pose - ee_poses, quat - ee_quats)
            #print('\n\n')
            print(aid)
                          
                


def verify_end_effector_data(mode="cartesian0",  dataset_path = "../datasets/without_extra_grabber/npys/"):
    """
    mode="cartesian_sim" is to read end-effector's cartesian position and rotations: [x,y,z,row,pitch,yall]
    mode="trajectory" is to read the joint angle trajectories
    """
    #file_pths = glob.glob(dataset_path+"*.npy")
    #for i in range(len(file_pths)):  
    #moveit_commander.roscpp_initialize(sys.argv)
    #group_name = "panda_arm"

    #robot = moveit_commander.RobotCommander()
    #scene = moveit_commander.PlanningSceneInterface()
    #group = moveit_commander.MoveGroupCommander(group_name)
    global EE_POSE 
    global JOINTS

    #pub = rospy.Publisher('/new_target', Float32MultiArray, queue_size=10)
    #pub = rospy.Publisher('/new_target', Pose, queue_size=10)
    pub = rospy.Publisher('/equilibrium_pose', PoseStamped, queue_size=10)

    cartesian_action_client = rospy.ServiceProxy('take_cartesian_action', TakeCartesianAction)
    ros_ee_pose_sub = rospy.Subscriber("/franka_state_controller/franka_states", FrankaState, record_ros_ee_pose)
    ros_joints_sub = rospy.Subscriber("/franka_state_controller/franka_states", FrankaState, record_ros_joints)

    req = TakeCartesianActionRequest()


    dirname = os.path.dirname(__file__)
    # dataset_path = os.path.join(dirname, dataset_path)
    dataset_path = "/home/alex/datasets/without_extra_grabber/npys/"
    for filename in os.listdir(dataset_path):
        if filename.endswith(".npy") and mode in filename:    
            #print(filename)  
            #print(os.path.join(dataset_path, filename))
            # traj = np.load(dataset_path+mode+str(i).zfill(3)+'.npy')
            traj = np.load('/home/alex/catkin_ws/src/imitation_learning_ros/src/franka_insertion_prediction/data/trajectories/ros_cartesian_traj.npy', 
                allow_pickle=True)


            #iterator = read_trajectory(traj[0][1:])
            rate = rospy.Rate(20.0)

            run = True

            while run:#not rospy.is_shutdown(): 
                explore_joints = []
                for aid, action in enumerate(traj):#traj[1:][::2]:
                    #aid, action = iterator.next()
                    # if need, convert rotation matrix into quaternion using R.from_rotvec(action[-3:]).as_quat()
                    """
                    r = R.from_quat([action[3], action[4], action[5], action[6]])
                    rotmat = r.as_matrix()
                    msg = create_impedance_msg(action[0:3],rotmat)
                    pub.publish(msg)
                    """
                    r = R.from_matrix([[action[0], action[4], action[8]],
                                       [action[1], action[5], action[9]],
                                       [action[2], action[6], action[10]]])
                    quat = r.as_quat()
                    print('python target:',aid, quat)

                    #"""
                    # ros c++ impedance control
                    msg = create_msg([action[12], action[13], action[14]], quat)
                    pub.publish(msg)
                    #"""
                    
                    #"""
                    time.sleep(1)
                    explore_joints.append(JOINTS)
                    
                    #run = False

                explore_joints = np.array(explore_joints)
                with open('/home/alex/catkin_ws/src/imitation_learning_ros/src/explore_joints.npy', 'wb') as f:
                    np.save(f, explore_joints)
                
                                

def read_joint_data(mode="trajectory",  dataset_path = "../datasets/without_extra_grabber/npys/"):
    """
    mode="cartesian_sim" is to read end-effector's cartesian position and rotations: [x,y,z,row,pitch,yall]
    mode="trajectory" is to read the joint angle trajectories
    """
    #file_pths = glob.glob(dataset_path+"*.npy")
    #for i in range(len(file_pths)):  

    pub = rospy.Publisher('/w2_controller/joint_target', Float32MultiArray, queue_size=10)

    dirname = os.path.dirname(__file__)
    dataset_path = os.path.join(dirname, dataset_path)
    for filename in os.listdir(dataset_path):
        if filename.endswith(".npy") and mode in filename:    
            #print(filename)  
            #print(os.path.join(dataset_path, filename))
            # traj = np.load(dataset_path+mode+str(i).zfill(3)+'.npy')
            traj = np.load(os.path.join(dataset_path, filename))
            iterator = read_trajectory(traj)
            rate = rospy.Rate(20.0)
            while not rospy.is_shutdown(): 
                aid, action = iterator.next()
                msg = create_joint_msg(action)
                #print(msg)
                pub.publish(msg)
                rate.sleep()



def read_joint_data_moveit(mode="trajectory",  dataset_path = "../datasets/without_extra_grabber/npys/"):
    """
    mode="cartesian_sim" is to read end-effector's cartesian position and rotations: [x,y,z,row,pitch,yall]
    mode="trajectory" is to read the joint angle trajectories
    """
    #file_pths = glob.glob(dataset_path+"*.npy")
    #for i in range(len(file_pths)):  
    moveit_commander.roscpp_initialize(sys.argv)
    group_name = "panda_arm"

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander(group_name)

    dirname = os.path.dirname(__file__)
    dataset_path = os.path.join(dirname, dataset_path)
    for filename in os.listdir(dataset_path):
        if filename.endswith(".npy") and mode in filename:    
            #print(filename)  
            #print(os.path.join(dataset_path, filename))
            # traj = np.load(dataset_path+mode+str(i).zfill(3)+'.npy')
            traj = np.load(os.path.join(dataset_path, filename))
            iterator = read_trajectory(traj)
            rate = rospy.Rate(20.0)
            while not rospy.is_shutdown(): 
                aid, action = iterator.next()
                
                arm_joints = group.get_current_joint_values()
                arm_joints[0] = action[0] 
                arm_joints[1] = action[1] 
                arm_joints[2] = action[2] 
                arm_joints[3] = action[3] 
                arm_joints[4] = action[4] 
                arm_joints[5] = action[5] 
                arm_joints[6] = action[6] 
                
                group.set_max_velocity_scaling_factor(0.5)
                group.allow_replanning(True)
                group.go(arm_joints, wait=True)
                #group.stop()
                group.clear_pose_targets()

                rate.sleep()

def test_start_position():
    """
    mode="cartesian_sim" is to read end-effector's cartesian position and rotations: [x,y,z,row,pitch,yall]
    mode="trajectory" is to read the joint angle trajectories
    """
    #file_pths = glob.glob(dataset_path+"*.npy")
    #for i in range(len(file_pths)):  
    moveit_commander.roscpp_initialize(sys.argv)
    group_name = "panda_arm"

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander(group_name)

    arm_joints = [0.38725121,  0.50633521, -0.44973267, -2.27083523, 0.0665176, 2.99140586, 2.3965908]
    #arm_joints = [0.38945949, 0.60078327, -0.46259381, -2.00665275, 0.06666632, 2.54331118, 2.3220701] 
                
    group.set_max_velocity_scaling_factor(0.5)
    group.allow_replanning(True)
    group.go(arm_joints, wait=True)
    group.stop()
    group.clear_pose_targets()




if __name__ == '__main__':
    rospy.init_node('recordings')
    # time.sleep(5)
    # controller_switcher = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)

    """
    switch_msg = SwitchControllerRequest()
    switch_msg.stop_controllers = ["insertion_controller"]    
    switch_msg.start_controllers = ["position_joint_trajectory_controller"]                    
    switch =  controller_switcher(switch_msg)
    #read_data()
    #read_joint_data()
    #read_joint_data_moveit()
    generate_end_effector_data()
    """

    #switch_msg = SwitchControllerRequest()
    #switch_msg.stop_controllers = ["position_joint_trajectory_controller"]    
    #switch_msg.start_controllers = ["impedance_insertion_controller"]                    
    #switch =  controller_switcher(switch_msg)
    #  """
    #verify_end_effector_data()
    #verify_forward_kinematics_conversion()
    test_start_position()
    rospy.spin()