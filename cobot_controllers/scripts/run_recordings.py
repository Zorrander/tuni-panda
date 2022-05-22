#!/usr/bin/env python3
import sys
sys.path.append('../')

import os
import time
import numpy as np
import tf_conversions
from scipy.spatial.transform import Rotation as R
from cobot_controllers.srv import *
import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose, PoseStamped
import moveit_commander

from franka_msgs.msg import FrankaState
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest

EE_POSE = np.zeros(16)

def read_trajectory(traj):
    #traj = traj[::2]
    for aid, action in enumerate(traj[::]):
        yield (aid, action)

def create_msg(action):
    msg = Float32MultiArray()
    for i in action:
        msg.data.append(i)
    #print(msg)
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

    pub = rospy.Publisher('/new_target', Float32MultiArray, queue_size=10)
    #pub = rospy.Publisher('/new_target', Pose, queue_size=10)
    #pub = rospy.Publisher('/equilibrium_pose', PoseStamped, queue_size=10)

    cartesian_action_client = rospy.ServiceProxy('take_cartesian_action', TakeCartesianAction)
    ros_ee_pose_sub = rospy.Subscriber("/franka_state_controller/franka_states", FrankaState, record_ros_ee_pose)

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
                for action in traj:#traj[1:][::2]:
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
                    print('python target:',quat)

                    #"""
                    # ros c++ impedance control
                    msg = create_msg(action)
                    pub.publish(msg)
                    #"""
                    
                    #"""
                    time.sleep(1)
                    #rate.sleep()
                    
                    #print("ACTION: ", action)
                    #print("ACTUAL: ", group.get_current_pose())
                #prev_action = action
                
                                

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


if __name__ == '__main__':
    rospy.init_node('recordings')
    time.sleep(5)
    controller_switcher = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)

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

    switch_msg = SwitchControllerRequest()
    switch_msg.stop_controllers = ["position_joint_trajectory_controller"]    
    switch_msg.start_controllers = ["impedance_insertion_controller"]                    
    switch =  controller_switcher(switch_msg)
    #  """
    verify_end_effector_data()

    rospy.spin()