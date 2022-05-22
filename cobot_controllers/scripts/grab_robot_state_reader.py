#!/usr/bin/env python2

import roslib
import rospy
import math
import tf
from geometry_msgs.msg import *
import time
from sensor_msgs.msg import JointState
from franka_msgs.msg import FrankaState
from franka_msgs.msg import *
import numpy as np 
from scipy.spatial.transform import Rotation as R

a = None
jv = None
js = None

# Variables for indicating listening to ROS nodes
LISTEN = False
EXTRA_SENSORS = []

# The ROS node read function for reading sensor
def update_franka_state(msg):
    #print(msg)
    #print('cc',msg.cartesian_contact)
    #print('co',msg.cartesian_collision)
    #print('of',msg.O_F_ext_hat_K)
    #print('kf',msg.K_F_ext_hat_K)
    #print('tau',msg.tau_ext_hat_filtered)
    #print(msg.current_errors)
    #print(np.array(msg.K_F_ext_hat_K) + np.array(msg.O_F_ext_hat_K))
    ee_p = msg.O_T_EE
    ee_position = [ee_p[-4],ee_p[-3],ee_p[-2]]
    ee_rot_mat = np.array([[ee_p[0],ee_p[4],ee_p[8]],
                  [ee_p[1],ee_p[5],ee_p[9]],
                  [ee_p[2],ee_p[6],ee_p[10]]])
    ee_rot_vec = rotationMatrixToEulerAngles(ee_rot_mat)
    external_torque = msg.O_F_ext_hat_K
    print('of',ee_position, ee_rot_vec, external_torque)

    # Recording the sensor data based on indication from LISTEN
    if LISTEN:
        EXTRA_SENSORS.append(ee_position, ee_rot_vec) # for example
    else:
        EXTRA_SENSORS = []
    time.sleep(0.1)
    return

# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R) :

    #assert(isRotationMatrix(R))

    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])

    singular = sy < 1e-6

    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0

    return np.array([x, y, z])


def extract_ee_pose():
    pose = PoseStamped()
    #pose.header.frame_id = "panda_leftfinger"
    pose.header.frame_id = "panda_EE"
    pose.header.stamp = rospy.Time(0)
    # t = listener.getLatestCommonTime("/panda_leftfinger", "/panda_link0")
    # pose.header.stamp = t
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 0
    pose.pose.orientation.w = 1.0

    return pose


def update_ext_force(msg):
    print('wrench',msg)
    #target_pose[0] = msg.x
    #target_pose[1] = msg.y
    return

def update_franka_state(msg):
    #print(msg)
    #print('cc',msg.cartesian_contact)
    #print('co',msg.cartesian_collision)
    #print('of',msg.O_F_ext_hat_K)
    #print('kf',msg.K_F_ext_hat_K)
    #print('tau',msg.tau_ext_hat_filtered)
    #print(msg.current_errors)
    #print(np.array(msg.K_F_ext_hat_K) + np.array(msg.O_F_ext_hat_K))
    ee_p = msg.O_T_EE
    ee_position = [ee_p[-4],ee_p[-3],ee_p[-2]]
    ee_rot_mat = np.array([[ee_p[0],ee_p[4],ee_p[8]],
                  [ee_p[1],ee_p[5],ee_p[9]],
                  [ee_p[2],ee_p[6],ee_p[10]]])
    print('of',ee_position, rotationMatrixToEulerAngles(ee_rot_mat))
    time.sleep(0.1)
    return


def update_joint_velo(msg):
    #print('joint_velocity:',msg)
    #target_pose[0] = msg.x
    #target_pose[1] = msg.y
    return

def update_joint_state(msg):
    print('!!!joint_state:',msg.position[0],msg.position[1],msg.position[2],msg.position[3],msg.position[4]
        ,msg.position[5],msg.position[6])
    #target_pose[0] = msg.x
    #target_pose[1] = msg.y

def callback(data):
    print(data)
    a = data
    rospy.loginfo("I heard %s",data.data)


if __name__ == '__main__':
     # init subscriber for force
    rospy.init_node('tf_broadcaster')
    rospy.Subscriber('/franka_state_controller/F_ext', WrenchStamped, update_ext_force) 
    rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, update_franka_state)
    #rospy.Subscriber('/joint_states', JointState, update_joint_state)
    #rospy.Subscriber('/joint_velocities', Pose, update_joint_velo)
    #rospy.Subscriber('/joint_states', JointState, update_joint_state)

    # init publisher
    ee_pose_pub = rospy.Publisher('/ee_pose', Pose, queue_size=10)
    #ee_force_pub = rospy.Publisher('/ee_force', WrenchStamped, queue_size=10)
    #joint_v_pub = rospy.Publisher('/joint_velocity', Pose, queue_size=10)
    #joint_s_pub = rospy.Publisher('/joint_state', Pose, queue_size=10)

    listener = tf.TransformListener()

    rate = rospy.Rate(20.0)
    rospy.sleep(1.0)
    rate.sleep()
    while not rospy.is_shutdown():
        '''
        Transformation between the tag and the base of the robot
        '''
        #transformed_pose = listener.transformPose("panda_link0", extract_ee_pose())
        #print(transformed_pose.pose)
        #ee_pose_pub.publish(transformed_pose.pose)
        rate.sleep()


"""
cart_pose_trans_mat = np.asarray(msg.O_T_EE).reshape(4, 4, order='F')

self._cartesian_pose = {
    'position': cart_pose_trans_mat[:3, 3],
    'orientation': quaternion.from_rotation_matrix(cart_pose_trans_mat[:3, :3]),
    'ori_mat': cart_pose_trans_mat[:3,:3]}
"""


   
