import tf
import time
import math
import rospy
import cv2
import numpy as np
from cobot_msgs.msg import *
from cobot_msgs.srv import *
from geometry_msgs.msg import Pose, PoseStamped


class PosesConverter():

    def __init__(self, camera_params, robot_coodinates_frame, ee_coordinates_frame, camera_coordinates_frame):
        self.camera_params = camera_params
        self.listener_tf = tf.TransformListener()
        self.robot_coodinates_frame = robot_coodinates_frame
        self.camera_coordinates_frame = camera_coordinates_frame
        self.ee_coordinates_frame = ee_coordinates_frame
        # self.tabletop_height = 0.135 # height difference between the robot base and the tabletop where objects are manipulated
        #self.tabletop_height = 0.196 # panda_link8
        self.tabletop_height = 0.093 # panda_EE
        self.timer = rospy.Timer(rospy.Duration(5), self.look_for_robot)
        self.robot_connected = False
        
    def is_robot_connected(self):
        return self.robot_connected
        
    def look_for_robot(self, timer):
        try:
            self.listener_tf.waitForTransform(self.robot_coodinates_frame, self.camera_coordinates_frame, rospy.Time(0), rospy.Duration(5.0))
            trans, rot = self.listener_tf.lookupTransform(self.robot_coodinates_frame, self.camera_coordinates_frame, rospy.Time(0))
        except Exception as e:
            self.robot_connected = False
        else:
            self.robot_connected = True

    def convert2Dpose(self, x, y):

        pntsCam = np.array([x,y], dtype=np.float32)
        matrix = np.array([[ 1.43587169e-05, -4.08321462e-04,  7.18769465e-01],
                             [-3.92702957e-04, -1.07446034e-05,  4.15051377e-01],
                             [ 9.15894192e-06, -3.04989368e-05,  1.00000000e+00]]
                        )
        dst = cv2.perspectiveTransform(pntsCam.reshape(-1,1,2), matrix)
        dst = dst.reshape((-1,2))
        
        my_point = Pose()
        my_point.position.x = dst[0][0]
        my_point.position.y = dst[0][1]
        my_point.position.z = 0

        return my_point


    def get_orientation(self, cnt):
        homMatrix = np.array([[ 1.43587169e-05, -4.08321462e-04,  7.18769465e-01],
                             [-3.92702957e-04, -1.07446034e-05,  4.15051377e-01],
                             [ 9.15894192e-06, -3.04989368e-05,  1.00000000e+00]]
                        )
        xVector = np.array([1.0,0])
        sz = len(cnt)
        data_pts = np.empty((sz, 2), dtype=np.float64)
        for i in range(data_pts.shape[0]):
            data_pts[i,0] = cnt[i,0,0]
            data_pts[i,1] = cnt[i,0,1]
        # Perform PCA analysis
        mean = np.empty((0))
        mean, eigenvectors, eigenvalues = cv2.PCACompute2(data_pts, mean)
        #print(eigenvectors)
        #print(eigenvalues)
        mainAxis = eigenvectors[0]
        pts_im = np.array([[0.,0],[mainAxis[0],mainAxis[1]]]).reshape(-1,1,2)
        pts_base = cv2.perspectiveTransform(pts_im, homMatrix)
        pts_base.reshape(-1,2)
        mainAxis = pts_base[0]-pts_base[1]
        mainAxis = mainAxis/np.linalg.norm(mainAxis)

        angle = np.arccos(np.clip(np.dot(mainAxis, xVector), -1.0, 1.0))
        if angle > np.pi/2:
            angle = angle-np.pi
        return angle


    def convert2DposeAngle(self, x, y, angle):
        #t = self.listener_tf.getLatestCommonTime(self.robot_coodinates_frame, self.camera_coordinates_frame)
        self.listener_tf.waitForTransform(self.robot_coodinates_frame, self.camera_coordinates_frame, rospy.Time(0), rospy.Duration(5.0))
        trans, rot = self.listener_tf.lookupTransform(self.robot_coodinates_frame, self.camera_coordinates_frame, rospy.Time(0)) 

        self.listener_tf.waitForTransform(self.camera_coordinates_frame, self.ee_coordinates_frame, rospy.Time(0), rospy.Duration(5.0))
        ps = self.listener_tf.lookupTransform(self.camera_coordinates_frame, self.ee_coordinates_frame, rospy.Time(0))
        # z_to_surface = trans1[2] - 0.13 # This was OKAY - centered but only along one dimension.
        z_to_surface = trans[2] - self.tabletop_height
        to_world_scale = z_to_surface / self.camera_params.camera_focal
        x_dist = x * to_world_scale
        y_dist = y * to_world_scale

        my_point = PoseStamped()
        my_point.header.frame_id = self.camera_coordinates_frame
        my_point.header.stamp = rospy.Time(0)
        my_point.pose.position.x = 0
        my_point.pose.position.y = -x_dist
        my_point.pose.position.z = y_dist
        '''
        theta = 0
        quat = tf.transformations.quaternion_from_euler(0, 0, theta)
        my_point.pose.orientation.x = quat[0]
        my_point.pose.orientation.y = quat[1]
        my_point.pose.orientation.z = quat[2]
        my_point.pose.orientation.w = quat[3]
        '''
        theta = angle / 180 * math.pi
        quat_rotcmd = tf.transformations.quaternion_from_euler(theta, 0, 0)
        quat = tf.transformations.quaternion_multiply(quat_rotcmd, ps[1])
        my_point.pose.orientation.x = quat[0]
        my_point.pose.orientation.y = quat[1]
        my_point.pose.orientation.z = quat[2]
        my_point.pose.orientation.w = quat[3]

        pose_stamped = self.listener_tf.transformPose(self.robot_coodinates_frame, my_point)
        print("[Converter]: ({},{}) -> ({},{})".format(x, y, pose_stamped.pose.position.x, pose_stamped.pose.position.y))
        return pose_stamped.pose
