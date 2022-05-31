import tf
import rospy
from cobot_msgs.msg import *
from cobot_msgs.srv import *
from geometry_msgs.msg import PoseStamped

class PosesConverter():

    def __init__(self, listener_tf, camera_params, robot_coodinates_frame, camera_coordinates_frame):
        self.camera_params = camera_params
        self.listener_tf = listener_tf
        self.robot_coodinates_frame = robot_coodinates_frame
        self.camera_coordinates_frame = camera_coordinates_frame
        self.tabletop_height = 0.13 # height difference between the robot base and the tabletop where objects are manipulated

    def convert2Dpose(self, x, y):
	    #t = self.listener_tf.getLatestCommonTime(self.robot_coodinates_frame, self.camera_coordinates_frame)
        self.listener_tf.waitForTransform(self.robot_coodinates_frame, self.camera_coordinates_frame, rospy.Time(0), rospy.Duration(4.0))
        trans, rot = self.listener_tf.lookupTransform(self.robot_coodinates_frame, self.camera_coordinates_frame, rospy.Time(0)) 
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
        theta = 0
        quat = tf.transformations.quaternion_from_euler(0, 0, theta)
        my_point.pose.orientation.x = quat[0]
        my_point.pose.orientation.y = quat[1]
        my_point.pose.orientation.z = quat[2]
        my_point.pose.orientation.w = quat[3]
        
        pose_stamped = self.listener_tf.transformPose(self.robot_coodinates_frame, my_point)
        print("[Converter]: ({},{}) -> ({},{})".format(x, y, pose_stamped.pose.position.x, pose_stamped.pose.position.y))
        return pose_stamped.pose