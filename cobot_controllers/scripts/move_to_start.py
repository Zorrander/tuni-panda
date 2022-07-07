#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
from geometry_msgs.msg import *
#which should have geometry_msgs/WrenchStamped 
from scipy.spatial.transform import Rotation as R

start_joint_positions_array = [0.5779521567746997e-09, 0.4745329201221466, 3.005009599330333e-01, -0.5726646304130554, -0.80409618139168742e-07, 1.0217304706573486, -0.7853981256484985]
ee_force = [0, 0, 0]

def update_ext_force(msg):
    #print(msg)
    #target_pose[0] = msg.x
    #target_pose[1] = msg.y
    return

def callback(data):
   rospy.loginfo("I heard %s",data.data)

def move_to_target(moveit_commander, robot, scene, group):
    arm_joints = group.get_current_joint_values()
    arm_joints[0] = 0.3853664 
    arm_joints[1] = 0.49662741
    arm_joints[2] = -0.44943702
    arm_joints[3] = -2.27425249  
    arm_joints[4] = 0.06632516
    arm_joints[5] = 2.99395411
    arm_joints[6] = 2.39860492
    
    group.set_max_velocity_scaling_factor(0.2)
    group.allow_replanning(True)

    group.go(arm_joints, wait=True)
    #group.stop()
    group.clear_pose_targets()
    return True
   


def move_to_cartesian_target(moveit_commander, robot, scene, group):
    target_pose1 = Pose() 
    r = R.from_dcm([
            [-0.7673566 ,  0.42430914, -0.48075524],
            [-0.37661595,  0.30856349,  0.87346952],
            [ 0.51896461,  0.85132269, -0.07697665]]
        )
    p = r.as_quat()

    target_pose1.orientation.w =  p[0];
    target_pose1.orientation.x =  p[1];
    target_pose1.orientation.y =  p[2]
    target_pose1.orientation.z =  p[3];
  
    target_pose1.position.x = 0.3853664;
    target_pose1.position.y = 0.49662741;
    target_pose1.position.z = 0.44943702;
    group.set_pose_target(target_pose1);

    group.set_max_velocity_scaling_factor(0.5)
    group.allow_replanning(True)

    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    return True

if __name__ == '__main__':
    rospy.init_node('tf_broadcaster', anonymous=True)
    #rospy.Subscriber('/franka_state_controller/F_ext', WrenchStamped, update_ext_force)

    moveit_commander.roscpp_initialize(sys.argv)
    group_name = "panda_arm"

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander(group_name)

    #rospy.Subscriber('reset', Empty, move_to_target, (moveit_commander, robot, scene, group))
    #for i in range(10):
    #    print(i)
    #    print(move_to_target(start_joint_positions_array, moveit_commander, robot, scene, group))
    # move_to_cartesian_target(moveit_commander, robot, scene, group)
    move_to_target(moveit_commander, robot, scene, group)

# moveit_commander.move_group.MoveGroupCommander.set_pose_target    (
