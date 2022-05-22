#!/usr/bin/env python

import time
import rospy
from sensor_msgs.msg import JointState
from cobot_msgs.srv import *


def main():
	gear_pose = [0.8314000340148536, 0.17001376770738894, 0.07777910043046214, -2.6012946816457427, -0.060050653517849904, 2.7874877791669634, 0.19530744828946062]
	backplate_approach_pose = [0.6813842747504251, 0.3863561676158779, -0.15252376738556644, -1.7943590762326294, 0.06897386904405396, 2.1883793025301634, -0.24329813243779871]
	backplate_pose = [0.591584522849635, 0.5963525362851327, -0.058518272329753186, -1.8352778794695759, 0.11483129848374259, 2.4540278837150997, -0.36387774687839874]
	backplate_insertion_pose = [0.5912686642429285, 0.6188222601165857, -0.05933526095776822, -1.829300385807981, 0.11483864723973802, 2.4606658658475284, -0.3639797757251395]
	top_plate_pose = [0.25216918792471477, 0.44560220148394525, 0.3697916431690843, -2.2037668119731704, -0.21863222708966998, 2.655778267092175, -0.001763459224771294]

	pub = rospy.Publisher('/new_joint_target', JointState, queue_size=10)

	rospy.wait_for_service('grasp')
	grasping_srv = rospy.ServiceProxy('grasp', Grasp)
	rospy.wait_for_service('move_gripper')
	release_gripper = rospy.ServiceProxy("move_gripper", MoveGripper)

	time.sleep(5)
	# Pick up the gear 
	msg = JointState()
	msg.position = gear_pose
	print(msg)
	pub.publish(msg)
	time.sleep(3)

	grasping_srv(0.046, 70)
	# Bring them above the back plate
	msg = JointState()
	msg.position = backplate_approach_pose
	print(msg)
	pub.publish(msg)
	time.sleep(3)
	# insert the gears 
	msg = JointState()
	msg.position = backplate_pose
	print(msg)
	pub.publish(msg)
	time.sleep(3)

	release_gripper(20.0, 0.8)
	# Come back above the back plate 
	msg = JointState()
	msg.position = backplate_approach_pose
	print(msg)
	pub.publish(msg)
	time.sleep(3)	
	# Pick up the top plate 
	msg = JointState()
	msg.position = top_plate_pose
	print(msg)
	pub.publish(msg)
	time.sleep(3)
	# Bring it above the back plate 
	msg = JointState()
	msg.position = backplate_insertion_pose
	print(msg)
	pub.publish(msg)
	time.sleep(3)

if __name__ == '__main__':
	rospy.init_node('metrics_assembly')
	main()
	rospy.spin()