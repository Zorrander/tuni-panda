#!/usr/bin/env python2
import sys
import rospy
import moveit_commander
from geometry_msgs.msg import *
from expert_dataset import dataset
#which should have geometry_msgs/WrenchStamped 
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState


start_joint_positions_array = [0.5779521567746997e-09, 0.4745329201221466, 3.005009599330333e-01, -0.5726646304130554, -0.80409618139168742e-07, 1.0217304706573486, -0.7853981256484985]
ee_force = [0, 0, 0]
a = 0


def update_ext_force(msg):
    print(msg)
    a = msg
    #target_pose[0] = msg.x
    #target_pose[1] = msg.y

def callback(data):
   rospy.loginfo("I heard %s",data.data)

def run_joint_settings( a, publisher):
    data_to_send.data = a
    publisher.publish(data_to_send)

    gap_time = 0
    start = time.time()
    while gap_time < 1:
        # read the external force during the movement command executing
        gap_time = time.time() - start

    print(a, gap_time)

if __name__ == '__main__':
    # load expert dataset
    exp_data = dataset.ExpertDataset(mode="trajectory",  dataset_path = "/home/atakan/catkin_ws/src/cobot_controllers/scripts/expert_dataset/datasets/without_extra_grabber/npys/")
    exp_traj = exp_data.trajs[0]
    print(exp_traj)
    
    # init ros
    rospy.init_node('tf_broadcaster', anonymous=True)
    rospy.Subscriber('/franka_state_controller/F_ext', WrenchStamped, update_ext_force)
    rospy.Publisher('/joint_states', JointState, run_joint_settings)


    #for i in range(10):
    #    print(i)
    rate = rospy.Rate(20.0)
    rospy.sleep(1.0)
    rate.sleep()

    j_id = 0

    pub = rospy.Publisher('/joint_states', Float32MultiArray, queue_size=7)
    print("Publisher...done")


    data_to_send = Float32MultiArray()
    rate = rospy.Rate(20.0)

    while not rospy.is_shutdown() and j_id < len(exp_traj):
        j_arr=exp_traj[j_id]
        data_to_send.data = j_arr
        run_joint_settings(data_to_send, pub)
        rate.sleep()
        a()
    

# moveit_commander.move_group.MoveGroupCommander.set_pose_target    (
