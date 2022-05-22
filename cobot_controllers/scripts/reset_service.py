#!/usr/bin/env python3

import rospy
from cobot_msgs.srv import *
from cobot_controllers.srv import *
from sensor_msgs.msg import JointState
import time


def move_to_target(joint_values):
    print("GOAL")
    print(list(joint_values))

    target_pub = rospy.Publisher("/new_target", JointState, queue_size=10);
    target_msg = JointState()
    target_msg.position = joint_values
    target_pub.publish(target_msg)



def move_to_reset_pose(reset_time):
    gripper_release_srv = rospy.ServiceProxy('/move_gripper', MoveGripper)
    gripper_grasp_srv = rospy.ServiceProxy('/grasp', Grasp)

    arm_joints_SOCKET = [0.17604044847321085, -0.004486328642619283, -0.6682706695587933, -2.449662917020028, -0.01548445735954576, 2.6170337975819904, 1.8612127985672815]
    arm_joints_GRAB = [0.09552836457114863,   0.4582943016897168,   -0.49671672280629475,   -2.3144507059799997 ,   0.2909820540878507,   2.7186551869710285,   1.6897436378393556]
    arm_joints_PANEL = [0.40683915349204014, 0.05011774442954318, -0.5303004122417656, -2.5375107996756565, -0.014617423742530854, 2.6364270890547146, 2.229896400718366]
    arm_joints_INIT =  [0.3853664 , 0.49662741, -0.44943702, -2.27425249  , 0.06632516, 2.99395411,2.39860492]
    
    # Release 
    print('release')
    gripper_release_srv(MoveGripperRequest(20.0, 0.08))

    # Move to socket's above loc 
    print('move to the top of socket')
    move_to_target(arm_joints_SOCKET)

    print('wait for manual reset')
    time.sleep(2)


    print('move to grab')
    # Go get the object
    #arm_joints = self.group.get_current_joint_values()
    move_to_target(arm_joints_GRAB)
    time.sleep(2)
    print('grab')
    req = GraspRequest()
    req.width = 0.045
    gripper_grasp_srv(req)

    # Move to socket's above loc again to avoid collision
    print('move to socket up')        
    move_to_target(arm_joints_SOCKET)
    time.sleep(2)
    print('move to the top of panel')
    move_to_target(arm_joints_PANEL)
    time.sleep(2)
    print("move_to_reset_pose")
    #arm_joints = self.group.get_current_joint_values()
    return move_to_target(arm_joints_INIT)


def handle_reset(req):
    print("handle_reset")
    ee_pose, ee_orientation = move_to_reset_pose(req.reset_time)
    return ResetEnvResponse(ee_pose, ee_orientation)



if __name__ == '__main__':
    rospy.init_node('reset_service', anonymous=True)

    reset_service = rospy.Service('/reset_env', ResetEnv, handle_reset)
    rospy.wait_for_service('/reset_env')
    print("/reset_env ready")

    rospy.spin()
