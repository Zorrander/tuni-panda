#! /usr/bin/env python

import rospy
from franka_control.msg import ErrorRecoveryActionGoal
from franka_msgs.msg import FrankaState
from std_msgs.msg import Empty

def recover():
    pub = rospy.Publisher('/franka_control/error_recovery/goal', ErrorRecoveryActionGoal, queue_size=10)
    goal = ErrorRecoveryActionGoal()
    goal.goal={}
    pub.publish(goal)

def error_recovery_client(state):
    if not (state.robot_mode in [1, 2]):
        prompt = "Try to recover from "
        if state.robot_mode == 4:
            prompt += "kReflex"
        prompt+= "? y/n"
        print(prompt)
        answer = raw_input()
        if answer=='y':
            recover()

if __name__ == '__main__':
    rospy.init_node('cobot_error_recovery')
    rospy.Subscriber("/franka_state_controller/franka_states", FrankaState, error_recovery_client)
    rospy.spin()
