#!/usr/bin/env python

import rospy
from moveit_commander import MoveGroupCommander
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import  Empty


commander = MoveGroupCommander('panda_arm')

def execute(vide):
    commander.set_named_target('ready')
    commander.go()

def wait_for_commands():
    rospy.Subscriber("command", Empty,execute)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('cobot_reset')
    wait_for_commands()
