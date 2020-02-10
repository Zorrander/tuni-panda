#!/usr/bin/env python

import rospy
from moveit_commander import MoveGroupCommander
from std_msgs.msg import  Empty


commander = MoveGroupCommander('panda_arm')

def execute(vide):
    commander.set_named_target('ready')
    commander.go()

def wait_for_commands():
    rospy.Subscriber("move_start", Empty,execute)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('cobot_reset')
    wait_for_commands()
