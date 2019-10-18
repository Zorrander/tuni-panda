#!/usr/bin/env python

import rospy
from moveit_commander import MoveGroupCommander
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import String


commander = MoveGroupCommander('panda_arm')

def execute(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    commander.set_named_target('ready')
    commander.go()

def wait_for_commands():
    rospy.Subscriber("command", String, execute)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('basic_control')
    wait_for_commands()
