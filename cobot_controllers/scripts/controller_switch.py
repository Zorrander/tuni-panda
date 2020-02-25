#!/usr/bin/env python

import rospy
from controller_manager_msgs.srv import SwitchController
from std_msgs.msg import Empty


def switch_controllers(strictness=1):
    rospy.wait_for_service('/controller_manager/switch_controller')
    try:
        print(rospy.get_param("running_controller"))
        switcher = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
        if rospy.get_param("running_controller")=='test_controller':
            switcher(['force_example_controller'], ['test_controller'], strictness)
            rospy.set_param('running_controller', 'force_example_controller')
        else:
            switcher(['test_controller'], ['force_example_controller'], strictness)
            rospy.set_param('running_controller', 'test_controller')
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def switch(vide):
    switch_controllers()

def wait_for_commands():
    rospy.set_param('running_controller', 'force_example_controller')
    rospy.Subscriber("switch_controllers", Empty, switch)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('cobot_control')
    wait_for_commands()
