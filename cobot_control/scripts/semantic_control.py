#!/usr/bin/env python

import rospy
from cobot_control.planner import RosPlanner
from cobot_control.dispatcher import Dispatcher

if __name__ == "__main__":
    rospy.init_node('semantic_control')
    planner = RosPlanner()
    dispatcher = Dispatcher()
    rospy.spin()
