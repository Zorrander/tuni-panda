#!/usr/bin/env python

import rospy
from cobot_control.pick_and_place import PickAndPlaceServer

if __name__ == '__main__':
    rospy.init_node('pick_and_place_as')
    server = PickAndPlaceServer()
    rospy.spin()
