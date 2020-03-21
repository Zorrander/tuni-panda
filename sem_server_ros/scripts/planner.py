#!/usr/bin/env python

import rospy

if __name__ == "__main__":
    rospy.init_node('sem_server_node')
    s = rospy.Service('add_data',    Update,   add_data)
    s = rospy.Service('select_data', Select,   select_data)
    s = rospy.Service('read_data',   Describe, read_data)
    s = rospy.Service('test_data',   Ask,      test_data)
    s = rospy.Service('remove_data', Update,   remove_data)
    rospy.spin()
