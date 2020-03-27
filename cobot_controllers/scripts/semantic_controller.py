#!/usr/bin/env python

import rospy
from cobot_controllers.semantic_controller import SemanticController

if __name__ == '__main__':
    try:
        rospy.init_node('controller_node')
        sem_controller = SemanticController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
