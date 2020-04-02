#!/usr/bin/env python

import rospy
from cobot_controllers.semantic_controller import SemanticController

if __name__ == '__main__':
    try:
        rospy.init_node('controller_node')
        host = rospy.get_param('/host', 'localhost:3030')
        dataset = rospy.get_param('/dataset', 'Panda')
        sem_controller = SemanticController(host, dataset)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
