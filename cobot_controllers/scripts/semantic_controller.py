#!/usr/bin/env python

import rospy
from cobot_controllers.semantic_controller import SemanticController
from cobot_tuni_msgs.msg import Command
from cobot_vision.srv import EstimatePose

def action_callback(action_msg, sem_controller):
    sem_controller.interpret(action_msg.action, action_msg.target)


if __name__ == '__main__':
    try:
        rospy.init_node('controller_node')
        host = rospy.get_param('/host', 'localhost:3030')
        dataset = rospy.get_param('/dataset', 'Panda')
        rospy.wait_for_service('estimate_pose')
        pose_estimator = rospy.ServiceProxy('calculate_tag_pose', EstimatePose)
        sem_controller = SemanticController(host, dataset, pose_estimator)
        rospy.Subscriber("/semantic_action", Command, action_callback, (sem_controller))
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
