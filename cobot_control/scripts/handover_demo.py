#!/usr/bin/env python
import time
import rospy
from cobot_control.planner import RosPlanner
from cobot_tuni_msgs.msg import Command, Object, Collection

class Tutorial(object):

    def __init__(self):
        rospy.init_node('handover_demo', anonymous=True)
        self.obj_detector_publisher = rospy.Publisher('/object', Object, queue_size=10)
        self.cmd_publisher = rospy.Publisher('/nlp_command', Command, queue_size=10)
        self.action_publisher = rospy.Publisher('/semantic_action', Command, queue_size=10)
        self.plan_subscriber = rospy.Subscriber('/semantic_command', Command, self.create_plan)
        pose_estimator = rospy.ServiceProxy('calculate_tag_pose', EstimatePose)
        sem_controller = SemanticController(host, dataset, pose_estimator)
        self.planner = RosPlanner('localhost:3030', 'Panda', self.action_publisher, sem_controller)

    def create_plan(self, cmd):
        print("create_plan {}".format(cmd.action))
        self.planner.create_plan(cmd.action)
        self.planner.run(plan)


def main():
    try:
        print "============ Press `Enter` to begin the tutorial by printing general info about each node (press ctrl-d to exit) ..."
        raw_input()
        tuto = Tutorial()
        print "============ Press `Enter` to spawn an object on the workspace ..."
        raw_input()

        tuto.obj_detector_publisher.publish(Object("Cup", 0.07, 1))

        print "============ Press `Enter` to send a command to the robot to handover the previous object ..."
        raw_input()
        action = "Give"
        target = "Cup"

        msg = Command("Give", "Cup")
        tuto.cmd_publisher.publish(msg)

        print "============ Press `Enter` to release the object ..."
        raw_input()

        print "============ Demo complete!"
    except Exception as e:
        print(e)

if __name__ == '__main__':
    main()