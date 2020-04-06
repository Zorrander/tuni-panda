#!/usr/bin/env python
import time
import rospy
from cobot_control.planner import RosPlanner
from cobot_control.dispatcher import Dispatcher
from cobot_tuni_msgs.msg import Command, Object, Collection

class Tutorial(object):

    def __init__(self):
        rospy.init_node('handover_demo', anonymous=True)
        self.planner = RosPlanner('localhost:3030', 'Panda')
        self.obj_detector_publisher = rospy.Publisher('/object', Object, queue_size=10)
        self.cmd_publisher = rospy.Publisher('/nlp_command', Command, queue_size=10)
        self.action_publisher = rospy.Publisher('/semantic_action', Command, queue_size=10)
        self.plan_subscriber = rospy.Subscriber('/semantic_command', Command, self.create_plan)

    def create_plan(self, cmd):
        print("create_plan {}".format(cmd.action))
        self.planner.create_plan(cmd.action)

    def apply_timestamp(self, cmd, dispatcher):
        print("done {}".format(cmd.action))
        dispatcher.apply_timestamp(cmd.action)

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
        print "============ Press `Enter` to visualize resulting changes in the KB ..."

        print "============ Press `Enter` to execute the command ..."
        raw_input()
        dispatcher = Dispatcher(tuto.planner.retrieve_plan(), 'localhost:3030', 'Panda')
        action_subscriber = rospy.Subscriber('/action_command', Command, tuto.apply_timestamp, (dispatcher))
        for task, object in dispatcher.dispatch():
            print("dispatching {} -{}".format(task,object))
            tuto.action_publisher.publish(Command(task, object))
        time.sleep(2)
        print "============ Press `Enter` to release the object ..."
        raw_input()

        print "============ Press `Enter` to clean the knowledge base ..."
        raw_input()

        print "============ Demo complete!"
    except Exception as e:
        print(e)

if __name__ == '__main__':
    main()
