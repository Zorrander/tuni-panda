import rospy
import actionlib
from cobot_control.msg import PickAndPlaceAction, PickAndPlaceFeedback, PickAndPlaceResult
from cobot_controller.srv import *

class PickAndPlaceServer(object):
    def __init__(self):
        self._as = actionlib.SimpleActionServer(
            rospy.get_name(),
            PickAndPlaceAction,
            execute_cb=self.execute_cb,
            auto_start = False)

        self._as.start()
        rospy.loginfo("Action server %s started." % rospy.get_name())
        self.reach_tag_client = rospy.ServiceProxy('add_two_ints', AddTwoInts)


    def execute_cb(self, goal):
        success = True
        rate = rospy.Rate(1)
        '''
        if self._as.is_preempt_requested():
            self._as.set_preempted()
            success = False
            break
        '''
        #self._as.publish_feedback(PickAndPlaceFeedback(i))
        #rate.sleep()
        print("NEW GOAL place {} to {}".format(goal.object_name, goal.place_name))
        resp1 = add_two_ints(x, y)
        return resp1.sum
        if success:
            self._as.set_succeeded(PickAndPlaceResult(True))
