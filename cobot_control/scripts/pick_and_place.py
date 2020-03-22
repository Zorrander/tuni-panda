import time
import rospy
import actionlib
from cobot_control.msg import PickAndPlaceAction, PickAndPlaceFeedback, PickAndPlaceResult
from cobot_controllers.srv import *
from cobot_vision.srv import TagPose
#from sem_server_ros.srv import *

class PickAndPlaceServer(object):
    def __init__(self):
        self._as = actionlib.SimpleActionServer(
            rospy.get_name(),
            PickAndPlaceAction,
            execute_cb=self.execute_cb,
            auto_start = False)

        rospy.wait_for_service('grasp')
        #rospy.wait_for_service('pose_query')
        #rospy.wait_for_service('width_query')
        rospy.wait_for_service('move_gripper')
        rospy.wait_for_service('calculate_tag_pose')
        rospy.wait_for_service('go_to_cartesian_goal')

        self.grasp_client        = rospy.ServiceProxy("grasp", Grasp)
        #self.width_query         = rospy.ServiceProxy("width_query", WidthQuery)
        #self.pose_query          = rospy.ServiceProxy('pose_query', PoseQuery)
        self.move_gripper_client = rospy.ServiceProxy("move_gripper", MoveGripper)
        self.tag_detector        = rospy.ServiceProxy('calculate_tag_pose', TagPose)
        self.move_to_pose        = rospy.ServiceProxy('go_to_cartesian_goal', ReachCartesianPose)

        self._as.start()
        rospy.loginfo("Action server %s started." % rospy.get_name())

    def execute_cb(self, goal):
        print("NEW GOAL place {} to {}".format(goal.object_name, goal.place_name))

        try:
            ### Approach
            self._as.publish_feedback(PickAndPlaceFeedback("Approaching the object"))
            tag_pose_resp = self.tag_detector(1)
            # tag_pose_resp = self.pose_query(goal.object_name).pose
            tag_pose_resp.position.z += 0.20
            self.move_to_pose(tag_pose_resp.tag_pose)
            time.sleep(1)
            ### Pick
            # pre-grasp
            self._as.publish_feedback(PickAndPlaceFeedback("Pre-grasp phase"))
            # for now only a vertical motion
            tag_pose_resp.position.z -= 0.175
            self.move_to_pose(tag_pose_resp.tag_pose)
            # grasp
            self._as.publish_feedback(PickAndPlaceFeedback("Grasping the object"))
            # obj_width = self.width_query(goal.object_name).width
            self.grasp_client(1, 20.0)
            # post-grasp
            self._as.publish_feedback(PickAndPlaceFeedback("Lifting object up"))
            tag_pose_resp.position.z += 0.175
            self.move_to_pose(tag_pose_resp.tag_pose)
            time.sleep(1)

            ### Place
            # transport
            self._as.publish_feedback(PickAndPlaceFeedback("Transporting the object"))
            #goal_pose_resp = self.pose_query(goal.place_name).pose
            goal_pose_resp = self.tag_detector(3)
            goal_pose_resp.position.z += 0.20
            self.move_to_pose(tag_pose_resp.tag_pose)
            time.sleep(1)
            # put down
            self._as.publish_feedback(PickAndPlaceFeedback("Putting the object down"))
            goal_pose_resp.position.z -= 0.175
            self.move_to_pose(tag_pose_resp.tag_pose)
            # release
            self._as.publish_feedback(PickAndPlaceFeedback("Releasing the object down"))
            self.move_gripper_client(20.0, 0.08)
            # post-relieve
            self._as.publish_feedback(PickAndPlaceFeedback("Approaching the object"))
            goal_pose_resp.position.z += 0.175
            self.move_to_pose(tag_pose_resp.tag_pose)

            self._as.set_succeeded(PickAndPlaceResult(True))
        except:
            '''
            if self._as.is_preempt_requested():
                self._as.set_preempted()
                success = False
                break
            '''
            self._as.set_succeeded(PickAndPlaceResult(False))
