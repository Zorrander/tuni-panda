import rospy
import actionlib
import franka_gripper.msg
from control_msgs.msg import GripperCommandActionGoal, GripperCommandGoal, GripperCommand

class Gripper():

    def __init__(self):
        self.move_client   = actionlib.SimpleActionClient('/franka_gripper/move', franka_gripper.msg.MoveAction)
        self.stop_client   = actionlib.SimpleActionClient('/franka_gripper/homing', franka_gripper.msg.StopAction)
        self.grasp_client  = actionlib.SimpleActionClient('/franka_gripper/grasp', franka_gripper.msg.GraspAction)
        self.homing_client = actionlib.SimpleActionClient('/franka_gripper/homing', franka_gripper.msg.HomingAction)

        self.generic_grasp_client =  rospy.Publisher('/franka_gripper/gripper_action/goal', GripperCommandActionGoal, queue_size=10)

        self.move_client.wait_for_server()
        rospy.loginfo("Move gripper client connected")
        self.stop_client.wait_for_server()
        rospy.loginfo("Stop gripper client connected")
        self.grasp_client.wait_for_server()
        rospy.loginfo("Grasping client connected")
        self.homing_client.wait_for_server()
        rospy.loginfo("Homing gripper client connected")

    def homing(self):
        goal = franka_gripper.msg.HomingActionGoal(goal={})
        self.homing_client.send_goal(goal)
        self.homing_client.wait_for_result()
        return self.homing_client.get_result()

    def move(self, speed=20.0, width=0.08):
        self.move_client.send_goal(
            franka_gripper.msg.MoveGoal(
                width,
                speed
            )
        )
        self.move_client.wait_for_result()
        return self.move_client.get_result()

    def stop(self):
        pass

    def grasp(self, force, width, speed=20.0, epsilon_inner=0.001, epsilon_outer=0.001):
        ''' width, epsilon_inner, epsilon_outer, speed, force '''
        self.grasp_client.send_goal(
            franka_gripper.msg.GraspGoal(
                width,
                franka_gripper.msg.GraspEpsilon(
                    epsilon_inner,
                    epsilon_outer
                ),
                speed,
                force
            )
        )
        self.grasp_client.wait_for_result()
        return self.grasp_client.get_result()

    def grasp2(self, width, force):
        ''' width, epsilon_inner, epsilon_outer, speed, force '''
        grasp_goal = GripperCommandGoal()
        grasp_goal.command.position = float(width)
        grasp_goal.command.max_effort = float(force)
        grasp_msg = GripperCommandActionGoal(goal=grasp_goal)
        self.generic_grasp_client.publish(grasp_msg)
