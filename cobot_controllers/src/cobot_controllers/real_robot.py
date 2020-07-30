from cobowl.robot import CollaborativeRobotInterface
from cobot_controllers.arm import Arm
from cobot_controllers.gripper import Gripper

class RealCollaborativeRobot(CollaborativeRobotInterface):

    def __init__(self, knowledge_base_path):
        self.arm = Arm()
        self.gripper = Gripper()
        super().__init__(knowledge_base_path)

    def move_operator(self, target):
        def move_to():
            print("Moving to {}...".format(target))
            # go_to_cartesian_goal (0=storage, 1=handover)
            # go_to_joint_space_goal(joint target)
        return move_to

    def close_operator(self, target):
        def grasp():
            print("Grasping {}...".format(target))
            self.gripper.grasp2( 0.01, 20.0)  # width, force
        return grasp

    def open_operator(self, target):
        def release():
            print("Releasing...{}...".format(target))
            self.gripper.move()
        return release

    def idle_operator(self):
        def wait():
            print("Waiting...")
        return wait

    def stop_operator(self):
        def stop():
            print("Stopping...")
        return stop

    def reset_operator(self):
        def reset():
            print("Reseting...")
            # publish error recovery message
            self.arm.move_to('ready')
        return reset
