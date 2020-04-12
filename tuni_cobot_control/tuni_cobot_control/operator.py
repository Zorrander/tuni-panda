from owlready2 import *
import time


class IdleOperator(Thing):
    def run(self, world, primitive, robot):
        if primitive.name == "WaitForTask":
            robot.wait_for()
        elif primitive.name == "IdleTask" and robot.isWaitingForSomething:
            print("Waiting...")
        else:
            print("Nothing to do for now...")

class MoveOperator(Thing):
    def run(self, world, primitive, robot):
        if primitive.name == "ReachTask":
            robot.reach()
        else:
            robot.move()

class OpenOperator(Thing):
    def run(self, world, primitive, robot):
        robot.open_gripper()

class CloseOperator(Thing):
    def run(self, world, primitive, robot):
        robot.close_gripper()
