from owlready2 import *
import time



class IdleOperator(Thing):
    def run(self, world, primitive, robot):
        return True

class MoveOperator(Thing):
    def run(self, world, primitive, robot):
        robot.move()

class OpenOperator(Thing):
    def run(self, world, primitive, robot):
        robot.open_gripper()

class CloseOperator(Thing):
    def run(self, world, primitive, robot):
        robot.close_gripper()
