from owlready2 import *
import time

class Robot(Thing):
    def move(self):
        self.isHoldingSomething = True
        print("I like to move it move it")
        time.sleep(3)

    def open_gripper(self):
        self.isHoldingSomething = False
        print("Knock knock")
        time.sleep(3)

    def close_gripper(self):
        self.isHoldingSomething = True
        print("Shut it ")
        time.sleep(3)


class IdleOperator(Thing):
    def run(self, world, primitive):
        print("Et les shadocks pompaient")
        return True

class MoveOperator(Thing):
    def run(self, world, primitive):
        robot = world.search(type = world['http://onto-server-tuni.herokuapp.com/Panda#Robot'])
        Robot(robot).move()


class OpenOperator(Thing):
    def run(self, world, primitive):
        robot = world.search(type = world['http://onto-server-tuni.herokuapp.com/Panda#Robot'])
        Robot(robot).open_gripper()

class CloseOperator(Thing):
    def run(self, world, primitive):
        robot = world.search(type = world['http://onto-server-tuni.herokuapp.com/Panda#Robot'])
        Robot(robot).close_gripper()
