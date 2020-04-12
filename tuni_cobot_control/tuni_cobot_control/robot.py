from owlready2 import *


class Robot(Thing):
    def move(self):
        self.isCapableOfReaching = True

    def open_gripper(self):
        self.isHoldingSomething = False

    def close_gripper(self):
        self.isHoldingSomething = True
