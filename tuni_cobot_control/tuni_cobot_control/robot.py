from owlready2 import *


class Robot(Thing):
    def move(self):
        pass

    def reach(self):
        self.isCapableOfReaching = True

    def open_gripper(self):
        self.isHoldingSomething = False
        if self.isWaitingForSomething:
            self.isWaitingForSomething = False

    def close_gripper(self):
        self.isHoldingSomething = True

    def wait_for(self):
        self.isWaitingForSomething = True

    def print_status(self):
        meta = ['namespace', 'storid']
        for key in self.__dict__:
            if not key in meta:
                print("{} - {}".format(key, self.__dict__[key]))
