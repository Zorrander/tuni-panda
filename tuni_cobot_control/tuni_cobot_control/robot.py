from owlready2 import *


class Robot(Thing):
    def move(self):
        pass

    def reach(self):
        self.isCapableOfReaching = True

    def open_gripper(self):
        del self.__dict__['isHoldingSomething']
        if self.isWaitingForSomething:
            self.isWaitingForSomething = False

    def close_gripper(self):
        self.isHoldingSomething = True
        del self.__dict__['isCapableOfReaching']

    def wait_for(self):
        self.isWaitingForSomething = True

    def print_status(self):
        meta = ['namespace', 'storid']
        for key in self.__dict__:
            if not key[0] == "_" and not key in meta:
                print("{} - {}".format(key, self.__dict__[key]))