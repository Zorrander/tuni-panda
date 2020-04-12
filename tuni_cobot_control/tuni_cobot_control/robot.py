from owlready2 import *


class Robot(Thing):
    def move(self):
        self.isCapableOfReaching = True
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
