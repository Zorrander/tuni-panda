from owlready2 import *

class ReceivedHandoverCommand(Thing):
    def evaluate(self, world, robot):
        cmd = world.search_one(type = world['http://onto-server-tuni.herokuapp.com/Panda#HandoverCommand'])
        if cmd:
            destroy_entity(cmd)
            return True
        else:
            return False

class IsHoldingSomething(Thing):
    def evaluate(self, world,robot):
        return True if ('isHoldingSomething' in robot.__dict__ and robot.isHoldingSomething == True) else False

class IsNotHoldingSomething(Thing):
    def evaluate(self, world,robot):
        return True if (not 'isHoldingSomething' in robot.__dict__ or robot.isHoldingSomething == False) else False

class IsCapableOfReaching(Thing):
    def evaluate(self, world,robot):
        return True if ('isCapableOfReaching' in robot.__dict__ and robot.isCapableOfReaching == True) else False

class IsNotCapableOfReaching(Thing):
    def evaluate(self, world,robot):
        return True if (not 'isCapableOfReaching' in robot.__dict__ or robot.isCapableOfReaching == False) else False

class IsReadyToBeTaken(Thing):
    def evaluate(self, world,robot):
        return False

class IsNotReadyToBeTaken(Thing):
    def evaluate(self, world,robot):
        pass
