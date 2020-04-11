from owlready2 import *

class ReceivedHandoverCommand(Thing):
    def evaluate(self, world):
        cmd = world.search(type = world['http://onto-server-tuni.herokuapp.com/Panda#HandoverCommand'])
        return True if cmd else False

class IsHoldingSomething(Thing):
    def evaluate(self, world):
        holds = world.search(isHoldingSomething = True)
        return True if holds else False

class IsNotHoldingSomething(Thing):
    def evaluate(self, world):
        pass

class IsCapableOfReaching(Thing):
    def evaluate(self, world):
        pass

class IsNotCapableOfReaching(Thing):
    def evaluate(self, world):
        pass

class IsReadyToBeTaken(Thing):
    def evaluate(self, world):
        return False

class IsNotReadyToBeTaken(Thing):
    def evaluate(self, world):
        pass
