from owlready2 import *

class State(Thing):

    def __init__(self, onto):
        self._is_holding_something = True if onto.Panda.isHoldingSomething == True else False
        self._received_command = False

    @property
    def is_holding_something(self):
        return self._is_holding_something

    @is_holding_something.setter
    def is_holding_something(self, value):
        print("OOOOOOOH {}".format(value))
        self._is_holding_something = value

    @property
    def received_command(self):
        return self._received_command

    @received_command.setter
    def received_command(self, value):
        print("AAH {}".format(value))
        self._received_command = value

    '''
    class IsHoldingSomething(Thing):
        def evaluate(self):


        def apply(self):
            self.onto.Panda.isHoldingSomething = True
            self.onto.Panda.isNotHoldingSomething = False

    class IsNotHoldingSomething(Thing):
        def evaluate(self):
            return True if self.onto.Panda.isNotHoldingSomething == True else False

        def apply(self):
            self.onto.Panda.isNotHoldingSomething = True
            self.onto.Panda.isHoldingSomething = True

    class IsCapableOfReaching(Thing):
        def apply(self):
            self.onto.Panda.canReach = True

    class IsNotCapableOfReaching(Thing):
        def apply(self):
            self.onto.Panda.canReach = False

    class IsReadyToBeTaken(Thing):
        def evaluate(self):
            return False

        def apply(self):
            pass

    class IsNotReadyToBeTaken(Thing):
        def apply(self):
            pass

    class ReceivedHandoverCommand(Thing):
        def evaluate(self):
            print("What")
            result = True if self.onto.search(type = self.onto.HandoverCommand) else False
            print("ReceivedHandoverCommand -- {}".format(result))
            return result

        def apply(self):
            pass
    '''
