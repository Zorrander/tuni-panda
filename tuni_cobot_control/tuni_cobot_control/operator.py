from owlready2 import *


class IdleOperator(Thing):
    def run(self, world):
        print("Et les shadocks pompaient")
        return True

class MoveOperator(Thing):
    def run(self, world, place):
        pass

class OpenOperator(Thing):
    def run(self, world):
        pass

class CloseOperator(Thing):
    def run(self, world, object):
        pass
