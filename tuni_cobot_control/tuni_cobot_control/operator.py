from tuni_cobot_control.state import *


class IdleOperator():
    def run(self, state_handler):
        state_handler.is_holding_something(True)
        print("Et les shadocks pompaient")


class MoveOperator():
    def run(self):
        pass

class OpenOperator():
    def run(self):
        pass
class CloseOperator():
    def run(self):
        pass
