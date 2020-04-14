from tuni_cobot_control.workspace import Workspace
from tuni_cobot_control.object import Object

class contains(Workspace >> Object):
    def compare(current, objective):
        return ((len(current) - objective.cardinality, current))
