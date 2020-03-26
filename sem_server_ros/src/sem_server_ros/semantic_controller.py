from cobot_controllers.arm import Arm
from cobot_controllers.gripper import Gripper
from .server_com import ROSFusekiServer

class SemanticController():

    def __init__(self):
        self.arm = Arm()
        self.gripper = Gripper()
        self.sem_server = ROSFusekiServer()


    def interpret(self, task):
        ''' decide what to do '''
        self.sem_server.identify_action(task)
