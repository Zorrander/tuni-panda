from sem_server_ros.server_com import FusekiEndpoint

class SemanticController():

    def __init__(self):
        self.sem_server = FusekiEndpoint()

    def interpret(self, task):
        ''' decide what to do '''
        pass
