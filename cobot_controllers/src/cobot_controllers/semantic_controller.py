from sem_server_ros.server_com import FusekiEndpoint

class SemanticController():

    def __init__(self, host, dataset):
        self.sem_server = FusekiEndpoint(host, dataset)

    def interpret(self, task):
        ''' decide what to do '''
        pass
