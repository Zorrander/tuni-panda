from sem_server_ros.server_com import FusekiEndpoint

class SemanticVision:

    def __init__(self):
        self.sem_server = FusekiEndpoint()

    def new_object(self, object_type):
        self.sem_server.create_instance(object_type)

    def store_width(self, object, pose):
        pass

    def store_pose(self, object, pose):
        pass
