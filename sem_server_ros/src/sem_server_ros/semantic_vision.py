from .server_com import ROSFusekiServer


class SemanticVision:

    def __init__(self):
        self.jena_fuseki_server = ROSFusekiServer()

    def new_object(self, object_type, width, pose):
        instance = self.new_instance(object_type)
        self.store_width(instance, width)
        self.store_pose(instance, pose)

    def store_width(self, object, pose):
        self.jena_fuseki_server.add_data(width)

    def store_pose(self, object, pose):
        self.jena_fuseki_server.add_data(pose)
