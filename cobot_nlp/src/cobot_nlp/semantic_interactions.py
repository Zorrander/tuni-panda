from sem_server_ros.server_com import FusekiEndpoint
from sem_server_ros.queries import QueryTemplateEngine
from cobot_tuni_msgs.msg import Collection, Command

class SemanticInterpreter():

    def __init__(self, publisher, host, dataset):
        self.sem_server = FusekiEndpoint(host, dataset)
        self.query_engine = QueryTemplateEngine(__file__)
        self.publisher = publisher

    def notify_listeners(self, new_cmd):
        self.publisher.publish(new_cmd)

    def new_command(self, action, target):
        instance = self.sem_server.create_instance("Command")
        self.sem_server.add_data(instance, "cogrob:has_action", "'"+action.lower()+"'" )
        self.sem_server.add_data(instance, "cogrob:has_target",  "'"+target.lower()+"'" )
        self.notify_listeners(Command(instance, target.lower()))
