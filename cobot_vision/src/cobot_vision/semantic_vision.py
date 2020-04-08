from sem_server_ros.server_com import FusekiEndpoint
from sem_server_ros.queries import QueryTemplateEngine
from cobot_tuni_msgs.msg import  Command

class SemanticVision:

    def __init__(self, publisher, host, dataset):
        self.sem_server = FusekiEndpoint(host, dataset)
        self.query_engine = QueryTemplateEngine(__file__)
        self.publisher = publisher

    def notify_listeners(self, cmd, obj):
        print("Sending {}".format(Command(cmd, obj)))
        self.publisher.publish(Command(cmd, obj))

    def new_object(self, object_type, width, id):
        instance = self.sem_server.create_instance(object_type)
        self.sem_server.add_data(instance, "cogrob:hasWidth", width)
        self.sem_server.add_data(instance, "cogrob:hasId", id)

    def anchor_object(self, cmd, target):
        print(cmd)
        template = self.query_engine.load_template('describe_instance_target.rq')
        query = self.query_engine.generate(template, "'"+target+"'")
        result = self.sem_server.select_data(query)
        obj = [x['obj']['value'] for x in result]
        print("VISION RESULT {}".format(obj))
        if not obj:
            print("I cannot see a {}".format(target))
        elif len(obj)>1:
            print("Ambiguous scene, I can see more than one {}".format(target))
        else:
            print("I can see a {}".format(target))
            obj = 'cogrob:'+str(obj[0]).split('#')[1]
            self.sem_server.add_data(cmd, "cogrob:has_target",  obj )
            template = self.query_engine.load_template('describe_command_results.rq')
            query = self.query_engine.generate(template, cmd)
            result = self.sem_server.select_data(query)
            task = 'cogrob:'+str(result[0]['task']['value']).split('#')[1] 
            self.notify_listeners(task, obj)
