from sem_server_ros.server_com import FusekiEndpoint
from sem_server_ros.queries import QueryTemplateEngine
from cobot_tuni_msgs.msg import Collection, Triple

class SemanticInterpreter():

    def __init__(self, publisher):
        self.sem_server = FusekiEndpoint()
        self.query_engine = QueryTemplateEngine(__file__)
        self.publisher = publisher

    def notify_listeners(self, collection):
        self.publisher.publish(collection)

    def check_action(self, symbol):
        template = self.query_engine.load_template('describe_command_results.rq')
        query = self.query_engine.generate(template, "'"+symbol+"'")
        result = self.sem_server.read_data(query)
        #steps = [object for subject, predicate, object in result if predicate=="cogrob:causes"]
        #print("found steps:{}".format(steps))
        return result

    def check_manipulation_preconditions(self, object_symbol):
        template = self.query_engine.load_template('describe_instance_target.rq')
        query = self.query_engine.generate(template, "cogrob:"+object_symbol)
        result = self.sem_server.read_data(query)
        object_seen = True if result else False
        if not object_seen:
            print("I cannot see a {}".format(object_symbol))
        else:
            print("I can see a {}".format(object_symbol))
        return result

    def new_command(self, action, target):
        instance = self.sem_server.create_instance("Command")
        self.sem_server.add_data(Triple(instance, "cogrob:has_action", "'"+action.lower()+"'" ))
        self.sem_server.add_data(Triple(instance, "cogrob:has_target",  "'"+target.lower()+"'" ))
        action_sem = self.check_action(target.lower())
        target_sem = self.check_manipulation_preconditions(target)
        list_triples = [Triple(subject, predicate, object) for subject, predicate, object in action_sem+target_sem]
        self.notify_listeners(Collection(list_triples))
