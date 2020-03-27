from sem_server_ros.server_com import FusekiEndpoint


class SemanticInterpreter():

    def __init__(self):
        self.sem_server = FusekiEndpoint()

    def check_syntax(self, symbol):
        symbol_sem = sem_server.test_data(symbol)
        if not symbol_sem:
            raise Exception("I don't know what {} means".format(symbol))

    def check_action(self, symbol):
        self.check_syntax(symbol)
        action_sem = sem_server.read_data(symbol)
        print(action_sem)
        steps = [object for subject, predicate, object in action_sem if predicate=="cogrob:causes"]
        print("found steps:{}".format(steps))
        return steps

    def check_manipulation_preconditions(self, object_symbol):
        self.check_syntax(object_symbol)
        object_seen = False
        object_sem = sem_server.read_data(object_symbol)
        for triple in object_sem:
            if triple.predicate == "rdf:type" and triple.object == cmd_msg.target:
                object_seen = True
        if not object_seen:
            speak.publish("I cannot see a {}".format(cmd_msg.target))
            raise ("I can't see a {}".format(object_symbol))
        return object_sem

    def new_command(self, action, target):
        steps_sem = self.check_action(action)
        target_sem = self.check_manipulation_preconditions(target)
