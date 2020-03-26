from .server_com import ROSFusekiServer

class SemanticInterpreter():

    def __init__(self):
        self.sem_server = ROSFusekiServer()

    def check_syntax(symbol):
        triple = Triple()
        triple.subject = symbol
        symbol_sem = sem_server.test_data(symbol)
        if not symbol_sem:
            speak.publish("I cannot understand {}".format(symbol))
            raise Exception("I don't know what {} means".format(symbol))

    def check_action(symbol):
        check_syntax(symbol)
        action_sem = sem_server.read_data(symbol)
        return action_sem

    def check_manipulation_preconditions(object_symbol):
        check_syntax(object_symbol)
        object_seen = False
        object_sem = sem_server.read_data(object_symbol)
        for triple in object_sem:
            if triple.predicate == "rdf:type" and triple.object == cmd_msg.target:
                object_seen = True
        if not object_seen:
            speak.publish("I cannot see a {}".format(cmd_msg.target))
            raise ("I can't see a {}".format(object_symbol))
        return object_sem
