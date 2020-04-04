from sem_server_ros.server_com import FusekiEndpoint
from sem_server_ros.queries import QueryTemplateEngine


class SemanticController():

    def __init__(self, host, dataset):
        self.sem_server = FusekiEndpoint(host, dataset)
        self.query_engine = QueryTemplateEngine(__file__)

    def interpret(self, action, target):
        ''' decide what to do '''
        template = self.query_engine.load_template('select_next_move.rq')
        query = self.query_engine.generate(template, 'cogrob:'+str(action).split('#')[1][:-1])
        print(query)
        result = self.sem_server.select_data(query)
        print("next move: {}".format(result))
