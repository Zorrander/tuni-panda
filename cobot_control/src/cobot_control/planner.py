from simple_net.planner import Planner
from sem_server_ros.server_com import FusekiEndpoint
from sem_server_ros.queries import QueryTemplateEngine

class RosPlanner:

    def __init__(self, host, dataset):
        self.planner = Planner()
        self.sem_server = FusekiEndpoint(host, dataset)
        self.query_engine = QueryTemplateEngine(__file__)

    def retrieve_plan(self):
        try:
            return self.planner.base_solution
        except Exception as e:
            print(e)

    def find_available_steps(self, time):
        try:
            return self.planner.find_available_steps(time)
        except Exception as e:
            print(e)

    def create_plan(self, sem_command):
        try:
            template = self.query_engine.load_template('select_semantic_plan.rq')
            query = self.query_engine.generate(template, sem_command)
            result = self.sem_server.construct_data(query)
            print("R E S U L T  P L A N:{}".format(result))
            return self.planner.create_plan(result)
        except:
            print(self.planner.base_solution)
            print("Error during planning")

    def print_plan(self):
        self.planner.print_plan()
